// ===== Arduino MEGA 2560 — motor_controller_mega_v3.ino =====
// Baseado no motor_controller_mega_v2, agora com LOG em microSD (CSV) incluindo timestamp de RTC.
//
// SPI (MEGA):
//  MISO = 50
//  MOSI = 51
//  SCK  = 52
//  SS   = 53 (tem que ficar como OUTPUT)
// Vamos usar CS do SD no pino 10 (comum em módulos/card-shield).
//
// CSV: a cada inicialização cria LOGxxx.CSV novo e grava cabeçalho.
// Linha: ts_iso,ms,volts,pct,temp,humi,rpm,speed_kmh,Ibat,Imot,dutyNow,dutyTarget,maxpct
//
// Pinos de controle iguais ao v2:
// A0 = acelerador
// A8 = RPM (PCINT2 / PK0)
// A2 = Ibat
// A3 = Imot
// D4 = DHT22
// D11 = PWM (Timer1 OC1A)

#include <Arduino.h>
#include <DHT.h>
#include <EEPROM.h>
#include <math.h>
#include <SPI.h>
#include <SD.h>
#include <RtcDS1302.h>

// ---------- Flags de controle ----------
static const bool USE_TEMP_DERATE  = false;
static const bool USE_CURR_DERATE  = false;
static const bool USE_STALL_DETECT = false;

// ---------- Pinos principais ----------
const uint8_t PIN_THROTTLE = A0;
const uint8_t PIN_RPM      = A8;   // tem PCINT no MEGA (PORTK)
const uint8_t PIN_IBAT     = A2;
const uint8_t PIN_IMOT     = A3;
const uint8_t PIN_DHT      = 4;
const uint8_t PIN_PWM      = 11;   // Timer1 OC1A no MEGA

// ---------- SD ----------
const uint8_t PIN_SD_CS    = 10;   // CS do módulo SD
File logFile;
bool sd_ok = false;
unsigned long lastFlushMs = 0;

// ---------- RTC DS1302 (ThreeWire) ----------
const uint8_t PIN_RTC_DATA = 22;
const uint8_t PIN_RTC_CLK  = 23;
const uint8_t PIN_RTC_RST  = 24;
ThreeWire rtcWire(PIN_RTC_DATA, PIN_RTC_CLK, PIN_RTC_RST);
RtcDS1302<ThreeWire> rtc(rtcWire);
bool rtc_ok = false;


// ---------- DHT ----------
#define DHTTYPE DHT22
DHT dht(PIN_DHT, DHTTYPE);

// ---------- PWM ----------
uint16_t PWM_FREQ_HZ   = 1000; // 100..8000

// ---------- Acelerador ----------
const float VREF_ADC = 5.0f;
const int   ADC_MAX  = 1023;
float V_MIN_REAL = 1.10f;
float V_MAX_REAL = 4.25f;
const float V_FAULT_LOW  = 0.30f;
const float V_FAULT_HIGH = 4.90f;

// ---------- Corrente (calib ACS712) ----------
const float ACS_MV_PER_A = 100.0f;
const float ACS_V_OFFSET = 2.5f;

// ---------- Controle / rampas ----------
volatile bool  g_override = false;
volatile int   g_hold_pct = 0;
float dutyNowPct = 0.0f;
float dutyTargetPct = 0.0f;
unsigned long rapidUntilMs = 0;

uint16_t RAPID_RAMP_MS = 250;
float    RAPID_UP_PCTPS= 150;
float    SLEW_UP_PCTPS = 40;
float    SLEW_DN_PCTPS = 60;
uint8_t  START_MIN_PCT = 8;

const uint8_t  DEADZONE_PWM        = 4;
const uint8_t  MIN_START_PWM       = 20;
const uint16_t REST_DEBOUNCE_MS    = 120;
const uint16_t SOFT_RAMP_MS        = 1500;
const uint16_t ACCEL_RAMP_MS       = 250;
const uint8_t  REST_PCT_THRESHOLD  = 2;

bool      atRest          = true;
uint32_t  restSinceMs     = 0;
bool      softCapActive   = false;
uint32_t  softStartMs     = 0;

bool      accelCapActive  = false;
uint32_t  accelStartMs    = 0;
float     accelFromPct    = 0.0f;
float     accelToPct      = 0.0f;
float     lastPedalPct    = 0.0f;

int thr_filtered = 0;
int last_raw_thr = 0;

// ---------- RPM (PCINT em A8 / PK0 / PCINT16) ----------
volatile unsigned long LastTimeWeMeasured = 0;
volatile unsigned long PeriodBetweenPulses = 700000UL;
volatile unsigned long PeriodAverage = 700000UL;
volatile unsigned long PeriodSum = 0;
volatile unsigned int  PulseCounter = 1;
unsigned long FrequencyRaw = 0;
unsigned int AmountOfReadings = 1;
unsigned int ZeroDebouncingExtra = 0;
unsigned long ZeroTimeout = 600000UL; // us

volatile uint8_t rpm_last_state = 0;
unsigned long RPM_MIN_PULSE_US = 200;

const byte numReadings = 2;
unsigned long readings[numReadings] = {0};
unsigned long readIndex = 0, total = 0;

volatile float g_wheel_cm = 50.8f;
volatile uint8_t g_ppr = 1;

float wheel_circumf_m(){ return 3.14159265f * (g_wheel_cm*0.01f); }
float rpm_to_kmh(float rpm){ return rpm * wheel_circumf_m() * 0.06f; }

// ---------- Amostragem ----------
const uint32_t LOOP_MS = 50;   // 20 Hz
const uint32_t DHT_MS  = 2000;
uint32_t lastLoop = 0, lastDht = 0;

// ---------- Estado publicado ----------
float g_volts=0, g_pct=0, g_temp=NAN, g_humi=NAN;
float g_current_bat_a=0, g_current_mot_a=0;
float g_rpm=0, g_speed_kmh=0;

// ---------- Modos ----------
uint8_t g_printmode = 0;
bool    g_step_mode = false;
bool    g_accelS_on = true;
uint16_t g_ramp_delay_ms = 0;

// ---------- Teto de aceleração ----------
uint8_t G_MAX_PCT = 100;

// ---------- Persistência (EEPROM) ----------
struct Cfg {
  uint8_t ver;      // = 2
  float vmin, vmax;
  uint8_t maxpct;
  uint16_t pwm_hz;
  uint16_t rapid_ms;
  float    rapid_up;
  float    slew_up, slew_dn;
  uint8_t  start_min;
} cfg;

void saveCfg(){
  cfg.ver=2;
  cfg.vmin=V_MIN_REAL; cfg.vmax=V_MAX_REAL;
  cfg.maxpct=G_MAX_PCT;
  cfg.pwm_hz=PWM_FREQ_HZ;
  cfg.rapid_ms=RAPID_RAMP_MS; cfg.rapid_up=RAPID_UP_PCTPS;
  cfg.slew_up=SLEW_UP_PCTPS; cfg.slew_dn=SLEW_DN_PCTPS;
  cfg.start_min=START_MIN_PCT;
  EEPROM.put(0, cfg);
}

bool loadCfg(){
  EEPROM.get(0, cfg);
  if (cfg.ver!=2) return false;
  V_MIN_REAL=cfg.vmin; V_MAX_REAL=cfg.vmax;
  G_MAX_PCT=cfg.maxpct;
  PWM_FREQ_HZ=cfg.pwm_hz;
  RAPID_RAMP_MS=cfg.rapid_ms; RAPID_UP_PCTPS=cfg.rapid_up;
  SLEW_UP_PCTPS=cfg.slew_up;  SLEW_DN_PCTPS=cfg.slew_dn;
  START_MIN_PCT=cfg.start_min;
  return true;
}

// ---------- Util ----------
void ack(const char* k, const String& v){
  Serial.print(F("ACK:")); Serial.print(k); Serial.print(' '); Serial.println(v);
}

float pct_from_voltage(float v){
  float pct = (v - V_MIN_REAL) / (V_MAX_REAL - V_MIN_REAL);
  if (!isfinite(pct)) pct = 0.0f;
  return constrain(pct, 0.0f, 1.0f) * 100.0f;
}
float read_voltage_throttle(){
  int raw = analogRead(PIN_THROTTLE);
  last_raw_thr = raw;
  thr_filtered = (thr_filtered*3 + raw*5)/8;
  float v_adc = (thr_filtered * VREF_ADC) / (float)ADC_MAX;
  return v_adc;
}

// Corrente
static float i_zero_bat = 0.0f, i_zero_mot = 0.0f;
static bool  i_zero_bat_ok=false, i_zero_mot_ok=false;

float read_current_filtered(uint8_t analogPin, float &i_zero, bool &zero_ok){
  const uint8_t N=7; int s[N];
  for(uint8_t k=0;k<N;k++){ (void)analogRead(analogPin); s[k]=analogRead(analogPin); }
  for(uint8_t i=0;i<N-1;i++) for(uint8_t j=i+1;j<N;j++) if (s[j]<s[i]){ int t=s[i];s[i]=s[j];s[j]=t; }
  int raw=s[N/2];
  float v=(raw*VREF_ADC)/ADC_MAX;
  float ia=((v-ACS_V_OFFSET)*1000.0f)/ACS_MV_PER_A;

  static float iir_bat=0,iir_mot=0; float &iir=(analogPin==PIN_IBAT)?iir_bat:iir_mot;
  const float alpha=0.20f; iir=(1-alpha)*iir+alpha*ia;

  bool idle=(dutyNowPct<1.0f)&&(g_rpm<5.0f);
  if(idle){ const float beta=0.02f; i_zero=(1-beta)*i_zero+beta*iir; zero_ok=true; }

  float i_corr=iir-(zero_ok?i_zero:0.0f);
  if (fabs(i_corr)<0.08f) i_corr=0.0f;
  return i_corr;
}

// ---------- RTC helpers ----------
void initRTC() {
  rtc.Begin();

  if (!rtc.GetIsRunning()) {
    rtc.SetIsRunning(true);
  }

  if (!rtc.IsDateTimeValid()) {
    // sincroniza automaticamente com horário de compilação
    RtcDateTime compileTime(__DATE__, __TIME__);
    rtc.SetDateTime(compileTime);
  }

  rtc_ok = true;
}


void printTimestamp(File &f){
  if (rtc_ok) {
    RtcDateTime now = rtc.GetDateTime();
    char buf[20];
    // yyyy-mm-dd HH:MM:SS
    snprintf(buf, sizeof(buf),
      "%04u-%02u-%02u %02u:%02u:%02u",
      now.Year(), now.Month(), now.Day(),
      now.Hour(), now.Minute(), now.Second()
    );
    f.print(buf);
  } else {
    f.print(millis());
  }
}


// ---------- Timer1 PWM ----------
void pwm1_set_freq(uint16_t hz){
  hz = constrain(hz,100,8000);
  PWM_FREQ_HZ=hz;
  pinMode(PIN_PWM,OUTPUT);

  uint16_t cs_bits=0x01;
  uint32_t top=(F_CPU/(1UL*hz))-1;
  if(top>65535UL){ cs_bits=0x02; top=(F_CPU/(8UL*hz))-1; }
  if(top>65535UL){ cs_bits=0x03; top=(F_CPU/(64UL*hz))-1; }
  if(top>65535UL){ top=65535UL; }

  TCCR1A=0; TCCR1B=0;
  TCCR1A|=(1<<WGM11)|(1<<COM1A1);
  TCCR1B|=(1<<WGM12)|(1<<WGM13);
  ICR1=(uint16_t)top; OCR1A=0;
  TCCR1B|=cs_bits;
}
void pwm1_set_pct(float pct){
  pct=constrain(pct,0,100);
  uint16_t top=ICR1;
  uint16_t val=(uint16_t)((pct/100.0f)*(float)top);
  if(val>top) val=top;
  OCR1A=val;
}

// ---------- SD helpers ----------
bool sd_create_new_file(){
  char filename[16];
  for (uint16_t i = 0; i < 1000; i++) {
    snprintf(filename, sizeof(filename), "LOG%03u.CSV", i);
    if (!SD.exists(filename)) {
      logFile = SD.open(filename, FILE_WRITE);
      if (logFile) {
        // cabeçalho
        logFile.println(F("ts_iso,ms,volts,pct,temp,humi,rpm,speed_kmh,Ibat,Imot,dutyNow,dutyTarget,maxpct"));
        logFile.flush();
        Serial.print(F("ACK:SD_FILE "));
        Serial.println(filename);
        return true;
      } else {
        return false;
      }
    }
  }
  return false;
}

void sd_log_line(){
  if (!sd_ok || !logFile) return;
  // monta linha CSV
  printTimestamp(logFile);            logFile.print(',');
  logFile.print(millis());            logFile.print(',');
  logFile.print(g_volts,3);           logFile.print(',');
  logFile.print(g_pct,1);             logFile.print(',');
  if (isnan(g_temp)) logFile.print(""); else logFile.print(g_temp,1);
  logFile.print(',');
  if (isnan(g_humi)) logFile.print(""); else logFile.print(g_humi,1);
  logFile.print(',');
  logFile.print(g_rpm,1);             logFile.print(',');
  logFile.print(g_speed_kmh,2);       logFile.print(',');
  logFile.print(g_current_bat_a,2);   logFile.print(',');
  logFile.print(g_current_mot_a,2);   logFile.print(',');
  logFile.print(dutyNowPct,1);        logFile.print(',');
  logFile.print(dutyTargetPct,1);     logFile.print(',');
  logFile.print((int)G_MAX_PCT);
  logFile.println();

  // flush suave
  unsigned long now = millis();
  if (now - lastFlushMs >= 1000) {
    logFile.flush();
    lastFlushMs = now;
  }
}

// ---------- Comandos ----------
void cmd_start(){ g_override=false; g_hold_pct=0; }
void cmd_stop(){  g_override=true;  g_hold_pct=0; dutyTargetPct=0; }
void cmd_hold(int pct){ pct=constrain(pct,0,100); g_override=true; g_hold_pct=pct; dutyTargetPct=pct; }

void process_line(String line){
  line.trim(); if(line.length()==0) return;
  int sp=line.indexOf(' ');
  String head=(sp<0)?line:line.substring(0,sp);
  String rest=(sp<0)?"":line.substring(sp+1);
  head.trim(); rest.trim();

  if (head=="START"){ cmd_start(); ack("START",""); return; }
  if (head=="STOP"){  cmd_stop();  ack("STOP","");  return; }
  if (head=="HOLD"){  cmd_hold(rest.toInt()); ack("HOLD",rest); return; }

  if (head=="SET_MIN_NOW"){ V_MIN_REAL=g_volts; ack("SET_MIN_NOW",String(V_MIN_REAL,3)); return; }
  if (head=="SET_MAX_NOW"){ V_MAX_REAL=g_volts; ack("SET_MAX_NOW",String(V_MAX_REAL,3)); return; }
  if (head=="DEFAULTS"){
    V_MIN_REAL=0.80f; V_MAX_REAL=4.20f; G_MAX_PCT=100; pwm1_set_freq(1000);
    RAPID_RAMP_MS=250; RAPID_UP_PCTPS=150; SLEW_UP_PCTPS=40; SLEW_DN_PCTPS=60; START_MIN_PCT=8;
    ack("DEFAULTS","OK"); return;
  }

  if (head=="SET_WHEEL"){ float cm=rest.toFloat(); cm=constrain(cm,0.5f,200.0f); g_wheel_cm=cm; ack("SET_WHEEL",String(cm,1)); return; }
  if (head=="SET_PPR"){ int n=rest.toInt(); n=constrain(n,1,16); g_ppr=(uint8_t)n; ack("SET_PPR",String(n)); return; }

  if (head=="SET_PWMF"){ uint16_t hz=(uint16_t)rest.toInt(); pwm1_set_freq(hz); ack("SET_PWMF",String(hz)); return; }
  if (head=="SET_STARTMIN"){ uint8_t x=(uint8_t)rest.toInt(); START_MIN_PCT=constrain(x,0,40); ack("SET_STARTMIN",String(START_MIN_PCT)); return; }
  if (head=="SET_RAPIDMS"){ uint16_t ms=(uint16_t)rest.toInt(); RAPID_RAMP_MS=constrain(ms,50,1500); ack("SET_RAPIDMS",String(RAPID_RAMP_MS)); return; }
  if (head=="SET_RAPIDUP"){ float up=rest.toFloat(); RAPID_UP_PCTPS=constrain(up,10.0f,400.0f); ack("SET_RAPIDUP",String(RAPID_UP_PCTPS,1)); return; }
  if (head=="SET_SLEW"){
    int sp2=rest.indexOf(' ');
    float up=rest.substring(0,sp2).toFloat();
    float dn=rest.substring(sp2+1).toFloat();
    SLEW_UP_PCTPS=constrain(up,5.0f,200.0f);
    SLEW_DN_PCTPS=constrain(dn,5.0f,300.0f);
    ack("SET_SLEW",String(SLEW_UP_PCTPS,1)+","+String(SLEW_DN_PCTPS,1));
    return;
  }
  if (head=="SET_ZEROTO"){ unsigned long us=strtoul(rest.c_str(),NULL,10); ZeroTimeout=us; ack("SET_ZEROTO",String(us)); return; }

  // persistência
  if (head=="SAVE"){ saveCfg(); ack("SAVE","OK"); return; }
  if (head=="LOAD_DEFAULTS"){
    V_MIN_REAL=0.80f; V_MAX_REAL=4.20f; G_MAX_PCT=100; pwm1_set_freq(1000);
    RAPID_RAMP_MS=250; RAPID_UP_PCTPS=150; SLEW_UP_PCTPS=40; SLEW_DN_PCTPS=60; START_MIN_PCT=8;
    ack("LOAD_DEFAULTS","OK"); return;
  }

  // diretos
  if (head=="SET_MINV"){ V_MIN_REAL=rest.toFloat(); ack("SET_MINV",String(V_MIN_REAL,3)); return; }
  if (head=="SET_MAXV"){ V_MAX_REAL=rest.toFloat(); ack("SET_MAXV",String(V_MAX_REAL,3)); return; }
  if (head=="SET_MAXPCT"){ int x=constrain(rest.toInt(),1,100); G_MAX_PCT=(uint8_t)x; ack("SET_MAXPCT",String(G_MAX_PCT)); return; }

  if (head=="SET_PRINTMODE"){ if (rest.equalsIgnoreCase("KVP")) g_printmode=0; else if (rest.equalsIgnoreCase("PLOTTER")) g_printmode=1; ack("SET_PRINTMODE",String(g_printmode)); return; }
  if (head=="SET_STEP_MODE"){ int x=rest.toInt(); g_step_mode=(x!=0); ack("SET_STEP_MODE",String(g_step_mode)); return; }
  if (head=="SET_ACCELS_ON"){ int x=rest.toInt(); g_accelS_on=(x!=0); ack("SET_ACCELS_ON",String(g_accelS_on)); return; }
  if (head=="SET_RAMPDELAY"){ int ms=rest.toInt(); g_ramp_delay_ms=(uint16_t)constrain(ms,0,10); ack("SET_RAMPDELAY",String(g_ramp_delay_ms)); return; }
  if (head=="SET_RPM_MINPULSE"){ unsigned long us=strtoul(rest.c_str(),NULL,10); RPM_MIN_PULSE_US=us; ack("SET_RPM_MINPULSE",String(us)); return; }
}

// ---------- PCINT para MEGA (PORTK / PCINT2) ----------
ISR(PCINT2_vect){
  uint8_t state = PINK & _BV(PK0); // A8
  if(state && !rpm_last_state){
    unsigned long now=micros();
    unsigned long pbp=now-LastTimeWeMeasured;
    if (pbp>=RPM_MIN_PULSE_US){
      PeriodBetweenPulses=pbp;
      LastTimeWeMeasured=now;
      if(PulseCounter>=AmountOfReadings){
        PeriodAverage=PeriodSum/AmountOfReadings;
        PulseCounter=1; PeriodSum=PeriodBetweenPulses;
        int Remaped=map((int)PeriodBetweenPulses,40000,5000,1,10);
        Remaped=constrain(Remaped,1,10);
        AmountOfReadings=Remaped;
      } else { PulseCounter++; PeriodSum+=PeriodBetweenPulses; }
    }
  }
  rpm_last_state=state;
}

// ---------- Setup ----------
String rx;
void setup(){
  Serial.begin(115200);
  Serial.setTimeout(30);

  initRTC();

  // EEPROM
  if (!loadCfg()){
    V_MIN_REAL=0.80f; V_MAX_REAL=4.20f; G_MAX_PCT=100; PWM_FREQ_HZ=1000;
    RAPID_RAMP_MS=250; RAPID_UP_PCTPS=150; SLEW_UP_PCTPS=40; SLEW_DN_PCTPS=60; START_MIN_PCT=8;
    saveCfg();
  }

  pwm1_set_freq(PWM_FREQ_HZ);
  pwm1_set_pct(0);

  // RPM em A8
  pinMode(PIN_RPM, INPUT_PULLUP);
  rpm_last_state = PINK & _BV(PK0);
  PCICR |= _BV(PCIE2);
  PCMSK2 |= _BV(PCINT16);

  dht.begin();

  // SD
  pinMode(53, OUTPUT); // SS do MEGA deve ser OUTPUT
  if (SD.begin(PIN_SD_CS)) {
    if (sd_create_new_file()) {
      sd_ok = true;
      Serial.println(F("ACK:SD OK"));
    } else {
      Serial.println(F("ERR:SD NOFILE"));
      sd_ok = false;
    }
  } else {
    Serial.println(F("ERR:SD INIT"));
    sd_ok = false;
  }

  atRest=true; restSinceMs=millis();
  accelCapActive=false; lastPedalPct=0.0f;

  Serial.println(F("ACK:BOOT OK"));
}

// ---------- Loop ----------
void loop(){
  // RX
  while(Serial.available()){
    char c=(char)Serial.read();
    if(c=='\n'){ process_line(rx); rx=""; }
    else if(c!='\r'){ rx+=c; if(rx.length()>160) rx=""; }
  }

  uint32_t now=millis();
  if(now - lastLoop >= LOOP_MS){
    uint32_t dt_ms=now - lastLoop;
    lastLoop=now;

    // Acelerador
    float v=read_voltage_throttle();
    bool fault = (v<V_FAULT_LOW)||(v>V_FAULT_HIGH);
    float pedalPct = fault?0.0f:pct_from_voltage(v);
    g_volts=v; g_pct=pedalPct;

    // Correntes
    g_current_bat_a = read_current_filtered(PIN_IBAT, i_zero_bat, i_zero_bat_ok);
    g_current_mot_a = read_current_filtered(PIN_IMOT, i_zero_mot, i_zero_mot_ok);

    // DHT
    if(now - lastDht >= DHT_MS){
      lastDht=now;
      float t=dht.readTemperature(), h=dht.readHumidity();
      if(!isnan(t)) g_temp=t;
      if(!isnan(h)) g_humi=h;
    }

    // Discretização
    uint8_t pedalDuty8=(uint8_t)lroundf(constrain(pedalPct,0.0f,100.0f)*255.0f/100.0f);
    if(pedalDuty8<DEADZONE_PWM) pedalDuty8=0;
    if(pedalDuty8>0 && pedalDuty8<MIN_START_PWM) pedalDuty8=0;
    float pedalPctAfter=(pedalDuty8*100.0f)/255.0f;

    // Teto
    pedalPctAfter *= ((float)G_MAX_PCT/100.0f);

    // repouso/soft
    bool belowRest=(pedalPct<=REST_PCT_THRESHOLD);
    if(belowRest || pedalDuty8==0){
      if(!atRest) restSinceMs=now;
      if((now - restSinceMs)>=REST_DEBOUNCE_MS){ atRest=true; softCapActive=false; }
    } else {
      if(atRest){ atRest=false; softCapActive=true; softStartMs=now; accelCapActive=false; }
    }

    float cappedPct=pedalPctAfter;
    if(softCapActive){
      float x=(float)(now-softStartMs)/(float)SOFT_RAMP_MS; x=constrain(x,0.0f,1.0f);
      float s=(3.0f*x*x)-(2.0f*x*x*x);
      float softCapPct=s*100.0f;
      if(cappedPct>softCapPct) cappedPct=softCapPct;
      if(x>=1.0f) softCapActive=false;
    }

    // rampa S
    if(g_accelS_on && !softCapActive){
      if(pedalPctAfter > lastPedalPct + 0.001f){
        accelCapActive=true; accelStartMs=now; accelFromPct=dutyNowPct; accelToPct=pedalPctAfter;
      } else if(pedalPctAfter + 0.001f < lastPedalPct){
        accelCapActive=false;
      }
    } else if(!g_accelS_on){ accelCapActive=false; }
    lastPedalPct=pedalPctAfter;

    if(accelCapActive){
      float x=(float)(now-accelStartMs)/(float)ACCEL_RAMP_MS; x=constrain(x,0.0f,1.0f);
      float s=(3.0f*x*x)-(2.0f*x*x*x);
      float capPct=accelFromPct + s*(accelToPct-accelFromPct);
      if(cappedPct>capPct) cappedPct=capPct;
      if(x>=1.0f) accelCapActive=false;
    }

    // alvo final
    if(g_override){ dutyTargetPct=(g_hold_pct>0)?(float)g_hold_pct:0.0f; }
    else {
      float target = max(cappedPct, (pedalPct>0.0f && pedalPct<START_MIN_PCT)?(float)START_MIN_PCT:0.0f);
      dutyTargetPct = target;
    }

    // aplicação
    if(!g_step_mode){
      static float lastTargetLegacy=0.0f;
      if(dutyTargetPct > lastTargetLegacy + 0.5f){ rapidUntilMs=now+RAPID_RAMP_MS; }
      lastTargetLegacy=dutyTargetPct;

      float slewUp=SLEW_UP_PCTPS;
      if((long)(rapidUntilMs - now)>0 && dutyTargetPct>dutyNowPct) slewUp=max(slewUp,RAPID_UP_PCTPS);

      float stepUp=(slewUp*dt_ms)/1000.0f;
      float stepDn=(SLEW_DN_PCTPS*dt_ms)/1000.0f;

      if(dutyNowPct<dutyTargetPct) dutyNowPct=min(dutyTargetPct,dutyNowPct+stepUp);
      else if(dutyNowPct>dutyTargetPct) dutyNowPct=max(dutyTargetPct,dutyNowPct-stepDn);
    } else {
      uint8_t cur8=(uint8_t)lroundf(constrain(dutyNowPct,0.0f,100.0f)*255.0f/100.0f);
      uint8_t tgt8=(uint8_t)lroundf(constrain(dutyTargetPct,0.0f,100.0f)*255.0f/100.0f);
      int diff8=(int)tgt8 - (int)cur8;
      int step=2 + (int)abs(diff8)/8; if(step>32) step=32;
      if(diff8>0){ uint16_t next=(uint16_t)cur8+step; if(next>tgt8) next=tgt8; cur8=(uint8_t)next; }
      else if(diff8<0){ int next=(int)cur8-step; if(next<tgt8) next=tgt8; cur8=(uint8_t)next; }
      dutyNowPct=(cur8*100.0f)/255.0f;
    }

    pwm1_set_pct(dutyNowPct);

    // RPM/velocidade
    noInterrupts();
    unsigned long LastTimeCycleMeasure_local=LastTimeWeMeasured;
    unsigned long PeriodBetweenPulses_local=PeriodBetweenPulses;
    unsigned long PeriodAverage_local=PeriodAverage;
    interrupts();

    unsigned long CurrentMicros=micros();
    if(CurrentMicros < LastTimeCycleMeasure_local) LastTimeCycleMeasure_local=CurrentMicros;

    if(PeriodBetweenPulses_local > ZeroTimeout - ZeroDebouncingExtra ||
       CurrentMicros - LastTimeCycleMeasure_local > ZeroTimeout - ZeroDebouncingExtra){
      FrequencyRaw=0; ZeroDebouncingExtra=2000;
    } else {
      ZeroDebouncingExtra=0;
      FrequencyRaw=10000000000UL / (PeriodAverage_local?PeriodAverage_local:1);
    }

    unsigned long rpm_inst=0; uint8_t ppr=(g_ppr==0)?1:g_ppr;
    if(FrequencyRaw>0){ rpm_inst=(FrequencyRaw*60UL)/ppr; rpm_inst/=10000UL; }
    total -= readings[readIndex]; readings[readIndex]=rpm_inst; total+=readings[readIndex];
    readIndex=(readIndex+1)%numReadings; unsigned long avg=total/numReadings;
    g_rpm=(float)avg; g_speed_kmh=rpm_to_kmh(g_rpm);

    // Telemetria serial (como antes)
    if(g_printmode==0){
      Serial.print(F("MS:"));    Serial.print(millis());       Serial.print(" ");
      Serial.print(F("V:"));     Serial.print(g_volts,3);      Serial.print(" ");
      Serial.print(F("Pct:"));   Serial.print(g_pct,1);        Serial.print(" ");
      Serial.print(F("Temp:"));  if(isnan(g_temp)) Serial.print("NaN"); else Serial.print(g_temp,1); Serial.print(" ");
      Serial.print(F("Humi:"));  if(isnan(g_humi)) Serial.print("NaN"); else Serial.print(g_humi,1); Serial.print(" ");
      Serial.print(F("RPM:"));   Serial.print(g_rpm,1);        Serial.print(" ");
      Serial.print(F("Speed:")); Serial.print(g_speed_kmh,2);  Serial.print(" ");
      Serial.print(F("I:"));     Serial.print(g_current_bat_a,2); Serial.print(" ");
      Serial.print(F("IMOT:"));  Serial.print(g_current_mot_a,2); Serial.print(" ");
      Serial.print(F("MIN:"));   Serial.print(V_MIN_REAL,3);   Serial.print(" ");
      Serial.print(F("MAX:"));   Serial.print(V_MAX_REAL,3);   Serial.print(" ");
      Serial.print(F("WHEEL:")); Serial.print(g_wheel_cm,1);   Serial.print(" ");
      Serial.print(F("PPR:"));   Serial.print((int)g_ppr);     Serial.print(" ");
      Serial.print(F("OVR:"));   Serial.print(g_override?1:0); Serial.print(" ");
      Serial.print(F("OVRPCT:"));Serial.print(g_hold_pct);     Serial.print(" ");
      Serial.print(F("MAXPCT:"));Serial.print((int)G_MAX_PCT); Serial.print("\n");
    }

    // LOG no SD
    sd_log_line();

    if(g_ramp_delay_ms) delay(g_ramp_delay_ms);
  }
}
