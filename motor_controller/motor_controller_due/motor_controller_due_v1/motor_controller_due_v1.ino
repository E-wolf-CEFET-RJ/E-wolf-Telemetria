/*
  ===== Arduino DUE — motor_controller_due_v1.ino —  Motor D9-like (PWM) + Rampas ajustáveis =====
  Porta para ARM Cortex-M3 (SAM3X8E) a partir do firmware do UNO/Nano.
  - PWM via analogWrite (12 bits). Freq padrão do Due (~1kHz).
  - RPM por attachInterrupt() (use pino D2 / RISING).
  - Persistência via FlashStorage (substitui EEPROM).
  - Telemetria K:V (compatível com o ESP que você já tem).

  ⚠️ DUE é 3,3V. Garanta que todos os sensores (especialmente ACS712, hall) não
  excedam 3,3V no pino do microcontrolador. Se estiverem a 5V, use divisor/resistor
  ou sensores próprios 3,3V (ex.: INA219/INA226) para segurança.

  Ajuste de hardware:
  - RPM → pino D2 (interrupt).
  - PWM_OUT → escolha um pino PWM do Due (ex.: D7, D8, D9, D10, D11, D12, D13).
*/

#include <Arduino.h>
#include <DHT.h>
#include <math.h>
#include <FlashStorage.h>   // Emula EEPROM no Due (persiste em Flash)

// ---------------- Pinos (DUE) ----------------
const uint8_t PIN_THROTTLE = A0;   // Acelerador (0..3.3V)
const uint8_t PIN_RPM      = 2;    // ⚠️ USAR D2 (interrupt) para RPM
const uint8_t PIN_IBAT     = A4;   // Corrente BAT (0..3.3V)
const uint8_t PIN_DHT      = 4;    // DHT22 (3.3V ok)
const uint8_t PIN_PWM      = 7;    // Saída PWM (pino PWM do Due)

// ---------------- DHT ----------------
#define DHTTYPE DHT22
DHT dht(PIN_DHT, DHTTYPE);

// ---------------- ADC (DUE: 12 bits / 3.3V) ----------------
const float VREF_ADC = 3.3f;
const int   ADC_MAX  = 4095;

// Limites “reais” (calibração do acelerador) — podem ser salvos na Flash
float V_MIN_REAL = 0.80f;
float V_MAX_REAL = 4.20f;

// Failsafe do acelerador (volts)
const float V_FAULT_LOW  = 0.15f;
const float V_FAULT_HIGH = 3.25f;

// Corrente (ex.: ACS712-30A 185mV/A @5V — AJUSTE para seu sensor real @3.3V!)
// Coloque a sensibilidade correta do seu sensor em mV/A e offset em V.
const float ACS_MV_PER_A = 100.0f;  // ajuste conforme sensor real @3.3V
const float ACS_V_OFFSET = 1.65f;   // meia escala em 3.3V

// ---------------- Controle / PWM ----------------
volatile bool  g_override = false;  // STOP/HOLD
volatile int   g_hold_pct = 0;      // % quando HOLD
float dutyNowPct = 0.0f;
float dutyTargetPct = 0.0f;
unsigned long rapidUntilMs = 0;

// Rampas (taxa)
uint16_t RAPID_RAMP_MS = 250;
float    RAPID_UP_PCTPS= 150;
float    SLEW_UP_PCTPS = 40;
float    SLEW_DN_PCTPS = 60;
uint8_t  START_MIN_PCT = 8;

// Soft/accel “S”
const uint8_t  DEADZONE_PWM        = 4;
const uint8_t  MIN_START_PWM       = 20;      // referencial 8-bit
const uint8_t  REST_PCT_THRESHOLD  = 2;
const uint16_t REST_DEBOUNCE_MS    = 120;
const uint16_t SOFT_RAMP_MS        = 1500;
const uint16_t ACCEL_RAMP_MS       = 250;

bool      atRest          = true;
uint32_t  restSinceMs     = 0;
bool      softCapActive   = false;
uint32_t  softStartMs     = 0;

bool      accelCapActive  = false;
uint32_t  accelStartMs    = 0;
float     accelFromPct    = 0.0f;
float     accelToPct      = 0.0f;
float     lastPedalPct    = 0.0f;

// Filtro leve do acelerador
int thr_filtered = 0;
int last_raw_thr = 0;

// ---------------- RPM (attachInterrupt) ----------------
volatile unsigned long isr_lastMicros  = 0;
volatile unsigned long isr_periodUs    = 700000UL;  // período último pulso
unsigned long          avg_periodUs    = 700000UL;  // média móvel simples
const byte             AVG_N           = 2;
unsigned long          bufPeriod[AVG_N]={700000,700000};
byte                   idxPeriod       = 0;

unsigned long ZeroTimeout = 600000UL;  // 600 ms
unsigned long RPM_MIN_PULSE_US = 200;  // antirruído
volatile uint8_t g_ppr = 1;

volatile float g_wheel_cm = 50.8f;     // padrão novo

// ---------------- Amostragem ----------------
const uint32_t LOOP_MS = 50;
const uint32_t DHT_MS  = 2000;
uint32_t lastLoop = 0, lastDht = 0;

// ---------------- Estado publicado ----------------
float g_volts=0, g_pct=0, g_temp=NAN, g_humi=NAN;
float g_current_bat_a=0, g_current_mot_a=0;
float g_rpm=0, g_speed_kmh=0;
float G_MAX_PCT = 100.0f;      // teto de aceleração (%)
uint16_t PWM_FREQ_HZ = 1000;   // armazenado; no Due fica “cosmético” por ora

// ---------------- Modos ----------------
uint8_t g_printmode = 0;   // 0=KVP, 1=Plotter
bool    g_step_mode = false;
bool    g_accelS_on = true;
uint16_t g_ramp_delay_ms = 0;

// ---------------- Persistência (FlashStorage no Due) ----------------
struct Config {
  uint32_t tag;       // magic
  uint16_t ver;       // versão da estrutura
  float vmin, vmax;
  float slew_up, slew_dn, rapid_up;
  uint16_t rapid_ms;
  uint8_t start_min_pct;
  float max_pct;
  float wheel_cm;
  uint8_t ppr;
  uint32_t zero_to_us;
  uint16_t pwm_hz;
};
FlashStorage(cfgStore, Config);

const uint32_t CFG_TAG = 0xD00DCAFE;
const uint16_t CFG_VER = 1;

Config cfgDefault(){
  Config c{};
  c.tag = CFG_TAG; c.ver = CFG_VER;
  c.vmin = V_MIN_REAL; c.vmax = V_MAX_REAL;
  c.slew_up = SLEW_UP_PCTPS; c.slew_dn = SLEW_DN_PCTPS; c.rapid_up = RAPID_UP_PCTPS;
  c.rapid_ms = RAPID_RAMP_MS; c.start_min_pct = START_MIN_PCT;
  c.max_pct = G_MAX_PCT; c.wheel_cm = g_wheel_cm; c.ppr = g_ppr;
  c.zero_to_us = ZeroTimeout; c.pwm_hz = PWM_FREQ_HZ;
  return c;
}
void applyConfig(const Config& c){
  V_MIN_REAL=c.vmin; V_MAX_REAL=c.vmax;
  SLEW_UP_PCTPS=c.slew_up; SLEW_DN_PCTPS=c.slew_dn; RAPID_UP_PCTPS=c.rapid_up;
  RAPID_RAMP_MS=c.rapid_ms; START_MIN_PCT=c.start_min_pct;
  G_MAX_PCT=c.max_pct; g_wheel_cm=c.wheel_cm; g_ppr=c.ppr;
  ZeroTimeout=c.zero_to_us; PWM_FREQ_HZ=c.pwm_hz;
}
void loadConfig(){
  Config c = cfgStore.read();
  if (c.tag==CFG_TAG && c.ver==CFG_VER){
    applyConfig(c);
    Serial.println(F("ACK:CFG_LOAD"));
  } else {
    Config d = cfgDefault();
    cfgStore.write(d);
    applyConfig(d);
    Serial.println(F("ACK:CFG_INIT"));
  }
}
void saveConfig(){
  Config c = cfgDefault();
  cfgStore.write(c);
  Serial.println(F("ACK:CFG_SAVE"));
}

// ---------------- Utils ----------------
float pct_from_voltage(float v){
  float pct = (v - V_MIN_REAL) / (V_MAX_REAL - V_MIN_REAL);
  if (!isfinite(pct)) pct = 0.0f;
  return constrain(pct, 0.0f, 1.0f) * 100.0f;
}
float read_voltage_throttle(){
  int raw = analogRead(PIN_THROTTLE);
  last_raw_thr = raw;
  thr_filtered = (thr_filtered * 3 + raw * 5) / 8; // IIR ~0.62
  float v_adc = (thr_filtered * VREF_ADC) / (float)ADC_MAX;
  return v_adc;
}
static inline float read_current_from_pin(uint8_t analogPin){
  int raw = analogRead(analogPin);
  float v = (raw * VREF_ADC) / ADC_MAX;
  float mv = (v - ACS_V_OFFSET) * 1000.0f;
  float a  = mv / ACS_MV_PER_A;
  return a;
}
float wheel_circumf_m(){ return 3.14159265f * (g_wheel_cm * 0.01f); }
float rpm_to_kmh(float rpm){ return rpm * wheel_circumf_m() * 0.06f; }

// ---------------- PWM (Due) ----------------
void pwm_init_due(){
  analogWriteResolution(12);  // 0..4095
  // DUE: frequência default ~1 kHz para pinos PWM. Guardamos PWM_FREQ_HZ para referência.
  pinMode(PIN_PWM, OUTPUT);
  analogWrite(PIN_PWM, 0);
}
void pwm_set_pct_due(float pct){
  pct = constrain(pct,0,100);
  uint32_t v = (uint32_t)lroundf((pct/100.0f)*4095.0f);
  analogWrite(PIN_PWM, v);
}

// ---------------- Comandos ----------------
void cmd_start(){ g_override=false; g_hold_pct=0; }
void cmd_stop(){  g_override=true;  g_hold_pct=0; dutyTargetPct=0; }
void cmd_hold(int pct){ pct=constrain(pct,0,100); g_override=true; g_hold_pct=pct; dutyTargetPct=pct; }

void process_line(String line){
  line.trim(); if(line.length()==0) return;
  int sp=line.indexOf(' ');
  String head=(sp<0)?line:line.substring(0,sp);
  String rest=(sp<0)?"":line.substring(sp+1);
  head.trim(); rest.trim();

  if(head=="START"){ cmd_start(); return; }
  if(head=="STOP"){  cmd_stop();  return; }
  if(head=="HOLD"){  cmd_hold(rest.toInt()); return; }

  if(head=="SET_MIN_NOW"){ V_MIN_REAL=g_volts; Serial.println(F("ACK:MIN_NOW")); return; }
  if(head=="SET_MAX_NOW"){ V_MAX_REAL=g_volts; Serial.println(F("ACK:MAX_NOW")); return; }
  if(head=="DEFAULTS" || head=="LOAD_DEFAULTS"){ applyConfig(cfgDefault()); Serial.println(F("ACK:DEFAULTS")); return; }
  if(head=="SAVE"){ saveConfig(); return; }

  if(head=="SET_MINV"){ float v=rest.toFloat(); V_MIN_REAL=constrain(v,0.0f,3.25f); Serial.println(F("ACK:MINV")); return; }
  if(head=="SET_MAXV"){ float v=rest.toFloat(); V_MAX_REAL=constrain(v,0.0f,3.25f); Serial.println(F("ACK:MAXV")); return; }

  if(head=="SET_WHEEL"){ float cm=rest.toFloat(); cm=constrain(cm,0.5f,200.0f); g_wheel_cm=cm; Serial.println(F("ACK:WHEEL")); return; }
  if(head=="SET_PPR"){ int n=rest.toInt(); n=constrain(n,1,16); g_ppr=(uint8_t)n; Serial.println(F("ACK:PPR")); return; }

  if(head=="SET_PWMF"){ // No Due default ~1kHz; armazenamos o valor.
    uint16_t hz=(uint16_t)rest.toInt(); PWM_FREQ_HZ = constrain(hz,100,8000);
    Serial.println(F("ACK:PWMF")); return;
  }
  if(head=="SET_STARTMIN"){ uint8_t x=(uint8_t)rest.toInt(); START_MIN_PCT=constrain(x,0,40); Serial.println(F("ACK:STARTMIN")); return; }
  if(head=="SET_RAPIDMS"){ uint16_t ms=(uint16_t)rest.toInt(); RAPID_RAMP_MS=constrain(ms,50,1500); Serial.println(F("ACK:RAPIDMS")); return; }
  if(head=="SET_RAPIDUP"){ float up=rest.toFloat(); RAPID_UP_PCTPS=constrain(up,10.0f,400.0f); Serial.println(F("ACK:RAPIDUP")); return; }
  if(head=="SET_SLEW"){
    int sp2=rest.indexOf(' ');
    float up=rest.substring(0,sp2).toFloat();
    float dn=rest.substring(sp2+1).toFloat();
    SLEW_UP_PCTPS=constrain(up,5.0f,200.0f);
    SLEW_DN_PCTPS=constrain(dn,5.0f,300.0f);
    Serial.println(F("ACK:SLEW")); return;
  }
  if(head=="SET_ZEROTO"){ unsigned long us=strtoul(rest.c_str(),NULL,10); ZeroTimeout=us; Serial.println(F("ACK:ZEROTO")); return; }

  if(head=="SET_PRINTMODE"){ if(rest.equalsIgnoreCase("KVP")) g_printmode=0; else if(rest.equalsIgnoreCase("PLOTTER")) g_printmode=1; Serial.println(F("ACK:PRINTMODE")); return; }
  if(head=="SET_STEP_MODE"){ int x=rest.toInt(); g_step_mode=(x!=0); Serial.println(F("ACK:STEP_MODE")); return; }
  if(head=="SET_ACCELS_ON"){ int x=rest.toInt(); g_accelS_on=(x!=0); Serial.println(F("ACK:ACCELS_ON")); return; }
  if(head=="SET_RAMPDELAY"){ int ms=rest.toInt(); g_ramp_delay_ms=(uint16_t)constrain(ms,0,10); Serial.println(F("ACK:RAMPDELAY")); return; }
  if(head=="SET_RPM_MINPULSE"){ unsigned long us=strtoul(rest.c_str(),NULL,10); RPM_MIN_PULSE_US=us; Serial.println(F("ACK:RPM_MINPULSE")); return; }

  if(head=="SET_MAXPCT"){ float x=rest.toFloat(); G_MAX_PCT=constrain(x,1.0f,100.0f); Serial.println(F("ACK:MAXPCT")); return; }
}

// ---------------- RPM ISR (Due) ----------------
void rpmISR(){
  unsigned long now = micros();
  unsigned long dt  = now - isr_lastMicros;
  if (dt >= RPM_MIN_PULSE_US){
    isr_periodUs = dt;
    isr_lastMicros = now;
  }
}

// ---------------- Setup ----------------
String rx;
void setup(){
  Serial.begin(115200);
  Serial.setTimeout(30);

  analogReadResolution(12);
  analogWriteResolution(12);

  // Entradas / saídas
  pinMode(PIN_THROTTLE, INPUT);
  pinMode(PIN_IBAT, INPUT);
  pinMode(PIN_PWM, OUTPUT);

  // DHT
  dht.begin();

  // RPM
  pinMode(PIN_RPM, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), rpmISR, RISING);

  // PWM
  pwm_init_due();
  pwm_set_pct_due(0);

  // Estados iniciais
  atRest = true; restSinceMs = millis(); softCapActive = false;
  accelCapActive = false; lastPedalPct = 0.0f;

  // Config (Flash)
  loadConfig();

  Serial.println(F("==== DUE READY ===="));
}

// ---------------- Loop ----------------
void loop(){
  // RX comandos
  while (Serial.available()){
    char c=(char)Serial.read();
    if(c=='\n'){ process_line(rx); rx=""; }
    else if(c!='\r'){ rx += c; if(rx.length()>160) rx=""; }
  }

  // Controle / telemetria
  uint32_t now = millis();
  if (now - lastLoop >= LOOP_MS){
    uint32_t dt_ms = now - lastLoop;
    lastLoop = now;

    // Acelerador
    float v = read_voltage_throttle();
    bool fault = (v < V_FAULT_LOW) || (v > V_FAULT_HIGH);

    float pedalPct = fault ? 0.0f : pct_from_voltage(v);
    g_volts = v;
    g_pct   = pedalPct;

    // Corrente
    g_current_bat_a = read_current_from_pin(PIN_IBAT);
    // g_current_mot_a = ... (se houver segundo canal)

    // DHT
    if (now - lastDht >= DHT_MS){
      lastDht = now;
      float t = dht.readTemperature();
      float h = dht.readHumidity();
      if (!isnan(t)) g_temp = t;
      if (!isnan(h)) g_humi = h;
    }

    // Mínimos práticos (referência 8-bit compat)
    uint8_t pedalDuty8 = (uint8_t)lroundf(constrain(pedalPct,0.0f,100.0f) * 255.0f / 100.0f);
    if (pedalDuty8 < DEADZONE_PWM) pedalDuty8 = 0;
    if (pedalDuty8 > 0 && pedalDuty8 < MIN_START_PWM) pedalDuty8 = 0;
    float pedalPctAfter = (pedalDuty8 * 100.0f) / 255.0f;

    // Repouso + Soft-Start
    bool belowRest = (pedalPct <= REST_PCT_THRESHOLD);
    if (belowRest || pedalDuty8==0){
      if (!atRest) restSinceMs = now;
      if ((now - restSinceMs) >= REST_DEBOUNCE_MS){ atRest=true; softCapActive=false; }
    } else {
      if (atRest){ atRest=false; softCapActive=true; softStartMs=now; accelCapActive=false; }
    }

    float cappedPct = pedalPctAfter;
    if (softCapActive){
      float x = (float)(now - softStartMs) / (float)SOFT_RAMP_MS; x = constrain(x,0.0f,1.0f);
      float s = (3.0f*x*x) - (2.0f*x*x*x);
      float softCapPct = s * 100.0f;
      if (cappedPct > softCapPct) cappedPct = softCapPct;
      if (x >= 1.0f) softCapActive = false;
    }

    // Rampa S de aceleração
    if (g_accelS_on && !softCapActive){
      if (pedalPctAfter > lastPedalPct + 0.001f){
        accelCapActive = true; accelStartMs = now; accelFromPct = dutyNowPct; accelToPct = pedalPctAfter;
      } else if (pedalPctAfter + 0.001f < lastPedalPct){
        accelCapActive = false;
      }
    } else if (!g_accelS_on){
      accelCapActive = false;
    }
    lastPedalPct = pedalPctAfter;

    if (accelCapActive){
      float x = (float)(now - accelStartMs) / (float)ACCEL_RAMP_MS; x = constrain(x,0.0f,1.0f);
      float s = (3.0f*x*x) - (2.0f*x*x*x);
      float capPct = accelFromPct + s*(accelToPct - accelFromPct);
      if (cappedPct > capPct) cappedPct = capPct;
      if (x >= 1.0f) accelCapActive = false;
    }

    // Alvo de PWM (START/STOP/HOLD) + teto
    if (g_override){
      dutyTargetPct = (g_hold_pct>0) ? (float)g_hold_pct : 0.0f;
    } else {
      float target = max(cappedPct, (pedalPct>0.0f && pedalPct<START_MIN_PCT) ? (float)START_MIN_PCT : 0.0f);
      dutyTargetPct = min(target, G_MAX_PCT); // aplica teto
    }

    // Aplicação: taxa (%/s) OU passo adaptativo (8b)
    if (!g_step_mode){
      static float lastTargetLegacy = 0.0f;
      if (dutyTargetPct > lastTargetLegacy + 0.5f){ rapidUntilMs = now + RAPID_RAMP_MS; }
      lastTargetLegacy = dutyTargetPct;

      float slewUp = SLEW_UP_PCTPS;
      if ((long)(rapidUntilMs - now) > 0 && dutyTargetPct > dutyNowPct){
        slewUp = max(slewUp, RAPID_UP_PCTPS);
      }

      float stepUp = (slewUp * dt_ms) / 1000.0f;
      float stepDn = (SLEW_DN_PCTPS * dt_ms) / 1000.0f;

      if (dutyNowPct < dutyTargetPct){ dutyNowPct = min(dutyTargetPct, dutyNowPct + stepUp); }
      else if (dutyNowPct > dutyTargetPct){ dutyNowPct = max(dutyTargetPct, dutyNowPct - stepDn); }

    } else {
      uint8_t cur8 = (uint8_t)lroundf(constrain(dutyNowPct,0.0f,100.0f) * 255.0f / 100.0f);
      uint8_t tgt8 = (uint8_t)lroundf(constrain(dutyTargetPct,0.0f,100.0f) * 255.0f / 100.0f);
      int diff8 = (int)tgt8 - (int)cur8;
      int step = 2 + (int)abs(diff8)/8; if(step>32) step=32;
      if (diff8 > 0){
        uint16_t next=(uint16_t)cur8 + step; if(next>tgt8) next=tgt8; cur8=(uint8_t)next;
      } else if (diff8 < 0){
        int next=(int)cur8 - step; if(next<tgt8) next=tgt8; cur8=(uint8_t)next;
      }
      dutyNowPct = (cur8 * 100.0f) / 255.0f;
    }

    // Aplica PWM
    pwm_set_pct_due(dutyNowPct);

    // RPM / velocidade
    unsigned long local_last = isr_lastMicros;
    unsigned long local_per  = isr_periodUs;

    unsigned long nowUs = micros();
    if (nowUs < local_last) local_last = nowUs; // wrap proteção

    unsigned long period;
    if (local_per > ZeroTimeout || (nowUs - local_last) > ZeroTimeout){
      period = 0;
    } else {
      period = local_per;
    }

    // Média simples
    if (period == 0){
      bufPeriod[idxPeriod] = 1000000UL; // muito grande → conta como zero rpm
    } else {
      bufPeriod[idxPeriod] = period;
    }
    idxPeriod = (idxPeriod+1) % AVG_N;
    unsigned long sum=0; for(byte i=0;i<AVG_N;i++) sum += bufPeriod[i];
    avg_periodUs = sum / AVG_N;

    unsigned long rpm_inst = 0;
    uint8_t ppr = (g_ppr==0)?1:g_ppr;
    if (avg_periodUs > 0 && avg_periodUs < 10000000UL){
      float freq = 1000000.0f / (float)avg_periodUs; // Hz
      rpm_inst = (unsigned long)((freq * 60.0f) / (float)ppr);
    }
    g_rpm = (float)rpm_inst;
    g_speed_kmh = rpm_to_kmh(g_rpm);

    // Telemetria
    if (g_printmode == 0){
      Serial.print(F("V:"));     Serial.print(g_volts, 3);    Serial.print(" ");
      Serial.print(F("Pct:"));   Serial.print(g_pct, 1);      Serial.print(" ");
      Serial.print(F("Temp:"));  if (isnan(g_temp)) Serial.print("NaN"); else Serial.print(g_temp,1); Serial.print(" ");
      Serial.print(F("Humi:"));  if (isnan(g_humi)) Serial.print("NaN"); else Serial.print(g_humi,1); Serial.print(" ");
      Serial.print(F("RPM:"));   Serial.print(g_rpm, 1);      Serial.print(" ");
      Serial.print(F("Speed:")); Serial.print(g_speed_kmh,2); Serial.print(" ");
      Serial.print(F("I:"));     Serial.print(g_current_bat_a,2); Serial.print(" ");
      //Serial.print(F("IMOT:"));  Serial.print(g_current_mot_a,2); Serial.print(" ");
      Serial.print(F("MIN:"));   Serial.print(V_MIN_REAL,3);  Serial.print(" ");
      Serial.print(F("MAX:"));   Serial.print(V_MAX_REAL,3);  Serial.print(" ");
      Serial.print(F("WHEEL:")); Serial.print(g_wheel_cm,1);  Serial.print(" ");
      Serial.print(F("PPR:"));   Serial.print((int)g_ppr);    Serial.print(" ");
      Serial.print(F("OVR:"));   Serial.print(g_override?1:0);Serial.print(" ");
      Serial.print(F("OVRPCT:"));Serial.print(g_hold_pct);    Serial.print(" ");
      Serial.print(F("MAXPCT:"));Serial.print(G_MAX_PCT,0);   Serial.print("\n");
    } else {
      uint8_t dutyCurrent8 = (uint8_t)lroundf(constrain(dutyNowPct,0.0f,100.0f) * 255.0f / 100.0f);
      uint8_t dutyTarget8  = (uint8_t)lroundf(constrain(dutyTargetPct,0.0f,100.0f) * 255.0f / 100.0f);
      Serial.print("raw:");     Serial.print(last_raw_thr);                Serial.print("\t");
      Serial.print("volts:");   Serial.print(g_volts,3);                   Serial.print("\t");
      Serial.print("pct:");     Serial.print(g_pct,1);                     Serial.print("\t");
      Serial.print("ibat:");    Serial.print(g_current_bat_a,2);           Serial.print("\t");
      Serial.print("imot:");    Serial.print(g_current_mot_a,2);           Serial.print("\t");
      Serial.print("dutyTgt:"); Serial.print((int)dutyTarget8);            Serial.print("\t");
      Serial.print("dutyCur:"); Serial.print((int)dutyCurrent8);           Serial.print("\t");
      Serial.print("rest:");    Serial.print(atRest?1:0);                  Serial.print("\t");
      Serial.print("soft:");    Serial.print(softCapActive?1:0);           Serial.print("\t");
      Serial.print("accel:");   Serial.print(accelCapActive?1:0);          Serial.print("\n");
    }

    if (g_ramp_delay_ms) delay(g_ramp_delay_ms);
  }
}
