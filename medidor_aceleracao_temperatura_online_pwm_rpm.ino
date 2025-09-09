// ESP8266 (Wemos D1 mini)
// A0 (acelerador), D4 (DHT11), D6 (PWM motor via IRLZ44N), D5 (Hall A3144)
// UI: gráficos separados (Volts, Pct, Temp, Humi, RPM, Speed), eixos com ticks
// Config persistente (LittleFS): wheel_cm e ppr
// STOP/START: /stop?duty= (0–100 = %, >100 = bruto 0..1023) e /start
// /data expõe override e override_pct
// Serial Plotter (115200): Volts, Pct, Temp, Humi, RPM, Speed_kmh

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <DHT.h>
#include <FS.h>
#include <LittleFS.h>

// ---------- PINOS ----------
const uint8_t THR_PIN = A0;
const uint8_t DHTPIN  = D4;
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

const uint8_t PWM_PIN  = D6;     // Gate MOSFET via ~100Ω
const uint8_t HALL_PIN = D5;     // A3144 OUT (pull-up interno)

// ---------- PWM ----------
const int PWM_MAX  = 1023;
const int PWM_FREQ = 2000;
const int RAMP_STEP = 12;
const uint32_t RAMP_MS = 10;
int dutyTarget = 0, dutyNow = 0;
uint32_t lastRamp = 0;

// ---- Override de PWM (STOP/START) ----
volatile bool g_override = false;     // true = ignora acelerador
volatile int  g_overrideDuty = 0;     // 0..1023
float override_pct() { return (100.0f * g_overrideDuty) / PWM_MAX; }

// ---------- ACELERADOR ----------
const float ADC_VREF = 1.0f;      // A0 nativo = 0..1.0V (muitas placas têm divisor interno p/ 0..3.3V)
const int   ADC_MAX  = 1023;
const float DIV_K    = 33.0f / (100.0f + 33.0f); // 100k/33k do seu projeto

float V_MIN_REAL = 0.855f;  // ajustável na UI
float V_MAX_REAL = 2.305f;

const uint32_t SAMPLE_MS     = 50;
const uint32_t DHT_SAMPLE_MS = 2500;
uint32_t lastSample = 0, lastDhtSample = 0;

// ---------- ESTADO PUBLICADO ----------
volatile float g_volts = 0.0f;   // V
volatile float g_pct   = 0.0f;   // 0..100 %
volatile float g_temp  = NAN;    // °C
volatile float g_humi  = NAN;    // %RH

// ---------- RPM / VELOCIDADE ----------
volatile unsigned long LastTimeWeMeasured = 0;
volatile unsigned long PeriodBetweenPulses = 700000UL;
volatile unsigned long PeriodAverage = 700000UL;
volatile unsigned long PeriodSum = 0;
volatile unsigned int  PulseCounter = 1;
unsigned long FrequencyRaw = 0;
const unsigned long ZeroTimeout = 600000UL;
unsigned int AmountOfReadings = 1;
unsigned int ZeroDebouncingExtra = 0;

const byte numReadings = 2;
unsigned long readings[numReadings] = {0};
unsigned long readIndex = 0, total = 0;

volatile float g_wheel_diam_cm = 4.0f; // persistente
volatile uint8_t g_ppr = 1;            // persistente
volatile float g_rpm = 0.0f;
volatile float g_speed_kmh = 0.0f;

float wheel_circumf_m() { return 3.14159265f * (g_wheel_diam_cm * 0.01f); }
float rpm_to_kmh(float rpm) { return rpm * wheel_circumf_m() * 0.06f; } // (RPM/60)*C *3.6 = RPM*C*0.06

// ---------- WIFI / HTTP ----------
ESP8266WebServer server(80);

// ---------- CONFIG PERSISTENTE ----------
struct Cfg { float wheel_cm; uint8_t ppr; };
bool cfg_save(const Cfg& c);
bool cfg_load(Cfg& out);

const char* CFG_PATH = "/cfg.json";

bool cfg_save(const Cfg& c) {
  File f = LittleFS.open(CFG_PATH, "w");
  if (!f) return false;
  String s = "{\"wheel_cm\":";
  s += String(c.wheel_cm, 3);
  s += ",\"ppr\":";
  s += String((int)c.ppr);
  s += "}";
  f.print(s);
  f.close();
  return true;
}

bool cfg_load(Cfg& out) {
  if (!LittleFS.exists(CFG_PATH)) return false;
  File f = LittleFS.open(CFG_PATH, "r");
  if (!f) return false;
  String json = f.readString();
  f.close();
  int iw = json.indexOf("wheel_cm");
  int ip = json.indexOf("ppr");
  if (iw < 0 || ip < 0) return false;
  int cw = json.indexOf(':', iw);
  int cm = json.indexOf(',', cw);
  int cp = json.indexOf(':', ip);
  if (cw < 0 || cp < 0) return false;
  float w = json.substring(cw+1, cm).toFloat();
  int   p = json.substring(cp+1).toInt();
  if (w < 0.5f || w > 200.0f) w = 4.0f;
  if (p < 1 || p > 16) p = 1;
  out.wheel_cm = w;
  out.ppr = (uint8_t)p;
  return true;
}

// ---------- HTML ----------
const char PAGE_HTML[] PROGMEM = R"(
<!doctype html><html><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Throttle + DHT + RPM</title>
<style>
body{font-family:system-ui;margin:0;padding:16px;background:#0b0c10;color:#eaeaea}
.card{max-width:1100px;margin:0 auto;background:#171a21;border-radius:16px;padding:16px}
h1{margin:0 0 12px;font-size:20px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:8px}
.metric{background:#0f1116;border-radius:12px;padding:12px}
h2{margin:0 0 6px;font-size:14px;color:#9aa4b2}
.val{font-size:28px;font-weight:700}
.controls{display:flex;gap:8px;flex-wrap:wrap;margin:12px 0}
button,input{background:#0f1116;color:#eaeaea;border:1px solid #2a2f3a;border-radius:10px;padding:8px 12px}
button:hover{background:#111722}
.info{font-size:12px;opacity:.85;margin-top:6px}
.canvas-grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.chart{background:#0f1116;border-radius:12px;padding:8px}
canvas{width:100%;height:260px;display:block}
</style></head><body>
<div class="card">
  <h1>Wemos Throttle + DHT + RPM</h1>

  <div class="grid">
    <div class="metric"><h2>Tensão (V)</h2><div class="val" id="v">--</div></div>
    <div class="metric"><h2>Aceleração (%)</h2><div class="val" id="p">--</div></div>
    <div class="metric"><h2>Temperatura (°C)</h2><div class="val" id="t">--</div></div>
    <div class="metric"><h2>Umidade (%)</h2><div class="val" id="h">--</div></div>
    <div class="metric"><h2>RPM</h2><div class="val" id="rpm">--</div></div>
    <div class="metric"><h2>Velocidade (km/h)</h2><div class="val" id="spd">--</div></div>
  </div>

  <div class="controls">
    <button id="setMin">Definir MIN = valor atual</button>
    <button id="setMax">Definir MAX = valor atual</button>
    <button id="resetMM">Restaurar MIN/MAX padrão</button>
    <span class="info">MIN: <span id="minv">--</span> V • MAX: <span id="maxv">--</span> V</span>
  </div>

  <div class="controls">
    <label>Diâmetro roda (cm):
      <input id="wheel" type="number" min="0.5" step="0.1" value="4.0" style="width:110px">
    </label>
    <button id="applyWheel">Aplicar</button>
    <label>Pulsos por volta (PPR):
      <input id="ppr" type="number" min="1" step="1" value="1" style="width:90px">
    </label>
    <button id="applyPpr">Aplicar</button>
  </div>

  <div class="controls">
    <label>Intervalo da leitura (ms):
      <input id="ival" type="number" min="200" step="100" value="1000" style="width:110px">
    </label>
    <button id="applyIval">Aplicar</button>

    <!-- STOP/START -->
    <label>hold (%):
      <input id="hold" type="number" min="0" max="100" step="1" value="0" style="width:90px">
    </label>
    <button id="stopBtn"  style="background:#3a0b11;border-color:#6c1d26">STOP (hold %)</button>
    <button id="startBtn" style="background:#0b113a;border-color:#1d286c">START (auto)</button>

    <span class="info" id="status">status: iniciando…</span>
  </div>

  <div class="canvas-grid">
    <div class="chart"><h2>Volts</h2><canvas id="cvV"></canvas></div>
    <div class="chart"><h2>Pct (%)</h2><canvas id="cvP"></canvas></div>
    <div class="chart"><h2>Temp (°C)</h2><canvas id="cvT"></canvas></div>
    <div class="chart"><h2>Humi (%)</h2><canvas id="cvH"></canvas></div>
    <div class="chart"><h2>RPM</h2><canvas id="cvR"></canvas></div>
    <div class="chart"><h2>Speed (km/h)</h2><canvas id="cvS"></canvas></div>
  </div>
</div>

<script>
const statusEl=document.getElementById('status');
const vEl=document.getElementById('v'), pEl=document.getElementById('p'),
      tEl=document.getElementById('t'), hEl=document.getElementById('h'),
      rpmEl=document.getElementById('rpm'), spdEl=document.getElementById('spd');
const minEl=document.getElementById('minv'), maxEl=document.getElementById('maxv');
const wheelIn=document.getElementById('wheel'), pprIn=document.getElementById('ppr');
const ivalIn=document.getElementById('ival');
const holdIn=document.getElementById('hold');
const stopBtn=document.getElementById('stopBtn');
const startBtn=document.getElementById('startBtn');

let POLL_MS=1000, filledOnce=false;
const MAXPTS=120;
const buf = { V:[], P:[], T:[], H:[], R:[], S:[] };

function pushBuf(a,val){ a.push(val); if(a.length>MAXPTS) a.shift(); }

function drawSeries(canvasId, data, yLabel, minY=null, maxY=null){
  const cv=document.getElementById(canvasId), ctx=cv.getContext('2d');
  const W=cv.width=cv.clientWidth, H=cv.height=cv.clientHeight;
  const padL=42, padB=24, padT=10, padR=8;
  ctx.fillStyle="#0f1116"; ctx.fillRect(0,0,W,H);

  const clean = data.filter(x=>x!=null && !Number.isNaN(x));
  if(clean.length<2){ ctx.fillStyle="#aaa"; ctx.fillText("Sem dados", W/2-30, H/2); return; }

  let dmin = (minY!==null)?minY:Math.min(...clean);
  let dmax = (maxY!==null)?maxY:Math.max(...clean);
  if (dmin===dmax){ dmin-=1; dmax+=1; }

  // grade
  ctx.strokeStyle="#222"; ctx.lineWidth=1;
  const ticks=5;
  for(let i=0;i<=ticks;i++){
    const y=padT+(H-padT-padB)*i/ticks;
    ctx.beginPath(); ctx.moveTo(padL,y); ctx.lineTo(W-padR,y); ctx.stroke();
  }
  ctx.strokeStyle="#333"; ctx.strokeRect(padL,padT,W-padL-padR,H-padT-padB);

  // labels eixo Y
  ctx.fillStyle="#9aa4b2"; ctx.font="12px system-ui";
  for(let i=0;i<=ticks;i++){
    const val = dmax-(dmax-dmin)*i/ticks;
    const y=padT+(H-padT-padB)*i/ticks+4;
    ctx.fillText(val.toFixed( (Math.abs(dmax-dmin)<5)?2:1 ), 4, y);
  }
  ctx.fillText(yLabel, 4, padT+12);

  // série
  ctx.strokeStyle="#7aa2f7"; ctx.lineWidth=2; ctx.beginPath();
  const n=data.length;
  for(let i=0;i<n;i++){
    const v=data[i]; if(v==null||Number.isNaN(v)) continue;
    const x=padL+(W-padL-padR)*i/(MAXPTS-1);
    const y=padT+(H-padT-padB)*(1-(v-dmin)/(dmax-dmin));
    if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  }
  ctx.stroke();
}

async function tick(){
  try{
    const r=await fetch('/data',{cache:'no-store'});
    if(!r.ok) throw new Error('HTTP '+r.status);
    const d=await r.json();

    const volts=Number(d.volts);
    const pct=Number(d.pct)*100;
    const temp=(d.temp==null)?null:Number(d.temp);
    const humi=(d.humi==null)?null:Number(d.humi);
    const rpm=(d.rpm==null)?null:Number(d.rpm);
    const spd=(d.speed_kmh==null)?null:Number(d.speed_kmh);

    if(!Number.isNaN(volts)) vEl.textContent=volts.toFixed(3);
    if(!Number.isNaN(pct))   pEl.textContent=pct.toFixed(1);
    tEl.textContent=(temp==null)?'--':temp.toFixed(1);
    hEl.textContent=(humi==null)?'--':humi.toFixed(1);
    rpmEl.textContent=(rpm==null||Number.isNaN(rpm))?'--':rpm.toFixed(1);
    spdEl.textContent=(spd==null||Number.isNaN(spd))?'--':spd.toFixed(2);

    if(typeof d.min==='number') minEl.textContent=d.min.toFixed(3);
    if(typeof d.max==='number') maxEl.textContent=d.max.toFixed(3);

    // Preenche inputs só uma vez
    if(!filledOnce){
      if(typeof d.wheel_cm==='number') wheelIn.value=d.wheel_cm.toFixed(1);
      if(typeof d.ppr==='number') pprIn.value=String(d.ppr);
      if(typeof d.override_pct==='number') holdIn.value=d.override_pct.toFixed(0);
      filledOnce=true;
    }

    // Status STOP/START
    if (d.override) {
      statusEl.textContent = 'status: STOP ativo ('+(d.override_pct||0)+'%)';
    } else {
      statusEl.textContent = 'status: ok';
    }

    pushBuf(buf.V, Number.isNaN(volts)?null:volts);
    pushBuf(buf.P, Number.isNaN(pct)?null:pct);
    pushBuf(buf.T, temp);
    pushBuf(buf.H, humi);
    pushBuf(buf.R, rpm);
    pushBuf(buf.S, spd);

    drawSeries('cvV', buf.V, 'V');
    drawSeries('cvP', buf.P, '%', 0, 100);
    drawSeries('cvT', buf.T, '°C');
    drawSeries('cvH', buf.H, '%', 0, 100);
    drawSeries('cvR', buf.R, 'RPM');
    drawSeries('cvS', buf.S, 'km/h');
  }catch(e){
    statusEl.textContent='status: ERRO '+e.message;
  }
  setTimeout(tick, POLL_MS);
}
tick();

// Controles
document.getElementById('setMin').onclick=async()=>{ try{ await fetch('/min_now'); statusEl.textContent='MIN atualizado'; }catch(_){ } };
document.getElementById('setMax').onclick=async()=>{ try{ await fetch('/max_now'); statusEl.textContent='MAX atualizado'; }catch(_){ } };
document.getElementById('resetMM').onclick=async()=>{ try{ await fetch('/defaults'); statusEl.textContent='MIN/MAX padrão'; }catch(_){ } };

document.getElementById('applyIval').onclick=()=>{ const ms=parseInt(ivalIn.value||"1000",10);
  POLL_MS=Math.max(200,Math.min(10000,ms)); statusEl.textContent='intervalo = '+POLL_MS+' ms'; };

document.getElementById('applyWheel').onclick=async()=>{
  const cm=parseFloat(wheelIn.value||"4.0");
  try{ await fetch('/set_wheel?cm='+encodeURIComponent(cm)); statusEl.textContent='roda = '+cm+' cm (salvo)'; }catch(_){}
};
document.getElementById('applyPpr').onclick=async()=>{
  const n=parseInt(pprIn.value||"1",10);
  try{ await fetch('/set_ppr?n='+encodeURIComponent(n)); statusEl.textContent='PPR = '+n+' (salvo)'; }catch(_){}
};

// STOP/START
stopBtn.onclick = async () => {
  const hold = parseInt(holdIn.value || "0", 10);
  const duty = Math.max(0, Math.min(100, hold)); // % 0..100
  try {
    await fetch('/stop?duty='+encodeURIComponent(duty));
    statusEl.textContent = 'status: STOP ativo ('+duty+'%)';
  } catch(e) {}
};
startBtn.onclick = async () => {
  try { await fetch('/start'); statusEl.textContent = 'status: START (auto)'; } catch(e) {}
};
</script>
</body></html>
)";

// ---------- HANDLERS ----------
void handleRoot(){ server.send_P(200, "text/html; charset=utf-8", PAGE_HTML); }

void handleData(){
  String s = F("{\"volts\":");
  s += String(g_volts, 3);
  s += F(",\"pct\":"); s += String(g_pct/100.0f, 3);
  s += F(",\"temp\":"); if (isnan(g_temp)) s += F("null"); else s += String(g_temp,1);
  s += F(",\"humi\":"); if (isnan(g_humi)) s += F("null"); else s += String(g_humi,1);
  s += F(",\"min\":");  s += String(V_MIN_REAL,3);
  s += F(",\"max\":");  s += String(V_MAX_REAL,3);
  s += F(",\"rpm\":");  s += String(g_rpm,1);
  s += F(",\"speed_kmh\":"); s += String(g_speed_kmh,2);
  s += F(",\"wheel_cm\":"); s += String(g_wheel_diam_cm,1);
  s += F(",\"ppr\":");  s += String(g_ppr);
  s += F(",\"override\":"); s += (g_override ? "1" : "0");
  s += F(",\"override_pct\":"); s += String(override_pct(), 0);
  s += "}";
  server.send(200, "application/json", s);
}

void handleMinNow(){ V_MIN_REAL = g_volts; server.send(200, "text/plain", "OK"); }
void handleMaxNow(){ V_MAX_REAL = g_volts; server.send(200, "text/plain", "OK"); }
void handleDefaults(){ V_MIN_REAL = 0.80f; V_MAX_REAL = 4.20f; server.send(200, "text/plain", "OK"); }

void handleSetWheel(){
  if(!server.hasArg("cm")) { server.send(400, "text/plain", "missing cm"); return; }
  float cm = server.arg("cm").toFloat();
  if (cm < 0.5f) cm = 0.5f; if (cm > 200.0f) cm = 200.0f;
  g_wheel_diam_cm = cm;
  Cfg c{(float)g_wheel_diam_cm, (uint8_t)g_ppr};
  cfg_save(c);
  server.send(200, "text/plain", "OK");
}
void handleSetPpr(){
  if(!server.hasArg("n")) { server.send(400, "text/plain", "missing n"); return; }
  int n = server.arg("n").toInt();
  if (n < 1) n = 1; if (n > 16) n = 16;
  g_ppr = (uint8_t)n;
  Cfg c{(float)g_wheel_diam_cm, (uint8_t)g_ppr};
  cfg_save(c);
  server.send(200, "text/plain", "OK");
}

// STOP/START
void handleStop(){
  // duty pode ser % (0..100) ou bruto (>100) 0..1023
  int duty = 0;
  if (server.hasArg("duty")) {
    int in = server.arg("duty").toInt();
    if (in <= 100) duty = map(in, 0, 100, 0, PWM_MAX);
    else duty = in;
  }
  if (duty < 0) duty = 0;
  if (duty > PWM_MAX) duty = PWM_MAX;

  g_override = true;
  g_overrideDuty = duty;
  dutyTarget = duty;     // aplica alvo imediatamente
  dutyNow = duty;
  analogWrite(PWM_PIN, dutyNow);

  server.send(200, "text/plain", "OK");
}
void handleStart(){
  g_override = false;
  server.send(200, "text/plain", "OK");
}

// ---------- HALL ISR ----------
ICACHE_RAM_ATTR void Pulse_Event() {
  unsigned long now = micros();
  PeriodBetweenPulses = now - LastTimeWeMeasured;
  LastTimeWeMeasured  = now;

  if (PulseCounter >= AmountOfReadings) {
    PeriodAverage = PeriodSum / AmountOfReadings;
    PulseCounter  = 1;
    PeriodSum     = PeriodBetweenPulses;

    int Remaped = map((int)PeriodBetweenPulses, 40000, 5000, 1, 10);
    Remaped = constrain(Remaped, 1, 10);
    AmountOfReadings = Remaped;
  } else {
    PulseCounter++;
    PeriodSum += PeriodBetweenPulses;
  }
}

// ---------- SETUP / LOOP ----------
int pctToDuty(float pct){
  if (pct < 0) pct = 0; if (pct > 100) pct = 100;
  return (int)(pct * PWM_MAX / 100.0f);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  LittleFS.begin();
  // carrega config persistente (se existir)
  Cfg c;
  if (cfg_load(c)) {
    g_wheel_diam_cm = c.wheel_cm;
    g_ppr = c.ppr;
  } else {
    Cfg def{4.0f, 1};
    cfg_save(def);
    g_wheel_diam_cm = def.wheel_cm;
    g_ppr = def.ppr;
  }

  // DHT
  pinMode(DHTPIN, INPUT_PULLUP);
  dht.begin();
  delay(1200);
  dht.readTemperature(); dht.readHumidity();

  // PWM motor
  pinMode(PWM_PIN, OUTPUT);
  analogWriteRange(PWM_MAX);
  analogWriteFreq(PWM_FREQ);
  analogWrite(PWM_PIN, 0);

  // Hall
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), Pulse_Event, RISING);
  LastTimeWeMeasured = micros();

  // WiFi + HTTP
  WiFi.mode(WIFI_STA);
  WiFiManager wm; wm.setConfigPortalBlocking(true); wm.setTimeout(120);
  wm.autoConnect("Throttle-Setup");

  server.on("/",          handleRoot);
  server.on("/data",      handleData);
  server.on("/min_now",   handleMinNow);
  server.on("/max_now",   handleMaxNow);
  server.on("/defaults",  handleDefaults);
  server.on("/set_wheel", handleSetWheel);
  server.on("/set_ppr",   handleSetPpr);
  server.on("/stop",      handleStop);
  server.on("/start",     handleStart);
  server.on("/ping", [](){ server.send(200, "text/plain", "pong"); });
  server.begin();
}

void loop() {
  server.handleClient();
  const uint32_t now = millis();

  // Acelerador
  if (now - lastSample >= SAMPLE_MS) {
    lastSample = now;

    int   raw   = analogRead(THR_PIN);
    float v_adc = (raw * ADC_VREF) / (float)ADC_MAX;
    float v     = v_adc / DIV_K;

    float pct = (v - V_MIN_REAL) / (V_MAX_REAL - V_MIN_REAL);
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 1.0f) pct = 1.0f;

    g_volts = v;
    g_pct   = pct * 100.0f;

    // Se override ativo, alvo vem do hold; senão, do acelerador
    if (g_override) {
      dutyTarget = g_overrideDuty;
    } else {
      dutyTarget = pctToDuty(g_pct);
    }

    // Serial Plotter
    Serial.print(F("Volts:")); Serial.print(g_volts, 3); Serial.print(' ');
    Serial.print(F("Pct:"));   Serial.print(g_pct, 1);   Serial.print(' ');
    Serial.print(F("Temp:"));  if (isnan(g_temp)) Serial.print("NaN"); else Serial.print(g_temp, 1);
    Serial.print(' ');
    Serial.print(F("Humi:"));  if (isnan(g_humi)) Serial.print("NaN"); else Serial.print(g_humi, 1);
    Serial.print(' ');
    Serial.print(F("RPM:"));   Serial.print(g_rpm, 1);   Serial.print(' ');
    Serial.print(F("Speed_kmh:")); Serial.println(g_speed_kmh, 2);
  }

  // DHT
  if (now - lastDhtSample >= DHT_SAMPLE_MS) {
    lastDhtSample = now;
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t) && !isnan(h)) { g_temp = t; g_humi = h; }
  }

  // PWM: se override está ativo, força dutyNow = g_overrideDuty (sem rampa).
  if (g_override) {
    dutyNow = g_overrideDuty;
    analogWrite(PWM_PIN, dutyNow);
  } else if (millis() - lastRamp >= RAMP_MS) {
    // Rampa PWM normal
    lastRamp = millis();
    if (dutyNow < dutyTarget) { dutyNow = min(dutyNow + RAMP_STEP, dutyTarget); analogWrite(PWM_PIN, dutyNow); }
    else if (dutyNow > dutyTarget) { dutyNow = max(dutyNow - RAMP_STEP, dutyTarget); analogWrite(PWM_PIN, dutyNow); }
  }

  // Cálculo RPM/Velocidade (não bloquear)
  noInterrupts();
  unsigned long LastTimeCycleMeasure_local = LastTimeWeMeasured;
  unsigned long PeriodBetweenPulses_local  = PeriodBetweenPulses;
  unsigned long PeriodAverage_local        = PeriodAverage;
  interrupts();

  unsigned long CurrentMicros = micros();
  if (CurrentMicros < LastTimeCycleMeasure_local) {
    LastTimeCycleMeasure_local = CurrentMicros;
  }

  if (PeriodBetweenPulses_local > ZeroTimeout - ZeroDebouncingExtra ||
      CurrentMicros - LastTimeCycleMeasure_local > ZeroTimeout - ZeroDebouncingExtra) {
    FrequencyRaw = 0;
    ZeroDebouncingExtra = 2000;
  } else {
    ZeroDebouncingExtra = 0;
    // Hz*10000
    FrequencyRaw = 10000000000UL / (PeriodAverage_local ? PeriodAverage_local : 1);
  }

  unsigned long rpm_inst = 0;
  if (g_ppr == 0) g_ppr = 1;
  if (FrequencyRaw > 0) {
    rpm_inst = (FrequencyRaw * 60UL) / g_ppr;
    rpm_inst /= 10000UL;
  }

  total -= readings[readIndex];
  readings[readIndex] = rpm_inst;
  total += readings[readIndex];
  readIndex++;
  if (readIndex >= numReadings) readIndex = 0;
  unsigned long avg = total / numReadings;

  g_rpm = (float)avg;
  g_speed_kmh = rpm_to_kmh(g_rpm);
}

