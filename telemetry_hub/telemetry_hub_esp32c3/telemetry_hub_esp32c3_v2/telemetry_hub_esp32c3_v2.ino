// ===== ESP32-C3 SuperMini — telemetry_hub_esp32c3_v2.ino =====
// Hub de telemetria + UI Web + MQTT + OTA + mDNS + Logger CSV + BLE UART.
// Sensores só em telemetria; o Arduino MEGA controla PWM/motor.
// BLE: envia telemetria básica (speed/temp/corrente) e recebe comandos (HOLD / START / STOP).

// ---------------------------- INCLUDES ----------------------------
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <FS.h>
#include <LittleFS.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <time.h>
#include <NimBLEDevice.h>

// ---------------------------- MQTT ----------------------------
const char* MQTT_HOST = "broker.mqtt-dashboard.com";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_ID_BASE = "EWolfTelemetryC3";
const char* TOP_TLM_JSON = "pb/telemetry/json";
const char* TOP_CMD_MOTOR = "pb/cmd/motor";   // 0 = STOP, 1 = START
const char* TOP_STATUS   = "pb/status";       // online/offline

WiFiClient espClient;
PubSubClient mqtt(espClient);
unsigned long lastMqtt = 0;
const unsigned long MQTT_IV_MS = 1000;

// ---------------------------- HTTP ----------------------------
WebServer server(80);

// ---------------------------- Estado (recebido do Arduino) --------------------
volatile float g_volts=0, g_pct=0, g_temp=NAN, g_humi=NAN;
volatile float g_rpm=0, g_speed_kmh=0;
volatile float g_min_v=0.80f, g_max_v=4.20f;
volatile float g_wheel_cm=50.8f;
volatile uint8_t g_ppr=1;
volatile bool   g_override=false;
volatile float  g_override_pct=0;

volatile float g_current_bat_a=0.0f;
volatile float g_current_mot_a=0.0f;
volatile float g_current_a=0.0f;

volatile float g_max_pct = 100.0f;
volatile uint32_t g_poll_ms = 1000;

String sensLine;           // buffer da linha K:V
String lastAck;            // último ACK:* recebido
unsigned long lastAckMs=0; // timestamp do ACK

// ---------------------------- Logging CSV no LittleFS -------------------------
static const char* LOG_PATH     = "/telemetry.csv";
static const char* LOG_PATH_OLD = "/telemetry.csv.1";
bool     log_enabled = false;
uint32_t LOG_IV_MS = 1000;
unsigned long lastLog = 0;

// -------- BLE UART characteristics globais --------
static NimBLECharacteristic* bleTxChar = nullptr; // ESP32 -> celular (notify/read)
static NimBLECharacteristic* bleRxChar = nullptr; // celular -> ESP32 (write)


// ================= HTML principal ===================
const char PAGE_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width,initial-scale=1"/>
  <title>Throttle + DHT + RPM + Corrente</title>
  <style>
    :root{--ink:#eaeaea;--muted:#9aa4b2;--bg:#0b0c10;--card:#171a21;--panel:#0f1116;--line:#2a2f3a;--accent:#7aa2f7}
    html,body{margin:0;background:var(--bg);color:var(--ink);font-family:system-ui,Segoe UI,Roboto,Helvetica,Arial,sans-serif}
    .wrap{max-width:1180px;margin:0 auto;padding:16px}
    .card{background:var(--card);border-radius:16px;padding:16px}
    h1{margin:0 0 12px;font-size:20px}
    .badge{display:inline-block;border:1px solid var(--line);border-radius:999px;padding:4px 10px;margin-left:6px;color:var(--muted)}
    .section{border:1px solid var(--line);background:var(--panel);border-radius:14px;padding:12px;margin:10px 0}
    .section h3{margin:0 0 10px 0;color:var(--muted);font-weight:600;font-size:14px;letter-spacing:.2px}
    .controls{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
    .kv{display:flex;gap:6px;align-items:center}
    .kv label{color:var(--muted)}
    button,input{background:var(--panel);color:var(--ink);border:1px solid var(--line);border-radius:10px;padding:8px 12px}
    button:hover{background:#111722}
    .btn-red{background:#3a0b11;border-color:#6c1d26}
    .btn-blue{background:#0b113a;border-color:#1d286c}
    .muted{color:var(--muted);font-size:12px}
    .grid-metrics{display:grid;grid-template-columns:repeat(2,minmax(220px,1fr));gap:12px;margin:8px 0}
    .metric{background:var(--panel);border-radius:12px;padding:12px}
    .metric h2{margin:0 0 6px;font-size:13px;color:var(--muted)}
    .val{font-size:28px;font-weight:800}
    @media (max-width:800px){ .grid-metrics{grid-template-columns:1fr} }
  </style>
</head>
<body>
<div class="wrap">
  <div class="card">
    <h1>Arduino Telemetry → ESP32-C3 Web + MQTT + BLE
      <span id="motorBadge" class="badge">motor: ?</span>
      <span id="ackBadge" class="badge">ack: —</span>
    </h1>

    <!-- Leituras rápidas -->
    <div class="grid-metrics">
      <div class="metric"><h2>Tensão (V)</h2><div class="val" id="v">--</div></div>
      <div class="metric"><h2>Aceleração (%)</h2><div class="val" id="p">--</div></div>
      <div class="metric"><h2>Temperatura (°C)</h2><div class="val" id="t">--</div></div>
      <div class="metric"><h2>Umidade (%)</h2><div class="val" id="h">--</div></div>
      <div class="metric"><h2>RPM</h2><div class="val" id="rpm">--</div></div>
      <div class="metric"><h2>Velocidade (km/h)</h2><div class="val" id="spd">--</div></div>
      <div class="metric"><h2>Ibateria (A)</h2><div class="val" id="ibat">--</div></div>
      <div class="metric"><h2>Imotor (A)</h2><div class="val" id="imot">--</div></div>
    </div>

    <!-- Grupo 1: Acelerador -->
    <div class="section">
      <h3>Acelerador (calibração e limites)</h3>
      <div class="controls">
        <div class="kv"><label>MIN (V):</label><input id="minIn" type="number" step="0.001" style="width:120px"><button id="applyMin">Aplicar</button></div>
        <div class="kv"><label>MAX (V):</label><input id="maxIn" type="number" step="0.001" style="width:120px"><button id="applyMax">Aplicar</button></div>
        <button id="setMin">Definir MIN = valor atual</button>
        <button id="setMax">Definir MAX = valor atual</button>
        <button id="resetMM">Restaurar MIN/MAX padrão</button>
        <span class="muted">MIN: <span id="minv">--</span> V • MAX: <span id="maxv">--</span> V</span>
        <div class="kv" style="margin-left:auto"><label>Teto acel. (%):</label><input id="maxpct" type="number" min="1" max="100" step="1" value="100" style="width:90px"><button id="applyMaxPct">Aplicar</button></div>
      </div>
    </div>

    <!-- Grupo 2: Motor & Controles -->
    <div class="section">
      <h3>Motor e controles</h3>
      <div class="controls">
        <button id="stopBtn"  class="btn-red">STOP</button>
        <button id="startBtn" class="btn-blue">START</button>
        <div class="kv"><label>Hold (%):</label><input id="hold" type="number" min="0" max="100" step="1" value="0" style="width:90px"><button id="applyHold">Aplicar Hold</button></div>
        <span class="muted" id="status">status: iniciando…</span>
      </div>
    </div>

    <!-- Exibição -->
    <div class="section">
      <h3>Exibição e atualização</h3>
      <div class="controls">
        <div class="kv"><label>Poll (ms):</label><input id="pollms" type="number" min="100" step="50" value="1000" style="width:110px"><button id="applyPoll">Aplicar</button></div>
        <a href="/speedo" style="text-decoration:none;margin-left:auto"><button>🔭 Abrir velocímetro (tela cheia)</button></a>
      </div>
    </div>

    <!-- CSV Logger (no ESP) -->
    <div class="section">
      <h3>CSV Logger (no ESP)</h3>
      <div class="controls">
        <button onclick="fetch('/log/start')">⏺️ Iniciar</button>
        <button onclick="fetch('/log/stop')">⏹️ Parar</button>
        <a href="/log/csv" download><button>⬇️ Baixar CSV</button></a>
        <button onclick="fetch('/log/clear')">🧹 Limpar</button>
        <div class="kv"><label>Intervalo (ms):</label><input id="logiv" type="number" min="100" step="100" value="1000" style="width:110px"><button onclick="fetch('/set_log_iv?ms='+document.getElementById('logiv').value)">Aplicar</button></div>
        <span class="muted">Status: <span id="logstatus">--</span></span>
      </div>
    </div>

  </div>
</div>

<script>
const statusEl=document.getElementById('status');
const vEl=document.getElementById('v'), pEl=document.getElementById('p'),
      tEl=document.getElementById('t'), hEl=document.getElementById('h'),
      rpmEl=document.getElementById('rpm'), spdEl=document.getElementById('spd');
const ibEl=document.getElementById('ibat'), imEl=document.getElementById('imot');
const minEl=document.getElementById('minv'), maxEl=document.getElementById('maxv');
const pollIn=document.getElementById('pollms');
const maxpctIn=document.getElementById('maxpct');
const logiv=document.getElementById('logiv');
const logstatus=document.getElementById('logstatus');
const holdIn=document.getElementById('hold');

let POLL_MS=1000;

async function tick(){
  try{
    const r=await fetch('/data',{cache:'no-store'});
    const d=await r.json();
    vEl.textContent=d.volts.toFixed(3);
    pEl.textContent=d.pct.toFixed(1);
    tEl.textContent=(d.temp==null)?'--':d.temp.toFixed(1);
    hEl.textContent=(d.humi==null)?'--':d.humi.toFixed(1);
    rpmEl.textContent=d.rpm.toFixed(1);
    spdEl.textContent=d.speed_kmh.toFixed(2);
    ibEl.textContent=d.current_bat_a.toFixed(2);
    imEl.textContent=d.current_mot_a.toFixed(2);
    minEl.textContent=d.min.toFixed(3);
    maxEl.textContent=d.max.toFixed(3);
    POLL_MS=d.poll_ms||POLL_MS;
    pollIn.value=POLL_MS;
    maxpctIn.value=d.max_pct||100;
    logstatus.textContent=(d.log_enabled?'ON':'OFF')+' • '+(d.log_size||0)+' bytes @ '+(d.log_iv_ms||0)+' ms';
    if(d.log_iv_ms) logiv.value=d.log_iv_ms;
    document.getElementById('motorBadge').textContent = d.override ? 'motor: OFF' : 'motor: ON';
    const ack=(d.ack||'').trim(); document.getElementById('ackBadge').textContent='ack: '+(ack?ack:'—');
    statusEl.textContent='status: ok';
  }catch(e){
    statusEl.textContent='status: ERRO '+e.message;
  }
  setTimeout(tick,POLL_MS);
}
tick();

// Controles simples
document.getElementById('setMin').onclick=()=>{ fetch('/min_now'); };
document.getElementById('setMax').onclick=()=>{ fetch('/max_now'); };
document.getElementById('resetMM').onclick=()=>{ fetch('/defaults'); };
document.getElementById('applyMin').onclick=()=>{ const v=parseFloat(minEl.textContent||'0'); fetch('/set_min?v='+v.toFixed(3)); };
document.getElementById('applyMax').onclick=()=>{ const v=parseFloat(maxEl.textContent||'0'); fetch('/set_max?v='+v.toFixed(3)); };
document.getElementById('stopBtn').onclick=()=>{ fetch('/stop'); };
document.getElementById('startBtn').onclick=()=>{ fetch('/start'); };
document.getElementById('applyHold').onclick=()=>{ const n=Math.max(0,Math.min(100,parseInt(holdIn.value||'0',10))); fetch('/hold_pct?x='+n); };
document.getElementById('applyPoll').onclick=()=>{ const ms=parseInt(pollIn.value||'1000',10); fetch('/set_pollms?ms='+ms); };
document.getElementById('applyMaxPct').onclick=()=>{ const x=Math.max(1,Math.min(100,parseInt(maxpctIn.value||'100',10))); fetch('/set_maxpct?x='+x); };
</script>
</body>
</html>)HTML";

// ---------------- /speedo (versão BLE-friendly) ----------------
const char SPEEDO_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no"/>
<title>Speedo</title>
<style>
  :root{ --bg:#0b0c10; --card:#11131a; --ink:#eaeaea; --muted:#97a0ab; --accent:#7aa2f7; }
  html,body{height:100%;margin:0;background:var(--bg);color:var(--ink);font-family:system-ui,Segoe UI,Roboto,Helvetica,Arial,sans-serif}
  .wrap{position:relative;height:100%;display:flex;align-items:center;justify-content:center}
  .gauge{width:min(90vw,90vh);height:min(90vw,90vh);background:var(--card);border-radius:24px;
         box-shadow:0 10px 30px rgba(0,0,0,.35);position:relative;overflow:hidden}
  canvas{width:100%;height:100%}
  .topbar{position:absolute;top:10px;left:10px;right:10px;display:flex;justify-content:space-between;align-items:center;pointer-events:none;z-index:3}
  .pill{pointer-events:auto;background:#0f1116;border:1px solid #2a2f3a;border-radius:999px;padding:4px 10px;color:#9aa2b2;font-size:12px}
  .readout{position:absolute;left:0;right:0;bottom:10%;text-align:center;pointer-events:none;z-index:2}
  .v{font-size:clamp(56px,14vw,140px);font-weight:900;letter-spacing:.5px}
  .unit{font-size:clamp(14px,3.6vw,26px);color:var(--muted)}
</style></head><body>
<div class="wrap">
  <div class="gauge">
    <div class="topbar">
      <span class="pill" id="status">MAX: <span id="maxLbl">60</span> km/h</span>
      <span class="pill">Origem: HTTP/BLE</span>
    </div>
    <canvas id="cv"></canvas>
    <div class="readout"><div class="v" id="speed">--</div><div class="unit">km/h</div></div>
  </div>
</div>
<script>
let MAX = 60;
let IV  = 250;

const cv = document.getElementById('cv'), ctx = cv.getContext('2d');
function resize(){ cv.width=cv.clientWidth*2; cv.height=cv.clientHeight*2; }
addEventListener('resize',resize); resize();

const START = Math.PI * 5/6, END = Math.PI * 13/6;
function arcGauge(val,max){
  const W=cv.width, H=cv.height, cx=W/2, cy=H*0.72, r=Math.min(W,H)*0.42;
  ctx.clearRect(0,0,W,H);
  ctx.lineCap='round'; ctx.lineWidth=r*0.16;
  ctx.strokeStyle='#1f2430'; ctx.beginPath(); ctx.arc(cx,cy,r,START,END,false); ctx.stroke();
  const pct = Math.max(0,Math.min(1,max>0?val/max:0));
  const col = (pct<0.7)?'#7aa2f7':(pct<0.9)?'#f7d27a':'#f77676';
  ctx.strokeStyle=col; ctx.beginPath(); ctx.arc(cx,cy,r,START,START+(END-START)*pct,false); ctx.stroke();
}

const speedEl=document.getElementById('speed');
const statusMid=document.getElementById('status');
const maxLbl=document.getElementById('maxLbl');

function updateFromData(spd){
  spd = Number(spd)||0;
  speedEl.textContent = spd.toFixed(1);
  arcGauge(spd,MAX);
}

async function tick(){
  try{
    const r=await fetch('/data',{cache:'no-store'});
    const d=await r.json();
    updateFromData(d.speed_kmh);
    statusMid.textContent = 'MAX: '+MAX+' km/h';
  }catch(e){
    statusMid.textContent='conectando…';
  }
  setTimeout(tick,IV);
}
tick();
</script>
</body></html>
)HTML";

// ---------------------------- Util Serial → Arduino ---------------------------
void sendCmd(const String& s){ Serial.println(s); }

// ---------------------------- Time/NTP ---------------------------------------
void setupTime() {
  configTime(0, 0, "pool.ntp.org", "time.google.com");
}
String isoTimestamp() {
  time_t now = time(nullptr);
  if (now <= 100000) return String("");
  struct tm t; gmtime_r(&now, &t);
  char buf[25];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &t);
  return String(buf);
}

// ---------------------------- LittleFS helpers -------------------------------
bool fileExistsNonEmpty(const char* path) {
  if (!LittleFS.exists(path)) return false;
  File f = LittleFS.open(path, "r"); if (!f) return false;
  size_t sz = f.size(); f.close(); return sz>0;
}
size_t fileSize(const char* path) {
  if (!LittleFS.exists(path)) return 0;
  File f = LittleFS.open(path, "r"); if (!f) return 0;
  size_t sz=f.size(); f.close(); return sz;
}
void rotateIfNeeded() {
  const size_t MAX_BYTES = 1024UL * 1024UL; // 1 MB
  size_t sz = fileSize(LOG_PATH);
  if (sz >= MAX_BYTES) {
    if (LittleFS.exists(LOG_PATH_OLD)) LittleFS.remove(LOG_PATH_OLD);
    LittleFS.rename(LOG_PATH, LOG_PATH_OLD);
  }
}
void writeCsvHeaderIfNeeded() {
  if (fileExistsNonEmpty(LOG_PATH)) return;
  File f = LittleFS.open(LOG_PATH, "a"); if (!f) return;
  f.println(F("ts_iso,ms,volts,pct,temp,humi,rpm,speed_kmh,current_bat_a,current_mot_a,min,max,wheel_cm,ppr,override,override_pct,max_pct,rssi"));
  f.close();
}
String fmtMaybeNan(float v, int digits=3) {
  if (isnan(v)) return String();
  return String(v, digits);
}
int wifiRSSI() {
  return (WiFi.status()==WL_CONNECTED) ? WiFi.RSSI() : 0;
}
void appendCsvRow() {
  rotateIfNeeded();
  writeCsvHeaderIfNeeded();
  File f = LittleFS.open(LOG_PATH, "a"); if (!f) return;
  String ts = isoTimestamp();
  f.print(ts); f.print(',');
  f.print(millis()); f.print(',');
  f.print(String(g_volts,3)); f.print(',');
  f.print(String(g_pct,1));   f.print(',');
  f.print(fmtMaybeNan(g_temp,1)); f.print(',');
  f.print(fmtMaybeNan(g_humi,1)); f.print(',');
  f.print(String(g_rpm,1));   f.print(',');
  f.print(String(g_speed_kmh,2)); f.print(',');
  f.print(String(g_current_bat_a,3)); f.print(',');
  f.print(String(g_current_mot_a,3)); f.print(',');
  f.print(String(g_min_v,3));  f.print(',');
  f.print(String(g_max_v,3));  f.print(',');
  f.print(String(g_wheel_cm,1)); f.print(',');
  f.print((int)g_ppr); f.print(',');
  f.print(g_override ? 1 : 0); f.print(',');
  f.print(String(g_override_pct,0)); f.print(',');
  f.print(String(g_max_pct,0)); f.print(',');
  f.print(wifiRSSI());
  f.println();
  f.close();
}

// ---------------------------- HTTP Handlers ----------------------------------
void handleRoot(){ server.send_P(200, "text/html; charset=utf-8", PAGE_HTML); }
void handleSpeedo(){ server.send_P(200, "text/html; charset=utf-8", SPEEDO_HTML); }

void handleData(){
  String s = F("{\"volts\":");
  s += String(g_volts, 3);
  s += F(",\"pct\":"); s += String(g_pct, 1);
  s += F(",\"temp\":"); if (isnan(g_temp)) s += F("null"); else s += String(g_temp,1);
  s += F(",\"humi\":"); if (isnan(g_humi)) s += F("null"); else s += String(g_humi,1);
  s += F(",\"min\":");  s += String(g_min_v,3);
  s += F(",\"max\":");  s += String(g_max_v,3);
  s += F(",\"rpm\":");  s += String(g_rpm,1);
  s += F(",\"speed_kmh\":"); s += String(g_speed_kmh,2);
  s += F(",\"wheel_cm\":"); s += String(g_wheel_cm,1);
  s += F(",\"ppr\":");  s += String(g_ppr);
  s += F(",\"override\":"); s += (g_override ? 1 : 0);
  s += F(",\"override_pct\":"); s += String(g_override_pct, 0);
  s += F(",\"current_bat_a\":"); s += String(g_current_bat_a, 3);
  s += F(",\"current_mot_a\":"); s += String(g_current_mot_a, 3);
  s += F(",\"current_a\":");     s += String(g_current_a, 3);
  s += F(",\"max_pct\":");       s += String(g_max_pct, 0);
  s += F(",\"poll_ms\":");       s += String(g_poll_ms);
  s += F(",\"ack\":\""); String a = lastAck; a.replace("\"","'"); s += a; s += F("\"");
  s += F(",\"log_enabled\":"); s += (log_enabled?1:0);
  s += F(",\"log_iv_ms\":");   s += LOG_IV_MS;
  s += F(",\"log_size\":");    s += fileSize(LOG_PATH);
  s += "}";
  server.send(200, "application/json", s);
}

// comandos que só repassam pro Arduino
void handleMinNow(){ sendCmd(F("SET_MIN_NOW")); server.send(200, "text/plain", "OK"); }
void handleMaxNow(){ sendCmd(F("SET_MAX_NOW")); server.send(200, "text/plain", "OK"); }
void handleDefaults(){ sendCmd(F("DEFAULTS"));  server.send(200, "text/plain", "OK"); }
void handleSetMin(){
  if(!server.hasArg("v")) { server.send(400, "text/plain", "missing v"); return; }
  float v = server.arg("v").toFloat(); g_min_v = v;
  sendCmd(String(F("SET_MINV "))+String(v,3)); server.send(200, "text/plain", "OK");
}
void handleSetMax(){
  if(!server.hasArg("v")) { server.send(400, "text/plain", "missing v"); return; }
  float v = server.arg("v").toFloat(); g_max_v = v;
  sendCmd(String(F("SET_MAXV "))+String(v,3)); server.send(200, "text/plain", "OK");
}
void handleSetWheel(){
  if(!server.hasArg("cm")) { server.send(400, "text/plain", "missing cm"); return; }
  float cm = server.arg("cm").toFloat(); g_wheel_cm = cm;
  sendCmd(String(F("SET_WHEEL "))+String(cm,1)); server.send(200, "text/plain", "OK");
}
void handleSetPpr(){
  if(!server.hasArg("n")) { server.send(400, "text/plain", "missing n"); return; }
  int n = server.arg("n").toInt(); g_ppr = (uint8_t)n;
  sendCmd(String(F("SET_PPR "))+String(n)); server.send(200, "text/plain", "OK");
}
void handleStop(){ sendCmd(F("STOP"));  server.send(200, "text/plain", "OK"); }
void handleStart(){ sendCmd(F("START")); server.send(200, "text/plain", "OK"); }
void handleHoldPct(){
  if(!server.hasArg("x")) { server.send(400, "text/plain", "missing x"); return; }
  int pct = constrain(server.arg("x").toInt(),0,100);
  g_override=true; g_override_pct=pct;
  sendCmd(String(F("HOLD "))+String(pct));
  server.send(200, "text/plain", "OK");
}
void handleSetPwmf(){
  if(!server.hasArg("hz")) { server.send(400, "text/plain", "missing hz"); return; }
  int hz = server.arg("hz").toInt();
  sendCmd(String(F("SET_PWMF "))+String(hz)); server.send(200, "text/plain", "OK");
}
void handleSetStartMin(){
  if(!server.hasArg("x")) { server.send(400, "text/plain", "missing x"); return; }
  int x = server.arg("x").toInt();
  sendCmd(String(F("SET_STARTMIN "))+String(x)); server.send(200, "text/plain", "OK");
}
void handleSetRapid(){
  if(!server.hasArg("ms")||!server.hasArg("up")) { server.send(400, "text/plain", "missing params"); return; }
  int ms = server.arg("ms").toInt();
  int up = server.arg("up").toInt();
  sendCmd(String(F("SET_RAPIDMS "))+String(ms));
  sendCmd(String(F("SET_RAPIDUP "))+String(up));
  server.send(200, "text/plain", "OK");
}
void handleSetSlew(){
  if(!server.hasArg("up")||!server.hasArg("down")) { server.send(400, "text/plain", "missing up/down"); return; }
  int up = server.arg("up").toInt();
  int dn = server.arg("down").toInt();
  sendCmd(String(F("SET_SLEW "))+String(up)+F(" ")+String(dn));
  server.send(200, "text/plain", "OK");
}
void handleSetZeroTo(){
  if(!server.hasArg("ms")) { server.send(400, "text/plain", "missing ms"); return; }
  long us = (long)server.arg("ms").toInt() * 1000L;
  sendCmd(String(F("SET_ZEROTO "))+String(us));
  server.send(200, "text/plain", "OK");
}
void handleSetMaxPct(){
  if(!server.hasArg("x")) { server.send(400, "text/plain", "missing x"); return; }
  int x = constrain(server.arg("x").toInt(), 1, 100);
  g_max_pct = x;
  sendCmd(String(F("SET_MAXPCT "))+String(x));
  server.send(200, "text/plain", "OK");
}
void handleSetPollMs(){
  if(!server.hasArg("ms")) { server.send(400, "text/plain", "missing ms"); return; }
  g_poll_ms = (uint32_t)server.arg("ms").toInt();
  server.send(200, "text/plain", "OK");
}

// Logging endpoints
void handleLogStart(){ log_enabled = true; server.send(200, "text/plain", "logging: on"); }
void handleLogStop(){  log_enabled = false; server.send(200, "text/plain", "logging: off"); }
void handleLogClear(){
  if (LittleFS.exists(LOG_PATH)) LittleFS.remove(LOG_PATH);
  if (LittleFS.exists(LOG_PATH_OLD)) LittleFS.remove(LOG_PATH_OLD);
  server.send(200, "text/plain", "cleared");
}
void handleLogCsv(){
  if (!LittleFS.exists(LOG_PATH)) { server.send(404, "text/plain", "no log"); return; }
  File f = LittleFS.open(LOG_PATH, "r");
  server.streamFile(f, "text/csv");
  f.close();
}
void handleLogStatus(){
  String s = "{";
  s += "\"enabled\":"; s += (log_enabled? "true":"false");
  s += ",\"interval_ms\":"; s += LOG_IV_MS;
  s += ",\"size\":"; s += fileSize(LOG_PATH);
  s += ",\"size_old\":"; s += fileSize(LOG_PATH_OLD);
  s += "}";
  server.send(200, "application/json", s);
}
void handleSetLogIv(){
  if (!server.hasArg("ms")) { server.send(400, "text/plain", "missing ms"); return; }
  uint32_t ms = (uint32_t)server.arg("ms").toInt();
  LOG_IV_MS = ms < 100 ? 100 : ms;
  server.send(200, "text/plain", "OK");
}

// ---------------------------- MQTT -------------------------------------------
void mqttCallback(char* topic, byte* payload, unsigned int length){
  if (String(topic) == TOP_CMD_MOTOR){
    char c=0; for (unsigned int i=0;i<length;i++){ if(!isspace(payload[i])){ c=(char)payload[i]; break; } }
    if (c=='0'){ sendCmd(F("START")); }
    else if (c=='1'){ sendCmd(F("STOP")); }
    else {
      String msg; msg.reserve(length);
      for(unsigned int i=0;i<length;i++) msg += (char)payload[i];
      msg.trim();
      if (msg.equalsIgnoreCase("ON")||msg.equalsIgnoreCase("START")) sendCmd(F("START"));
      else if (msg.equalsIgnoreCase("OFF")||msg.equalsIgnoreCase("STOP")) sendCmd(F("STOP"));
    }
  }
}

void ensureMqtt(){
  if (mqtt.connected()) return;
  while (!mqtt.connected()){
    String cid = String(MQTT_ID_BASE) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (mqtt.connect(cid.c_str(), TOP_STATUS, 0, true, "offline")){
      mqtt.publish(TOP_STATUS, "online", true);
      mqtt.subscribe(TOP_CMD_MOTOR);
    } else {
      delay(1500);
    }
  }
}

void mqttPublishTelemetry(){
  String j = "{";
  j += "\"volts\":" + String(g_volts,3);
  j += ",\"pct\":" + String(g_pct,1);
  if (isnan(g_temp)) j += ",\"temp\":null"; else j += ",\"temp\":" + String(g_temp,1);
  if (isnan(g_humi)) j += ",\"humi\":null"; else j += ",\"humi\":" + String(g_humi,1);
  j += ",\"rpm\":" + String(g_rpm,1);
  j += ",\"speed_kmh\":" + String(g_speed_kmh,2);
  j += ",\"current_a\":"   + String(g_current_a,2);
  j += ",\"current_bat_a\":"+ String(g_current_bat_a,2);
  j += ",\"current_mot_a\":"+ String(g_current_mot_a,2);
  j += ",\"min\":" + String(g_min_v,3);
  j += ",\"max\":" + String(g_max_v,3);
  j += ",\"wheel_cm\":" + String(g_wheel_cm,1);
  j += ",\"ppr\":" + String((int)g_ppr);
  j += ",\"override\":" + String(g_override ? 1 : 0);
  j += ",\"override_pct\":" + String(g_override_pct,0);
  j += ",\"max_pct\":" + String(g_max_pct,0);
  j += "}";
  mqtt.publish(TOP_TLM_JSON, j.c_str(), false);
}

// ---------------------------- OTA + mDNS -------------------------------------
void setupOTA(){
  if (MDNS.begin("telemetry")){ /* ok */ }
  ArduinoOTA.setHostname("telemetry-c3");
  ArduinoOTA.setPassword("espota"); // troque!
  ArduinoOTA.begin();
}

// ======================== BLE (UART + slider remoto) =========================
static NimBLEUUID serviceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static NimBLEUUID charRxUUID ("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
static NimBLEUUID charTxUUID ("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

static bool bleClientConnected = false;

// modo: 0 = normal, 1 = RPM, 2 = acel físico ignorado + controle BLE
int bleMode = 0;

// modo remoto via slider BLE
volatile bool  g_ble_remote_mode = false;
int g_ble_remote_pct = 0;

class MyServerCallbacks : public NimBLEServerCallbacks {
 public:
  void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
    bleClientConnected = true;
    Serial.println("[BLE] Client connected");
  }

  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
    bleClientConnected = false;
    Serial.printf("[BLE] Client disconnected (reason=%d)\n", reason);

    // REINICIA ADVERTISING SEMPRE
    NimBLEDevice::startAdvertising();
  }
  void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) override {
        Serial.print("[BLE] MTU negociado = ");
        Serial.println(MTU);
  }
};

class MyRxCallbacks : public NimBLECharacteristicCallbacks {
 public:
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
    std::string v = c->getValue();
    if (v.empty()) return;

    String j = String(v.c_str());
    j.trim();

    Serial.print("[BLE RX] ");
    Serial.println(j);

    // Lê JSON enviado pelo Android
    int modeIdx = j.indexOf("\"mode\"");
    int pctIdx  = j.indexOf("\"pct\"");

    if (modeIdx >= 0) {
      int col = j.indexOf(':', modeIdx);
      if (col > 0) bleMode = j.substring(col+1).toInt();
    }
    if (pctIdx >= 0) {
      int col = j.indexOf(':', pctIdx);
      if (col > 0) g_ble_remote_pct = constrain(j.substring(col+1).toInt(), 0, 100);
    }
  }
};

void setupBLE() {
  // Nome GAP do dispositivo
  NimBLEDevice::init("EWolf-Telemetry");
  NimBLEDevice::setDeviceName("EWolf-Telemetry");
  NimBLEDevice::setMTU(100);

  NimBLEServer* pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Serviço Nordic UART
  NimBLEService* pService = pServer->createService(serviceUUID);

  // TX: ESP32 -> Celular (notify + read)
  bleTxChar = pService->createCharacteristic(
      charTxUUID,
      NIMBLE_PROPERTY::NOTIFY
  );

  // RX: Celular -> ESP32 (write)
  bleRxChar = pService->createCharacteristic(
      charRxUUID,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  bleRxChar->setCallbacks(new MyRxCallbacks());

  pService->start();

  // Advertising com nome visível
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(serviceUUID);
  adv->setName("EWolf-Telemetry");
  adv->setAppearance(0x0000);
  adv->start();

  Serial.println(F("[BLE] Advertising como 'EWolf-Telemetry'"));
}

void bleSendSpeedo() {
  if (!bleClientConnected || bleTxChar == nullptr) return;

  String j = "{";
  j += "\"mode\":"; j += bleMode;
  j += ",\"speed_kmh\":"; j += String(g_speed_kmh,1);
  j += ",\"rpm\":";       j += String(g_rpm,1);
  j += ",\"pct\":";       j += String(g_pct,1);
  j += ",\"temp\":";
  if (isnan(g_temp)) j += "null"; else j += String(g_temp,1);
  j += ",\"current\":"; j += String(g_current_bat_a,2);
  j += "}";

  Serial.print("[BLE TX] ");
  Serial.println(j);

  bleTxChar->setValue(j.c_str());
  bleTxChar->notify();
}


// ---------------------------- Setup ------------------------------------------
void setup(){
  Serial.begin(115200);
  Serial.setTimeout(50);

  LittleFS.begin(true); // formatOnFail=true

  WiFi.mode(WIFI_STA);
  WiFiManager wm; wm.setConfigPortalBlocking(true); wm.setTimeout(120);
  wm.setDebugOutput(false);
  wm.autoConnect("Telemetry-Setup");

  // HTTP
  server.on("/",            handleRoot);
  server.on("/speedo",      handleSpeedo);
  server.on("/data",        handleData);
  server.on("/min_now",     handleMinNow);
  server.on("/max_now",     handleMaxNow);
  server.on("/defaults",    handleDefaults);
  server.on("/set_min",     handleSetMin);
  server.on("/set_max",     handleSetMax);
  server.on("/set_wheel",   handleSetWheel);
  server.on("/set_ppr",     handleSetPpr);
  server.on("/stop",        handleStop);
  server.on("/start",       handleStart);
  server.on("/hold_pct",    handleHoldPct);
  server.on("/set_pwmf",    handleSetPwmf);
  server.on("/set_startmin",handleSetStartMin);
  server.on("/set_rapid",   handleSetRapid);
  server.on("/set_slew",    handleSetSlew);
  server.on("/set_zeroto",  handleSetZeroTo);
  server.on("/set_maxpct",  handleSetMaxPct);
  server.on("/set_pollms",  handleSetPollMs);

  // Logging endpoints
  server.on("/log/start",   handleLogStart);
  server.on("/log/stop",    handleLogStop);
  server.on("/log/clear",   handleLogClear);
  server.on("/log/csv",     handleLogCsv);
  server.on("/log/status",  handleLogStatus);
  server.on("/set_log_iv",  handleSetLogIv);

  server.on("/ping", [](){ server.send(200, "text/plain", "pong"); });
  server.begin();

  Serial.println(F("======================================"));
  Serial.print  (F(" Web UI:    http://")); Serial.print(WiFi.localIP()); Serial.println(F("/"));
  Serial.print  (F(" Speedo:    http://")); Serial.print(WiFi.localIP()); Serial.println(F("/speedo"));
  Serial.print  (F(" MQTT pub:  ")); Serial.println(TOP_TLM_JSON);
  Serial.print  (F(" MQTT sub:  ")); Serial.println(TOP_CMD_MOTOR);
  Serial.println(F("======================================"));

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  setupOTA();
  setupTime();
  setupBLE();
}

// ---------------------------- Loop -------------------------------------------
void loop(){
  server.handleClient();
  ArduinoOTA.handle();

  // Ler telemetria K:V do Arduino (e capturar ACK:*)
  while (Serial.available()){
    char c=(char)Serial.read();
    if (c=='\n'){
      String line=sensLine; sensLine="";
      line.replace(",", " "); line.replace("\r", "");
      line.trim();
      if (line.startsWith("ACK:")){
        lastAck = line.substring(4); lastAck.trim(); lastAckMs = millis();
      } else {
        int pos=0;
        while (pos < (int)line.length()){
          int next=line.indexOf(' ',pos); if(next<0) next=line.length();
          String tok=line.substring(pos,next); tok.trim();
          int dp=tok.indexOf(':');
          if (dp>0){
            String k=tok.substring(0,dp);
            String v=tok.substring(dp+1);
            if      (k=="MS")      { }
            else if (k=="V")       g_volts = v.toFloat();
            else if (k=="Pct")     g_pct = v.toFloat();
            else if (k=="Temp")    g_temp = v.equalsIgnoreCase("NaN")?NAN:v.toFloat();
            else if (k=="Humi")    g_humi = v.equalsIgnoreCase("NaN")?NAN:v.toFloat();
            else if (k=="RPM")     g_rpm = v.toFloat();
            else if (k=="Speed")   g_speed_kmh = v.toFloat();
            else if (k=="I")      { g_current_bat_a = v.toFloat(); g_current_a = g_current_bat_a; }
            else if (k=="IMOT")    g_current_mot_a = v.toFloat();
            else if (k=="MIN")     g_min_v = v.toFloat();
            else if (k=="MAX")     g_max_v = v.toFloat();
            else if (k=="WHEEL")   g_wheel_cm = v.toFloat();
            else if (k=="PPR")     g_ppr = (uint8_t)v.toInt();
            else if (k=="OVR")     g_override = (v.toInt()!=0);
            else if (k=="OVRPCT")  g_override_pct = v.toFloat();
            else if (k=="MAXPCT")  g_max_pct = v.toFloat();
          }
          pos = next+1;
        }
      }
    } else if (c!='\r'){
      sensLine += c;
      if (sensLine.length()>400) sensLine="";
    }
  }

  ensureMqtt();
  mqtt.loop();
  unsigned long now=millis();
  if (now - lastMqtt >= MQTT_IV_MS){
    lastMqtt=now;
    mqttPublishTelemetry();
    if (lastAck.length()>0 && (now - lastAckMs) > 2000) lastAck="";
  }

  if (log_enabled) {
    unsigned long nowMs = millis();
    if (nowMs - lastLog >= LOG_IV_MS) {
      lastLog = nowMs;
      appendCsvRow();
    }
  }

  static unsigned long lastBle = 0;
  if (now - lastBle >= 200) {
    lastBle = now;
    bleSendSpeedo();
  }
}
