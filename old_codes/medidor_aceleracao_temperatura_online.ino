// Wemos D1 mini (ESP8266)
// Acelerador A0 + DHT D4
// Web: / (HTML com gráfico + botões), /data (JSON), /ping
// Serial Plotter (115200): Volts, Pct, Temp, Humi (label:valor)

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <DHT.h>

// --------- Pinos ---------
const uint8_t THR_PIN = A0;
const uint8_t DHTPIN  = D4;
#define DHTTYPE DHT11    // troque para DHT11 se for o seu
DHT dht(DHTPIN, DHTTYPE);

// --------- Acelerador ---------
const float ADC_VREF = 1.0f;
const int   ADC_MAX  = 1023;
// Divisor 100k/33k => k ≈ 0.247
const float DIV_K    = 33.0f / (100.0f + 33.0f);

// Faixa típica (pode ser alterada na página pelos botões)
float V_MIN_REAL = 0.855f;   // padrão
float V_MAX_REAL = 2.305f;   // padrão

// --------- Amostragem ---------
const uint32_t SAMPLE_MS     = 50;     // acelerador ~20 Hz
const uint32_t DHT_SAMPLE_MS = 2500;   // DHT ≥2 s
uint32_t lastSample    = 0;
uint32_t lastDhtSample = 0;

// --------- Estado publicado ---------
volatile float g_volts = 0.0f;   // V (reais)
volatile float g_pct   = 0.0f;   // 0..100 %
volatile float g_temp  = NAN;    // °C
volatile float g_humi  = NAN;    // %RH

ESP8266WebServer server(80);

// --------- Página HTML (sem CDN) ---------
const char PAGE_HTML[] PROGMEM = R"(
<!doctype html><html><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Throttle + DHT Monitor</title>
<style>
body{font-family:system-ui;margin:0;padding:16px;background:#0b0c10;color:#eaeaea}
.card{max-width:960px;margin:0 auto;background:#171a21;border-radius:16px;padding:16px}
h1{margin:0 0 12px;font-size:20px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.metric{background:#0f1116;border-radius:12px;padding:12px}
h2{margin:0 0 6px;font-size:14px;color:#9aa4b2}
.val{font-size:28px;font-weight:700}
.bar{height:12px;background:#222;border-radius:8px;overflow:hidden}
.fill{height:100%;width:0%;background:#7aa2f7;transition:width .2s}
.controls{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}
button,input{background:#0f1116;color:#eaeaea;border:1px solid #2a2f3a;border-radius:10px;padding:8px 12px}
button:hover{background:#111722}
.info{font-size:12px;opacity:.85;margin-top:6px}
canvas{background:#0f1116;border-radius:12px;margin-top:16px}
.legend{display:flex;gap:12px;margin-top:6px;font-size:12px;opacity:.9}
.badge{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:6px}
.bv{background:#7aa2f7}.bp{background:#3cb371}.bt{background:#f7768e}.bh{background:#e0af68}
</style></head><body>
<div class="card">
  <h1>Wemos Throttle + DHT Monitor</h1>

  <div class="grid">
    <div class="metric"><h2>Tensão (V)</h2><div class="val" id="v">--</div></div>
    <div class="metric"><h2>Aceleração (%)</h2>
      <div class="val" id="p">--</div>
      <div class="bar"><div id="b" class="fill"></div></div>
    </div>
    <div class="metric"><h2>Temperatura (°C)</h2><div class="val" id="t">--</div></div>
    <div class="metric"><h2>Umidade (%)</h2><div class="val" id="h">--</div></div>
  </div>

  <div class="controls">
    <button id="setMin">Definir MIN = valor atual</button>
    <button id="setMax">Definir MAX = valor atual</button>
    <button id="resetMM">Restaurar MIN/MAX padrão</button>
    <span class="info">MIN: <span id="minv">--</span> V &nbsp;•&nbsp; MAX: <span id="maxv">--</span> V</span>
  </div>

  <div class="controls">
    <label>Intervalo do gráfico (ms):
      <input id="ival" type="number" min="200" step="100" value="1000" style="width:110px">
    </label>
    <button id="applyIval">Aplicar</button>
  </div>

  <canvas id="chart" width="900" height="260"></canvas>
  <div class="legend">
    <span><span class="badge bv"></span>Volts</span>
    <span><span class="badge bp"></span>Pct (%)</span>
    <span><span class="badge bt"></span>Temp (°C)</span>
    <span><span class="badge bh"></span>Humi (%)</span>
  </div>

  <p class="info" id="status">status: iniciando…</p>
</div>

<script>
// ------- elementos -------
const statusEl=document.getElementById('status');
const vEl=document.getElementById('v'), pEl=document.getElementById('p'),
      bar=document.getElementById('b'), tEl=document.getElementById('t'), hEl=document.getElementById('h');
const minEl=document.getElementById('minv'), maxEl=document.getElementById('maxv');
const cv=document.getElementById('chart'); const ctx=cv.getContext('2d');
const btnMin=document.getElementById('setMin'), btnMax=document.getElementById('setMax'),
      btnReset=document.getElementById('resetMM');
const ivalIn=document.getElementById('ival'), ivalBtn=document.getElementById('applyIval');

// ------- buffers (últimos N pontos) -------
let POLL_MS=1000;
const MAXPTS=90; // ~90s padrão
const volts=[], pct=[], temp=[], humi=[], labels=[];

// ------- desenho leve -------
function draw(){
  const W=cv.width, H=cv.height, pad=36;
  ctx.clearRect(0,0,W,H);
  ctx.fillStyle="#0f1116"; ctx.fillRect(0,0,W,H);

  ctx.strokeStyle="#222"; ctx.lineWidth=1;
  for(let i=0;i<=5;i++){
    const y=pad + (H-2*pad)*i/5;
    ctx.beginPath(); ctx.moveTo(pad,y); ctx.lineTo(W-pad,y); ctx.stroke();
  }
  ctx.strokeStyle="#333"; ctx.strokeRect(pad,pad,W-2*pad,H-2*pad);

  function plot(arr, color, maxY){
    const n=arr.length; if(n<2) return;
    ctx.strokeStyle=color; ctx.lineWidth=2; ctx.beginPath();
    for(let i=0;i<n;i++){
      let v=arr[i];
      if(v==null||Number.isNaN(v)) continue;
      if(maxY>0) v = v/maxY;
      const x = pad + (W-2*pad)*(i)/(MAXPTS-1);
      const y = pad + (H-2*pad)*(1 - Math.max(0,Math.min(1,v)));
      if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.stroke();
  }

  const maxVolts = Math.max(5, ...volts.map(v=>v||0));
  plot(volts, "#7aa2f7", maxVolts);
  plot(pct,   "#3cb371", 100);
  const maxT = Math.max(40, ...temp.map(v=>v||0));
  plot(temp,  "#f7768e", maxT);
  plot(humi,  "#e0af68", 100);
}

let timer=null;
function schedule(){ if(timer) clearTimeout(timer); timer=setTimeout(tick,POLL_MS); }

async function tick(){
  try{
    const r=await fetch('/data',{cache:'no-store'});
    if(!r.ok) throw new Error('HTTP '+r.status);
    const d=await r.json();
    statusEl.textContent='status: ok';

    const v = Number(d.volts);
    const p = Number(d.pct)*100;
    const t = (d.temp===null||Number.isNaN(d.temp))?null:Number(d.temp);
    const h = (d.humi===null||Number.isNaN(d.humi))?null:Number(d.humi);

    if(!Number.isNaN(v)) vEl.textContent=v.toFixed(3);
    if(!Number.isNaN(p)) { pEl.textContent=p.toFixed(1); bar.style.width=p.toFixed(1)+'%'; }
    tEl.textContent = (t==null)?'--':t.toFixed(1);
    hEl.textContent = (h==null)?'--':h.toFixed(1);

    // min/max vindos do servidor
    if (typeof d.min === 'number') minEl.textContent = d.min.toFixed(3);
    if (typeof d.max === 'number') maxEl.textContent = d.max.toFixed(3);

    labels.push(Date.now());
    volts.push(Number.isNaN(v)?null:v);
    pct.push(Number.isNaN(p)?null:p);
    temp.push(t); humi.push(h);
    if(labels.length>MAXPTS){ labels.shift(); volts.shift(); pct.shift(); temp.shift(); humi.shift(); }

    draw();
  }catch(e){
    statusEl.textContent='status: ERRO ao buscar /data → '+e.message;
  }
  schedule();
}
schedule();

// --------- ações ---------
btnMin.onclick = async () => {
  try{ await fetch('/min_now'); statusEl.textContent='status: MIN atualizado'; }catch(_){}
};
btnMax.onclick = async () => {
  try{ await fetch('/max_now'); statusEl.textContent='status: MAX atualizado'; }catch(_){}
};
btnReset.onclick = async () => {
  try{ await fetch('/defaults'); statusEl.textContent='status: MIN/MAX restaurados'; }catch(_){}
};
ivalBtn.onclick = () => {
  const ms = parseInt(ivalIn.value||"1000",10);
  POLL_MS = Math.max(200, Math.min(10000, ms));
  statusEl.textContent='status: intervalo = '+POLL_MS+' ms';
  schedule();
};
</script>
</body></html>
)";

// ---------- Handlers ----------
void handleRoot(){ server.send_P(200, "text/html; charset=utf-8", PAGE_HTML); }

void handleData(){
  // JSON: volts (V), pct (0..1), temp (°C), humi (%), min/max (V)
  String s = F("{\"volts\":");
  s += String(g_volts, 3);
  s += F(",\"pct\":");
  s += String(g_pct / 100.0f, 3);
  s += F(",\"temp\":");
  if (isnan(g_temp)) s += F("null"); else s += String(g_temp, 1);
  s += F(",\"humi\":");
  if (isnan(g_humi)) s += F("null"); else s += String(g_humi, 1);
  s += F(",\"min\":");
  s += String(V_MIN_REAL, 3);
  s += F(",\"max\":");
  s += String(V_MAX_REAL, 3);
  s += "}";
  server.send(200, "application/json", s);
}

// define MIN = valor atual
void handleMinNow(){
  V_MIN_REAL = g_volts;
  server.send(200, "text/plain", "OK");
}

// define MAX = valor atual
void handleMaxNow(){
  V_MAX_REAL = g_volts;
  server.send(200, "text/plain", "OK");
}

// restaura padrões
void handleDefaults(){
  V_MIN_REAL = 0.80f;
  V_MAX_REAL = 4.20f;
  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // DHT
  pinMode(DHTPIN, INPUT_PULLUP);  // assegura alto no boot
  dht.begin();
  delay(1200);
  dht.readTemperature(); // descarta 1ª
  dht.readHumidity();

  // Wi-Fi via WiFiManager
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.setConfigPortalBlocking(true);
  wm.setTimeout(120);
  if (!wm.autoConnect("Throttle-Setup")) {
    Serial.println(F("WiFiManager: sem Wi-Fi (timeout)."));
  } else {
    Serial.print(F("Conectado. IP: "));
    Serial.println(WiFi.localIP());
  }

  // HTTP
  server.on("/",        handleRoot);
  server.on("/data",    handleData);
  server.on("/min_now", handleMinNow);
  server.on("/max_now", handleMaxNow);
  server.on("/defaults",handleDefaults);
  server.on("/ping", [](){ server.send(200, "text/plain", "pong"); });
  server.begin();
  Serial.println(F("HTTP pronto: / , /data , /min_now , /max_now , /defaults , /ping"));
}

void loop() {
  server.handleClient();

  const uint32_t now = millis();

  // --- Acelerador (20 Hz) ---
  if (now - lastSample >= SAMPLE_MS) {
    lastSample = now;

    int   raw   = analogRead(THR_PIN);
    float v_adc = (raw * ADC_VREF) / (float)ADC_MAX; // 0..1.0 V no A0
    float v     = v_adc / DIV_K;                     // volts reais do sensor

    float pct = (v - V_MIN_REAL) / (V_MAX_REAL - V_MIN_REAL);
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 1.0f) pct = 1.0f;

    g_volts = v;
    g_pct   = pct * 100.0f;

    // Serial Plotter (label:valor)
    Serial.print(F("Volts:")); Serial.print(g_volts, 3); Serial.print(' ');
    Serial.print(F("Pct:"));   Serial.print(g_pct, 1);   Serial.print(' ');
    Serial.print(F("Temp:"));
    if (isnan(g_temp)) Serial.print("NaN"); else Serial.print(g_temp, 1);
    Serial.print(' ');
    Serial.print(F("Humi:"));
    if (isnan(g_humi)) Serial.println("NaN"); else Serial.println(g_humi, 1);
  }

  // --- DHT (2.5 s) ---
  if (now - lastDhtSample >= DHT_SAMPLE_MS) {
    lastDhtSample = now;
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t) && !isnan(h)) { g_temp = t; g_humi = h; }
  }
}
