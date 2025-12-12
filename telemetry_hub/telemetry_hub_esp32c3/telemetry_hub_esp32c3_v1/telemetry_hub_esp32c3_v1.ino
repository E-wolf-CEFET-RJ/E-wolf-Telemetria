// ===== ESP32-C3 SuperMini telemetry_hub_esp32c3_v1.ino — Web/UI + Serial Bridge + MQTT + OTA/mDNS + CSV Logging =====
// Sensores só em telemetria; Arduino não usa para controle nesse estágio.
// UI: campos de MIN/MAX (com botões também), Poll (ms), teto de aceleração (%), botões de log CSV.
// MQTT: publica pb/telemetry/json; assina pb/cmd/motor (0=STOP,1=START); LWT em pb/status.
// mDNS: telemetry.local  | OTA: senha "espota" (troque!).

#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <FS.h>
#include <LittleFS.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <time.h>

// ---------- MQTT ----------
const char* MQTT_HOST = "broker.mqtt-dashboard.com";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_ID_BASE = "JONASgostoso";
const char* TOP_TLM_JSON = "pb/telemetry/json";
const char* TOP_CMD_MOTOR = "pb/cmd/motor";   // 0 = STOP, 1 = START
const char* TOP_STATUS   = "pb/status";       // online/offline

WiFiClient espClient;
PubSubClient mqtt(espClient);
unsigned long lastMqtt = 0;
const unsigned long MQTT_IV_MS = 1000;

// ---------- HTTP ----------
WebServer server(80);

// ---------- Estado (recebido do Arduino) ----------
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

// ---------- Logging CSV no LittleFS ----------
static const char* LOG_PATH = "/telemetry.csv";
static const char* LOG_PATH_OLD = "/telemetry.csv.1";
bool     log_enabled = false;
uint32_t LOG_IV_MS = 1000;
unsigned long lastLog = 0;

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
    .canvas-grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}
    .chart{background:var(--panel);border-radius:12px;padding:8px}
    canvas{width:100%;height:260px;display:block}
    @media (max-width:800px){ .canvas-grid{grid-template-columns:1fr} .grid-metrics{grid-template-columns:1fr} }
  </style>
</head>
<body>
<div class="wrap">
  <div class="card">
    <h1>Arduino Telemetry → ESP Web + MQTT
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
      <div class="metric" style="grid-column:1/-1">
        <h2>Relação Ibateria/Imotor</h2>
        <div class="val" id="iratio">--</div>
        <div class="muted">Quanto a corrente da bateria difere da do motor (perdas/inversor).</div>
      </div>
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

    <!-- Grupo 2: Roda & RPM -->
    <div class="section">
      <h3>Roda e sensor de RPM</h3>
      <div class="controls">
        <div class="kv"><label>Diâmetro roda (cm):</label><input id="wheel" type="number" min="0.5" step="0.1" value="50.8" style="width:110px"><button id="applyWheel">Aplicar</button></div>
        <div class="kv"><label>Pulsos por volta (PPR):</label><input id="ppr" type="number" min="1" step="1" value="1" style="width:90px"><button id="applyPpr">Aplicar</button></div>
      </div>
    </div>

    <!-- Grupo 3: Motor & Controles -->
    <div class="section">
      <h3>Motor e controles</h3>
      <div class="controls">
        <button id="stopBtn"  class="btn-red">STOP</button>
        <button id="startBtn" class="btn-blue">START</button>
        <div class="kv"><label>Hold (%):</label><input id="hold" type="number" min="0" max="100" step="1" value="0" style="width:90px"><button id="applyHold">Aplicar Hold</button></div>
        <span class="muted" id="status">status: iniciando…</span>
      </div>
    </div>

    <!-- Grupo 4: Exibição -->
    <div class="section">
      <h3>Exibição e atualização</h3>
      <div class="controls">
        <div class="kv"><label>Poll gráficos (ms):</label><input id="pollms" type="number" min="100" step="50" value="1000" style="width:110px"><button id="applyPoll">Aplicar</button></div>
        <a href="/speedo" style="text-decoration:none;margin-left:auto"><button>🔭 Abrir velocímetro (tela cheia)</button></a>
      </div>
    </div>

    <!-- Grupo 5: CSV Logger (no ESP) -->
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

    <!-- Gráficos -->
    <div class="section">
      <h3>Gráficos</h3>
      <div class="canvas-grid">
        <div class="chart"><h2>Volts</h2><canvas id="cvV"></canvas></div>
        <div class="chart"><h2>Pct (%)</h2><canvas id="cvP"></canvas></div>
        <div class="chart"><h2>Temp (°C)</h2><canvas id="cvT"></canvas></div>
        <div class="chart"><h2>Humi (%)</h2><canvas id="cvH"></canvas></div>
        <div class="chart"><h2>RPM</h2><canvas id="cvR"></canvas></div>
        <div class="chart"><h2>Speed (km/h)</h2><canvas id="cvS"></canvas></div>
        <div class="chart"><h2>Ibateria (A)</h2><canvas id="cvIB"></canvas></div>
        <div class="chart"><h2>Imotor (A)</h2><canvas id="cvIM"></canvas></div>
        <div class="chart"><h2>Ib/Im (adim.)</h2><canvas id="cvIR"></canvas></div>
      </div>
    </div>

    <!-- Parâmetros Arduino -->
    <div class="section">
      <h3>Parâmetros de rampa / PWM (Arduino)</h3>
      <div class="controls">
        <label>PWM (Hz): <input id="pwmf" type="number" min="100" max="8000" step="50" value="1000" style="width:100px"></label>
        <button id="applyPwmf">Aplicar</button>
        <label>Start mín (%): <input id="startmin" type="number" min="0" max="40" step="1" value="8" style="width:80px"></label>
        <button id="applyStartMin">Aplicar</button>
      </div>
      <div class="controls">
        <label>Rampa rápida (ms): <input id="rapidms" type="number" min="50" max="1500" step="10" value="250" style="width:100px"></label>
        <label>RapUp (pct/s): <input id="rapup" type="number" min="10" max="400" step="5" value="150" style="width:90px"></label>
        <button id="applyRapid">Aplicar</button>
      </div>
      <div class="controls">
        <label>Slew UP (pct/s): <input id="slewup" type="number" min="5" max="200" step="5" value="40" style="width:90px"></label>
        <label>Slew DN (pct/s): <input id="slewdn" type="number" min="5" max="300" step="5" value="60" style="width:90px"></label>
        <button id="applySlew">Aplicar</button>
      </div>
      <div class="controls">
        <label>ZeroTimeout RPM (ms): <input id="zeroms" type="number" min="50" max="2000" step="10" value="600" style="width:100px"></label>
        <button id="applyZero">Aplicar</button>
      </div>
    </div>

  </div>
</div>

<script>
/* --------- refs DOM --------- */
const statusEl=document.getElementById('status');
const vEl=document.getElementById('v'), pEl=document.getElementById('p'),
      tEl=document.getElementById('t'), hEl=document.getElementById('h'),
      rpmEl=document.getElementById('rpm'), spdEl=document.getElementById('spd');
const ibEl=document.getElementById('ibat'), imEl=document.getElementById('imot'), irEl=document.getElementById('iratio');
const minEl=document.getElementById('minv'), maxEl=document.getElementById('maxv');
const wheelIn=document.getElementById('wheel'), pprIn=document.getElementById('ppr');
const holdIn=document.getElementById('hold');
const stopBtn=document.getElementById('stopBtn');
const startBtn=document.getElementById('startBtn');
const applyHold=document.getElementById('applyHold');
const badge=document.getElementById('motorBadge');
const ackBadge=document.getElementById('ackBadge');

const pwmf=document.getElementById('pwmf');
const startmin=document.getElementById('startmin');
const rapidms=document.getElementById('rapidms');
const rapup=document.getElementById('rapup');
const slewup=document.getElementById('slewup');
const slewdn=document.getElementById('slewdn');
const zeroms=document.getElementById('zeroms');

const minIn=document.getElementById('minIn');
const maxIn=document.getElementById('maxIn');
const pollIn=document.getElementById('pollms');
const maxpctIn=document.getElementById('maxpct');
const logiv=document.getElementById('logiv');
const logstatus=document.getElementById('logstatus');

/* --------- estado UI --------- */
let POLL_MS=1000, filledOnce=false;
const MAXPTS=120;
const buf = { V:[], P:[], T:[], H:[], R:[], S:[], IB:[], IM:[], IR:[] };

/* --------- helpers: não sobrescrever inputs em edição --------- */
function isIdle(el){ return document.activeElement !== el && !el.dataset.editing; }
function setIfIdle(el, v){ if(isIdle(el)) el.value = v; }
[minIn,maxIn,pollIn,maxpctIn,wheelIn,pprIn,logiv].forEach(el=>{
  el.addEventListener('focus',()=>el.dataset.editing='1');
  el.addEventListener('blur', ()=>delete el.dataset.editing);
});

/* --------- gráficos --------- */
function pushBuf(a,val){ a.push(val); if(a.length>MAXPTS) a.shift(); }
function drawSeries(canvasId, data, yLabel, minY=null, maxY=null){
  const cv=document.getElementById(canvasId), ctx=cv.getContext('2d');
  const W=cv.width=cv.clientWidth, H=cv.height=cv.clientHeight;
  const padL=42, padB=24, padT=10, padR=8;
  ctx.fillStyle="#0f1116"; ctx.fillRect(0,0,W,H);
  const clean=data.filter(x=>x!=null && !Number.isNaN(x));
  if(clean.length<2){ ctx.fillStyle="#aaa"; ctx.fillText("Sem dados", W/2-30, H/2); return; }
  let dmin=(minY!==null)?minY:Math.min(...clean);
  let dmax=(maxY!==null)?maxY:Math.max(...clean);
  if(dmin===dmax){ dmin-=1; dmax+=1; }
  ctx.strokeStyle="#222"; ctx.lineWidth=1;
  const ticks=5;
  for(let i=0;i<=ticks;i++){ const y=padT+(H-padT-padB)*i/ticks; ctx.beginPath(); ctx.moveTo(padL,y); ctx.lineTo(W-padR,y); ctx.stroke(); }
  ctx.strokeStyle="#333"; ctx.strokeRect(padL,padT,W-padL-padR,H-padT-padB);
  ctx.fillStyle="#9aa4b2"; ctx.font="12px system-ui";
  for(let i=0;i<=ticks;i++){ const val=dmax-(dmax-dmin)*i/ticks; const y=padT+(H-padT-padB)*i/ticks+4; ctx.fillText(val.toFixed((Math.abs(dmax-dmin)<5)?2:1),4,y); }
  ctx.fillText(yLabel,4,padT+12);
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

/* --------- loop principal --------- */
async function tick(){
  try{
    const r=await fetch('/data',{cache:'no-store'}); if(!r.ok) throw new Error('HTTP '+r.status);
    const d=await r.json();

    const volts=Number(d.volts);
    const pct=Number(d.pct);
    const temp=(d.temp==null)?null:Number(d.temp);
    const humi=(d.humi==null)?null:Number(d.humi);
    const rpm=(d.rpm==null)?null:Number(d.rpm);
    const spd=(d.speed_kmh==null)?null:Number(d.speed_kmh);
    const ib =(d.current_bat_a==null)?null:Number(d.current_bat_a);
    const im =(d.current_mot_a==null)?null:Number(d.current_mot_a);
    const ratio=(ib!=null && im!=null && !isNaN(ib) && !isNaN(im) && Math.abs(im)>1e-6)?(ib/im):null;
    const motor_on = d.override ? 0 : 1;

    if(!Number.isNaN(volts)) vEl.textContent=volts.toFixed(3);
    if(!Number.isNaN(pct))   pEl.textContent=pct.toFixed(1);
    tEl.textContent=(temp==null)?'--':temp.toFixed(1);
    hEl.textContent=(humi==null)?'--':humi.toFixed(1);
    rpmEl.textContent=(rpm==null||Number.isNaN(rpm))?'--':rpm.toFixed(1);
    spdEl.textContent=(spd==null||Number.isNaN(spd))?'--':spd.toFixed(2);

    ibEl.textContent=(ib==null||Number.isNaN(ib))?'--':ib.toFixed(2);
    imEl.textContent=(im==null||Number.isNaN(im))?'--':im.toFixed(2);
    irEl.textContent=(ratio==null)?'--':ratio.toFixed(2);

    if(typeof d.min==='number'){ minEl.textContent=d.min.toFixed(3); setIfIdle(minIn,d.min.toFixed(3)); }
    if(typeof d.max==='number'){ maxEl.textContent=d.max.toFixed(3); setIfIdle(maxIn,d.max.toFixed(3)); }
    if(typeof d.poll_ms==='number'){ POLL_MS=d.poll_ms; setIfIdle(pollIn,POLL_MS); }
    if(typeof d.max_pct==='number'){ setIfIdle(maxpctIn,d.max_pct); }

    if(typeof d.log_enabled!=='undefined'){
      const sz=d.log_size||0, iv=d.log_iv_ms||0;
      logstatus.textContent=(d.log_enabled?'ON':'OFF')+' • '+sz+' bytes @ '+iv+' ms';
      if(d.log_iv_ms) setIfIdle(logiv,d.log_iv_ms);
    }

    if(!filledOnce){
      if(typeof d.wheel_cm==='number') setIfIdle(wheelIn,d.wheel_cm.toFixed(1));
      if(typeof d.ppr==='number') setIfIdle(pprIn,String(d.ppr));
      filledOnce=true;
    }

    pushBuf(buf.V, Number.isNaN(volts)?null:volts);
    pushBuf(buf.P, Number.isNaN(pct)?null:pct);
    pushBuf(buf.T, temp); pushBuf(buf.H, humi);
    pushBuf(buf.R, rpm);  pushBuf(buf.S, spd);
    pushBuf(buf.IB, ib);  pushBuf(buf.IM, im); pushBuf(buf.IR, ratio);

    drawSeries('cvV',buf.V,'V');
    drawSeries('cvP',buf.P,'%',0,100);
    drawSeries('cvT',buf.T,'°C');
    drawSeries('cvH',buf.H,'%',0,100);
    drawSeries('cvR',buf.R,'RPM');
    drawSeries('cvS',buf.S,'km/h');
    drawSeries('cvIB',buf.IB,'A');
    drawSeries('cvIM',buf.IM,'A');
    drawSeries('cvIR',buf.IR,'Ib/Im');

    document.getElementById('motorBadge').textContent = motor_on ? 'motor: ON' : 'motor: OFF';
    const ack=(d.ack||'').trim(); document.getElementById('ackBadge').textContent='ack: '+(ack?ack:'—');
    statusEl.textContent='status: ok';
  }catch(e){ statusEl.textContent='status: ERRO '+e.message; }
  setTimeout(tick, POLL_MS);
}
tick();

/* --------- endpoints --------- */
document.getElementById('setMin').onclick=async()=>{ try{ await fetch('/min_now'); }catch(_){ } };
document.getElementById('setMax').onclick=async()=>{ try{ await fetch('/max_now'); }catch(_){ } };
document.getElementById('resetMM').onclick=async()=>{ try{ await fetch('/defaults'); }catch(_){ } };

document.getElementById('applyMin').onclick=async()=>{
  const v=parseFloat(minIn.value||"0");
  try{ await fetch('/set_min?v='+encodeURIComponent(v.toFixed(3))); }catch(_){}
};
document.getElementById('applyMax').onclick=async()=>{
  const v=parseFloat(maxIn.value||"0");
  try{ await fetch('/set_max?v='+encodeURIComponent(v.toFixed(3))); }catch(_){}
};

document.getElementById('applyWheel').onclick=async()=>{
  const cm=parseFloat(wheelIn.value||"50.8");
  try{ await fetch('/set_wheel?cm='+encodeURIComponent(cm)); }catch(_){}
};
document.getElementById('applyPpr').onclick=async()=>{
  const n=parseInt(pprIn.value||"1",10);
  try{ await fetch('/set_ppr?n='+encodeURIComponent(n)); }catch(_){}
};

stopBtn.onclick=async()=>{ try{ await fetch('/stop'); }catch(_){ } };
startBtn.onclick=async()=>{ try{ await fetch('/start'); }catch(_){ } };
applyHold.onclick=async()=>{
  const n=Math.max(0,Math.min(100,parseInt(holdIn.value||"0",10)));
  try{ await fetch('/hold_pct?x='+encodeURIComponent(n)); }catch(_){}
};

document.getElementById('applyPoll').onclick=async()=>{
  const ms=parseInt(pollIn.value||"1000",10);
  try{ await fetch('/set_pollms?ms='+encodeURIComponent(ms)); }catch(_){}
};
document.getElementById('applyMaxPct').onclick=async()=>{
  const x=Math.max(1,Math.min(100,parseInt(maxpctIn.value||"100",10)));
  try{ await fetch('/set_maxpct?x='+encodeURIComponent(x)); }catch(_){}
};

document.getElementById('applyPwmf').onclick=async()=>{
  const hz=Math.max(100,Math.min(8000,parseInt(pwmf.value||"1000",10)));
  try{ await fetch('/set_pwmf?hz='+hz); }catch(_){}
};
document.getElementById('applyStartMin').onclick=async()=>{
  const x=Math.max(0,Math.min(40,parseInt(startmin.value||"8",10)));
  try{ await fetch('/set_startmin?x='+x); }catch(_){}
};
document.getElementById('applyRapid').onclick=async()=>{
  const ms=Math.max(50,Math.min(1500,parseInt(rapidms.value||"250",10)));
  const up=Math.max(10,Math.min(400,parseInt(rapup.value||"150",10)));
  try{ await fetch('/set_rapid?ms='+ms+'&up='+up); }catch(_){}
};
document.getElementById('applySlew').onclick=async()=>{
  const up=Math.max(5,Math.min(200,parseInt(slewup.value||"40",10)));
  const dn=Math.max(5,Math.min(300,parseInt(slewdn.value||"60",10)));
  try{ await fetch('/set_slew?up='+up+'&down='+dn); }catch(_){}
};
document.getElementById('applyZero').onclick=async()=>{
  const ms=Math.max(50,Math.min(2000,parseInt(zeroms.value||"600",10)));
  try{ await fetch('/set_zeroto?ms='+ms); }catch(_){}
};
</script>
</body>
</html>)HTML";


// ---------------- /speedo ----------------
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
  .fab{pointer-events:auto;width:48px;height:48px;border-radius:999px;border:1px solid #2a2f3a;
       background:#0f1116;color:#eaeaea;display:flex;align-items:center;justify-content:center;
       font-size:20px;box-shadow:0 8px 24px rgba(0,0,0,.3);user-select:none}
  .midpill{pointer-events:none;background:#0f1116;border:1px solid #2a2f3a;border-radius:999px;padding:6px 12px;color:#9aa2b2;font-size:14px}
  .readout{position:absolute;left:0;right:0;bottom:10%;text-align:center;pointer-events:none;z-index:2}
  .v{font-size:clamp(56px,14vw,140px);font-weight:900;letter-spacing:.5px}
  .unit{font-size:clamp(14px,3.6vw,26px);color:var(--muted)}
  .edge{position:fixed;top:50%;transform:translateY(-50%);z-index:1}
  .edge .pill{background:#0f1116;border:1px solid #2a2f3a;border-radius:14px;padding:10px 14px;
              color:#cfd6df;font-weight:700;font-size:clamp(14px,2.8vw,24px);white-space:nowrap}
  #edgeL{left:10px}
  #edgeR{right:10px}
  .warnTxt{color:#f7d27a} .alarmTxt{color:#ff5b5b}
  .modal{position:fixed;inset:0;background:rgba(0,0,0,.55);display:none;align-items:center;justify-content:center;z-index:4}
  .card{background:#12151c;border:1px solid #2a2f3a;border-radius:16px;padding:16px;min-width:min(92vw,440px)}
  .row{display:flex;gap:10px;align-items:center;justify-content:space-between;margin:10px 0}
  input[type=number]{width:120px;background:#0f1116;color:#eaeaea;border:1px solid #2a2f3a;border-radius:10px;padding:8px}
  .btn{background:#0f1116;color:#eaeaea;border:1px solid #2a2f3a;border-radius:10px;padding:8px 12px}
  label{color:#97a0ab}
  @media (max-width:720px){ .edge{display:none;} }
</style></head><body>
<div class="wrap">
  <div id="edgeL" class="edge"><span class="pill">Temp: <span id="tempVal">--</span> °C</span></div>
  <div id="edgeR" class="edge"><span class="pill">Corrente: <span id="currVal">--</span> A</span></div>
  <div class="gauge">
    <div class="topbar">
      <button id="btnFs" class="fab" title="Tela cheia">⛶</button>
      <span class="midpill" id="status">MAX: <span id="maxLbl">60</span> km/h</span>
      <button id="btnCfg" class="fab" title="Configurações">⚙</button>
    </div>
    <canvas id="cv"></canvas>
    <div class="readout"><div class="v" id="speed">--</div><div class="unit">km/h</div></div>
  </div>
</div>

<div id="modal" class="modal">
  <div class="card">
    <h3 style="margin:0 0 10px 0">Ajustes</h3>
    <div class="row"><label>Máximo (km/h):</label><input id="maxIn" type="number" min="10" max="300" step="5" value="60"></div>
    <div class="row"><label>Atualização (ms):</label><input id="ivalIn" type="number" min="100" max="2000" step="50" value="250"></div>
    <div class="row"><label>Alarme de temperatura (°C):</label><input id="tAlarmIn" type="number" min="20" max="120" step="1" value="70"></div>
    <div class="row"><label>Som do alarme:</label><input id="beepChk" type="checkbox" checked></div>
    <div class="row" style="justify-content:flex-end"><button id="closeBtn" class="btn">Fechar</button><button id="saveBtn" class="btn">Salvar</button></div>
  </div>
</div>

<script>
let MAX = Number(localStorage.getItem('max_kmh')||60);
let IV  = Number(localStorage.getItem('ival_ms')||250);
let T_ALARM = Number(localStorage.getItem('t_alarm_c')||70);
let BEEP_ON = (localStorage.getItem('beep_on')!=='0');

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
const tempEl=document.getElementById('tempVal');
const currEl=document.getElementById('currVal');
const statusMid=document.getElementById('status');
const maxLbl=document.getElementById('maxLbl');

let audioCtx=null;
function toneOnce(freq=1550, gain=0.6, dur=0.35){
  try{
    audioCtx = audioCtx || new (window.AudioContext||window.webkitAudioContext)();
    const o=audioCtx.createOscillator(), g=audioCtx.createGain();
    o.type='square'; o.frequency.value=freq;
    const t0=audioCtx.currentTime;
    g.gain.setValueAtTime(0.0001,t0);
    g.gain.exponentialRampToValueAtTime(gain,t0+0.03);
    g.gain.exponentialRampToValueAtTime(0.0001,t0+dur);
    o.connect(g).connect(audioCtx.destination);
    o.start(); o.stop(t0+dur+0.02);
  }catch(e){}
}

async function tick(){
  try{
    const r=await fetch('/data',{cache:'no-store'});
    const d=await r.json();
    const spd = Number(d.speed_kmh)||0;
    const t   = (d.temp==null)?NaN:Number(d.temp);
    const ia  = (d.current_bat_a==null)?NaN:Number(d.current_bat_a);

    speedEl.textContent = spd.toFixed(1);
    arcGauge(spd, MAX);
    tempEl.textContent = isNaN(t)?'--':t.toFixed(1);

    currEl.textContent = isNaN(ia)?'--':ia.toFixed(2);
    statusMid.textContent = 'MAX: '+MAX+' km/h'; maxLbl.textContent = MAX;
  }catch(e){ statusMid.textContent = 'conectando…'; }
  setTimeout(tick, IV);
}
tick();

document.getElementById('btnFs').onclick=()=>{
  const root=document.documentElement;
  if(!document.fullscreenElement) (root.requestFullscreen?root.requestFullscreen():root.webkitRequestFullscreen&&root.webkitRequestFullscreen());
  else (document.exitFullscreen?document.exitFullscreen():document.webkitExitFullscreen&&document.webkitExitFullscreen());
};
const modal=document.getElementById('modal');
const maxIn=document.getElementById('maxIn');
const ivalIn=document.getElementById('ivalIn');
const tAlarmIn=document.getElementById('tAlarmIn');
const beepChk=document.getElementById('beepChk');
document.getElementById('btnCfg').onclick=()=>{ maxIn.value=MAX; ivalIn.value=IV; tAlarmIn.value=T_ALARM; beepChk.checked=BEEP_ON; modal.style.display='flex'; };
document.getElementById('closeBtn').onclick=()=>{ modal.style.display='none'; };
document.getElementById('saveBtn').onclick=()=>{
  MAX=Math.max(10,Math.min(300,Number(maxIn.value)||60));
  IV=Math.max(100,Math.min(2000,Number(ivalIn.value)||250));
  T_ALARM=Math.max(20,Math.min(120,Number(tAlarmIn.value)||70));
  BEEP_ON=!!beepChk.checked;
  localStorage.setItem('max_kmh',MAX);
  localStorage.setItem('ival_ms',IV);
  localStorage.setItem('t_alarm_c',T_ALARM);
  localStorage.setItem('beep_on',BEEP_ON?'1':'0');
  modal.style.display='none';
};
</script>
</body></html>
)HTML";

// ---------- Util ----------
void sendCmd(const String& s){ Serial.println(s); }

// Time/NTP
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

// LittleFS helpers
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

// ---------- HTTP Handlers ----------
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

// ---------- MQTT ----------
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
    uint64_t chipid = ESP.getEfuseMac();
    String cid = String(MQTT_ID_BASE) + "-" + String((uint32_t)(chipid & 0xFFFFFFFF), HEX);
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

// ---------- OTA + mDNS ----------
void setupOTA(){
  if (MDNS.begin("telemetry")){ /* ok */ }
  ArduinoOTA.setHostname("telemetry");
  ArduinoOTA.setPassword("espota"); // troque!
  ArduinoOTA.begin();
}

// ---------- Setup ----------
void setup(){
  Serial.begin(115200);
  Serial.setTimeout(50);

  // LittleFS no ESP32: formatOnFail = true ajuda no primeiro boot
  LittleFS.begin(true);

  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.setConfigPortalBlocking(true);
  wm.setTimeout(120);
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

  // Log inicial
  Serial.println(F("======================================"));
  Serial.print  (F(" Web UI:    http://")); Serial.print(WiFi.localIP()); Serial.println(F("/"));
  Serial.print  (F(" Speedo:    http://")); Serial.print(WiFi.localIP()); Serial.println(F("/speedo"));
  Serial.print  (F(" MQTT pub:  ")); Serial.println(TOP_TLM_JSON);
  Serial.print  (F(" MQTT sub:  ")); Serial.println(TOP_CMD_MOTOR);
  Serial.println(F("======================================"));

  // MQTT
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  // OTA/mDNS + NTP
  setupOTA();
  setupTime();
}

// ---------- Loop ----------
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
            if      (k=="MS")      { /* opcional */ }
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

  // MQTT keepalive + telemetria
  ensureMqtt();
  mqtt.loop();
  unsigned long now=millis();
  if (now - lastMqtt >= MQTT_IV_MS){
    lastMqtt=now;
    mqttPublishTelemetry();
    // limpar ACK antigo no UI após 2s
    if (lastAck.length()>0 && (now - lastAckMs) > 2000) lastAck="";
  }

  // Logging CSV
  if (log_enabled) {
    unsigned long nowMs = millis();
    if (nowMs - lastLog >= LOG_IV_MS) {
      lastLog = nowMs;
      appendCsvRow();
    }
  }
}
