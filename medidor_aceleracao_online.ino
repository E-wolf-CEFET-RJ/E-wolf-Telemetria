// Wemos D1 mini (ESP8266)
// Lê acelerador (Hall 3 fios) no A0 e publica no Serial e via rede (HTML + JSON)
// WiFi via WiFiManager: se não houver credenciais, abre AP "Throttle-Setup"

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>   // instale a lib "WiFiManager" (tzapu)

// ---------- Ajustes do sensor ----------
const uint8_t THR_PIN = A0;

// ADC do ESP8266: 0..1.0 V
const float ADC_VREF = 1.0f;
const int   ADC_MAX  = 1023;

// Divisor sugerido: 100k (sinal->A0) + 33k (A0->GND) => k ≈ 0.247
const float DIV_K = 33.0f / (100.0f + 33.0f);

// Faixa típica do acelerador (tensão REAL, antes do divisor)
float V_MIN_REAL = 0.879f;   // repouso típico
float V_MAX_REAL = 2.305f;   // fim de curso típico

// ---------- Amostragem ----------
const uint32_t SAMPLE_MS = 50;   // ~20 Hz
uint32_t lastSample = 0;

// ---------- Estado publicado ----------
volatile float g_volts = 0.0f;   // tensão real (antes do divisor)
volatile float g_pct   = 0.0f;   // 0..100 %

ESP8266WebServer server(80);

// Página HTML super simples
const char PAGE_HTML[] PROGMEM = R"(
<!doctype html><html><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Throttle Monitor</title>
<style>
body{font-family:system-ui;margin:0;padding:16px;background:#0b0c10;color:#eaeaea}
.card{max-width:540px;margin:0 auto;background:#171a21;border-radius:16px;padding:16px}
h1{margin:0 0 12px;font-size:20px}
.metric{background:#0f1116;border-radius:12px;padding:12px;margin:8px 0}
h2{margin:0 0 6px;font-size:14px;color:#9aa4b2}
.val{font-size:28px;font-weight:700}
.bar{height:12px;background:#222;border-radius:8px;overflow:hidden}
.fill{height:100%;width:0%;background:#7aa2f7;transition:width .2s}
footer{opacity:.7;text-align:center;margin-top:8px;font-size:12px}
</style></head><body>
<div class="card">
  <h1>Wemos Throttle Monitor</h1>
  <div class="metric"><h2>Tensão (V)</h2><div class="val" id="v">--</div></div>
  <div class="metric"><h2>Aceleração (%)</h2>
    <div class="val" id="p">--</div>
    <div class="bar"><div id="b" class="fill"></div></div>
  </div>
  <footer>Atualiza automaticamente ~3×/s</footer>
</div>
<script>
async function tick(){
  try{
    const r = await fetch('/data',{cache:'no-store'});
    const d = await r.json();
    document.getElementById('v').textContent = d.volts.toFixed(3);
    const pct = (d.pct*100).toFixed(1);
    document.getElementById('p').textContent = pct;
    document.getElementById('b').style.width = pct + '%';
  }catch(e){}
  setTimeout(tick, 300);
}
tick();
</script></body></html>
)";

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", PAGE_HTML);
}

void handleData() {
  // monta JSON com os últimos valores
  String s = F("{\"volts\":");
  s += String(g_volts, 3);
  s += F(",\"pct\":");
  s += String(g_pct / 100.0f, 3); // em 0..1 para facilitar consumo
  s += "}";
  server.send(200, "application/json", s);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\nWemos Throttle Reader (Serial + Web)"));

  // ---- WiFi via WiFiManager ----
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.setConfigPortalBlocking(true);
  wm.setTimeout(120); // 2 min
  if (!wm.autoConnect("Throttle-Setup")) {
    Serial.println(F("WiFiManager: sem Wi-Fi configurado (tempo esgotado). "
                     "Você pode reabrir segurando RESET e regravando o sketch."));
  } else {
    Serial.print(F("Conectado. IP: "));
    Serial.println(WiFi.localIP());
  }

  // HTTP
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  Serial.println(F("HTTP pronto. Use / e /data"));
}

void loop() {
  server.handleClient();

  uint32_t now = millis();
  if (now - lastSample < SAMPLE_MS) return;
  lastSample = now;

  // ===== Leitura do acelerador =====
  int raw = analogRead(THR_PIN);
  float v_adc  = (raw * ADC_VREF) / (float)ADC_MAX; // volts no pino A0 (0..1.0 V)
  float v_real = v_adc / DIV_K;                     // volts reais do sensor

  // Mapeia para % usando limites típicos (ajuste se necessário)
  float pct = (v_real - V_MIN_REAL) / (V_MAX_REAL - V_MIN_REAL);
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 1.0f) pct = 1.0f;

  // Atualiza globais (publicação)
  g_volts = v_real;
  g_pct   = pct * 100.0f;

  // ===== Serial Plotter (formato robusto: label:valor por linha) =====
  // Exemplo de linha: "Volts:1.234 Pct:57.8"
  Serial.print(F("Volts:")); Serial.print(g_volts, 3); Serial.print(' ');
  Serial.print(F("Pct:"));   Serial.println(g_pct, 1);


}
