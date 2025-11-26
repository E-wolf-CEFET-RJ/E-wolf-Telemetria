// ===== ESP32-C3 SuperMini — telemetry_hub_esp32c3_v5_serial1_full.ino =====
// Hub de telemetria + MQTT + OTA + mDNS + Logger CSV + BLE UART.
// NÃO tem página HTTP: só WiFi + MQTT + BLE + logging.
// Sensores só em telemetria; o Arduino MEGA controla PWM/motor.
//
// UART0 provavelmente queimada -> usamos Serial1 em GPIO6/7 para falar com o MEGA.
// Serial (USB) fica só para debug/log.
//
// Ligações físicas:
//   ESP32-C3 GPIO6 (RX)  <- TX1 (D18) do MEGA  (via divisor resistivo 5V -> 3.3V)
//   ESP32-C3 GPIO7 (TX)  -> RX1 (D19) do MEGA  (direto)
//   GND em comum.
//
// ========= INCLUDES =========
#include <WiFi.h>
#include <WiFiManager.h>
#include <FS.h>
#include <LittleFS.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <time.h>
#include <NimBLEDevice.h>

// ========= UART COM ARDUINO (Serial1) =========
HardwareSerial SerialArd(1);
static const int ARD_RX_PIN = 6;  // ESP32-C3 RX para Arduino TX1
static const int ARD_TX_PIN = 7;  // ESP32-C3 TX para Arduino RX1

// ========= MQTT =========
const char* MQTT_HOST = "broker.mqtt-dashboard.com";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_ID_BASE = "EWolfTelemetryC3";

const char* TOP_TLM_JSON    = "pb/telemetry/json";
const char* TOP_CMD_MOTOR   = "pb/cmd/motor";    // 0/1, START/STOP
const char* TOP_STATUS      = "pb/status";       // LWT
const char* TOP_CMD_THROTTLE= "pb/cmd/throttle"; // override de aceleração
const char* TOP_CMD_CONFIG  = "pb/cmd/config";   // ajustes de config

WiFiClient espClient;
PubSubClient mqtt(espClient);
unsigned long lastMqtt = 0;
const unsigned long MQTT_IV_MS = 1000;

// ========= ESTADO VINDO DO ARDUINO =========
volatile float g_volts=0, g_pct=0, g_temp=NAN, g_humi=NAN;
volatile float g_rpm=0, g_speed_kmh=0;
volatile float g_min_v=0.80f, g_max_v=4.20f;
volatile float g_wheel_cm=50.8f;
volatile uint8_t g_ppr=1;
volatile bool   g_override=false;      // override vindo de algum lugar (local/BLE/MQTT)
volatile float  g_override_pct=0;      // valor do override (0–100%)

volatile float g_current_bat_a=0.0f;
volatile float g_current_mot_a=0.0f;
volatile float g_current_a=0.0f;

volatile float g_max_pct = 100.0f;
volatile uint32_t g_poll_ms = 1000;    // só espelhado/telemetria (quem manda é o MEGA)

String sensLine;
String lastAck;
unsigned long lastAckMs=0;

// ========= PARÂMETROS EXTRA (PWM/RAMPAS) =========
// Esses vêm de pb/cmd/config e são espelhados no JSON/CSV.
// Implementação no MEGA fica para depois (via protocolo serial).
float g_pwm_hz        = 1000.0f;  // PWM (Hz)
float g_start_min_pct = 8.0f;     // start mínimo (%)
float g_rapid_ms      = 250.0f;   // rampa rápida (ms)
float g_rapid_up      = 150.0f;   // rapid up (pct/s)
float g_slew_up       = 40.0f;    // slew up (pct/s)
float g_slew_dn       = 60.0f;    // slew down (pct/s)
float g_zero_timeout_ms = 600.0f; // zerar RPM após

// ========= DETECÇÃO INICIAL DE TELEMETRIA DO ARDUINO =========
bool haveSerialData = false;            // se já chegou QUALQUER byte do Arduino
bool reportedSerialStatus = false;      // se já falamos no Serial (USB) o status da telemetria
unsigned long serialCheckStartMs = 0;   // começa a contar no fim do setup
const unsigned long SERIAL_CHECK_WINDOW_MS = 5000; // janela inicial de 5 s

// ========= CONTROLE: QUEM MANDA? =========
enum ControlSource {
  CTRL_LOCAL = 0,   // acelerador físico / Arduino
  CTRL_BLE   = 1,   // app BLE
  CTRL_MQTT  = 2    // dashboard MQTT
};

ControlSource g_ctrl_src = CTRL_LOCAL;
unsigned long g_ctrl_src_last_ms = 0;
const unsigned long CTRL_TIMEOUT_MS = 2000; // 2s sem comando remoto -> volta pro local

// ========= LOGGING CSV (LittleFS) =========
static const char* LOG_PATH     = "/telemetry.csv";
static const char* LOG_PATH_OLD = "/telemetry.csv.1";
bool     log_enabled = false;
uint32_t LOG_IV_MS = 1000;
unsigned long lastLog = 0;

// ========= BLE UART =========
static NimBLECharacteristic* bleTxChar = nullptr; // ESP32 -> celular
static NimBLECharacteristic* bleRxChar = nullptr; // celular -> ESP32

// modo BLE: 0 = normal, 1 = RPM, 2 = controle remoto
int bleMode = 0;
volatile bool  g_ble_remote_mode = false;
int g_ble_remote_pct = 0;

// ========= HELPERS GERAIS =========
void sendCmd(const String& s){
  // Comando para o Arduino
  SerialArd.println(s);
}

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

// ========= LittleFS helpers =========
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
  f.println(F("ts_iso,ms,volts,pct,temp,humi,rpm,speed_kmh,current_bat_a,current_mot_a,min,max,wheel_cm,ppr,override,override_pct,max_pct,rssi,src,log_enabled,log_iv_ms,pwm_hz,start_min_pct,rapid_ms,rapid_up,slew_up,slew_dn,zero_timeout_ms,ack"));
  f.close();
}
String fmtMaybeNan(float v, int digits=3) {
  if (isnan(v)) return String();
  return String(v, digits);
}
int wifiRSSI() {
  return (WiFi.status()==WL_CONNECTED) ? WiFi.RSSI() : 0;
}
const char* ctrlSrcName(ControlSource s){
  switch(s){
    case CTRL_BLE:   return "BLE";
    case CTRL_MQTT:  return "MQTT";
    case CTRL_LOCAL:
    default:         return "LOCAL";
  }
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
  f.print(wifiRSSI()); f.print(',');
  f.print(ctrlSrcName(g_ctrl_src)); f.print(',');
  f.print(log_enabled ? 1 : 0); f.print(',');
  f.print(LOG_IV_MS); f.print(',');
  f.print(String(g_pwm_hz,1)); f.print(',');
  f.print(String(g_start_min_pct,1)); f.print(',');
  f.print(String(g_rapid_ms,1)); f.print(',');
  f.print(String(g_rapid_up,1)); f.print(',');
  f.print(String(g_slew_up,1)); f.print(',');
  f.print(String(g_slew_dn,1)); f.print(',');
  f.print(String(g_zero_timeout_ms,1)); f.print(',');
  f.print(lastAck);
  f.println();
  f.close();
}

// ========= MQTT =========
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
  j += ",\"src\":\""; j += ctrlSrcName(g_ctrl_src); j += "\"";
  j += ",\"poll_ms\":" + String(g_poll_ms);
  // logger
  j += ",\"log_enabled\":"; j += (log_enabled ? "true" : "false");
  j += ",\"log_iv_ms\":" + String(LOG_IV_MS);
  j += ",\"log_size\":" + String(fileSize(LOG_PATH));
  // parâmetros extras
  j += ",\"pwm_hz\":" + String(g_pwm_hz,1);
  j += ",\"start_min_pct\":" + String(g_start_min_pct,1);
  j += ",\"rapid_ms\":" + String(g_rapid_ms,1);
  j += ",\"rapid_up\":" + String(g_rapid_up,1);
  j += ",\"slew_up\":" + String(g_slew_up,1);
  j += ",\"slew_dn\":" + String(g_slew_dn,1);
  j += ",\"zero_timeout_ms\":" + String(g_zero_timeout_ms,1);
  // ack
  j += ",\"ack\":";
  if (lastAck.length() > 0) {
    j += "\"";
    String esc = lastAck; esc.replace("\"","\\\"");
    j += esc;
    j += "\"";
  } else {
    j += "null";
  }
  j += "}";

  bool ok = mqtt.publish(TOP_TLM_JSON, j.c_str(), false);
  if (!ok) {
    Serial.print("[MQTT] publish FAIL, len = ");
    Serial.println(j.length());
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length){
  String top = String(topic);

  // ----- Comando de motor -----
  if (top == TOP_CMD_MOTOR){
    String msg;
    msg.reserve(length);
    for (unsigned int i=0;i<length;i++) msg += (char)payload[i];
    msg.trim();

    if (msg.equalsIgnoreCase("0") || msg.equalsIgnoreCase("OFF") || msg.equalsIgnoreCase("STOP")){
      sendCmd(F("STOP"));
    } else if (msg.equalsIgnoreCase("1") || msg.equalsIgnoreCase("ON") || msg.equalsIgnoreCase("START")){
      sendCmd(F("START"));
    }
  }

  // ----- Throttle / override MQTT -----
  else if (top == TOP_CMD_THROTTLE){
    String s;
    for (unsigned int i=0;i<length;i++) s += (char)payload[i];
    s.trim();

    bool isOverride = (s.indexOf("override") >= 0);
    bool isAuto     = (s.indexOf("auto")     >= 0);

    if (isOverride) {
      int pIdx = s.indexOf("\"pct\"");
      float pct = g_override_pct;
      if (pIdx >= 0) {
        int col = s.indexOf(':', pIdx);
        if (col > 0) pct = s.substring(col+1).toFloat();
      }
      g_override = true;
      g_override_pct = constrain(pct, 0, 100);
      g_ctrl_src = CTRL_MQTT;
      g_ctrl_src_last_ms = millis();
      // Aqui, se quiser controlar o MEGA de fato, pode mandar comando serial no futuro.
    }
    else if (isAuto) {
      g_override = false;
      g_ctrl_src = CTRL_LOCAL;
      g_ctrl_src_last_ms = millis();
      // Idem, aqui poderia mandar "OVR:0" pro MEGA se ele suportar.
    }
  }

  // ----- Config remota via MQTT -----
  else if (top == TOP_CMD_CONFIG){
    String s;
    for (unsigned int i=0;i<length;i++) s += (char)payload[i];
    s.trim();

    int idx;

    // max_pct
    idx = s.indexOf("max_pct");
    if (idx >= 0) {
      int col = s.indexOf(':', idx);
      if (col > 0) g_max_pct = constrain(s.substring(col+1).toFloat(), 0, 100);
    }

    // min_v
    idx = s.indexOf("min_v");
    if (idx >= 0) {
      int col = s.indexOf(':', idx);
      if (col > 0) g_min_v = s.substring(col+1).toFloat();
    }

    // max_v
    idx = s.indexOf("max_v");
    if (idx >= 0) {
      int col = s.indexOf(':', idx);
      if (col > 0) g_max_v = s.substring(col+1).toFloat();
    }

    // wheel_cm
    idx = s.indexOf("wheel_cm");
    if (idx >= 0) {
      int col = s.indexOf(':', idx);
      if (col > 0) g_wheel_cm = s.substring(col+1).toFloat();
    }

    // ppr
    idx = s.indexOf("ppr");
    if (idx >= 0) {
      int col = s.indexOf(':', idx);
      if (col > 0) g_ppr = (uint8_t)s.substring(col+1).toInt();
    }

    // log on/off
    idx = s.indexOf("log");
    if (idx >= 0) {
      int col = s.indexOf(':', idx);
      if (col > 0) {
        String v = s.substring(col+1);
        v.toLowerCase();
        if (v.indexOf("true")>=0 || v.indexOf("1")>=0) log_enabled = true;
        else log_enabled = false;
      }
    }

    // poll_ms (espelho)
    idx = s.indexOf("poll_ms");
    if (idx >= 0) {
      int col = s.indexOf(':', idx);
      if (col > 0) g_poll_ms = (uint32_t)s.substring(col+1).toInt();
    }

    // log_iv_ms
    idx = s.indexOf("log_iv_ms");
    if (idx >= 0) {
      int col = s.indexOf(':', idx);
      if (col > 0) {
        uint32_t ms = (uint32_t)s.substring(col+1).toInt();
        if (ms < 100) ms = 100;
        if (ms > 60000) ms = 60000;
        LOG_IV_MS = ms;
      }
    }

    // log_clear
    idx = s.indexOf("log_clear");
    if (idx >= 0) {
      int col = s.indexOf(':', idx);
      if (col > 0) {
        String v = s.substring(col+1);
        v.toLowerCase();
        bool clear = (v.indexOf("true")>=0 || v.indexOf("1")>=0);
        if (clear) {
          if (LittleFS.exists(LOG_PATH)) LittleFS.remove(LOG_PATH);
          if (LittleFS.exists(LOG_PATH_OLD)) LittleFS.remove(LOG_PATH_OLD);
        }
      }
    }

    // pwm_hz
    idx = s.indexOf("pwm_hz");
    if (idx >= 0) {
      int col = s.indexOf(':', idx);
      if (col > 0) {
        float hz = s.substring(col+1).toFloat();
        if (hz < 100.0f) hz = 100.0f;
        if (hz > 8000.0f) hz = 8000.0f;
        g_pwm_hz = hz;
      }
    }

    // start_min_pct
    idx = s.indexOf("start_min_pct");
    if (idx >= 0) {
      int col = s.indexOf(':', idx);
      if (col > 0) {
        float v = s.substring(col+1).toFloat();
        if (v < 0.0f) v = 0.0f;
        if (v > 40.0f) v = 40.0f;
        g_start_min_pct = v;
      }
    }

    // rapid_ms
    idx = s.indexOf("rapid_ms");
    if (idx >= 0) {
      int col = s.indexOf(':', idx);
      if (col > 0) {
        float ms = s.substring(col+1).toFloat();
        if (ms < 50.0f) ms = 50.0f;
        if (ms > 1500.0f) ms = 1500.0f;
        g_rapid_ms = ms;
      }
    }

    // rapid_up
    idx = s.indexOf("rapid_up");
    if (idx >= 0) {
      int col = s.indexOf(':', idx);
      if (col > 0) {
        float v = s.substring(col+1).toFloat();
        if (v < 10.0f) v = 10.0f;
        if (v > 400.0f) v = 400.0f;
        g_rapid_up = v;
      }
    }

    // slew_up
    idx = s.indexOf("slew_up");
    if (idx >= 0) {
      int col = s.indexOf(':', idx);
      if (col > 0) {
        float v = s.substring(col+1).toFloat();
        if (v < 5.0f) v = 5.0f;
        if (v > 200.0f) v = 200.0f;
        g_slew_up = v;
      }
    }

    // slew_dn
    idx = s.indexOf("slew_dn");
    if (idx >= 0) {
      int col = s.indexOf(':', idx);
      if (col > 0) {
        float v = s.substring(col+1).toFloat();
        if (v < 5.0f) v = 5.0f;
        if (v > 300.0f) v = 300.0f;
        g_slew_dn = v;
      }
    }

    // zero_timeout_ms
    idx = s.indexOf("zero_timeout_ms");
    if (idx >= 0) {
      int col = s.indexOf(':', idx);
      if (col > 0) {
        float v = s.substring(col+1).toFloat();
        if (v < 50.0f) v = 50.0f;
        if (v > 2000.0f) v = 2000.0f;
        g_zero_timeout_ms = v;
      }
    }

    // TODO: se quiser mandar esses parâmetros para o MEGA por serial,
    // este é o lugar para montar comandos tipo "CFG_PWM:1000" etc.
  }
}

void ensureMqtt(){
  if (mqtt.connected()) return;
  while (!mqtt.connected()){
    String cid = String(MQTT_ID_BASE) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (mqtt.connect(cid.c_str(), TOP_STATUS, 0, true, "offline")){
      mqtt.publish(TOP_STATUS, "online", true);
      mqtt.subscribe(TOP_CMD_MOTOR);
      mqtt.subscribe(TOP_CMD_THROTTLE);
      mqtt.subscribe(TOP_CMD_CONFIG);
    } else {
      delay(1500);
    }
  }
}

// ========= OTA + mDNS =========
void setupOTA(){
  if (MDNS.begin("telemetry")){ /* ok */ }
  ArduinoOTA.setHostname("telemetry-c3");
  ArduinoOTA.setPassword("espota"); // troque!
  ArduinoOTA.begin();
}

// ========= BLE (Nordic UART-like) =========
static NimBLEUUID serviceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static NimBLEUUID charRxUUID ("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
static NimBLEUUID charTxUUID ("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

static bool bleClientConnected = false;

class MyServerCallbacks : public NimBLEServerCallbacks {
 public:
  void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
    bleClientConnected = true;
    Serial.println("[BLE] Client connected");
  }

  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
    bleClientConnected = false;
    Serial.printf("[BLE] Client disconnected (reason=%d)\n", reason);
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

    int modeIdx = j.indexOf("\"mode\"");
    int pctIdx  = j.indexOf("\"pct\"");

    if (modeIdx >= 0) {
      int col = j.indexOf(':', modeIdx);
      if (col > 0) {
        bleMode = j.substring(col+1).toInt();
        if (bleMode == 0) {
          g_override = false;
          g_ctrl_src = CTRL_LOCAL;
          g_ctrl_src_last_ms = millis();
        }
      }
    }

    if (pctIdx >= 0) {
      int col = j.indexOf(':', pctIdx);
      if (col > 0) {
        g_ble_remote_pct = constrain(j.substring(col+1).toInt(), 0, 100);
        g_override       = true;
        g_override_pct   = g_ble_remote_pct;
        g_ctrl_src       = CTRL_BLE;
        g_ctrl_src_last_ms = millis();
      }
    }
  }
};

void setupBLE() {
  NimBLEDevice::init("EWolf-Telemetry");
  NimBLEDevice::setDeviceName("EWolf-Telemetry");
  NimBLEDevice::setMTU(100);

  NimBLEServer* pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  NimBLEService* pService = pServer->createService(serviceUUID);

  bleTxChar = pService->createCharacteristic(
      charTxUUID,
      NIMBLE_PROPERTY::NOTIFY
  );

  bleRxChar = pService->createCharacteristic(
      charRxUUID,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  bleRxChar->setCallbacks(new MyRxCallbacks());

  pService->start();

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
  bleTxChar->setValue(j.c_str());
  bleTxChar->notify();
}

// ========= SETUP =========
void setup(){
  // Serial USB para debug
  Serial.begin(115200);
  Serial.setTimeout(50);

  // Serial1 exclusiva para o Arduino
  SerialArd.begin(115200, SERIAL_8N1, ARD_RX_PIN, ARD_TX_PIN);
  SerialArd.setTimeout(50);

  LittleFS.begin(true); // formatOnFail=true

  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.setConfigPortalBlocking(true);
  wm.setTimeout(120);
  wm.setDebugOutput(false);
  wm.autoConnect("Telemetry-Setup");

  Serial.println(F("======================================"));
  Serial.print  (F(" WiFi IP:  ")); Serial.println(WiFi.localIP());
  Serial.print  (F(" MQTT pub: ")); Serial.println(TOP_TLM_JSON);
  Serial.print  (F(" MQTT sub: ")); Serial.println(TOP_CMD_MOTOR);
  Serial.print  (F(" MQTT sub: ")); Serial.println(TOP_CMD_THROTTLE);
  Serial.print  (F(" MQTT sub: ")); Serial.println(TOP_CMD_CONFIG);
  Serial.println(F(" UART Arduino em Serial1 (GPIO6 RX, GPIO7 TX)"));
  Serial.println(F("======================================"));

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(2048);   // buffer maior pro JSON gordo

  setupOTA();
  setupTime();
  setupBLE();

  serialCheckStartMs = millis();
}

// ========= LOOP =========
void loop(){
  ArduinoOTA.handle();

  // ---- Ler telemetria do Arduino (linha K:V) via Serial1 ----
  while (SerialArd.available()){
    haveSerialData = true;
    char c=(char)SerialArd.read();
    if (c=='\n'){
      String line=sensLine; sensLine="";
      line.replace(",", " ");
      line.replace("\r", "");
      line.trim();
      if (line.startsWith("ACK:")){
        lastAck = line.substring(4);
        lastAck.trim();
        lastAckMs = millis();
      } else {
        int pos=0;
        while (pos < (int)line.length()){
          int next=line.indexOf(' ',pos);
          if(next<0) next=line.length();
          String tok=line.substring(pos,next);
          tok.trim();
          int dp=tok.indexOf(':');
          if (dp>0){
            String k=tok.substring(0,dp);
            String v=tok.substring(dp+1);
            if      (k=="MS")      { /* opcional, já usado no Arduino */ }
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
            // se no futuro o MEGA mandar PWM/RAMPAS em K:V, dá pra atualizar aqui também.
          }
          pos = next+1;
        }
      }
    } else if (c!='\r'){
      sensLine += c;
      if (sensLine.length()>400) sensLine="";
    }
  }

  // ---- Checagem única se está recebendo algo do Arduino ----
  if (!reportedSerialStatus) {
    unsigned long nowMs = millis();
    if (nowMs - serialCheckStartMs > SERIAL_CHECK_WINDOW_MS) {
      if (haveSerialData) {
        Serial.println(F("DBG: Recebendo telemetria do Arduino (dados detectados na Serial1)."));
      } else {
        Serial.println(F("DBG: Nenhuma telemetria recebida do Arduino nos primeiros 5s na Serial1."));
      }
      reportedSerialStatus = true;
    }
  }

  // ---- Timeout de controle remoto -> volta pro local ----
  unsigned long now = millis();
  if (g_ctrl_src != CTRL_LOCAL &&
      (now - g_ctrl_src_last_ms) > CTRL_TIMEOUT_MS) {
    g_ctrl_src = CTRL_LOCAL;
    g_override = false;
  }

  // ---- MQTT keepalive + publicação ----
  ensureMqtt();
  mqtt.loop();
  if (now - lastMqtt >= MQTT_IV_MS){
    lastMqtt=now;
    mqttPublishTelemetry();
    if (lastAck.length()>0 && (now - lastAckMs) > 2000) lastAck="";
  }

  // ---- Logging CSV ----
  if (log_enabled) {
    unsigned long nowMs2 = millis();
    if (nowMs2 - lastLog >= LOG_IV_MS) {
      lastLog = nowMs2;
      appendCsvRow();
    }
  }

  // ---- BLE broadcast ----
  static unsigned long lastBle = 0;
  if (now - lastBle >= 200) {
    lastBle = now;
    bleSendSpeedo();
  }
}
