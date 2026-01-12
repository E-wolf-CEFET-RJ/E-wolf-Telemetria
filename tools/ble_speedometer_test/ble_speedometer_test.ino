#include <Arduino.h>
#include <NimBLEDevice.h>

#define NUS_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_TX_UUID      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_RX_UUID      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

NimBLECharacteristic* txChar;

float speed = 0;
float rpm = 0;
float pct = 0;
float temp = 35;
float current = 5;
bool up = true;

void setup() {
  Serial.begin(115200);

  NimBLEDevice::init("E-Wolf-Telemetry");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEServer* server = NimBLEDevice::createServer();
  NimBLEService* service = server->createService(NUS_SERVICE_UUID);

  txChar = service->createCharacteristic(
    NUS_TX_UUID,
    NIMBLE_PROPERTY::NOTIFY
  );

  service->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->start();

  Serial.println("E-Wolf BLE Telemetry ativo");
}

void loop() {
  // ---- Simulação realista ----
  if (up) {
    speed += 0.3;
    if (speed >= 120) up = false;
  } else {
    speed -= 0.3;
    if (speed <= 0) up = true;
  }

  rpm = speed * 45;
  pct = (speed / 120.0) * 100.0;
  temp += 0.01;
  current = 5 + pct * 0.15;

  char json[160];
  snprintf(json, sizeof(json),
    "{\"speed_kmh\":%.1f,\"rpm\":%.0f,\"pct\":%.0f,\"temp\":%.1f,\"current\":%.1f}",
    speed, rpm, pct, temp, current
  );

  txChar->setValue((uint8_t*)json, strlen(json));
  txChar->notify();

  Serial.println(json);
  delay(100); // 10 Hz
}
