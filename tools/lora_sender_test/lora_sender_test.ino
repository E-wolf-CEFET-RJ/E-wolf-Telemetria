// SEND - JSON
#include <SoftwareSerial.h>

SoftwareSerial LoRa(2, 3); // RX=3, TX=2

unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL_MS = 200; // 5 Hz (bem agressivo, mas ok)

uint32_t counter = 0;

void sendAT(const char* cmd) {
  LoRa.println(cmd);
  delay(100); // só aqui, setup
  while (LoRa.available()) Serial.write(LoRa.read());
}

void setup() {
  Serial.begin(9600);
  LoRa.begin(9600);

  Serial.println("LoRa JSON Transmissor");

  delay(500);
  LoRa.print("+++"); 
  delay(300);


  sendAT("AT+BAUD4");      // 9600
  sendAT("AT+CHANNEL0F");  // Channel 15
  sendAT("AT+MAC01,00");   // Adress 0x0001
  sendAT("AT+POWE5");      // Max power
  sendAT("AT+MODE0");      // Transparent
  sendAT("AT+RESET");      // Restart module
     

  Serial.println("Config OK");
}

void loop() {
  unsigned long now = millis();

  if (now - lastSend >= SEND_INTERVAL_MS) {
    lastSend = now;
    counter++;

    // JSON minificado
    LoRa.print("{\"id\":1,\"cnt\":");
    LoRa.print(counter);
    LoRa.print(",\"ms\":");
    LoRa.print(now);
    LoRa.println("}");

    Serial.print("TX -> ");
    Serial.println(counter);
  }
}
