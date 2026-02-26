// RECEIVER - JSON
#include <SoftwareSerial.h>

SoftwareSerial LoRa(2, 3); // RX=3, TX=2

String rxBuffer = "";

void sendAT(const char* cmd) {
  LoRa.println(cmd);
  delay(100);
  while (LoRa.available()) Serial.write(LoRa.read());
}

void setup() {
  Serial.begin(9600);
  LoRa.begin(9600);

  Serial.println("LoRa JSON Receptor");

  delay(500);
  LoRa.print("+++");
  delay(300);

  sendAT("AT+BAUD4");      // 9600
  sendAT("AT+CHANNEL0F");  // Channel 15
  sendAT("AT+MAC02,00");   // Adress 0x0002
  sendAT("AT+POWE5");      // Max power
  sendAT("AT+MODE0");      // Transparent
  sendAT("AT+RESET");      // Restart module

  Serial.println("Config OK");
}

void loop() {
  while (LoRa.available()) {
    char c = LoRa.read();

    if (c == '\n') {
      // mensagem completa
      Serial.print("RX -> ");
      Serial.println(rxBuffer);
      rxBuffer = "";
    } else if (rxBuffer.length() < 128) {
      rxBuffer += c;
    } else {
      // overflow de segurança
      rxBuffer = "";
    }
  }
}
