#include <SdFat.h>

#define CS_SD 10

SdFat sd;
SdFile file;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("TESTE SdFat - UNO"));

  // No UNO, SS é o pino 10
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  // Inicializa SD com clock baixo
  if (!sd.begin(CS_SD, SD_SCK_MHZ(4))) {
    Serial.println(F("SdFat INIT FALHOU"));
    sd.initErrorPrint();
    while (1);
  }

  Serial.println(F("SdFat OK"));

  if (!file.open("TESTE.TXT", O_WRITE | O_CREAT | O_APPEND)) {
    Serial.println(F("OPEN FALHOU"));
    while (1);
  }

  file.println("SD FUNCIONANDO NO UNO");
  file.close();

  Serial.println(F("ARQUIVO GRAVADO"));
}

void loop() {
  // nada
}
