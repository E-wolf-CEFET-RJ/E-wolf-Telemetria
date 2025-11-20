#include <EEPROM.h>

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

  Serial.println(F("Iniciando limpeza completa da EEPROM..."));
  for (int endereco = 0; endereco < EEPROM.length(); endereco++) {
    EEPROM.write(endereco, 0xFF);
  }
  Serial.println(F("EEPROM apagada (todos os bytes = 0xFF)."));
}

void loop() {
  // Nada a fazer no loop principal.
}
