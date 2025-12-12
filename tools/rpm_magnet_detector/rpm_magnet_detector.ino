// ===== Detector de Ímã usando Sensor de RPM (Hall) =====
// Arduino Mega
// Entrada: A8 (PORTK / PCINT16)
// Saída: LED onboard (D13)
// Sem uso de #include

// ---------- Pinos ----------
const uint8_t PIN_RPM = A8;    // PK0 / PCINT16
const uint8_t PIN_LED = 13;    // LED onboard

// ---------- Controle ----------
volatile bool magnetDetected = false;
volatile unsigned long lastPulseMicros = 0;

// ---------- ISR de Pin Change Interrupt (PORTK) ----------
ISR(PCINT2_vect) {
  // A8 = PK0 = bit 0 do PORTK
  if (!(PINK & (1 << 0))) {   // nível baixo = pulso do Hall
    lastPulseMicros = micros();
    magnetDetected = true;
  }
}

void setup() {
  pinMode(PIN_RPM, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);

  // Habilita PCINT para PORTK (A8–A15)
  PCICR  |= (1 << PCIE2);     // Grupo PCINT2 → PORTK
  PCMSK2 |= (1 << PCINT16);   // Habilita A8
}

void loop() {
  static unsigned long lastSeen = 0;

  noInterrupts();
  bool detected = magnetDetected;
  magnetDetected = false;
  interrupts();

  if (detected) {
    digitalWrite(PIN_LED, HIGH);   // Ímã detectado
    lastSeen = millis();
  }

  // Apaga o LED se não houver pulso por 500 ms
  if (millis() - lastSeen > 500) {
    digitalWrite(PIN_LED, LOW);
  }
}
