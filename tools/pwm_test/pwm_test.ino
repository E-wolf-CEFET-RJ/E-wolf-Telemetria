// === PWM 10 kHz em D9 (OC1A) – Teste simples de varredura ===
// Placa: Arduino UNO (16 MHz)

#include <Arduino.h>

const uint8_t PWM_PIN = 11;          // OC1A
#define PWM_FREQ_HZ 10000UL         // 10 kHz

// Configura Timer1 em Fast PWM (modo 14), TOP = ICR1, OC1A non-inverting
static void setupPwmTimer1(uint32_t freq_hz) {
  // tenta prescaler 1 (melhor resolução)
  uint16_t presc_val = 1;
  uint16_t cs_bits = _BV(CS10); // prescaler = 1
  uint32_t top = (F_CPU / (presc_val * freq_hz)) - 1;

  // se precisar, aumenta prescaler (não deve precisar pra 10 kHz no UNO)
  if (top > 65535UL) {
    presc_val = 8;   cs_bits = _BV(CS11);                // /8
    top = (F_CPU / (presc_val * freq_hz)) - 1;
  }
  if (top > 65535UL) {
    presc_val = 64;  cs_bits = _BV(CS11) | _BV(CS10);    // /64
    top = (F_CPU / (presc_val * freq_hz)) - 1;
  }
  if (top > 65535UL) top = 65535UL; // segurança

  // Modo 14: Fast PWM, TOP = ICR1
  TCCR1A = 0;
  TCCR1B = 0;
  // COM1A1=1 -> non-inverting em OC1A (D9)
  TCCR1A |= _BV(COM1A1);
  // WGM13:WGM10 = 1110
  TCCR1A |= _BV(WGM11);
  TCCR1B |= _BV(WGM13) | _BV(WGM12);

  ICR1 = (uint16_t)top;   // define TOP (frequência)
  OCR1A = 0;              // duty inicial 0%

  // habilita clock com o prescaler escolhido
  TCCR1B |= cs_bits;
}

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  setupPwmTimer1(PWM_FREQ_HZ);

  Serial.begin(115200);
  Serial.println(F("PWM 10 kHz em D9 - varrendo duty 0..100%..0"));
}

void loop() {
  // --- Sobe 0% -> 100% ---
  for (uint16_t d = 0; d <= 255; d++) {
    uint16_t top = ICR1;
    uint16_t ocr = (uint32_t)d * top / 255UL;
    OCR1A = ocr;
    if ((d % 32) == 0) {
      Serial.print(F("Subindo  duty="));
      Serial.print((d * 100) / 255);
      Serial.println(F("%"));
    }
    delay(5); // ajuste pra ver a rampa (remova se quiser mais rápido)
  }

  // --- Desce 100% -> 0% ---
  for (int d = 255; d >= 0; d--) {
    uint16_t top = ICR1;
    uint16_t ocr = (uint32_t)d * top / 255UL;
    OCR1A = ocr;
    if ((d % 32) == 0) {
      Serial.print(F("Descendo duty="));
      Serial.print((d * 100) / 255);
      Serial.println(F("%"));
    }
    delay(5);
  }
}

/*
  Quer duty fixo (ex.: 50%) em 10 kHz?
  - No setup(), depois de setupPwmTimer1(PWM_FREQ_HZ), faça:
      uint16_t top = ICR1;
      OCR1A = (uint32_t)top * 50 / 100;  // 50%
  - E deixe o loop() vazio.
*/
