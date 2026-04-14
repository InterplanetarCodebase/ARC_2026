// Arduino Nano FlySky PWM reader
// Reads RC PWM on D10 (CH1) and D11 (CH2)
// and streams pulse widths in microseconds over Serial.

#include <Arduino.h>

static const uint8_t CH1_PIN = 10;  // PB2 / PCINT2
static const uint8_t CH2_PIN = 11;  // PB3 / PCINT3

volatile uint32_t ch1_rise_us = 0;
volatile uint32_t ch2_rise_us = 0;
volatile uint16_t ch1_width_us = 1500;
volatile uint16_t ch2_width_us = 1500;
volatile uint32_t ch1_last_update_ms = 0;
volatile uint32_t ch2_last_update_ms = 0;
volatile uint8_t prev_pinb_state = 0;

ISR(PCINT0_vect) {
  uint32_t now_us = micros();
  uint8_t pinb_state = PINB;

  bool ch1_prev_high = (prev_pinb_state & _BV(PB2)) != 0;
  bool ch1_now_high = (pinb_state & _BV(PB2)) != 0;
  bool ch2_prev_high = (prev_pinb_state & _BV(PB3)) != 0;
  bool ch2_now_high = (pinb_state & _BV(PB3)) != 0;

  if (!ch1_prev_high && ch1_now_high) {
    ch1_rise_us = now_us;
  } else if (ch1_prev_high && !ch1_now_high) {
    uint32_t width = now_us - ch1_rise_us;
    if (width >= 750 && width <= 2250) {
      ch1_width_us = (uint16_t)width;
      ch1_last_update_ms = millis();
    }
  }

  if (!ch2_prev_high && ch2_now_high) {
    ch2_rise_us = now_us;
  } else if (ch2_prev_high && !ch2_now_high) {
    uint32_t width = now_us - ch2_rise_us;
    if (width >= 750 && width <= 2250) {
      ch2_width_us = (uint16_t)width;
      ch2_last_update_ms = millis();
    }
  }

  prev_pinb_state = pinb_state;
}

void setup() {
  Serial.begin(115200);

  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);

  // Enable pin-change interrupts for Port B (D8-D13) and mask D10/D11.
  prev_pinb_state = PINB;
  PCICR |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT2) | _BV(PCINT3);

  Serial.println("Arduino Nano PWM reader ready (D10/D11)");
}

void loop() {
  static uint32_t last_print_ms = 0;
  uint32_t now_ms = millis();

  if (now_ms - last_print_ms >= 50) {
    last_print_ms = now_ms;

    noInterrupts();
    uint16_t ch1 = ch1_width_us;
    uint16_t ch2 = ch2_width_us;
    uint32_t ch1_age = now_ms - ch1_last_update_ms;
    uint32_t ch2_age = now_ms - ch2_last_update_ms;
    interrupts();

    // If signal is lost, print 0 for that channel.
    if (ch1_age > 200) {
      ch1 = 0;
    }
    if (ch2_age > 200) {
      ch2 = 0;
    }

    Serial.print("CH1:");
    Serial.print(ch1);
    Serial.print(" CH2:");
    Serial.println(ch2);
  }
}
