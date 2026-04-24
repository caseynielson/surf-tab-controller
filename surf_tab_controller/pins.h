#pragma once

/*
 * pins.h — GPIO assignments for ESP32 dev board (motor controller)
 *
 * Target: standard ESP32-WROOM-32 / ESP32 Dev Module
 * Two IBT-2 (BTS7960) motor drivers, one per Lenco tab actuator
 *
 * IBT-2 wiring per channel:
 *   RPWM  → extend (forward)
 *   LPWM  → retract (reverse)
 *   R_EN  → forward enable  (HIGH = enabled)
 *   L_EN  → reverse enable  (HIGH = enabled)
 *   R_IS  → forward current sense (analog, needs 500Ω–1KΩ to GND)
 *   L_IS  → reverse current sense (analog, needs 500Ω–1KΩ to GND)
 *   VCC   → 3.3V
 *   GND   → GND
 *   B+/B- → actuator wires
 *   12V/GND → boat 12V power
 */

// ── Port tab actuator (IBT-2 #1) ─────────────────────────────────────────────
#define PORT_RPWM   25   // Extend PWM
#define PORT_LPWM   26   // Retract PWM
#define PORT_R_EN   27   // Forward enable
#define PORT_L_EN   14   // Reverse enable
#define PORT_R_IS   34   // Forward current sense (ADC, input only)
#define PORT_L_IS   35   // Reverse current sense (ADC, input only)

// ── Starboard tab actuator (IBT-2 #2) ────────────────────────────────────────
#define STBD_RPWM   32   // Extend PWM
#define STBD_LPWM   33   // Retract PWM
#define STBD_R_EN   12   // Forward enable
#define STBD_L_EN   13   // Reverse enable
#define STBD_R_IS   36   // Forward current sense (ADC, input only)
#define STBD_L_IS   39   // Reverse current sense (ADC, input only)

// ── UART to CYD display board ────────────────────────────────────────────────
// Connect CYD P1 connector: TX→RX, RX→TX, GND→GND
#define UI_TX       17   // ESP32 TX2 → CYD RX
#define UI_RX       16   // ESP32 RX2 ← CYD TX

// ── Status LED (onboard, optional) ───────────────────────────────────────────
#define STATUS_LED   2   // Built-in LED on most ESP32 dev boards
