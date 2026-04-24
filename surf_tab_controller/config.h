#pragma once

/*
 * config.h — timing constants, preset positions, and tuning values
 *
 * All positions are in milliseconds from home (fully retracted = 0).
 * Calibrate ACTUATOR_FULL_TRAVEL_MS by timing full retract→extend on bench.
 *
 * Current sense stall detection:
 *   IBT-2 IS pins output I_motor / 8500 as a current source.
 *   Add a 680Ω resistor from each IS pin to GND.
 *   V_IS = I_motor × 680 / 8500
 *   At 3A (running): V ≈ 0.24V → ADC ≈ 298
 *   At stall (~6A):  V ≈ 0.48V → ADC ≈ 596
 *   Threshold set conservatively below stall, above running.
 *   Adjust STALL_THRESHOLD after bench testing.
 */

// ── Actuator travel ───────────────────────────────────────────────────────────
#define ACTUATOR_FULL_TRAVEL_MS   5000   // ms for full retract→extend (calibrate!)
#define ACTUATOR_PWM_FREQ         1000   // Hz
#define ACTUATOR_PWM_RES          8      // bits (0–255)
#define ACTUATOR_FULL_SPEED       220    // PWM duty (0–255), leave headroom

// ── Stall detection ───────────────────────────────────────────────────────────
#define STALL_THRESHOLD           450    // ADC counts (0–4095), tune after testing
#define STALL_CONFIRM_MS          150    // Must exceed threshold for this long
#define HOME_RETRACT_TIMEOUT_MS   8000   // Abort homing if no stall in this time

// ── Preset positions (ms from home / fully retracted) ────────────────────────
// Adjust these after calibrating full travel time on the actual boat
#define PRESET_NEUTRAL_PORT       0
#define PRESET_NEUTRAL_STBD       0

#define PRESET_SURF_LEFT_PORT     (ACTUATOR_FULL_TRAVEL_MS * 80 / 100)  // 80% down
#define PRESET_SURF_LEFT_STBD     (ACTUATOR_FULL_TRAVEL_MS * 20 / 100)  // 20% down

#define PRESET_SURF_RIGHT_PORT    (ACTUATOR_FULL_TRAVEL_MS * 20 / 100)
#define PRESET_SURF_RIGHT_STBD    (ACTUATOR_FULL_TRAVEL_MS * 80 / 100)

#define PRESET_FULL_DOWN_PORT     (ACTUATOR_FULL_TRAVEL_MS * 95 / 100)  // 95%, not 100%
#define PRESET_FULL_DOWN_STBD     (ACTUATOR_FULL_TRAVEL_MS * 95 / 100)  // leave stall headroom

// ── PWM ledc channels ─────────────────────────────────────────────────────────
#define PORT_RPWM_CH   0
#define PORT_LPWM_CH   1
#define STBD_RPWM_CH   2
#define STBD_LPWM_CH   3

// ── Serial baud rates ─────────────────────────────────────────────────────────
#define SERIAL_BAUD    115200
#define UI_SERIAL_BAUD 115200
