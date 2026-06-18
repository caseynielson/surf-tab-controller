#pragma once

/*
 * config.h — timing constants and tuning values
 *
 * ACTUATOR_DEFAULT_TRAVEL_MS is the fallback before calibration.
 * After running "CAL", measured travel times are saved to NVS and loaded on boot.
 *
 * Preset positions are percentages (0-100) of each actuator's full travel.
 * Actual ms values are computed at runtime, so presets stay correct after re-cal.
 *
 * Current sense stall detection (IBT-2 IS pins):
 *   IBT-2 IS output = I_motor / 8500  (current source)
 *   Wire 680 Ohm from each IS pin to GND.
 *   V_IS = I_motor * 680 / 8500
 *   At ~3A running : V ~= 0.24V  ->  ADC ~= 298
 *   At ~6A stall   : V ~= 0.48V  ->  ADC ~= 596
 *   Run "CAL" and read the suggested threshold from serial output.
 */

// Actuator travel
#define ACTUATOR_DEFAULT_TRAVEL_MS  5000   // ms fallback; overridden by NVS after CAL
#define ACTUATOR_PWM_FREQ           1000   // Hz
#define ACTUATOR_PWM_RES            8      // bits (0-255)
#define ACTUATOR_FULL_SPEED         220    // PWM duty (0-255)

// Stall detection (used during CAL extend-to-stall phase)
// Running current: ADC ~298 at 3A; stall: ADC ~596 at 6A (with 680Ω IS resistor)
// Run "CAL" and read suggested threshold from serial output.
#define STALL_THRESHOLD           450    // ADC counts (0-4095); tune after running CAL
#define STALL_CONFIRM_MS          150    // ms: must exceed threshold continuously to confirm stall
#define CAL_EXTEND_TIMEOUT_MS    15000   // abort calibration extend if no stall

// Homing: Lenco actuators have internal limit switches that OPEN at end-of-travel.
// When the switch opens, motor current drops to near-zero — we detect the dropout.
// We first extend briefly to pull off the retract limit, then retract into it.
#define HOME_PREEXTEND_MS         600    // ms to extend before retracting (clears limit switch)
#define HOME_RUNNING_THRESHOLD    100    // ADC: motor is drawing current (switch closed, moving)
#define HOME_DROPOUT_THRESHOLD     60    // ADC: current gone (limit switch opened = at home)
#define HOME_RETRACT_TIMEOUT_MS  20000   // abort if limit switch dropout never detected

// PWM ledc channels
#define PORT_RPWM_CH   0
#define PORT_LPWM_CH   1
#define STBD_RPWM_CH   2
#define STBD_LPWM_CH   3

// Serial baud rates
#define SERIAL_BAUD    115200
#define UI_SERIAL_BAUD 115200

// Actuator enable flags — set false to skip homing/operation on bench
// without that actuator wired. Both true for normal boat operation.
#define PORT_ENABLED   true
#define STBD_ENABLED   true
