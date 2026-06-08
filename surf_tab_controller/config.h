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

// Stall detection
// Running current: ADC ~298 at 3A; stall: ADC ~596 at 6A (with 680Ω IS resistor)
// Observed stall ADC range: 450–741 — raise STALL_THRESHOLD if false triggers occur.
// Run "CAL" and read suggested threshold from serial output.
#define STALL_THRESHOLD           450    // ADC counts (0-4095); tune after running CAL
#define STALL_CONFIRM_MS          150    // must exceed threshold for this long to confirm
#define HOME_RETRACT_TIMEOUT_MS   8000   // abort homing if no stall within this time
#define CAL_EXTEND_TIMEOUT_MS    15000   // abort calibration extend if no stall

// PWM ledc channels
#define PORT_RPWM_CH   0
#define PORT_LPWM_CH   1
#define STBD_RPWM_CH   2
#define STBD_LPWM_CH   3

// Serial baud rates
#define SERIAL_BAUD    115200
#define UI_SERIAL_BAUD 115200
