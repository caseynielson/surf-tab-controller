#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include "config.h"

/*
 * ActuatorController - drives one IBT-2 (BTS7960) H-bridge + Lenco linear actuator
 *
 * Position tracking is time-based (ms from home).
 * Home = fully retracted, detected by current stall on the IS pin.
 *
 * Calibration:
 *   Call calibrate() once on the bench with the actuator free to travel fully.
 *   It homes, then extends to stall, measures travel time, and saves to NVS.
 *   On every subsequent boot, loadCalibration() (called inside begin()) restores it.
 *   Presets in the .ino use percentages * getFullTravel(), so they stay correct.
 */

enum class ActuatorState {
  IDLE,
  HOMING,
  EXTENDING,
  RETRACTING,
  STALLED,
  ERROR
};

class ActuatorController {
public:
  ActuatorController(
    const char* name,
    uint8_t pin_rpwm, uint8_t pin_lpwm,
    uint8_t pin_r_en, uint8_t pin_l_en,
    uint8_t pin_r_is, uint8_t pin_l_is,
    uint8_t ledc_rpwm_ch, uint8_t ledc_lpwm_ch,
    int stallThreshold = STALL_THRESHOLD  // per-actuator override; default from config.h
  );

  // Initialize GPIO/PWM and load calibration from NVS
  void begin();

  // Blocking home: retract to stall, set position = 0
  bool home();

  // Non-blocking: start moving toward target position (ms from home)
  void goToPosition(uint32_t targetMs);

  // Immediate stop, updates position estimate
  void stop();

  // Call from loop() - advances state machine, checks for stall
  void update();

  // Calibrate: home then extend to stall, measuring full travel time.
  // Prints ADC every 250ms throughout - use output to tune STALL_THRESHOLD.
  // Saves result to NVS. Actuator left at full extension on return.
  // Returns measured travel ms, or 0 on failure.
  uint32_t calibrate();

  // Diagnostic: drive motor for durationMs and print both IS ADC values at 100ms.
  // extend=true → forward (R_IS active), extend=false → retract (L_IS active).
  // Blocking. Returns peak active-channel ADC seen.
  int driveTest(bool extend, uint16_t durationMs = 3000);

  // NVS persistence - loadCalibration() called automatically in begin()
  void loadCalibration();
  void saveCalibration(uint32_t travelMs);

  uint32_t      getPosition()   const { return _positionMs; }
  uint32_t      getTarget()     const { return _targetMs; }
  uint32_t      getFullTravel() const { return _fullTravelMs; }
  ActuatorState getState()      const { return _state; }
  bool isHomed() const { return _homed; }
  bool isBusy()  const { return _state == ActuatorState::EXTENDING  ||
                                _state == ActuatorState::RETRACTING ||
                                _state == ActuatorState::HOMING; }
  int readCurrentADC() const;

private:
  const char* _name;
  char        _nvs_key[16];  // e.g. "port_ms", "stbd_ms"

  uint8_t _pin_rpwm, _pin_lpwm;
  uint8_t _pin_r_en, _pin_l_en;
  uint8_t _pin_r_is, _pin_l_is;
  uint8_t _ledc_rpwm_ch, _ledc_lpwm_ch;

  ActuatorState _state        = ActuatorState::IDLE;
  uint32_t      _positionMs   = 0;
  uint32_t      _targetMs     = 0;
  uint32_t      _fullTravelMs = ACTUATOR_DEFAULT_TRAVEL_MS;
  bool          _homed        = false;

  int      _stallThreshold = STALL_THRESHOLD;  // per-instance, set in constructor

  uint32_t _moveStartMs    = 0;
  uint32_t _posAtMoveStart = 0;
  uint32_t _stallStartMs   = 0;
  bool     _stallPending   = false;

  void _extend(uint8_t speed = ACTUATOR_FULL_SPEED);
  void _retract(uint8_t speed = ACTUATOR_FULL_SPEED);
  void _stopMotor();
  bool _checkStall();
};
