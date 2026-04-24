#pragma once

#include <Arduino.h>
#include "config.h"

/*
 * ActuatorController — drives one IBT-2 (BTS7960) H-bridge + Lenco linear actuator
 *
 * Position tracking is time-based (ms from home).
 * Home = fully retracted, detected by current stall.
 * No external position sensor required.
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
    uint8_t ledc_rpwm_ch, uint8_t ledc_lpwm_ch
  );

  void begin();

  // Blocking home: retract to stall, set position = 0
  // Returns true if homed successfully, false on timeout
  bool home();

  // Non-blocking: start moving toward target position (ms from home)
  void goToPosition(uint32_t targetMs);

  // Immediate stop
  void stop();

  // Call from loop() — updates position estimate, checks for stall
  void update();

  // Accessors
  uint32_t getPosition() const { return _positionMs; }
  uint32_t getTarget()   const { return _targetMs; }
  ActuatorState getState() const { return _state; }
  bool isHomed()   const { return _homed; }
  bool isBusy()    const { return _state == ActuatorState::EXTENDING ||
                                  _state == ActuatorState::RETRACTING ||
                                  _state == ActuatorState::HOMING; }
  int  readCurrentADC() const;   // raw ADC of active IS pin

private:
  const char* _name;

  // Pins
  uint8_t _pin_rpwm, _pin_lpwm;
  uint8_t _pin_r_en, _pin_l_en;
  uint8_t _pin_r_is, _pin_l_is;
  uint8_t _ledc_rpwm_ch, _ledc_lpwm_ch;

  // State
  ActuatorState _state     = ActuatorState::IDLE;
  uint32_t      _positionMs = 0;
  uint32_t      _targetMs   = 0;
  bool          _homed       = false;

  // Internal timing
  uint32_t _moveStartMs    = 0;   // millis() when move began
  uint32_t _posAtMoveStart = 0;   // _positionMs when move began
  uint32_t _stallStartMs   = 0;   // when current first exceeded threshold
  bool     _stallPending   = false;

  void _extend(uint8_t speed = ACTUATOR_FULL_SPEED);
  void _retract(uint8_t speed = ACTUATOR_FULL_SPEED);
  void _stopMotor();
  bool _checkStall();
};
