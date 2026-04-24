#include "ActuatorController.h"

ActuatorController::ActuatorController(
  const char* name,
  uint8_t pin_rpwm, uint8_t pin_lpwm,
  uint8_t pin_r_en, uint8_t pin_l_en,
  uint8_t pin_r_is, uint8_t pin_l_is,
  uint8_t ledc_rpwm_ch, uint8_t ledc_lpwm_ch
) :
  _name(name),
  _pin_rpwm(pin_rpwm), _pin_lpwm(pin_lpwm),
  _pin_r_en(pin_r_en), _pin_l_en(pin_l_en),
  _pin_r_is(pin_r_is), _pin_l_is(pin_l_is),
  _ledc_rpwm_ch(ledc_rpwm_ch), _ledc_lpwm_ch(ledc_lpwm_ch)
{}

void ActuatorController::begin() {
  // Setup PWM via ledc (ESP32 Arduino core v3.x API)
  ledcAttach(_pin_rpwm, ACTUATOR_PWM_FREQ, ACTUATOR_PWM_RES);
  ledcAttach(_pin_lpwm, ACTUATOR_PWM_FREQ, ACTUATOR_PWM_RES);

  // Enable pins
  pinMode(_pin_r_en, OUTPUT);
  pinMode(_pin_l_en, OUTPUT);
  digitalWrite(_pin_r_en, LOW);
  digitalWrite(_pin_l_en, LOW);

  // Current sense — ADC input only pins, no setup needed
  // (IO34, IO35, IO36, IO39 are input-only on ESP32)

  _stopMotor();
  Serial.printf("[%s] initialized\n", _name);
}

bool ActuatorController::home() {
  Serial.printf("[%s] homing — retracting to stall...\n", _name);
  _state = ActuatorState::HOMING;
  _homed = false;

  _retract(ACTUATOR_FULL_SPEED);

  uint32_t startMs = millis();
  uint32_t stallStartMs = 0;
  bool stallPending = false;

  while (true) {
    // Timeout guard
    if (millis() - startMs > HOME_RETRACT_TIMEOUT_MS) {
      _stopMotor();
      _state = ActuatorState::ERROR;
      Serial.printf("[%s] homing TIMEOUT — check wiring\n", _name);
      return false;
    }

    // Read current sense on retract (L_IS)
    int adc = analogRead(_pin_l_is);

    if (adc > STALL_THRESHOLD) {
      if (!stallPending) {
        stallPending = true;
        stallStartMs = millis();
      } else if (millis() - stallStartMs >= STALL_CONFIRM_MS) {
        // Confirmed stall — we're home
        _stopMotor();
        _positionMs = 0;
        _homed = true;
        _state = ActuatorState::IDLE;
        Serial.printf("[%s] homed OK (stall ADC=%d, t=%lums)\n",
                      _name, adc, millis() - startMs);
        return true;
      }
    } else {
      stallPending = false;
    }

    delay(10);
  }
}

void ActuatorController::goToPosition(uint32_t targetMs) {
  if (!_homed) {
    Serial.printf("[%s] goToPosition called before homing — ignored\n", _name);
    return;
  }

  targetMs = constrain(targetMs, 0, ACTUATOR_FULL_TRAVEL_MS);
  _targetMs = targetMs;

  if (targetMs == _positionMs) {
    return;
  }

  _posAtMoveStart = _positionMs;
  _moveStartMs    = millis();
  _stallPending   = false;

  if (targetMs > _positionMs) {
    Serial.printf("[%s] extending: %lums → %lums\n", _name, _positionMs, targetMs);
    _state = ActuatorState::EXTENDING;
    _extend(ACTUATOR_FULL_SPEED);
  } else {
    Serial.printf("[%s] retracting: %lums → %lums\n", _name, _positionMs, targetMs);
    _state = ActuatorState::RETRACTING;
    _retract(ACTUATOR_FULL_SPEED);
  }
}

void ActuatorController::stop() {
  _stopMotor();
  // Update position estimate based on time moved
  if (_state == ActuatorState::EXTENDING || _state == ActuatorState::RETRACTING) {
    uint32_t elapsed = millis() - _moveStartMs;
    if (_state == ActuatorState::EXTENDING) {
      _positionMs = min(_posAtMoveStart + elapsed, (uint32_t)ACTUATOR_FULL_TRAVEL_MS);
    } else {
      _positionMs = (_posAtMoveStart > elapsed) ? _posAtMoveStart - elapsed : 0;
    }
  }
  _state = ActuatorState::IDLE;
  Serial.printf("[%s] stopped at ~%lums\n", _name, _positionMs);
}

void ActuatorController::update() {
  if (_state == ActuatorState::IDLE || _state == ActuatorState::ERROR) return;

  uint32_t elapsed = millis() - _moveStartMs;

  if (_state == ActuatorState::EXTENDING) {
    _positionMs = min(_posAtMoveStart + elapsed, (uint32_t)ACTUATOR_FULL_TRAVEL_MS);

    // Check for stall (unexpected obstruction)
    if (_checkStall()) {
      _stopMotor();
      _state = ActuatorState::STALLED;
      Serial.printf("[%s] STALL during extend at ~%lums\n", _name, _positionMs);
      return;
    }

    // Reached target?
    if (_positionMs >= _targetMs) {
      _stopMotor();
      _positionMs = _targetMs;
      _state = ActuatorState::IDLE;
      Serial.printf("[%s] reached %lums\n", _name, _positionMs);
    }

  } else if (_state == ActuatorState::RETRACTING) {
    uint32_t newPos = (_posAtMoveStart > elapsed) ? _posAtMoveStart - elapsed : 0;
    _positionMs = newPos;

    if (_checkStall()) {
      _stopMotor();
      // If we stall near home during a retract, treat as homed
      if (_positionMs < 200) {
        _positionMs = 0;
        _state = ActuatorState::IDLE;
        Serial.printf("[%s] stall at near-home, position reset to 0\n", _name);
      } else {
        _state = ActuatorState::STALLED;
        Serial.printf("[%s] STALL during retract at ~%lums\n", _name, _positionMs);
      }
      return;
    }

    if (_positionMs <= _targetMs) {
      _stopMotor();
      _positionMs = _targetMs;
      _state = ActuatorState::IDLE;
      Serial.printf("[%s] reached %lums\n", _name, _positionMs);
    }
  }
}

int ActuatorController::readCurrentADC() const {
  if (_state == ActuatorState::EXTENDING || _state == ActuatorState::HOMING) {
    return analogRead(_pin_r_is);
  }
  return analogRead(_pin_l_is);
}

// ── Private ───────────────────────────────────────────────────────────────────

void ActuatorController::_extend(uint8_t speed) {
  digitalWrite(_pin_l_en, LOW);
  ledcWrite(_pin_lpwm, 0);
  digitalWrite(_pin_r_en, HIGH);
  ledcWrite(_pin_rpwm, speed);
}

void ActuatorController::_retract(uint8_t speed) {
  digitalWrite(_pin_r_en, LOW);
  ledcWrite(_pin_rpwm, 0);
  digitalWrite(_pin_l_en, HIGH);
  ledcWrite(_pin_lpwm, speed);
}

void ActuatorController::_stopMotor() {
  digitalWrite(_pin_r_en, LOW);
  digitalWrite(_pin_l_en, LOW);
  ledcWrite(_pin_rpwm, 0);
  ledcWrite(_pin_lpwm, 0);
}

bool ActuatorController::_checkStall() {
  int adc = readCurrentADC();
  if (adc > STALL_THRESHOLD) {
    if (!_stallPending) {
      _stallPending = true;
      _stallStartMs = millis();
    } else if (millis() - _stallStartMs >= STALL_CONFIRM_MS) {
      return true;
    }
  } else {
    _stallPending = false;
  }
  return false;
}
