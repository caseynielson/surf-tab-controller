#include "ActuatorController.h"

ActuatorController::ActuatorController(
  const char* name,
  uint8_t pin_rpwm, uint8_t pin_lpwm,
  uint8_t pin_r_en, uint8_t pin_l_en,
  uint8_t pin_r_is, uint8_t pin_l_is,
  uint8_t ledc_rpwm_ch, uint8_t ledc_lpwm_ch,
  int stallThreshold
) :
  _name(name),
  _pin_rpwm(pin_rpwm), _pin_lpwm(pin_lpwm),
  _pin_r_en(pin_r_en), _pin_l_en(pin_l_en),
  _pin_r_is(pin_r_is), _pin_l_is(pin_l_is),
  _ledc_rpwm_ch(ledc_rpwm_ch), _ledc_lpwm_ch(ledc_lpwm_ch)
{
  // NVS key: lowercase name + "_ms", max 15 chars (e.g. "PORT" -> "port_ms")
  snprintf(_nvs_key, sizeof(_nvs_key), "%.8s_ms", name);
  for (char* p = _nvs_key; *p; p++) *p = tolower((unsigned char)*p);
  _stallThreshold = stallThreshold;
}

// ---- Init -------------------------------------------------------------------

void ActuatorController::begin() {
  ledcAttach(_pin_rpwm, ACTUATOR_PWM_FREQ, ACTUATOR_PWM_RES);
  ledcAttach(_pin_lpwm, ACTUATOR_PWM_FREQ, ACTUATOR_PWM_RES);
  pinMode(_pin_r_en, OUTPUT);
  pinMode(_pin_l_en, OUTPUT);
  digitalWrite(_pin_r_en, LOW);
  digitalWrite(_pin_l_en, LOW);
  _stopMotor();
  loadCalibration();  // sets _fullTravelMs from NVS if available
  Serial.printf("[%s] ready (fullTravel=%lums)\n", _name, _fullTravelMs);
}

// ---- Calibration ------------------------------------------------------------

void ActuatorController::loadCalibration() {
  Preferences prefs;
  prefs.begin("surftab", true);
  uint32_t stored = prefs.getUInt(_nvs_key, 0);
  prefs.end();
  if (stored >= 1000 && stored <= 30000) {
    _fullTravelMs = stored;
    Serial.printf("[%s] calibration loaded: %lums\n", _name, _fullTravelMs);
  } else {
    Serial.printf("[%s] no NVS calibration -- using default %lums (run CAL)\n",
                  _name, _fullTravelMs);
  }
}

void ActuatorController::saveCalibration(uint32_t travelMs) {
  Preferences prefs;
  prefs.begin("surftab", false);
  prefs.putUInt(_nvs_key, travelMs);
  prefs.end();
  Serial.printf("[%s] calibration saved: %lums\n", _name, travelMs);
}

uint32_t ActuatorController::calibrate() {
  Serial.printf("\n[%s] === CALIBRATION START ===\n", _name);
  Serial.printf("[%s] Step 1: homing...\n", _name);

  if (!home()) {
    Serial.printf("[%s] CALIBRATION FAILED: homing failed\n", _name);
    return 0;
  }

  Serial.printf("[%s] Step 2: extending to stall -- watch ADC values below\n", _name);
  Serial.printf("[%s]   (stallThreshold currently = %d)\n", _name, _stallThreshold);
  Serial.printf("[%s]   t(ms)   ADC\n", _name);

  _extend(ACTUATOR_FULL_SPEED);

  uint32_t startMs      = millis();
  uint32_t lastPrintMs  = 0;
  uint32_t stallStartMs = 0;
  bool     stallPending = false;
  int      runningMax   = 0;
  int      stallAdc     = 0;

  while (true) {
    if (millis() - startMs > CAL_EXTEND_TIMEOUT_MS) {
      _stopMotor();
      _state = ActuatorState::ERROR;
      Serial.printf("[%s] CALIBRATION TIMEOUT (%dms) -- stall never detected\n",
                    _name, CAL_EXTEND_TIMEOUT_MS);
      Serial.printf("[%s]   Check: 680 Ohm on IS pins? Motor powered? Direction correct?\n",
                    _name);
      return 0;
    }

    int adc = analogRead(_pin_r_is);

    if (millis() - lastPrintMs >= 250) {
      Serial.printf("[%s]   %5lu   %4d%s\n",
                    _name, millis() - startMs, adc,
                    (adc > _stallThreshold) ? "  <- STALL?" : "");
      lastPrintMs = millis();
    }

    if (adc > _stallThreshold) {
      if (!stallPending) {
        stallPending = true;
        stallStartMs = millis();
        stallAdc     = adc;
      } else if (millis() - stallStartMs >= STALL_CONFIRM_MS) {
        uint32_t travelMs = millis() - startMs;
        _stopMotor();
        _positionMs   = travelMs;
        _fullTravelMs = travelMs;
        _state        = ActuatorState::IDLE;

        Serial.printf("\n[%s] === CALIBRATION COMPLETE ===\n",        _name);
        Serial.printf("[%s]   Full travel time         : %lu ms\n",   _name, travelMs);
        Serial.printf("[%s]   Running ADC peak         : %d\n",       _name, runningMax);
        Serial.printf("[%s]   Stall ADC                : %d\n",       _name, stallAdc);
        Serial.printf("[%s]   Suggested stallThreshold: %d\n",
                      _name, (runningMax + stallAdc) / 2);
        Serial.printf("[%s]   Actuator at full extension -- caller will home()\n", _name);

        saveCalibration(travelMs);
        return travelMs;
      }
    } else {
      stallPending = false;
      if (adc > runningMax) runningMax = adc;
    }

    delay(10);
  }
}

// ---- Home -------------------------------------------------------------------

bool ActuatorController::home() {
  Serial.printf("[%s] homing -- retracting to stall (THRESHOLD=%d)...\n",
                _name, _stallThreshold);
  Serial.printf("[%s]   t(ms)  L_IS\n", _name);
  _state = ActuatorState::HOMING;
  _homed = false;
  _retract(ACTUATOR_FULL_SPEED);

  uint32_t startMs      = millis();
  uint32_t stallStartMs = 0;
  uint32_t lastPrintMs  = 0;
  bool     stallPending = false;

  while (true) {
    if (millis() - startMs > HOME_RETRACT_TIMEOUT_MS) {
      _stopMotor();
      _state = ActuatorState::ERROR;
      Serial.printf("[%s] homing TIMEOUT (%lums) -- L_IS never exceeded %d\n",
                    _name, millis() - startMs, _stallThreshold);
      Serial.printf("[%s]   Check: IS resistors to GND? IBT-2 12V present? EN pins connected?\n", _name);
      return false;
    }
    int adc = analogRead(_pin_l_is);

    if (millis() - lastPrintMs >= 250) {
      Serial.printf("[%s]   %5lu  %4d%s\n",
                    _name, millis() - startMs, adc,
                    (adc > _stallThreshold) ? "  <- STALL?" : "");
      lastPrintMs = millis();
    }

    if (adc > _stallThreshold) {
      if (!stallPending) {
        stallPending = true;
        stallStartMs = millis();
      } else if (millis() - stallStartMs >= STALL_CONFIRM_MS) {
        _stopMotor();
        _positionMs = 0;
        _homed      = true;
        _state      = ActuatorState::IDLE;
        Serial.printf("[%s] homed OK (stallADC=%d, t=%lums)\n",
                      _name, adc, millis() - startMs);
        return true;
      }
    } else {
      stallPending = false;
    }
    delay(10);
  }
}

// ---- Drive test ------------------------------------------------------------

int ActuatorController::driveTest(bool extend, uint16_t durationMs) {
  Serial.printf("[%s] DRIVE TEST: %s for %ums (threshold=%d)\n",
                _name, extend ? "EXTEND (R_IS=GPIO active)" : "RETRACT (L_IS=GPIO active)",
                durationMs, _stallThreshold);
  Serial.printf("[%s]   t(ms)   R_IS   L_IS   active\n", _name);

  if (extend) _extend(ACTUATOR_FULL_SPEED);
  else         _retract(ACTUATOR_FULL_SPEED);

  uint32_t startMs     = millis();
  uint32_t lastPrintMs = 0;
  int      peakAdc     = 0;

  while (millis() - startMs < durationMs) {
    int r      = analogRead(_pin_r_is);
    int l      = analogRead(_pin_l_is);
    int active = extend ? r : l;  // only the active half-bridge carries current
    if (active > peakAdc) peakAdc = active;

    if (millis() - lastPrintMs >= 100) {
      Serial.printf("[%s]   %5lu   %4d   %4d   %4d%s\n",
                    _name, millis() - startMs, r, l, active,
                    (active > _stallThreshold) ? "  <- STALL" : "");
      lastPrintMs = millis();
    }
    delay(10);
  }
  _stopMotor();
  Serial.printf("[%s] DRIVE TEST done -- peak active IS = %d  (threshold=%d)\n",
                _name, peakAdc, _stallThreshold);
  return peakAdc;
}

// ---- Movement ---------------------------------------------------------------

void ActuatorController::goToPosition(uint32_t targetMs) {
  if (!_homed) {
    Serial.printf("[%s] goToPosition called before homing -- ignored\n", _name);
    return;
  }
  targetMs = constrain(targetMs, 0, _fullTravelMs);
  _targetMs = targetMs;
  if (targetMs == _positionMs) return;

  _posAtMoveStart = _positionMs;
  _moveStartMs    = millis();
  _stallPending   = false;

  if (targetMs > _positionMs) {
    Serial.printf("[%s] extending: %lums -> %lums\n", _name, _positionMs, targetMs);
    _state = ActuatorState::EXTENDING;
    _extend(ACTUATOR_FULL_SPEED);
  } else {
    Serial.printf("[%s] retracting: %lums -> %lums\n", _name, _positionMs, targetMs);
    _state = ActuatorState::RETRACTING;
    _retract(ACTUATOR_FULL_SPEED);
  }
}

void ActuatorController::stop() {
  _stopMotor();
  if (_state == ActuatorState::EXTENDING || _state == ActuatorState::RETRACTING) {
    uint32_t elapsed = millis() - _moveStartMs;
    if (_state == ActuatorState::EXTENDING)
      _positionMs = min(_posAtMoveStart + elapsed, _fullTravelMs);
    else
      _positionMs = (_posAtMoveStart > elapsed) ? _posAtMoveStart - elapsed : 0;
  }
  _state = ActuatorState::IDLE;
  Serial.printf("[%s] stopped at ~%lums\n", _name, _positionMs);
}

void ActuatorController::update() {
  if (_state == ActuatorState::IDLE || _state == ActuatorState::ERROR) return;

  uint32_t elapsed = millis() - _moveStartMs;

  if (_state == ActuatorState::EXTENDING) {
    _positionMs = min(_posAtMoveStart + elapsed, _fullTravelMs);
    if (_checkStall()) {
      _stopMotor();
      _state = ActuatorState::STALLED;
      Serial.printf("[%s] STALL during extend at ~%lums\n", _name, _positionMs);
      return;
    }
    if (_positionMs >= _targetMs) {
      _stopMotor();
      _positionMs = _targetMs;
      _state      = ActuatorState::IDLE;
      Serial.printf("[%s] reached %lums\n", _name, _positionMs);
    }

  } else if (_state == ActuatorState::RETRACTING) {
    _positionMs = (_posAtMoveStart > elapsed) ? _posAtMoveStart - elapsed : 0;
    if (_checkStall()) {
      _stopMotor();
      if (_positionMs < 200) {
        _positionMs = 0;
        _state      = ActuatorState::IDLE;
        Serial.printf("[%s] stall near home -- position reset to 0\n", _name);
      } else {
        _state = ActuatorState::STALLED;
        Serial.printf("[%s] STALL during retract at ~%lums\n", _name, _positionMs);
      }
      return;
    }
    if (_positionMs <= _targetMs) {
      _stopMotor();
      _positionMs = _targetMs;
      _state      = ActuatorState::IDLE;
      Serial.printf("[%s] reached %lums\n", _name, _positionMs);
    }
  }
}

// ---- ADC / Stall ------------------------------------------------------------

int ActuatorController::readCurrentADC() const {
  if (_state == ActuatorState::EXTENDING || _state == ActuatorState::HOMING)
    return analogRead(_pin_r_is);
  return analogRead(_pin_l_is);
}

bool ActuatorController::_checkStall() {
  int adc = readCurrentADC();
  if (adc > _stallThreshold) {
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

// ---- Motor control ----------------------------------------------------------

void ActuatorController::_extend(uint8_t speed) {
  // Both EN pins must be HIGH to complete the H-bridge circuit.
  // Direction is controlled by which PWM channel is active.
  ledcWrite(_pin_lpwm, 0);
  ledcWrite(_pin_rpwm, speed);
  digitalWrite(_pin_r_en, HIGH);
  digitalWrite(_pin_l_en, HIGH);
}

void ActuatorController::_retract(uint8_t speed) {
  ledcWrite(_pin_rpwm, 0);
  ledcWrite(_pin_lpwm, speed);
  digitalWrite(_pin_r_en, HIGH);
  digitalWrite(_pin_l_en, HIGH);
}

void ActuatorController::_stopMotor() {
  // Zero PWM first, then disable — motor brakes via low-side FETs, then goes hi-Z
  ledcWrite(_pin_rpwm, 0);
  ledcWrite(_pin_lpwm, 0);
  digitalWrite(_pin_r_en, LOW);
  digitalWrite(_pin_l_en, LOW);
}
