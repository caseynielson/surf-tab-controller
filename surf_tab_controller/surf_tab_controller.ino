/*
 * surf_tab_controller.ino
 * Motor controller for 2007 Malibu Wakesetter 247 surf tab system.
 * Drives two Lenco 15054-001 actuators via IBT-2 (BTS7960) H-bridges.
 *
 * Preset positions are percentages of full travel so they stay valid after re-cal.
 *
 * Serial commands (USB or UI UART):
 *   CAL          - calibrate both actuators (bench only, tabs must be free to move)
 *   HOME         - re-home both tabs
 *   N / NEUTRAL  - both tabs up (0%)
 *   L / SURF_LEFT  - port 80%, stbd 20%
 *   R / SURF_RIGHT - port 20%, stbd 80%
 *   D / FULL_DOWN  - both 95%
 *   STOP         - immediate stop
 *   STATUS       - print positions, travel, and state
 *   HELP         - command list
 */

#include "ActuatorController.h"
#include "pins.h"
#include "config.h"

HardwareSerial uiSerial(2);

ActuatorController portTab(
  "PORT",
  PORT_RPWM, PORT_LPWM, PORT_R_EN, PORT_L_EN, PORT_R_IS, PORT_L_IS,
  PORT_RPWM_CH, PORT_LPWM_CH
);
ActuatorController stbdTab(
  "STBD",
  STBD_RPWM, STBD_LPWM, STBD_R_EN, STBD_L_EN, STBD_R_IS, STBD_L_IS,
  STBD_RPWM_CH, STBD_LPWM_CH
);

// Presets: percentages of full travel (0-100).
// goToPreset() multiplies by getFullTravel() at runtime, so values stay correct
// after re-calibration without any code change.
struct Preset {
  const char* name;
  uint8_t     portPct;
  uint8_t     stbdPct;
};
const Preset PRESETS[] = {
  { "NEUTRAL",     0,   0 },
  { "SURF_LEFT",  80,  20 },   // port heavy
  { "SURF_RIGHT", 20,  80 },   // stbd heavy
  { "FULL_DOWN",  95,  95 },   // 5% headroom from end-stop
};
const int NUM_PRESETS = sizeof(PRESETS) / sizeof(PRESETS[0]);

int      currentPreset = -1;
bool     systemReady   = false;
uint32_t lastStatusMs  = 0;

// ---- Helpers ----------------------------------------------------------------

void goToPreset(int idx) {
  if (idx < 0 || idx >= NUM_PRESETS) return;
  Serial.printf("-> Preset: %s\n", PRESETS[idx].name);
  portTab.goToPosition(portTab.getFullTravel() * PRESETS[idx].portPct / 100);
  stbdTab.goToPosition(stbdTab.getFullTravel() * PRESETS[idx].stbdPct / 100);
  currentPreset = idx;
  uiSerial.printf("PRESET:%d:%s\n", idx, PRESETS[idx].name);
}

void broadcastStatus() {
  uiSerial.printf("STATUS:%lu:%lu:%d:%d\n",
    portTab.getPosition(), stbdTab.getPosition(),
    (int)portTab.getState(), (int)stbdTab.getState());
}

void runCalibration() {
  Serial.println("\n=== CALIBRATION MODE ===");
  Serial.println("Both actuators must be free to travel fully (bench only).");
  systemReady = false;
  uiSerial.println("CAL:PORT");

  uint32_t portTravel = portTab.calibrate();
  if (portTravel == 0) {
    Serial.println("PORT calibration FAILED -- aborting.");
    uiSerial.println("ERROR:CAL_FAILED");
    return;
  }

  uiSerial.println("CAL:STBD");
  uint32_t stbdTravel = stbdTab.calibrate();
  if (stbdTravel == 0) {
    Serial.println("STBD calibration FAILED.");
    uiSerial.println("ERROR:CAL_FAILED");
    return;
  }

  Serial.println("\n=== CALIBRATION SUMMARY ===");
  Serial.printf("  PORT full travel: %lu ms\n", portTravel);
  Serial.printf("  STBD full travel: %lu ms\n", stbdTravel);
  Serial.println("  See STALL_THRESHOLD suggestions above; update config.h if needed.");
  Serial.println("  Homing both tabs...");

  bool ok = portTab.home() && stbdTab.home();
  if (ok) {
    systemReady = true;
    digitalWrite(STATUS_LED, HIGH);
    Serial.println("Calibration done. System ready.");
    uiSerial.println("READY");
    goToPreset(0);
  } else {
    Serial.println("Post-calibration home FAILED.");
    uiSerial.println("ERROR:HOMING_FAILED");
  }
}

void handleCommand(const String& raw) {
  String c = raw;
  c.trim();
  c.toUpperCase();

  if      (c == "CAL")                    runCalibration();
  else if (c == "HOME") {
    systemReady = false;
    Serial.println("Homing...");
    bool portOK = PORT_ENABLED ? portTab.home() : true;
    bool stbdOK = STBD_ENABLED ? stbdTab.home() : true;
    if (portOK && stbdOK) {
      systemReady = true;
      digitalWrite(STATUS_LED, HIGH);
      Serial.println("Homed OK. System ready.");
      uiSerial.println("READY");
      goToPreset(0);
    } else {
      Serial.println("Home FAILED. Type HOME to retry, CAL to calibrate.");
    }
  }
  else if (c == "N" || c == "NEUTRAL")    goToPreset(0);
  else if (c == "L" || c == "SURF_LEFT")  goToPreset(1);
  else if (c == "R" || c == "SURF_RIGHT") goToPreset(2);
  else if (c == "D" || c == "FULL_DOWN")  goToPreset(3);
  else if (c == "STOP") {
    portTab.stop();
    stbdTab.stop();
    currentPreset = -1;
  }
  else if (c == "STATUS") {
    uint32_t pFull = portTab.getFullTravel();
    uint32_t sFull = stbdTab.getFullTravel();
    Serial.printf("PORT: %4lums / %4lums  (%3.0f%%)  state=%d\n",
      portTab.getPosition(), pFull,
      pFull ? 100.0f * portTab.getPosition() / pFull : 0.0f,
      (int)portTab.getState());
    Serial.printf("STBD: %4lums / %4lums  (%3.0f%%)  state=%d\n",
      stbdTab.getPosition(), sFull,
      sFull ? 100.0f * stbdTab.getPosition() / sFull : 0.0f,
      (int)stbdTab.getState());
    Serial.printf("preset=%d  ready=%d  calibrated=%s\n",
      currentPreset, systemReady,
      (pFull != ACTUATOR_DEFAULT_TRAVEL_MS) ? "yes" : "no (run CAL)");
  }
  else if (c == "A" || c == "ADC") {
    // Snapshot read (motor stopped). Use DTP/DTS for live drive-test readings.
    Serial.printf("PORT: R_IS(34)=%4d  L_IS(35)=%4d\n",
      analogRead(PORT_R_IS), analogRead(PORT_L_IS));
    Serial.printf("STBD: R_IS(36)=%4d  L_IS(39)=%4d\n",
      analogRead(STBD_R_IS), analogRead(STBD_L_IS));
  }
  else if (c == "DTP") {
    // Drive PORT RETRACT for 3s; prints both IS channels live at 100ms.
    // Run this to confirm IS wiring before home/cal.
    portTab.driveTest(false, 3000);
  }
  else if (c == "DTS") {
    stbdTab.driveTest(false, 3000);
  }
  else if (c == "HELP") {
    Serial.println("Commands:");
    Serial.println("  CAL          -- measure full travel, save to NVS, suggest threshold");
    Serial.println("  HOME         -- re-home both tabs");
    Serial.println("  N/NEUTRAL    -- both tabs up");
    Serial.println("  L/SURF_LEFT  -- port 80%, stbd 20%");
    Serial.println("  R/SURF_RIGHT -- port 20%, stbd 80%");
    Serial.println("  D/FULL_DOWN  -- both 95%");
    Serial.println("  STOP         -- immediate stop");
    Serial.println("  STATUS       -- positions and calibration state");
    Serial.println("  ADC          -- snapshot IS ADC values (motor stopped)");
    Serial.println("  DTP          -- PORT retract 3s, print IS live (diagnose stall detect)");
    Serial.println("  DTS          -- STBD retract 3s, print IS live");
  }
  else if (c.length() > 0) {
    Serial.printf("Unknown: %s (type HELP)\n", c.c_str());
  }
}

// ---- Setup ------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(50);  // Don't block >50ms waiting for newline — prevents IDE freeze
  Serial.println("\n=== Surf Tab Controller ===");
  Serial.println("2007 Malibu Wakesetter 247");

  uiSerial.begin(UI_SERIAL_BAUD, SERIAL_8N1, UI_RX, UI_TX);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  portTab.begin();   // loads calibration from NVS
  stbdTab.begin();

  Serial.println("Homing tabs...");
  bool portOK = PORT_ENABLED ? portTab.home() : true;  // skip if not wired
  bool stbdOK = STBD_ENABLED ? stbdTab.home() : true;  // skip if not wired

  if (portOK && stbdOK) {
    systemReady = true;
    digitalWrite(STATUS_LED, HIGH);
    Serial.println("System ready. Type HELP for commands.");
    uiSerial.println("READY");
    goToPreset(0);
  } else {
    Serial.println("HOMING FAILED -- check wiring. Type HOME to retry, CAL to calibrate.");
    uiSerial.println("ERROR:HOMING_FAILED");
  }
}

// ---- Loop -------------------------------------------------------------------

void loop() {
  if (!systemReady) {
    // Do NOT auto-retry homing -- motors grinding against end-stops is harmful.
    // User must type HOME to retry or CAL to calibrate.
    return;
  }

  portTab.update();
  stbdTab.update();

  if (portTab.isBusy() || stbdTab.isBusy()) {
    if (millis() - lastStatusMs > 250) {
      broadcastStatus();
      lastStatusMs = millis();
    }
  }

  // USB serial commands (for dev/debug) — non-blocking accumulation
  // Only accept printable ASCII (32-126) to discard motor EMI garbage bytes.
  static String serialBuf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuf.length() > 0) { handleCommand(serialBuf); serialBuf = ""; }
    } else if (c >= 32 && c <= 126) {   // printable ASCII only
      if (serialBuf.length() < 64) serialBuf += c;  // cap at 64 chars
    }
    // non-printable bytes silently dropped (motor EMI, etc.)
  }

  // UI serial commands from CYD — non-blocking accumulation
  static String uiBuf;
  while (uiSerial.available()) {
    char c = uiSerial.read();
    if (c == '\n' || c == '\r') {
      if (uiBuf.length() > 0) { handleCommand(uiBuf); uiBuf = ""; }
    } else if (c >= 32 && c <= 126) {
      if (uiBuf.length() < 64) uiBuf += c;
    }
  }
}
