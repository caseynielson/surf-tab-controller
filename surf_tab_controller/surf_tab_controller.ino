/*
 * surf_tab_controller.ino
 *
 * Surf tab controller for 2007 Malibu Wakesetter 247
 * Replaces Bennett Marine OBI9000-E Bolt
 *
 * Hardware:
 *   - ESP32-WROOM-32 dev board (motor controller)
 *   - 2× IBT-2 (BTS7960 43A H-bridge) motor drivers
 *   - 2× Lenco 15054-001 linear actuators (2-wire, no feedback)
 *   - CYD (ESP32-2432S028R) as UI display via UART2
 *
 * Position tracking: time-based, home = fully retracted (stall detected)
 * See config.h for preset timing values — calibrate on bench before install.
 */

#include "pins.h"
#include "config.h"
#include "ActuatorController.h"

// ── Actuators ─────────────────────────────────────────────────────────────────
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

// ── UI serial ─────────────────────────────────────────────────────────────────
HardwareSerial uiSerial(2);  // UART2

// ── Presets ───────────────────────────────────────────────────────────────────
typedef struct {
  const char* name;
  uint32_t    portMs;
  uint32_t    stbdMs;
} Preset;

const Preset PRESETS[] = {
  { "NEUTRAL",    PRESET_NEUTRAL_PORT,    PRESET_NEUTRAL_STBD    },
  { "SURF_LEFT",  PRESET_SURF_LEFT_PORT,  PRESET_SURF_LEFT_STBD  },
  { "SURF_RIGHT", PRESET_SURF_RIGHT_PORT, PRESET_SURF_RIGHT_STBD },
  { "FULL_DOWN",  PRESET_FULL_DOWN_PORT,  PRESET_FULL_DOWN_STBD  },
};
const int NUM_PRESETS = sizeof(PRESETS) / sizeof(PRESETS[0]);

// ── State ─────────────────────────────────────────────────────────────────────
int     currentPreset = -1;   // -1 = none/manual
bool    systemReady   = false;
uint32_t lastStatusMs = 0;

// ── Helpers ───────────────────────────────────────────────────────────────────
void goToPreset(int idx) {
  if (idx < 0 || idx >= NUM_PRESETS) return;
  Serial.printf("→ Preset: %s\n", PRESETS[idx].name);
  portTab.goToPosition(PRESETS[idx].portMs);
  stbdTab.goToPosition(PRESETS[idx].stbdMs);
  currentPreset = idx;
  // Notify UI
  uiSerial.printf("PRESET:%d:%s\n", idx, PRESETS[idx].name);
}

void broadcastStatus() {
  // Send tab positions to UI every 250ms while moving
  uiSerial.printf("STATUS:%lu:%lu:%d:%d\n",
    portTab.getPosition(),
    stbdTab.getPosition(),
    (int)portTab.getState(),
    (int)stbdTab.getState()
  );
}

// Handle serial commands from USB console and UI board
void handleCommand(const String& cmd) {
  String c = cmd;
  c.trim();
  c.toUpperCase();

  if      (c == "HOME")    { systemReady = false; /* re-home */ }
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
    Serial.printf("PORT: %lums  STBD: %lums  preset=%d  ready=%d\n",
      portTab.getPosition(), stbdTab.getPosition(), currentPreset, systemReady);
  }
  else if (c == "HELP") {
    Serial.println("Commands: HOME | N | L | R | D | STOP | STATUS | HELP");
  }
  else if (c.length() > 0) {
    Serial.printf("Unknown: %s\n", c.c_str());
  }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println("\n=== Surf Tab Controller ===");
  Serial.println("2007 Malibu Wakesetter 247");

  uiSerial.begin(UI_SERIAL_BAUD, SERIAL_8N1, UI_RX, UI_TX);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  portTab.begin();
  stbdTab.begin();

  Serial.println("Homing tabs...");
  bool portOK = portTab.home();
  bool stbdOK = stbdTab.home();

  if (portOK && stbdOK) {
    systemReady = true;
    digitalWrite(STATUS_LED, HIGH);
    Serial.println("System ready. Type HELP for commands.");
    uiSerial.println("READY");
    goToPreset(0);  // Start neutral
  } else {
    Serial.println("HOMING FAILED — check wiring and power. Type HOME to retry.");
    uiSerial.println("ERROR:HOMING_FAILED");
  }
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  // Re-home if requested
  if (!systemReady) {
    delay(100);
    bool portOK = portTab.home();
    bool stbdOK = stbdTab.home();
    if (portOK && stbdOK) {
      systemReady = true;
      digitalWrite(STATUS_LED, HIGH);
      Serial.println("Re-homed OK.");
      uiSerial.println("READY");
      goToPreset(0);
    }
    return;
  }

  // Update actuator state machines
  portTab.update();
  stbdTab.update();

  // Broadcast position to UI while moving
  if (portTab.isBusy() || stbdTab.isBusy()) {
    if (millis() - lastStatusMs > 250) {
      broadcastStatus();
      lastStatusMs = millis();
    }
  }

  // USB serial commands (for dev/debug)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }

  // UI serial commands from CYD
  if (uiSerial.available()) {
    String cmd = uiSerial.readStringUntil('\n');
    handleCommand(cmd);
  }
}
