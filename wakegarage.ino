#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <EZButton.h>
#include <Preferences.h>
#include <math.h>

// ============================================================
// Configuration Layer
// ============================================================
namespace Config {

constexpr int TFT_DC  = 2;
constexpr int TFT_CS  = 5;
constexpr int TFT_RST = 4;

constexpr int pinL_EXT    = 32;
constexpr int pinL_RET    = 33;
constexpr int pinR_EXT    = 25;
constexpr int pinR_RET    = 26;
constexpr int pinBOTH_EXT = 34;
constexpr int pinBOTH_RET = 35;
constexpr int pinLAUNCH   = 16;
constexpr int pinSURF_L   = 17;
constexpr int pinSURF_R   = 27;

constexpr int ledL_EXT = 13;
constexpr int ledL_RET = 14;
constexpr int ledR_EXT = 21;
constexpr int ledR_RET = 22;

constexpr float fullStrokeTime             = 5.0f;
constexpr float actuatorRatePctPerSec      = 100.0f / fullStrokeTime;
constexpr float posDeadband                = 0.25f;
constexpr float targetSettleBand           = 1.5f;
constexpr float reverseInhibitBand         = 2.0f;
constexpr float launchCompleteOffMargin    = 3.0f;
constexpr float zeroSeatThresholdPct       = 1.0f;

constexpr unsigned long tripleTapMinWindowMs   = 150;
constexpr unsigned long tripleTapMaxWindowMs   = 2000;
constexpr unsigned long surfThresholdDwellMs   = 1000;
constexpr unsigned long launchThresholdDwellMs = 250;
constexpr unsigned long launchTimeoutMs        = 60000;
constexpr unsigned long endpointCalMs          = 500;
constexpr unsigned long tapPressMinMs          = 40;
constexpr unsigned long surfExitHighMs         = 3000;
constexpr unsigned long surfTargetSettleMs     = 400;
constexpr unsigned long faultClearStableMs     = 500;
constexpr unsigned long holdThresholdMs        = 450;
constexpr unsigned long holdStartRepeatDelayMs = 350;
constexpr unsigned long holdRepeatMs           = 125;

// New: release must stay stable this long before another press is accepted
constexpr unsigned long tapRearmReleaseMs      = 120;

constexpr unsigned long manualDirectionMaxRunMs =
  (unsigned long)(fullStrokeTime * 2000.0f);

constexpr float autoTargetMinStepPct = 1.0f;
constexpr float autoTargetHoldRatePctPerSec = actuatorRatePctPerSec * 1.10f;

constexpr float surfEnableSpeedOn      = 9.0f;
constexpr float surfEnableSpeedOff     = 8.5f;
constexpr float manualLaunchCompleteOn = 13.0f;
constexpr float surfLaunchCompleteOn   = 9.0f;
constexpr float surfExitHighSpeedMph   = 15.0f;

constexpr unsigned long endpointCooldownMs   = 8000;
constexpr unsigned long stuckInputFaultMs    = (unsigned long)(fullStrokeTime * 2500.0f);
constexpr unsigned long endpointExtraAllowMs = (unsigned long)(fullStrokeTime * 1000.0f);

constexpr float surfLPitchTarget = 10.0f;
constexpr float surfLRollTarget  = -2.0f;
constexpr float surfRPitchTarget = 10.5f;
constexpr float surfRRollTarget  = 5.0f;

constexpr int MODE_X   = 8;
constexpr int MODE_Y   = 8;
constexpr int MODE_W   = 102;
constexpr int MODE_H   = 28;

constexpr int SPEED_X  = 118;
constexpr int SPEED_Y  = 8;
constexpr int SPEED_W  = 114;
constexpr int SPEED_H  = 28;

constexpr int STATUS_X = 8;
constexpr int STATUS_Y = 52;
constexpr int STATUS_W = 224;
constexpr int STATUS_H = 108;

constexpr int LTAB_X   = 8;
constexpr int LTAB_Y   = 172;
constexpr int LTAB_W   = 104;
constexpr int LTAB_H   = 72;

constexpr int RTAB_X   = 128;
constexpr int RTAB_Y   = 172;
constexpr int RTAB_W   = 104;
constexpr int RTAB_H   = 72;

}  // namespace Config

// ============================================================
// Core Types
// ============================================================
enum BaseMode {
  MODE_MANUAL = 0,
  MODE_SURF_L = 1,
  MODE_SURF_R = 2
};

enum FaultCause {
  FAULT_NONE = 0,
  FAULT_STUCK_INPUT,
  FAULT_ILLEGAL_DRIVE,
  FAULT_RUNTIME_OVERRUN
};

enum OverlayKind : uint8_t {
  OVERLAY_NONE  = 0,
  OVERLAY_MSG   = 1,
  OVERLAY_FAULT = 2
};

enum StatusMsgPri : uint8_t {
  MSG_NONE = 0,
  MSG_INFO = 1,
  MSG_WARN = 2,
  MSG_CRIT = 3
};

enum ButtonIndex {
  BTN_L_EXT = 0,
  BTN_L_RET,
  BTN_R_EXT,
  BTN_R_RET,
  BTN_BOTH_EXT,
  BTN_BOTH_RET,
  BTN_LAUNCH,
  BTN_SURF_L,
  BTN_SURF_R,
  BTN_COUNT
};

struct DriveState {
  bool calActive = false;
  int calDir = 0;
  unsigned long calEndMs = 0;

  int runtimeDir = 0;
  unsigned long runMs = 0;
  bool timedOutLockout = false;

  unsigned long endpointPushMs = 0;
  bool endpointCooldown = false;
  unsigned long endpointCooldownEndMs = 0;
  int cooldownDir = 0;

  bool suppressSeatUntilRelease = false;
  int endpointOvertravelLatchedDir = 0;
};

struct StatusPanelCache {
  bool valid = false;
  int pitch10 = 0;
  int roll10 = 0;
  BaseMode mode = MODE_MANUAL;
  bool fault = false;
  uint8_t overlayKind = 0;
  uint32_t msgSig = 0;
  uint8_t displayState = 0;
};

struct AdjustState {
  bool extPrev = false;
  bool retPrev = false;

  unsigned long extPressStart = 0;
  unsigned long retPressStart = 0;

  bool extWasHold = false;
  bool retWasHold = false;

  unsigned long extLastRepeatMs = 0;
  unsigned long retLastRepeatMs = 0;

  // New: require a stable release before accepting another press
  unsigned long extReleaseMs = 0;
  unsigned long retReleaseMs = 0;
  bool extArmed = true;
  bool retArmed = true;
};

struct TapTracker {
  uint8_t count = 0;
  unsigned long firstTapMs = 0;
  bool pressed = false;
  unsigned long pressStartMs = 0;
};

struct UiMsg {
  StatusMsgPri pri = MSG_NONE;
  uint16_t color = ILI9341_WHITE;
  unsigned long untilMs = 0;
  char line1[22] = "";
  char line2[22] = "";
};

struct DisplayCache {
  BaseMode lastModeDisplay = (BaseMode)-1;
  bool lastLaunchBlinkState = false;
  bool lastSpeedInverted = false;
  int lastSpeedTenths = -9999;
  bool lastModeFaultDisplay = false;

  bool statusNormalStaticDrawn = false;
  int lastPitchMarkerX = -9999;
  int lastRollMarkerX = -9999;
  int lastPitchTargetX = -9999;
  int lastRollTargetX = -9999;
  int lastPitchValue10 = 99999;
  int lastRollValue10 = 99999;
  bool lastShowTargets = false;

  int lastLeftTextAct = -999;
  int lastRightTextAct = -999;
  int lastLeftTextTgt = -999;
  int lastRightTextTgt = -999;
  int lastLeftFillH = -1;
  int lastRightFillH = -1;
  int lastLeftLineY = -1;
  int lastRightLineY = -1;
  bool lastLeftShowLimit = false;
  bool lastRightShowLimit = false;
  bool lastLeftActive = false;
  bool lastRightActive = false;
};

struct AppState {
  BaseMode baseMode = MODE_MANUAL;
  FaultCause faultCause = FAULT_NONE;

  bool launchActive = false;
  bool postLaunchManualRetract = false;
  bool surfTransferActive = false;
  bool faultLatched = false;
  bool bootZeroingActive = false;
  bool manualEntryZeroing = false;
  bool wasSpeedRetracting = false;

  bool bootSeatL = false;
  bool bootSeatR = false;

  float actualPosL = 0.0f;
  float actualPosR = 0.0f;
  float surfLTarget = 75.0f;
  float surfRTarget = 75.0f;

  float boatSpeed = 0.0f;
  float simPitch  = 0.0f;
  float simRoll   = 0.0f;

  unsigned long lastUpdate = 0;
  unsigned long lastSaveMs = 0;
  unsigned long surfOverSpeedStartMs = 0;
  unsigned long surfAboveStartMs = 0;
  unsigned long surfBelowStartMs = 0;
  unsigned long launchAboveStartMs = 0;
  unsigned long launchStartMs = 0;
  unsigned long surfLTargetLastChangeMs = 0;
  unsigned long surfRTargetLastChangeMs = 0;
  unsigned long stuckInputStartMs = 0;
  unsigned long faultClearStartMs = 0;

  uint16_t lastInputMask = 0;
  bool prefsInitialized = false;
  bool surfTabsEnabled = false;

  DriveState driveL;
  DriveState driveR;
  AdjustState adjustL;
  AdjustState adjustR;
  TapTracker tapSurfL;
  TapTracker tapSurfR;
  TapTracker tapBothRet;
  StatusPanelCache statusCache;
  UiMsg uiMsg;
  DisplayCache display;
};

// ============================================================
// Global App State
// ============================================================
AppState app;

// ============================================================
// Hardware Layer
// ============================================================
namespace Hardware {

Adafruit_ILI9341 tft(Config::TFT_CS, Config::TFT_DC, Config::TFT_RST);
Preferences prefs;

bool buttonStates[BTN_COUNT] = {false};
void ReadButtons(bool *states, int num);
EZButton buttons(BTN_COUNT, ReadButtons, 600, 250, 40);

void ReadButtons(bool *states, int num) {
  if (num < BTN_COUNT) return;

  states[BTN_L_EXT]    = (digitalRead(Config::pinL_EXT)    == LOW);
  states[BTN_L_RET]    = (digitalRead(Config::pinL_RET)    == LOW);
  states[BTN_R_EXT]    = (digitalRead(Config::pinR_EXT)    == LOW);
  states[BTN_R_RET]    = (digitalRead(Config::pinR_RET)    == LOW);
  states[BTN_BOTH_EXT] = (digitalRead(Config::pinBOTH_EXT) == LOW);
  states[BTN_BOTH_RET] = (digitalRead(Config::pinBOTH_RET) == LOW);
  states[BTN_LAUNCH]   = (digitalRead(Config::pinLAUNCH)   == LOW);
  states[BTN_SURF_L]   = (digitalRead(Config::pinSURF_L)   == LOW);
  states[BTN_SURF_R]   = (digitalRead(Config::pinSURF_R)   == LOW);

  for (int i = 0; i < BTN_COUNT; i++) {
    buttonStates[i] = states[i];
  }
}

}  // namespace Hardware

// ============================================================
// Forward Declarations
// ============================================================
namespace UI {
uint32_t fnv1a32(const char* s);
void uiMsgSet(StatusMsgPri pri, const char* l1, const char* l2,
              unsigned long durMs, uint16_t color);
void uiMsgClear();
bool uiMsgActive();
uint32_t currentMsgSig();
OverlayKind currentOverlayKind();
}

namespace Persistence {
void maybeSaveState(bool force = false);
void loadState();
}

namespace Simulation {
void processSerialLine(String line);
}

namespace Control {
uint16_t getInputMask();
void resetTapTracker(TapTracker &tracker);
bool registerTap(TapTracker &tracker, unsigned long releaseTimeMs);
float manualMaxAllowedExtension(float speedMph);
void armZeroSeat(DriveState &st, float &pos);
void clearSurfAdjustState();
void enterFault(FaultCause cause);
void driveWithCommand(float &pos, int requestedDir, bool enforceRuntimeLimit,
                      DriveState &st, int ledExt, int ledRet, float dt);
void driveTowardTarget(float &pos, float target, DriveState &st,
                       int ledExt, int ledRet, float dt);
void beginManualZeroing();
void updateFaultState(float dt);
void updateSurfSpeedState();
void updateLaunchState();
void updateSurfOverspeedExit();
void updateBootZeroing(float dt);
void updateManualEntryZeroing(float dt);
void updateManualMode(float dt);
void updateSurfTargetAdjust(float dt);
void updateAutoMode(float dt);
void updateSurfTransferState();
void updateTapTrackers();
}

namespace Display {
void drawStaticLayout();
void drawModePanel(bool force = false);
void drawSpeedPanel(bool force = false);
void drawStatusPanelNormalStatic(bool showTargets, float tgtPitch, float tgtRoll);
void drawStatusPanelNormalDynamic(bool showTargets, float tgtPitch, float tgtRoll);
void drawStatusPanel(bool force = false);
void drawTabPanelLeft(bool force = false);
void drawTabPanelRight(bool force = false);
void drawDashboard();
}

// ============================================================
// Utility / UI State
// ============================================================
namespace UI {

uint32_t fnv1a32(const char* s) {
  uint32_t h = 2166136261u;
  for (const char* p = s; p && *p; ++p) {
    h ^= (uint8_t)(*p);
    h *= 16777619u;
  }
  return h;
}

void uiMsgSet(StatusMsgPri pri, const char* l1, const char* l2,
              unsigned long durMs, uint16_t color) {
  if (app.uiMsg.pri != MSG_NONE && pri < app.uiMsg.pri) {
    return;
  }

  app.uiMsg.pri = pri;
  app.uiMsg.color = color;
  app.uiMsg.untilMs = millis() + durMs;

  strncpy(app.uiMsg.line1, l1 ? l1 : "", sizeof(app.uiMsg.line1) - 1);
  app.uiMsg.line1[sizeof(app.uiMsg.line1) - 1] = '\0';

  strncpy(app.uiMsg.line2, l2 ? l2 : "", sizeof(app.uiMsg.line2) - 1);
  app.uiMsg.line2[sizeof(app.uiMsg.line2) - 1] = '\0';
}

void uiMsgClear() {
  app.uiMsg.pri = MSG_NONE;
  app.uiMsg.untilMs = 0;
  app.uiMsg.line1[0] = '\0';
  app.uiMsg.line2[0] = '\0';
}

bool uiMsgActive() {
  if (app.uiMsg.pri == MSG_NONE) {
    return false;
  }

  if ((long)(millis() - app.uiMsg.untilMs) > 0) {
    uiMsgClear();
    return false;
  }

  return true;
}

uint32_t currentMsgSig() {
  if (!uiMsgActive()) {
    return 0;
  }

  uint32_t h1 = fnv1a32(app.uiMsg.line1);
  uint32_t h2 = fnv1a32(app.uiMsg.line2);
  uint32_t h = 2166136261u;
  h ^= h1; h *= 16777619u;
  h ^= h2; h *= 16777619u;
  h ^= (uint32_t)app.uiMsg.color; h *= 16777619u;
  h ^= (uint32_t)app.uiMsg.pri;   h *= 16777619u;
  return h;
}

OverlayKind currentOverlayKind() {
  if (app.faultLatched) return OVERLAY_FAULT;
  if (uiMsgActive()) return OVERLAY_MSG;
  return OVERLAY_NONE;
}

}  // namespace UI

// ============================================================
// Persistence
// ============================================================
namespace Persistence {

void maybeSaveState(bool force) {
  if (!app.prefsInitialized) return;

  const int modeNow  = (int)app.baseMode;
  const int posLT    = (int)roundf(app.actualPosL * 10.0f);
  const int posRT    = (int)roundf(app.actualPosR * 10.0f);
  const int surfLT   = (int)roundf(app.surfLTarget * 10.0f);
  const int surfRT   = (int)roundf(app.surfRTarget * 10.0f);
  const int faultNow = app.faultLatched ? 1 : 0;

  static int lastSavedMode   = -999;
  static int lastSavedPosLT  = -99999;
  static int lastSavedPosRT  = -99999;
  static int lastSavedSurfLT = -99999;
  static int lastSavedSurfRT = -99999;
  static int lastSavedFault  = -1;

  const bool changed = force ||
                       modeNow  != lastSavedMode ||
                       posLT    != lastSavedPosLT ||
                       posRT    != lastSavedPosRT ||
                       surfLT   != lastSavedSurfLT ||
                       surfRT   != lastSavedSurfRT ||
                       faultNow != lastSavedFault;

  if (!changed) return;
  if (!force && millis() - app.lastSaveMs < 1000) return;

  Hardware::prefs.putInt("baseMode", modeNow);
  Hardware::prefs.putFloat("posL", app.actualPosL);
  Hardware::prefs.putFloat("posR", app.actualPosR);
  Hardware::prefs.putFloat("surfLT", app.surfLTarget);
  Hardware::prefs.putFloat("surfRT", app.surfRTarget);
  Hardware::prefs.putBool("fault", app.faultLatched);

  lastSavedMode   = modeNow;
  lastSavedPosLT  = posLT;
  lastSavedPosRT  = posRT;
  lastSavedSurfLT = surfLT;
  lastSavedSurfRT = surfRT;
  lastSavedFault  = faultNow;
  app.lastSaveMs  = millis();
}

void loadState() {
  bool init = Hardware::prefs.getBool("init", false);

  app.bootSeatL = false;
  app.bootSeatR = false;
  app.bootZeroingActive = false;

  if (!init) {
    app.baseMode = MODE_MANUAL;
    app.launchActive = false;
    app.postLaunchManualRetract = false;
    app.faultLatched = false;

    app.actualPosL = 0.0f;
    app.actualPosR = 0.0f;
    app.surfLTarget = 75.0f;
    app.surfRTarget = 75.0f;

    Hardware::prefs.putBool("init", true);
    maybeSaveState(true);

    if (app.actualPosL <= Config::zeroSeatThresholdPct) app.bootSeatL = true;
    if (app.actualPosR <= Config::zeroSeatThresholdPct) app.bootSeatR = true;
    return;
  }

  int savedMode = Hardware::prefs.getInt("baseMode", (int)MODE_MANUAL);
  if (savedMode < 0 || savedMode > 2) {
    savedMode = (int)MODE_MANUAL;
  }

  app.baseMode = (BaseMode)savedMode;
  app.launchActive = false;
  app.postLaunchManualRetract = false;
  app.faultLatched = Hardware::prefs.getBool("fault", false);

  app.actualPosL = constrain(Hardware::prefs.getFloat("posL", 0.0f), 0.0f, 100.0f);
  app.actualPosR = constrain(Hardware::prefs.getFloat("posR", 0.0f), 0.0f, 100.0f);
  app.surfLTarget = constrain(Hardware::prefs.getFloat("surfLT", 75.0f), 0.0f, 100.0f);
  app.surfRTarget = constrain(Hardware::prefs.getFloat("surfRT", 75.0f), 0.0f, 100.0f);

  app.surfTabsEnabled = false;
  app.surfAboveStartMs = 0;
  app.surfBelowStartMs = 0;
  app.surfOverSpeedStartMs = 0;
  app.launchAboveStartMs = 0;

  if (app.actualPosL <= Config::zeroSeatThresholdPct) app.bootSeatL = true;
  if (app.actualPosR <= Config::zeroSeatThresholdPct) app.bootSeatR = true;
}

}  // namespace Persistence

// ============================================================
// Simulation Input
// ============================================================
namespace Simulation {

void processSerialLine(String line) {
  String lower = line;
  lower.toLowerCase();

  if (lower == "help") {
    Serial.println("Commands:");
    Serial.println("  11.2      -> speed 11.2 mph");
    Serial.println("  s 11.2    -> speed 11.2 mph");
    Serial.println("  p 8.5     -> pitch 8.5 deg");
    Serial.println("  r -1.2    -> roll -1.2 deg");
    return;
  }

  bool looksNumeric = true;
  for (unsigned int i = 0; i < line.length(); i++) {
    const char c = line[i];
    if (!(isDigit(c) || c == '-' || c == '+' || c == '.' || c == ' ')) {
      looksNumeric = false;
      break;
    }
  }

  if (looksNumeric && lower.indexOf(' ') == -1) {
    app.boatSpeed = line.toFloat();
    Serial.print("SPEED UPDATED: ");
    Serial.println(app.boatSpeed, 1);
    return;
  }

  if (lower.startsWith("s ")) {
    app.boatSpeed = line.substring(2).toFloat();
    Serial.print("SPEED UPDATED: ");
    Serial.println(app.boatSpeed, 1);
    return;
  }

  if (lower.startsWith("p ")) {
    app.simPitch = line.substring(2).toFloat();
    Serial.print("PITCH UPDATED: ");
    Serial.println(app.simPitch, 1);
    return;
  }

  if (lower.startsWith("r ")) {
    app.simRoll = line.substring(2).toFloat();
    Serial.print("ROLL UPDATED: ");
    Serial.println(app.simRoll, 1);
    return;
  }

  Serial.print("Unrecognized command: ");
  Serial.println(line);
}

}  // namespace Simulation

// ============================================================
// Control Layer
// ============================================================
namespace Control {

uint16_t getInputMask() {
  uint16_t mask = 0;
  if (Hardware::buttonStates[BTN_L_EXT])    mask |= (1 << 0);
  if (Hardware::buttonStates[BTN_L_RET])    mask |= (1 << 1);
  if (Hardware::buttonStates[BTN_R_EXT])    mask |= (1 << 2);
  if (Hardware::buttonStates[BTN_R_RET])    mask |= (1 << 3);
  if (Hardware::buttonStates[BTN_BOTH_EXT]) mask |= (1 << 4);
  if (Hardware::buttonStates[BTN_BOTH_RET]) mask |= (1 << 5);
  if (Hardware::buttonStates[BTN_LAUNCH])   mask |= (1 << 6);
  if (Hardware::buttonStates[BTN_SURF_L])   mask |= (1 << 7);
  if (Hardware::buttonStates[BTN_SURF_R])   mask |= (1 << 8);
  return mask;
}

void resetTapTracker(TapTracker &tracker) {
  tracker.count = 0;
  tracker.firstTapMs = 0;
  tracker.pressed = false;
  tracker.pressStartMs = 0;
}

bool registerTap(TapTracker &tracker, unsigned long releaseTimeMs) {
  if (tracker.count == 0) {
    tracker.count = 1;
    tracker.firstTapMs = releaseTimeMs;
    return false;
  }

  if (releaseTimeMs - tracker.firstTapMs > Config::tripleTapMaxWindowMs) {
    tracker.count = 1;
    tracker.firstTapMs = releaseTimeMs;
    return false;
  }

  tracker.count++;

  if (tracker.count >= 3) {
    const unsigned long elapsed = releaseTimeMs - tracker.firstTapMs;
    tracker.count = 0;
    tracker.firstTapMs = 0;
    return (elapsed >= Config::tripleTapMinWindowMs && elapsed <= Config::tripleTapMaxWindowMs);
  }

  return false;
}

float manualMaxAllowedExtension(float speedMph) {
  if (speedMph <= 15.0f) return 100.0f;

  if (speedMph < 20.0f) {
    const float t = (speedMph - 15.0f) / 5.0f;
    return 100.0f + t * (20.0f - 100.0f);
  }

  if (speedMph < 25.0f) {
    const float t = (speedMph - 20.0f) / 5.0f;
    return 20.0f + t * (0.0f - 20.0f);
  }

  return 0.0f;
}

void armZeroSeat(DriveState &st, float &pos) {
  pos = 0.0f;
  st.calActive = true;
  st.calDir = -1;
  st.calEndMs = millis() + Config::endpointCalMs;
}

void clearSurfAdjustState() {
  app.adjustL = AdjustState{};
  app.adjustR = AdjustState{};
  app.surfLTargetLastChangeMs = 0;
  app.surfRTargetLastChangeMs = 0;
}

void enterFault(FaultCause cause) {
  app.faultLatched = true;
  app.faultCause = cause;

  app.launchActive = false;
  app.postLaunchManualRetract = false;
  app.manualEntryZeroing = false;
  app.bootZeroingActive = false;
  app.surfTransferActive = false;

  resetTapTracker(app.tapSurfL);
  resetTapTracker(app.tapSurfR);
  resetTapTracker(app.tapBothRet);
  clearSurfAdjustState();

  app.driveL.calActive = false;
  app.driveR.calActive = false;
  app.driveL.runtimeDir = 0;
  app.driveR.runtimeDir = 0;
  app.driveL.runMs = 0;
  app.driveR.runMs = 0;
  app.driveL.timedOutLockout = false;
  app.driveR.timedOutLockout = false;
  app.driveL.endpointOvertravelLatchedDir = 0;
  app.driveR.endpointOvertravelLatchedDir = 0;

  digitalWrite(Config::ledL_EXT, LOW);
  digitalWrite(Config::ledL_RET, LOW);
  digitalWrite(Config::ledR_EXT, LOW);
  digitalWrite(Config::ledR_RET, LOW);

  Persistence::maybeSaveState(true);
}

void driveWithCommand(float &pos, int requestedDir, bool enforceRuntimeLimit,
                      DriveState &st, int ledExt, int ledRet, float dt) {
  const unsigned long now = millis();
  const unsigned long dtMs = (unsigned long)(dt * 1000.0f);

  if (st.calActive && now >= st.calEndMs) {
    st.calActive = false;
  }

  if (st.suppressSeatUntilRelease) {
    if (requestedDir == 0) {
      st.calActive = false;
      st.endpointPushMs = 0;
      st.runtimeDir = 0;
      st.runMs = 0;
      st.timedOutLockout = false;

      if (!st.endpointCooldown || now >= st.endpointCooldownEndMs) {
        st.endpointCooldown = false;
        st.cooldownDir = 0;
      }

      st.suppressSeatUntilRelease = false;
    }

    digitalWrite(ledExt, LOW);
    digitalWrite(ledRet, LOW);
    return;
  }

  if (requestedDir == 0) {
    st.timedOutLockout = false;
    st.runtimeDir = 0;
    st.runMs = 0;
  }

  if (st.endpointCooldown) {
    if (now >= st.endpointCooldownEndMs && requestedDir == 0) {
      st.endpointCooldown = false;
      st.endpointPushMs = 0;
      st.cooldownDir = 0;
    } else if (requestedDir == st.cooldownDir) {
      digitalWrite(ledExt, LOW);
      digitalWrite(ledRet, LOW);
      return;
    }
  }

  if (st.timedOutLockout && requestedDir != 0) {
    st.calActive = false;
    st.suppressSeatUntilRelease = true;
    digitalWrite(ledExt, LOW);
    digitalWrite(ledRet, LOW);
    return;
  }

  if (st.calActive && requestedDir != 0 && requestedDir == -st.calDir) {
    st.calActive = false;
  }

  int finalDir = requestedDir;
  if (finalDir == 0 && st.calActive) {
    finalDir = st.calDir;
  }

  const bool atMin = (pos <= Config::posDeadband);
  const bool atMax = (pos >= (100.0f - Config::posDeadband));
  const bool pushingIntoEndpoint = (finalDir < 0 && atMin) || (finalDir > 0 && atMax);

  if (pushingIntoEndpoint) {
    if (st.endpointOvertravelLatchedDir == finalDir) {
      st.calActive = false;
      st.suppressSeatUntilRelease = true;
      UI::uiMsgSet(MSG_WARN, "END STOP USED", "MOVE OPPOSITE", 1200, ILI9341_YELLOW);
      digitalWrite(ledExt, LOW);
      digitalWrite(ledRet, LOW);
      return;
    }

    st.runMs = 0;
    st.endpointPushMs += dtMs;

    if (st.endpointPushMs >= Config::endpointExtraAllowMs) {
      st.endpointCooldown = true;
      st.endpointCooldownEndMs = now + Config::endpointCooldownMs;
      st.cooldownDir = finalDir;
      st.endpointOvertravelLatchedDir = finalDir;
      st.calActive = false;
      st.suppressSeatUntilRelease = true;
      UI::uiMsgSet(MSG_WARN, "COOLDOWN", "RELEASE BUTTON", 1200, ILI9341_YELLOW);
      digitalWrite(ledExt, LOW);
      digitalWrite(ledRet, LOW);
      return;
    }
  } else {
    st.endpointPushMs = 0;
  }

  if (enforceRuntimeLimit) {
    if (requestedDir != st.runtimeDir) {
      st.runtimeDir = requestedDir;
      st.runMs = 0;
    }

    if (requestedDir != 0) {
      if (!pushingIntoEndpoint) {
        st.runMs += dtMs;
      }

      if (st.runMs >= Config::manualDirectionMaxRunMs) {
        st.timedOutLockout = true;
        st.calActive = false;
        st.suppressSeatUntilRelease = true;
        digitalWrite(ledExt, LOW);
        digitalWrite(ledRet, LOW);
        return;
      }
    }
  } else {
    st.runtimeDir = 0;
    st.runMs = 0;
    st.timedOutLockout = false;
  }

  if (finalDir == 0) {
    digitalWrite(ledExt, LOW);
    digitalWrite(ledRet, LOW);
    return;
  }

  const float posBefore = pos;

  if (finalDir > 0) {
    pos += Config::actuatorRatePctPerSec * dt;
  } else {
    pos -= Config::actuatorRatePctPerSec * dt;
  }

  if (finalDir > 0 && pos > posBefore + 0.001f && st.endpointOvertravelLatchedDir < 0) {
    st.endpointOvertravelLatchedDir = 0;
  }
  if (finalDir < 0 && pos < posBefore - 0.001f && st.endpointOvertravelLatchedDir > 0) {
    st.endpointOvertravelLatchedDir = 0;
  }

  bool hitEndpoint = false;
  if (pos >= 100.0f) { pos = 100.0f; hitEndpoint = true; }
  if (pos <= 0.0f)   { pos = 0.0f;   hitEndpoint = true; }

  if (hitEndpoint && !st.calActive && !st.timedOutLockout && !st.suppressSeatUntilRelease) {
    st.calActive = true;
    st.calDir = finalDir;
    st.calEndMs = now + Config::endpointCalMs;
  }

  digitalWrite(ledExt, finalDir > 0 ? HIGH : LOW);
  digitalWrite(ledRet, finalDir < 0 ? HIGH : LOW);
}

void driveTowardTarget(float &pos, float target, DriveState &st,
                       int ledExt, int ledRet, float dt) {
  static int lastDirL = 0;
  static int lastDirR = 0;

  int *lastDir = (&st == &app.driveL) ? &lastDirL : &lastDirR;
  int dir = 0;

  // Endpoint-aware target handling:
  // Near 0% and 100%, do not use the normal settle band because it can stop
  // short of the hard endpoint. Instead, drive until the endpoint is actually
  // reached, then allow normal seat behavior through driveWithCommand().
  if (target <= Config::zeroSeatThresholdPct) {
    dir = (pos > Config::posDeadband) ? -1 : 0;
  } else if (target >= (100.0f - Config::zeroSeatThresholdPct)) {
    dir = (pos < (100.0f - Config::posDeadband)) ? +1 : 0;
  } else {
    const float error = target - pos;

    if (fabs(error) <= Config::targetSettleBand) {
      dir = 0;
    } else {
      dir = (error > 0.0f) ? +1 : -1;

      if (*lastDir != 0 &&
          dir != *lastDir &&
          fabs(error) < Config::reverseInhibitBand) {
        dir = 0;
      }
    }
  }

  driveWithCommand(pos, dir, false, st, ledExt, ledRet, dt);

  if (dir != 0) {
    *lastDir = dir;
  }
}

void beginManualZeroing() {
  app.baseMode = MODE_MANUAL;
  app.launchActive = false;
  app.postLaunchManualRetract = false;
  app.manualEntryZeroing = true;

  resetTapTracker(app.tapSurfL);
  resetTapTracker(app.tapSurfR);
  resetTapTracker(app.tapBothRet);
  clearSurfAdjustState();

  app.driveL.timedOutLockout = false;
  app.driveL.runtimeDir = 0;
  app.driveL.runMs = 0;
  app.driveL.endpointOvertravelLatchedDir = 0;

  app.driveR.timedOutLockout = false;
  app.driveR.runtimeDir = 0;
  app.driveR.runMs = 0;
  app.driveR.endpointOvertravelLatchedDir = 0;

  if (app.actualPosL <= Config::zeroSeatThresholdPct) {
    armZeroSeat(app.driveL, app.actualPosL);
  }
  if (app.actualPosR <= Config::zeroSeatThresholdPct) {
    armZeroSeat(app.driveR, app.actualPosR);
  }
}

void updateFaultState(float dt) {
  (void)dt;

  const uint16_t inputMask = getInputMask();
  const unsigned long now = millis();

  if (inputMask != 0 && inputMask == app.lastInputMask) {
    if (app.stuckInputStartMs == 0) {
      app.stuckInputStartMs = now;
    } else if (!app.faultLatched && now - app.stuckInputStartMs >= Config::stuckInputFaultMs) {
      enterFault(FAULT_STUCK_INPUT);
      UI::uiMsgSet(MSG_CRIT, "INPUT FAULT", "RELEASE ALL", 1500, ILI9341_RED);
    }
  } else {
    app.stuckInputStartMs = (inputMask != 0) ? now : 0;
    app.lastInputMask = inputMask;
  }

  if (app.faultLatched) {
    if (inputMask == 0) {
      if (app.faultClearStartMs == 0) {
        app.faultClearStartMs = now;
      } else if (now - app.faultClearStartMs >= Config::faultClearStableMs) {
        app.faultLatched = false;
        app.faultCause = FAULT_NONE;
        app.faultClearStartMs = 0;
        app.stuckInputStartMs = 0;
        app.lastInputMask = 0;
        Persistence::maybeSaveState(true);
      }
    } else {
      app.faultClearStartMs = 0;
    }
  } else {
    app.faultClearStartMs = 0;
  }
}

void updateSurfSpeedState() {
  if (app.baseMode == MODE_MANUAL) {
    app.surfTabsEnabled = false;
    app.surfAboveStartMs = 0;
    app.surfBelowStartMs = 0;
    return;
  }

  const unsigned long now = millis();

  if (!app.surfTabsEnabled) {
    if (app.boatSpeed >= Config::surfEnableSpeedOn) {
      if (app.surfAboveStartMs == 0) app.surfAboveStartMs = now;
      if (now - app.surfAboveStartMs >= Config::surfThresholdDwellMs) {
        app.surfTabsEnabled = true;
      }
    } else {
      app.surfAboveStartMs = 0;
    }
  } else {
    if (app.boatSpeed <= Config::surfEnableSpeedOff) {
      if (app.surfBelowStartMs == 0) app.surfBelowStartMs = now;
      if (now - app.surfBelowStartMs >= Config::surfThresholdDwellMs) {
        app.surfTabsEnabled = false;
      }
    } else {
      app.surfBelowStartMs = 0;
    }
  }
}

void updateLaunchState() {
  if (!app.launchActive) return;

  const unsigned long now = millis();

  if (now - app.launchStartMs >= Config::launchTimeoutMs) {
    app.launchActive = false;
    app.launchAboveStartMs = 0;

    if (app.baseMode == MODE_MANUAL) {
      app.postLaunchManualRetract = true;
      Serial.println("LAUNCH TIMEOUT -> MANUAL RETRACT");
    } else {
      Serial.println("LAUNCH TIMEOUT -> RESUME SURF RULES");
    }
    return;
  }

  const float thresholdOn  = (app.baseMode == MODE_MANUAL) ? Config::manualLaunchCompleteOn : Config::surfLaunchCompleteOn;
  const float thresholdOff = thresholdOn - Config::launchCompleteOffMargin;

  if (app.boatSpeed >= thresholdOn) {
    if (app.launchAboveStartMs == 0) {
      app.launchAboveStartMs = now;
    }

    if (now - app.launchAboveStartMs >= Config::launchThresholdDwellMs) {
      app.launchActive = false;
      app.launchAboveStartMs = 0;

      if (app.baseMode == MODE_MANUAL) {
        app.postLaunchManualRetract = true;
        Serial.println("LAUNCH COMPLETE -> MANUAL RETRACT");
      } else {
        Serial.println("LAUNCH COMPLETE -> RESUME SURF");
      }
    }
  } else if (app.boatSpeed < thresholdOff) {
    app.launchAboveStartMs = 0;
  }
}

void updateSurfOverspeedExit() {
  if (app.baseMode == MODE_MANUAL) {
    app.surfOverSpeedStartMs = 0;
    return;
  }

  const unsigned long now = millis();

  if (app.boatSpeed > Config::surfExitHighSpeedMph) {
    if (app.surfOverSpeedStartMs == 0) {
      app.surfOverSpeedStartMs = now;
    } else if (now - app.surfOverSpeedStartMs >= Config::surfExitHighMs) {
      app.surfTabsEnabled = false;
      app.surfAboveStartMs = 0;
      app.surfBelowStartMs = 0;
      app.surfOverSpeedStartMs = 0;

      UI::uiMsgSet(MSG_WARN, "OVERSPEED", "RETURN MANUAL", 1200, ILI9341_ORANGE);
      beginManualZeroing();

      Serial.println("SURF EXIT: overspeed -> MANUAL ZEROING");
    }
  } else {
    app.surfOverSpeedStartMs = 0;
  }
}

void updateBootZeroing(float dt) {
  driveWithCommand(app.actualPosL, 0, false, app.driveL, Config::ledL_EXT, Config::ledL_RET, dt);
  driveWithCommand(app.actualPosR, 0, false, app.driveR, Config::ledR_EXT, Config::ledR_RET, dt);

  if (!app.driveL.calActive && !app.driveR.calActive) {
    app.bootZeroingActive = false;
  }
}

void updateManualEntryZeroing(float dt) {
  driveTowardTarget(app.actualPosL, 0.0f, app.driveL, Config::ledL_EXT, Config::ledL_RET, dt);
  driveTowardTarget(app.actualPosR, 0.0f, app.driveR, Config::ledR_EXT, Config::ledR_RET, dt);

  const bool leftDone  = (app.actualPosL <= Config::posDeadband && !app.driveL.calActive);
  const bool rightDone = (app.actualPosR <= Config::posDeadband && !app.driveR.calActive);

  if (leftDone && rightDone) {
    app.manualEntryZeroing = false;
    app.driveL.timedOutLockout = false;
    app.driveL.runtimeDir = 0;
    app.driveL.runMs = 0;
    app.driveR.timedOutLockout = false;
    app.driveR.runtimeDir = 0;
    app.driveR.runMs = 0;
    clearSurfAdjustState();
  }
}

void updateManualMode(float dt) {
  const float maxAllowed = manualMaxAllowedExtension(app.boatSpeed);

  bool lExt = Hardware::buttonStates[BTN_L_EXT];
  bool lRet = Hardware::buttonStates[BTN_L_RET];
  bool rExt = Hardware::buttonStates[BTN_R_EXT];
  bool rRet = Hardware::buttonStates[BTN_R_RET];

  if (Hardware::buttonStates[BTN_BOTH_EXT] && !Hardware::buttonStates[BTN_BOTH_RET]) {
    lExt = true; lRet = false; rExt = true; rRet = false;
  } else if (Hardware::buttonStates[BTN_BOTH_RET] && !Hardware::buttonStates[BTN_BOTH_EXT]) {
    lRet = true; lExt = false; rRet = true; rExt = false;
  }

  int reqL = 0;
  int reqR = 0;
  const bool hardZeroLimit = (maxAllowed <= Config::zeroSeatThresholdPct);

  if (hardZeroLimit) {
    reqL = (app.actualPosL > Config::posDeadband || app.driveL.calActive) ? -1 : 0;
  } else if (app.actualPosL > maxAllowed + Config::targetSettleBand) {
    reqL = -1;
  } else {
    if (lExt && !lRet) {
      reqL = (maxAllowed >= 100.0f - Config::posDeadband) ? +1 : ((app.actualPosL < maxAllowed) ? +1 : 0);
    } else if (lRet && !lExt) {
      reqL = -1;
    }
  }

  if (hardZeroLimit) {
    reqR = (app.actualPosR > Config::posDeadband || app.driveR.calActive) ? -1 : 0;
  } else if (app.actualPosR > maxAllowed + Config::targetSettleBand) {
    reqR = -1;
  } else {
    if (rExt && !rRet) {
      reqR = (maxAllowed >= 100.0f - Config::posDeadband) ? +1 : ((app.actualPosR < maxAllowed) ? +1 : 0);
    } else if (rRet && !rExt) {
      reqR = -1;
    }
  }

  const bool speedRetractingNow =
    (hardZeroLimit && (app.actualPosL > Config::posDeadband || app.actualPosR > Config::posDeadband)) ||
    ((reqL == -1 && app.actualPosL > maxAllowed + Config::targetSettleBand) ||
     (reqR == -1 && app.actualPosR > maxAllowed + Config::targetSettleBand));

  if (speedRetractingNow && !app.wasSpeedRetracting) {
    UI::uiMsgSet(MSG_WARN, "SPEED LIMIT", "RETRACTING TABS", 1200, ILI9341_ORANGE);
  }
  app.wasSpeedRetracting = speedRetractingNow;

  const bool userDrivenL = ((lExt && !lRet) || (lRet && !lExt));
  const bool userDrivenR = ((rExt && !rRet) || (rRet && !rExt));

  driveWithCommand(app.actualPosL, reqL, userDrivenL, app.driveL, Config::ledL_EXT, Config::ledL_RET, dt);
  driveWithCommand(app.actualPosR, reqR, userDrivenR, app.driveR, Config::ledR_EXT, Config::ledR_RET, dt);
}

void updateSurfTargetAdjust(float dt) {
  if (app.launchActive) return;
  if (app.baseMode == MODE_MANUAL) return;
  if (app.surfTransferActive) return;

  AdjustState *adj = (app.baseMode == MODE_SURF_L) ? &app.adjustL : &app.adjustR;

  const bool extPressed =
    (app.baseMode == MODE_SURF_L) ? Hardware::buttonStates[BTN_L_EXT]
                                  : Hardware::buttonStates[BTN_R_EXT];

  const bool retPressed =
    (app.baseMode == MODE_SURF_L) ? Hardware::buttonStates[BTN_L_RET]
                                  : Hardware::buttonStates[BTN_R_RET];

  float *target =
    (app.baseMode == MODE_SURF_L) ? &app.surfLTarget : &app.surfRTarget;

  float *actual =
    (app.baseMode == MODE_SURF_L) ? &app.actualPosR : &app.actualPosL;

  DriveState *drive =
    (app.baseMode == MODE_SURF_L) ? &app.driveR : &app.driveL;

  const int ledExt =
    (app.baseMode == MODE_SURF_L) ? Config::ledR_EXT : Config::ledL_EXT;

  const int ledRet =
    (app.baseMode == MODE_SURF_L) ? Config::ledR_RET : Config::ledL_RET;

  unsigned long *lastChangeMs =
    (app.baseMode == MODE_SURF_L) ? &app.surfLTargetLastChangeMs
                                  : &app.surfRTargetLastChangeMs;

  const unsigned long now = millis();

  // If both are pressed, cancel adjustment state and require re-release
  if (extPressed && retPressed) {
    adj->extPressStart = 0;
    adj->retPressStart = 0;
    adj->extWasHold = false;
    adj->retWasHold = false;
    adj->extLastRepeatMs = 0;
    adj->retLastRepeatMs = 0;
    adj->extPrev = extPressed;
    adj->retPrev = retPressed;
    return;
  }

  // Rearm only after stable release
  if (!extPressed) {
    if (adj->extPrev) {
      adj->extReleaseMs = now;
    } else if (!adj->extArmed && adj->extReleaseMs != 0 &&
               (now - adj->extReleaseMs >= Config::tapRearmReleaseMs)) {
      adj->extArmed = true;
    }
  }

  if (!retPressed) {
    if (adj->retPrev) {
      adj->retReleaseMs = now;
    } else if (!adj->retArmed && adj->retReleaseMs != 0 &&
               (now - adj->retReleaseMs >= Config::tapRearmReleaseMs)) {
      adj->retArmed = true;
    }
  }

  // =========================
  // EXTEND
  // =========================
  if (extPressed && !adj->extPrev && adj->extArmed) {
    adj->extArmed = false;
    adj->extPressStart = now;
    adj->extWasHold = false;
    adj->extLastRepeatMs = 0;
    adj->extReleaseMs = 0;

    // Immediate single tap step
    *target = min(100.0f, *target + Config::autoTargetMinStepPct);
    *lastChangeMs = now;
  }

  if (extPressed && !retPressed) {
    if (!adj->extWasHold) {
      if (adj->extPressStart != 0 &&
          (now - adj->extPressStart >= Config::holdThresholdMs)) {
        adj->extWasHold = true;
        adj->extLastRepeatMs = now + Config::holdStartRepeatDelayMs;
      }
    } else {
      if ((long)(now - adj->extLastRepeatMs) >= 0) {
        const float step =
          Config::autoTargetHoldRatePctPerSec * (Config::holdRepeatMs / 1000.0f);

        *target = min(100.0f, *target + step);
        *lastChangeMs = now;
        adj->extLastRepeatMs = now + Config::holdRepeatMs;

        if (app.surfTabsEnabled) {
          driveWithCommand(*actual, +1, false, *drive, ledExt, ledRet, dt);
        }
      }
    }
  }

  if (!extPressed && adj->extPrev) {
    adj->extPressStart = 0;
    adj->extWasHold = false;
    adj->extLastRepeatMs = 0;
  }

  // =========================
  // RETRACT
  // =========================
  if (retPressed && !adj->retPrev && adj->retArmed) {
    adj->retArmed = false;
    adj->retPressStart = now;
    adj->retWasHold = false;
    adj->retLastRepeatMs = 0;
    adj->retReleaseMs = 0;

    // Immediate single tap step
    *target = max(0.0f, *target - Config::autoTargetMinStepPct);
    *lastChangeMs = now;
  }

  if (retPressed && !extPressed) {
    if (!adj->retWasHold) {
      if (adj->retPressStart != 0 &&
          (now - adj->retPressStart >= Config::holdThresholdMs)) {
        adj->retWasHold = true;
        adj->retLastRepeatMs = now + Config::holdStartRepeatDelayMs;
      }
    } else {
      if ((long)(now - adj->retLastRepeatMs) >= 0) {
        const float step =
          Config::autoTargetHoldRatePctPerSec * (Config::holdRepeatMs / 1000.0f);

        *target = max(0.0f, *target - step);
        *lastChangeMs = now;
        adj->retLastRepeatMs = now + Config::holdRepeatMs;

        if (app.surfTabsEnabled) {
          driveWithCommand(*actual, -1, false, *drive, ledExt, ledRet, dt);
        }
      }
    }
  }

  if (!retPressed && adj->retPrev) {
    adj->retPressStart = 0;
    adj->retWasHold = false;
    adj->retLastRepeatMs = 0;
  }

  adj->extPrev = extPressed;
  adj->retPrev = retPressed;
}

void updateAutoMode(float dt) {
  const unsigned long now = millis();

  float desiredL = app.actualPosL;
  float desiredR = app.actualPosR;

  if (app.launchActive) {
    if (app.baseMode == MODE_MANUAL) {
      const float cap = manualMaxAllowedExtension(app.boatSpeed);

      // Special case:
      // In manual launch, if speed rules currently allow full extension,
      // drive directly to 100 without using continuous endpoint push logic.
      // This prevents auto-launch from consuming endpoint overtravel and
      // incorrectly triggering cooldown/release behavior.
      if (cap >= 100.0f - Config::posDeadband) {
        desiredL = 100.0f;
        desiredR = 100.0f;
      } else {
        desiredL = cap;
        desiredR = cap;
      }
    } else {
      desiredL = 100.0f;
      desiredR = 100.0f;
    }
  } else if (app.postLaunchManualRetract) {
    desiredL = 0.0f;
    desiredR = 0.0f;
  } else {
    if (app.baseMode == MODE_SURF_L) {
      desiredL = 0.0f;

      if (!app.surfTabsEnabled) {
        desiredR = 0.0f;
      } else if (app.surfTransferActive) {
        desiredR = app.surfLTarget;
      } else {
        const bool surfLAdjustHeld =
          ((Hardware::buttonStates[BTN_L_EXT] &&
            !Hardware::buttonStates[BTN_L_RET] &&
            app.adjustL.extWasHold) ||
           (Hardware::buttonStates[BTN_L_RET] &&
            !Hardware::buttonStates[BTN_L_EXT] &&
            app.adjustL.retWasHold));

        const bool surfLWaitingForSettle =
          (!surfLAdjustHeld &&
           app.surfLTargetLastChangeMs != 0 &&
           (now - app.surfLTargetLastChangeMs < Config::surfTargetSettleMs));

        desiredR = (surfLAdjustHeld || surfLWaitingForSettle)
                     ? app.actualPosR
                     : app.surfLTarget;
      }
    } else if (app.baseMode == MODE_SURF_R) {
      desiredR = 0.0f;

      if (!app.surfTabsEnabled) {
        desiredL = 0.0f;
      } else if (app.surfTransferActive) {
        desiredL = app.surfRTarget;
      } else {
        const bool surfRAdjustHeld =
          ((Hardware::buttonStates[BTN_R_EXT] &&
            !Hardware::buttonStates[BTN_R_RET] &&
            app.adjustR.extWasHold) ||
           (Hardware::buttonStates[BTN_R_RET] &&
            !Hardware::buttonStates[BTN_R_EXT] &&
            app.adjustR.retWasHold));

        const bool surfRWaitingForSettle =
          (!surfRAdjustHeld &&
           app.surfRTargetLastChangeMs != 0 &&
           (now - app.surfRTargetLastChangeMs < Config::surfTargetSettleMs));

        desiredL = (surfRAdjustHeld || surfRWaitingForSettle)
                     ? app.actualPosL
                     : app.surfRTarget;
      }
    } else {
      desiredL = 0.0f;
      desiredR = 0.0f;
    }
  }

  driveTowardTarget(app.actualPosL, desiredL,
                    app.driveL, Config::ledL_EXT, Config::ledL_RET, dt);
  driveTowardTarget(app.actualPosR, desiredR,
                    app.driveR, Config::ledR_EXT, Config::ledR_RET, dt);

  if (app.postLaunchManualRetract &&
      app.actualPosL <= Config::posDeadband &&
      app.actualPosR <= Config::posDeadband &&
      !app.driveL.calActive &&
      !app.driveR.calActive) {
    app.postLaunchManualRetract = false;
  }
}

void updateSurfTransferState() {
  if (!app.surfTransferActive) return;

  float desiredL = 0.0f;
  float desiredR = 0.0f;

  if (app.baseMode == MODE_SURF_L) {
    desiredL = 0.0f;
    desiredR = app.surfTabsEnabled ? app.surfLTarget : 0.0f;
  } else if (app.baseMode == MODE_SURF_R) {
    desiredL = app.surfTabsEnabled ? app.surfRTarget : 0.0f;
    desiredR = 0.0f;
  } else {
    app.surfTransferActive = false;
    return;
  }

  bool leftDone  = fabs(app.actualPosL - desiredL) <= Config::targetSettleBand;
  bool rightDone = fabs(app.actualPosR - desiredR) <= Config::targetSettleBand;

  if (desiredL <= Config::zeroSeatThresholdPct) {
    leftDone = (app.actualPosL <= Config::posDeadband && !app.driveL.calActive);
  }

  if (desiredR <= Config::zeroSeatThresholdPct) {
    rightDone = (app.actualPosR <= Config::posDeadband && !app.driveR.calActive);
  }

  if (leftDone && rightDone) {
    app.surfTransferActive = false;
    clearSurfAdjustState();
    UI::uiMsgClear();
  }
}

void updateTapTrackers() {
  const unsigned long now = millis();

  const bool surfLPressed   = Hardware::buttonStates[BTN_SURF_L];
  const bool surfRPressed   = Hardware::buttonStates[BTN_SURF_R];
  const bool bothRetPressed = Hardware::buttonStates[BTN_BOTH_RET];

  const bool anyOtherForSurfL =
    Hardware::buttonStates[BTN_L_EXT] || Hardware::buttonStates[BTN_L_RET] ||
    Hardware::buttonStates[BTN_R_EXT] || Hardware::buttonStates[BTN_R_RET] ||
    Hardware::buttonStates[BTN_BOTH_EXT] || Hardware::buttonStates[BTN_BOTH_RET] ||
    Hardware::buttonStates[BTN_LAUNCH] || Hardware::buttonStates[BTN_SURF_R];

  const bool anyOtherForSurfR =
    Hardware::buttonStates[BTN_L_EXT] || Hardware::buttonStates[BTN_L_RET] ||
    Hardware::buttonStates[BTN_R_EXT] || Hardware::buttonStates[BTN_R_RET] ||
    Hardware::buttonStates[BTN_BOTH_EXT] || Hardware::buttonStates[BTN_BOTH_RET] ||
    Hardware::buttonStates[BTN_LAUNCH] || Hardware::buttonStates[BTN_SURF_L];

  const bool anyOtherForBothRet =
    Hardware::buttonStates[BTN_L_EXT] || Hardware::buttonStates[BTN_L_RET] ||
    Hardware::buttonStates[BTN_R_EXT] || Hardware::buttonStates[BTN_R_RET] ||
    Hardware::buttonStates[BTN_BOTH_EXT] || Hardware::buttonStates[BTN_LAUNCH] ||
    Hardware::buttonStates[BTN_SURF_L] || Hardware::buttonStates[BTN_SURF_R];

  if (app.tapSurfL.count > 0 && anyOtherForSurfL)     resetTapTracker(app.tapSurfL);
  if (app.tapSurfR.count > 0 && anyOtherForSurfR)     resetTapTracker(app.tapSurfR);
  if (app.tapBothRet.count > 0 && anyOtherForBothRet) resetTapTracker(app.tapBothRet);

  if (surfLPressed && !app.tapSurfL.pressed) {
    app.tapSurfL.pressed = true;
    app.tapSurfL.pressStartMs = now;
  }

  if (!surfLPressed && app.tapSurfL.pressed) {
    app.tapSurfL.pressed = false;
    const unsigned long pressDur = now - app.tapSurfL.pressStartMs;

    if (pressDur >= Config::tapPressMinMs && registerTap(app.tapSurfL, now)) {
      if (app.baseMode == MODE_MANUAL) {
        if (app.boatSpeed < 5.0f) {
          app.baseMode = MODE_SURF_L;
          app.launchActive = false;
          app.postLaunchManualRetract = false;
          app.surfTabsEnabled = false;
          app.surfAboveStartMs = 0;
          app.surfBelowStartMs = 0;
          app.surfOverSpeedStartMs = 0;
        }
      } else if (app.baseMode == MODE_SURF_R) {
        app.baseMode = MODE_SURF_L;
        app.launchActive = false;
        clearSurfAdjustState();
        app.surfTransferActive = true;
        UI::uiMsgSet(MSG_INFO, "SWITCHING", "L <-- R", 900, ILI9341_CYAN);

        if (app.boatSpeed >= Config::surfEnableSpeedOn) {
          app.surfTabsEnabled = true;
          app.surfAboveStartMs = 0;
          app.surfBelowStartMs = 0;
        } else {
          app.surfTabsEnabled = false;
          app.surfAboveStartMs = 0;
          app.surfBelowStartMs = 0;
        }

        app.surfOverSpeedStartMs = 0;
      }
    }
  }

  if (surfRPressed && !app.tapSurfR.pressed) {
    app.tapSurfR.pressed = true;
    app.tapSurfR.pressStartMs = now;
  }

  if (!surfRPressed && app.tapSurfR.pressed) {
    app.tapSurfR.pressed = false;
    const unsigned long pressDur = now - app.tapSurfR.pressStartMs;

    if (pressDur >= Config::tapPressMinMs && registerTap(app.tapSurfR, now)) {
      if (app.baseMode == MODE_MANUAL) {
        if (app.boatSpeed < 5.0f) {
          app.baseMode = MODE_SURF_R;
          app.launchActive = false;
          app.postLaunchManualRetract = false;
          app.surfTabsEnabled = false;
          app.surfAboveStartMs = 0;
          app.surfBelowStartMs = 0;
          app.surfOverSpeedStartMs = 0;
        }
      } else if (app.baseMode == MODE_SURF_L) {
        app.baseMode = MODE_SURF_R;
        app.launchActive = false;
        clearSurfAdjustState();
        app.surfTransferActive = true;
        UI::uiMsgSet(MSG_INFO, "SWITCHING", "L --> R", 900, ILI9341_CYAN);

        if (app.boatSpeed >= Config::surfEnableSpeedOn) {
          app.surfTabsEnabled = true;
          app.surfAboveStartMs = 0;
          app.surfBelowStartMs = 0;
        } else {
          app.surfTabsEnabled = false;
          app.surfAboveStartMs = 0;
          app.surfBelowStartMs = 0;
        }

        app.surfOverSpeedStartMs = 0;
      }
    }
  }

  if (bothRetPressed && !app.tapBothRet.pressed) {
    app.tapBothRet.pressed = true;
    app.tapBothRet.pressStartMs = now;
  }

  if (!bothRetPressed && app.tapBothRet.pressed) {
    app.tapBothRet.pressed = false;
    const unsigned long pressDur = now - app.tapBothRet.pressStartMs;

    if (pressDur >= Config::tapPressMinMs && registerTap(app.tapBothRet, now)) {
      if (app.baseMode != MODE_MANUAL) {
        app.surfTabsEnabled = false;
        app.surfAboveStartMs = 0;
        app.surfBelowStartMs = 0;
        app.surfOverSpeedStartMs = 0;
        beginManualZeroing();
      } else {
        app.baseMode = MODE_MANUAL;
        app.launchActive = false;
        app.postLaunchManualRetract = false;
      }
    }
  }

  static bool prevLaunchPressed = false;
  const bool launchPressed = Hardware::buttonStates[BTN_LAUNCH];

  if (launchPressed && !prevLaunchPressed) {
    if (app.launchActive) {
      app.launchActive = false;
      app.launchAboveStartMs = 0;
      UI::uiMsgSet(MSG_WARN, "LAUNCH", "CANCELED", 900, ILI9341_YELLOW);

      if (app.baseMode == MODE_MANUAL) {
        app.postLaunchManualRetract = true;
      }
    } else {
      if (app.boatSpeed < 5.0f) {
        app.launchActive = true;
        app.postLaunchManualRetract = false;
        app.launchStartMs = now;
        app.launchAboveStartMs = 0;
        UI::uiMsgSet(MSG_INFO, "LAUNCH", "ACTIVE", 800, ILI9341_GREEN);
      }
    }
  }

  prevLaunchPressed = launchPressed;
}

}  // namespace Control

// ============================================================
// Display Layer
// ============================================================
namespace Display {

void drawStaticLayout() {
  auto &tft = Hardware::tft;

  tft.fillScreen(ILI9341_BLACK);
  tft.drawRoundRect(Config::MODE_X, Config::MODE_Y, Config::MODE_W, Config::MODE_H, 4, ILI9341_DARKCYAN);
  tft.drawRoundRect(Config::SPEED_X, Config::SPEED_Y, Config::SPEED_W, Config::SPEED_H, 4, ILI9341_DARKCYAN);
  tft.drawRoundRect(Config::STATUS_X, Config::STATUS_Y, Config::STATUS_W, Config::STATUS_H, 6, ILI9341_DARKCYAN);
  tft.drawRoundRect(Config::LTAB_X, Config::LTAB_Y, Config::LTAB_W, Config::LTAB_H, 6, ILI9341_DARKCYAN);
  tft.drawRoundRect(Config::RTAB_X, Config::RTAB_Y, Config::RTAB_W, Config::RTAB_H, 6, ILI9341_DARKCYAN);

  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setCursor(Config::MODE_X + 6, Config::MODE_Y - 7);      tft.print("MODE");
  tft.setCursor(Config::SPEED_X + 6, Config::SPEED_Y - 7);    tft.print("SPEED");
  tft.setCursor(Config::STATUS_X + 6, Config::STATUS_Y - 7);  tft.print("ATTITUDE");
  tft.setCursor(Config::LTAB_X + 6, Config::LTAB_Y - 7);      tft.print("LEFT TAB");
  tft.setCursor(Config::RTAB_X + 6, Config::RTAB_Y - 7);      tft.print("RIGHT TAB");

  app.statusCache.valid = false;
  app.display.statusNormalStaticDrawn = false;
  app.display.lastLeftFillH = -1;
  app.display.lastRightFillH = -1;
  app.display.lastLeftLineY = -1;
  app.display.lastRightLineY = -1;
  app.display.lastLeftTextAct = -999;
  app.display.lastRightTextAct = -999;
  app.display.lastLeftTextTgt = -999;
  app.display.lastRightTextTgt = -999;
}

void drawModePanel(bool force) {
  auto &tft = Hardware::tft;
  const bool blinkState = app.launchActive ? (((millis() / 500) % 2) == 0) : false;

  if (!force &&
      app.baseMode == app.display.lastModeDisplay &&
      blinkState == app.display.lastLaunchBlinkState &&
      app.faultLatched == app.display.lastModeFaultDisplay) {
    return;
  }

  tft.fillRoundRect(Config::MODE_X + 1, Config::MODE_Y + 1, Config::MODE_W - 2, Config::MODE_H - 2, 4, ILI9341_BLACK);

  uint16_t fill = ILI9341_DARKCYAN;
  const char* label = "MANUAL";

  if (app.faultLatched) {
    fill = ILI9341_RED;
    label = "FAULT";
  } else if (app.launchActive && blinkState) {
    fill = ILI9341_YELLOW;
    label = "LAUNCH";
  } else {
    switch (app.baseMode) {
      case MODE_MANUAL: fill = ILI9341_DARKCYAN; label = "MANUAL"; break;
      case MODE_SURF_L: fill = ILI9341_ORANGE;   label = "SURF L"; break;
      case MODE_SURF_R: fill = ILI9341_MAGENTA;  label = "SURF R"; break;
    }
  }

  tft.fillRoundRect(Config::MODE_X + 2, Config::MODE_Y + 2, Config::MODE_W - 4, Config::MODE_H - 4, 4, fill);
  tft.setTextColor(ILI9341_BLACK, fill);
  tft.setTextSize(2);
  tft.setCursor(Config::MODE_X + 10, Config::MODE_Y + 7);
  tft.print(label);

  app.display.lastModeDisplay = app.baseMode;
  app.display.lastLaunchBlinkState = blinkState;
  app.display.lastModeFaultDisplay = app.faultLatched;
}

void drawSpeedPanel(bool force) {
  auto &tft = Hardware::tft;
  const int speedTenths = (int)(app.boatSpeed * 10.0f + (app.boatSpeed >= 0 ? 0.5f : -0.5f));

  bool invert = false;
  uint16_t bg = ILI9341_BLACK;
  uint16_t fg = ILI9341_GREEN;
  uint16_t mphFg = ILI9341_WHITE;

  if (app.faultLatched) {
    bg = ILI9341_RED;
    fg = ILI9341_BLACK;
    mphFg = ILI9341_BLACK;
    invert = true;
  } else if (app.launchActive || ((app.baseMode == MODE_SURF_L || app.baseMode == MODE_SURF_R) && !app.surfTabsEnabled)) {
    bg = ILI9341_GREEN;
    fg = ILI9341_BLACK;
    mphFg = ILI9341_BLACK;
    invert = true;
  }

  if (!force && speedTenths == app.display.lastSpeedTenths && invert == app.display.lastSpeedInverted) return;

  tft.fillRect(Config::SPEED_X + 3, Config::SPEED_Y + 3, Config::SPEED_W - 6, Config::SPEED_H - 6, bg);
  tft.setTextColor(fg, bg);
  tft.setTextSize(2);
  tft.setCursor(Config::SPEED_X + 8, Config::SPEED_Y + 6);
  tft.print("     ");
  tft.setCursor(Config::SPEED_X + 8, Config::SPEED_Y + 6);
  tft.print(app.boatSpeed, 1);

  tft.setTextSize(1);
  tft.setTextColor(mphFg, bg);
  tft.setCursor(Config::SPEED_X + 82, Config::SPEED_Y + 11);
  tft.print("MPH");

  app.display.lastSpeedTenths = speedTenths;
  app.display.lastSpeedInverted = invert;
}

void drawStatusPanelNormalStatic(bool showTargets, float tgtPitch, float tgtRoll) {
  auto &tft = Hardware::tft;
  (void)tgtPitch;
  (void)tgtRoll;

  tft.fillRoundRect(Config::STATUS_X + 1, Config::STATUS_Y + 1, Config::STATUS_W - 2, Config::STATUS_H - 2, 6, ILI9341_BLACK);

  tft.setTextSize(1);
  tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
  tft.setCursor(Config::STATUS_X + 10, Config::STATUS_Y + 8);
  tft.print("PITCH");

  const int pitchBarX = Config::STATUS_X + 22;
  const int pitchBarY = Config::STATUS_Y + 30;
  const int pitchBarW = Config::STATUS_W - 44;
  const int pitchBarH = 8;
  const int pitchMidX = pitchBarX + pitchBarW / 2;

  tft.drawRect(pitchBarX, pitchBarY, pitchBarW, pitchBarH, ILI9341_DARKGREY);
  tft.drawFastVLine(pitchMidX, pitchBarY - 2, pitchBarH + 4, ILI9341_WHITE);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setCursor(pitchBarX + 2, pitchBarY + 12);
  tft.print("-15");
  tft.setCursor(pitchBarX + pitchBarW - 16, pitchBarY + 12);
  tft.print("15");

  tft.setTextSize(1);
  tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
  tft.setCursor(Config::STATUS_X + 10, Config::STATUS_Y + 56);
  tft.print("ROLL");

  const int rollBarX = Config::STATUS_X + 22;
  const int rollBarY = Config::STATUS_Y + 78;
  const int rollBarW = Config::STATUS_W - 44;
  const int rollBarH = 8;
  const int rollMidX = rollBarX + rollBarW / 2;

  tft.drawRect(rollBarX, rollBarY, rollBarW, rollBarH, ILI9341_DARKGREY);
  tft.drawFastVLine(rollMidX, rollBarY - 2, rollBarH + 4, ILI9341_WHITE);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setCursor(rollBarX + 6, rollBarY + 12);
  tft.print("-7");
  tft.setCursor(rollBarX + rollBarW - 10, rollBarY + 12);
  tft.print("7");

  app.display.lastPitchMarkerX = -9999;
  app.display.lastRollMarkerX = -9999;
  app.display.lastPitchTargetX = -9999;
  app.display.lastRollTargetX = -9999;
  app.display.lastPitchValue10 = 99999;
  app.display.lastRollValue10 = 99999;
  app.display.lastShowTargets = showTargets;
  app.display.statusNormalStaticDrawn = true;
}

void drawStatusPanelNormalDynamic(bool showTargets, float tgtPitch, float tgtRoll) {
  auto &tft = Hardware::tft;

  const int pitchBarX = Config::STATUS_X + 22;
  const int pitchBarY = Config::STATUS_Y + 30;
  const int pitchBarW = Config::STATUS_W - 44;
  const int pitchBarH = 8;
  const int pitchMidX = pitchBarX + pitchBarW / 2;

  const int rollBarX = Config::STATUS_X + 22;
  const int rollBarY = Config::STATUS_Y + 78;
  const int rollBarW = Config::STATUS_W - 44;
  const int rollBarH = 8;
  const int rollMidX = rollBarX + rollBarW / 2;

  tft.fillRect(pitchMidX - 28, pitchBarY - 10, 60, 8, ILI9341_BLACK);
  tft.fillRect(rollMidX - 28, rollBarY - 10, 60, 8, ILI9341_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

  tft.setCursor(pitchMidX - 24, pitchBarY - 10);
  if (showTargets) { tft.print("TGT "); tft.print(tgtPitch, 1); }
  else { tft.print("TGT --.-"); }

  tft.setCursor(rollMidX - 24, rollBarY - 10);
  if (showTargets) { tft.print("TGT "); tft.print(tgtRoll, 1); }
  else { tft.print("TGT --.-"); }

  const float pitchClamped = constrain(app.simPitch, -15.0f, 15.0f);
  const int pitchMarkerX = pitchBarX + (int)(((pitchClamped + 15.0f) / 30.0f) * pitchBarW);
  const float rollClamped = constrain(app.simRoll, -7.0f, 7.0f);
  const int rollMarkerX = rollBarX + (int)(((rollClamped + 7.0f) / 14.0f) * rollBarW);

  if (app.display.lastPitchMarkerX != -9999 && app.display.lastPitchMarkerX != pitchMarkerX) {
    tft.fillRect(app.display.lastPitchMarkerX - 2, pitchBarY + 1, 4, pitchBarH - 2, ILI9341_BLACK);
  }
  if (app.display.lastRollMarkerX != -9999 && app.display.lastRollMarkerX != rollMarkerX) {
    tft.fillRect(app.display.lastRollMarkerX - 2, rollBarY + 1, 4, rollBarH - 2, ILI9341_BLACK);
  }

  tft.drawFastVLine(pitchMidX, pitchBarY - 2, pitchBarH + 4, ILI9341_WHITE);
  tft.drawFastVLine(rollMidX, rollBarY - 2, rollBarH + 4, ILI9341_WHITE);
  tft.fillRect(pitchMarkerX - 2, pitchBarY + 1, 4, pitchBarH - 2, ILI9341_YELLOW);
  tft.fillRect(rollMarkerX - 2, rollBarY + 1, 4, rollBarH - 2, ILI9341_ORANGE);

  app.display.lastPitchMarkerX = pitchMarkerX;
  app.display.lastRollMarkerX = rollMarkerX;

  int pitchTargetX = -9999;
  int rollTargetX = -9999;
  if (showTargets) {
    const float pitchTgtClamped = constrain(tgtPitch, -15.0f, 15.0f);
    pitchTargetX = pitchBarX + (int)(((pitchTgtClamped + 15.0f) / 30.0f) * pitchBarW);
    const float rollTgtClamped = constrain(tgtRoll, -7.0f, 7.0f);
    rollTargetX = rollBarX + (int)(((rollTgtClamped + 7.0f) / 14.0f) * rollBarW);
  }

  if (app.display.lastPitchTargetX != -9999 && app.display.lastPitchTargetX != pitchTargetX) {
    tft.drawFastVLine(app.display.lastPitchTargetX, pitchBarY - 3, pitchBarH + 6, ILI9341_BLACK);
  }
  if (app.display.lastRollTargetX != -9999 && app.display.lastRollTargetX != rollTargetX) {
    tft.drawFastVLine(app.display.lastRollTargetX, rollBarY - 3, rollBarH + 6, ILI9341_BLACK);
  }

  if (showTargets) {
    tft.drawFastVLine(pitchTargetX, pitchBarY - 3, pitchBarH + 6, ILI9341_GREEN);
    tft.drawFastVLine(rollTargetX, rollBarY - 3, rollBarH + 6, ILI9341_GREEN);
  }

  app.display.lastPitchTargetX = pitchTargetX;
  app.display.lastRollTargetX = rollTargetX;
  app.display.lastShowTargets = showTargets;

  const int pitch10Now = (int)(app.simPitch * 10.0f);
  if (app.display.lastPitchValue10 != pitch10Now) {
    tft.fillRect(pitchMidX - 24, pitchBarY + 10, 52, 18, ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setCursor(pitchMidX - 18, pitchBarY + 10);
    tft.print(app.simPitch, 1);
    app.display.lastPitchValue10 = pitch10Now;
  }

  const int roll10Now = (int)(app.simRoll * 10.0f);
  if (app.display.lastRollValue10 != roll10Now) {
    tft.fillRect(rollMidX - 24, rollBarY + 10, 52, 18, ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setCursor(rollMidX - 18, rollBarY + 10);
    tft.print(app.simRoll, 1);
    app.display.lastRollValue10 = roll10Now;
  }
}

void drawStatusPanel(bool force) {
  auto &tft = Hardware::tft;
  const int p10 = (int)(app.simPitch * 10.0f);
  const int r10 = (int)(app.simRoll * 10.0f);
  const OverlayKind ok = UI::currentOverlayKind();
  const uint32_t sig = (ok == OVERLAY_MSG) ? UI::currentMsgSig() : 0;

  const bool waitingForSurfSpeed =
    !app.faultLatched &&
    !app.launchActive &&
    (app.baseMode == MODE_SURF_L || app.baseMode == MODE_SURF_R) &&
    !app.surfTabsEnabled;

  uint8_t displayState = 0;
  if (ok == OVERLAY_FAULT) displayState = 3;
  else if (ok == OVERLAY_MSG) displayState = 2;
  else if (waitingForSurfSpeed) displayState = 1;
  else displayState = 0;

  const bool showingNormal = (displayState == 0);

  const bool changed =
    !app.statusCache.valid ||
    app.statusCache.mode != app.baseMode ||
    app.statusCache.fault != app.faultLatched ||
    app.statusCache.overlayKind != (uint8_t)ok ||
    app.statusCache.msgSig != sig ||
    app.statusCache.displayState != displayState ||
    (showingNormal &&
     (app.statusCache.pitch10 != p10 || app.statusCache.roll10 != r10));

  if (!force && !changed) return;

  if (displayState == 3) {
    app.display.statusNormalStaticDrawn = false;
    tft.fillRoundRect(Config::STATUS_X + 1, Config::STATUS_Y + 1, Config::STATUS_W - 2, Config::STATUS_H - 2, 6, ILI9341_BLACK);
    tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setCursor(Config::STATUS_X + 62, Config::STATUS_Y + 22);
    tft.print("FAULT");
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setCursor(Config::STATUS_X + 40, Config::STATUS_Y + 54);
    tft.print("RELEASE INPUTS");
    tft.setCursor(Config::STATUS_X + 44, Config::STATUS_Y + 68);
    tft.print("TO CLEAR");
  }
  else if (displayState == 2) {
    app.display.statusNormalStaticDrawn = false;
    tft.fillRoundRect(Config::STATUS_X + 1, Config::STATUS_Y + 1, Config::STATUS_W - 2, Config::STATUS_H - 2, 6, ILI9341_BLACK);
    tft.setTextColor(app.uiMsg.color, ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setCursor(Config::STATUS_X + 10, Config::STATUS_Y + 26);
    tft.print(app.uiMsg.line1);
    tft.setTextSize(1);
    tft.setCursor(Config::STATUS_X + 10, Config::STATUS_Y + 56);
    tft.print(app.uiMsg.line2);
  }
  else if (displayState == 1) {
    app.display.statusNormalStaticDrawn = false;
    tft.fillRoundRect(Config::STATUS_X + 1, Config::STATUS_Y + 1, Config::STATUS_W - 2, Config::STATUS_H - 2, 6, ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
    tft.setCursor(Config::STATUS_X + 18, Config::STATUS_Y + 26);
    tft.print("WAITING FOR");
    tft.setCursor(Config::STATUS_X + 26, Config::STATUS_Y + 56);
    tft.print("SURF SPEED");
  }
  else {
    float tgtPitch = 0.0f;
    float tgtRoll = 0.0f;
    bool showTargets = false;

    if (app.baseMode == MODE_SURF_L) {
      tgtPitch = Config::surfLPitchTarget;
      tgtRoll  = Config::surfLRollTarget;
      showTargets = true;
    } else if (app.baseMode == MODE_SURF_R) {
      tgtPitch = Config::surfRPitchTarget;
      tgtRoll  = Config::surfRRollTarget;
      showTargets = true;
    }

    if (!app.display.statusNormalStaticDrawn || force || app.display.lastShowTargets != showTargets) {
      drawStatusPanelNormalStatic(showTargets, tgtPitch, tgtRoll);
    }
    drawStatusPanelNormalDynamic(showTargets, tgtPitch, tgtRoll);
  }

  app.statusCache.valid = true;
  app.statusCache.pitch10 = p10;
  app.statusCache.roll10 = r10;
  app.statusCache.mode = app.baseMode;
  app.statusCache.fault = app.faultLatched;
  app.statusCache.overlayKind = (uint8_t)ok;
  app.statusCache.msgSig = sig;
  app.statusCache.displayState = displayState;
}

void drawTabPanelLeft(bool force) {
  auto &tft = Hardware::tft;
  const int act = (int)app.actualPosL;
  int valueLine = 0;

  const bool showLimit =
    (app.baseMode == MODE_MANUAL && !app.launchActive && !app.postLaunchManualRetract && !app.manualEntryZeroing && !app.faultLatched);
  const bool active = (app.baseMode == MODE_SURF_L);

  if (showLimit) valueLine = (int)Control::manualMaxAllowedExtension(app.boatSpeed);
  else {
    if (app.launchActive && app.baseMode == MODE_MANUAL) valueLine = (int)Control::manualMaxAllowedExtension(app.boatSpeed);
    else if (app.launchActive) valueLine = 100;
    else if (app.postLaunchManualRetract || app.manualEntryZeroing || app.faultLatched) valueLine = 0;
    else if (app.baseMode == MODE_SURF_L) valueLine = 0;
    else valueLine = (int)app.surfRTarget;
  }

  const int barW = 22;
  const int barH = Config::LTAB_H - 18;
  const int barX = Config::LTAB_X + Config::LTAB_W - barW - 10;
  const int barY = Config::LTAB_Y + 9;
  const int fillH = ((barH - 2) * act) / 100;
  const int lineY = barY + (barH * valueLine) / 100;

  const bool panelStateChanged = force ||
    showLimit != app.display.lastLeftShowLimit ||
    active != app.display.lastLeftActive;

  if (panelStateChanged) {
    tft.fillRect(Config::LTAB_X + 2, Config::LTAB_Y + 2, Config::LTAB_W - 4, Config::LTAB_H - 4, ILI9341_BLACK);
    tft.setTextColor(active ? ILI9341_ORANGE : ILI9341_WHITE, ILI9341_BLACK);
    tft.setTextSize(1);
    tft.setCursor(Config::LTAB_X + 8, Config::LTAB_Y + 8);
    tft.print("ACT");
    tft.setTextSize(1);
    tft.setCursor(Config::LTAB_X + 8, Config::LTAB_Y + 42);
    if (showLimit) {
      const bool limitingNow = (app.actualPosL > Control::manualMaxAllowedExtension(app.boatSpeed) + Config::targetSettleBand);
      tft.setTextColor(limitingNow ? ILI9341_RED : ILI9341_WHITE, ILI9341_BLACK);
      tft.print("LIM");
    } else {
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      tft.print("TGT");
    }
    tft.drawRect(barX, barY, barW, barH, ILI9341_DARKGREY);
    app.display.lastLeftFillH = -1;
    app.display.lastLeftLineY = -1;
    app.display.lastLeftTextAct = -999;
    app.display.lastLeftTextTgt = -999;
  }

  if (force || act != app.display.lastLeftTextAct) {
    tft.fillRect(Config::LTAB_X + 8, Config::LTAB_Y + 20, 52, 16, ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setCursor(Config::LTAB_X + 8, Config::LTAB_Y + 20);
    tft.print(act);
    tft.print("%");
    app.display.lastLeftTextAct = act;
  }

  if (panelStateChanged) {
    tft.fillRect(Config::LTAB_X + 8, Config::LTAB_Y + 42, 30, 10, ILI9341_BLACK);
    tft.setTextSize(1);
    if (showLimit) {
      const bool limitingNow = (app.actualPosL > Control::manualMaxAllowedExtension(app.boatSpeed) + Config::targetSettleBand);
      tft.setTextColor(limitingNow ? ILI9341_RED : ILI9341_WHITE, ILI9341_BLACK);
      tft.setCursor(Config::LTAB_X + 8, Config::LTAB_Y + 42);
      tft.print("LIM");
    } else {
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      tft.setCursor(Config::LTAB_X + 8, Config::LTAB_Y + 42);
      tft.print("TGT");
    }
  }

  if (force || valueLine != app.display.lastLeftTextTgt) {
    tft.fillRect(Config::LTAB_X + 8, Config::LTAB_Y + 54, 52, 16, ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setCursor(Config::LTAB_X + 8, Config::LTAB_Y + 54);
    tft.print(valueLine);
    tft.print("%");
    app.display.lastLeftTextTgt = valueLine;
  }

  if (force || fillH != app.display.lastLeftFillH) {
    if (app.display.lastLeftFillH < 0) {
      tft.fillRect(barX + 1, barY + 1, barW - 2, barH - 2, ILI9341_BLACK);
      if (fillH > 0) tft.fillRect(barX + 1, barY + 1, barW - 2, fillH, ILI9341_YELLOW);
    } else if (fillH > app.display.lastLeftFillH) {
      tft.fillRect(barX + 1, barY + 1 + app.display.lastLeftFillH, barW - 2, fillH - app.display.lastLeftFillH, ILI9341_YELLOW);
    } else if (fillH < app.display.lastLeftFillH) {
      tft.fillRect(barX + 1, barY + 1 + fillH, barW - 2, app.display.lastLeftFillH - fillH, ILI9341_BLACK);
    }
    app.display.lastLeftFillH = fillH;
  }

  if (force || lineY != app.display.lastLeftLineY || showLimit != app.display.lastLeftShowLimit) {
    if (app.display.lastLeftLineY >= 0) {
      tft.drawFastHLine(barX - 4, app.display.lastLeftLineY, barW + 8, ILI9341_BLACK);
    }
    if (showLimit) tft.drawFastHLine(barX - 4, lineY, barW + 8, ILI9341_RED);
    else tft.drawFastHLine(barX - 4, lineY, barW + 8, ILI9341_CYAN);
    app.display.lastLeftLineY = lineY;
  }

  app.display.lastLeftShowLimit = showLimit;
  app.display.lastLeftActive = active;
}

void drawTabPanelRight(bool force) {
  auto &tft = Hardware::tft;
  const int act = (int)app.actualPosR;
  int valueLine = 0;

  const bool showLimit =
    (app.baseMode == MODE_MANUAL && !app.launchActive && !app.postLaunchManualRetract && !app.manualEntryZeroing) || app.faultLatched;
  const bool active = (app.baseMode == MODE_SURF_R);

  if (showLimit) valueLine = app.faultLatched ? 0 : (int)Control::manualMaxAllowedExtension(app.boatSpeed);
  else {
    if (app.launchActive && app.baseMode == MODE_MANUAL) valueLine = (int)Control::manualMaxAllowedExtension(app.boatSpeed);
    else if (app.launchActive) valueLine = 100;
    else if (app.postLaunchManualRetract || app.manualEntryZeroing || app.faultLatched) valueLine = 0;
    else if (app.baseMode == MODE_SURF_R) valueLine = 0;
    else valueLine = (int)app.surfLTarget;
  }

  const int barW = 22;
  const int barH = Config::RTAB_H - 18;
  const int barX = Config::RTAB_X + 10;
  const int barY = Config::RTAB_Y + 9;
  const int textX = Config::RTAB_X + 42;
  const int fillH = ((barH - 2) * act) / 100;
  const int lineY = barY + (barH * valueLine) / 100;

  const bool panelStateChanged = force ||
    showLimit != app.display.lastRightShowLimit ||
    active != app.display.lastRightActive;

  if (panelStateChanged) {
    tft.fillRect(Config::RTAB_X + 2, Config::RTAB_Y + 2, Config::RTAB_W - 4, Config::RTAB_H - 4, ILI9341_BLACK);
    tft.setTextColor(active ? ILI9341_MAGENTA : ILI9341_WHITE, ILI9341_BLACK);
    tft.setTextSize(1);
    tft.setCursor(textX, Config::RTAB_Y + 8);
    tft.print("ACT");
    tft.setTextSize(1);
    tft.setCursor(textX, Config::RTAB_Y + 42);
    if (showLimit) {
      const bool limitingNow = (app.actualPosR > Control::manualMaxAllowedExtension(app.boatSpeed) + Config::targetSettleBand);
      tft.setTextColor(limitingNow ? ILI9341_RED : ILI9341_WHITE, ILI9341_BLACK);
      tft.print("LIM");
    } else {
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      tft.print("TGT");
    }
    tft.drawRect(barX, barY, barW, barH, ILI9341_DARKGREY);
    app.display.lastRightFillH = -1;
    app.display.lastRightLineY = -1;
    app.display.lastRightTextAct = -999;
    app.display.lastRightTextTgt = -999;
  }

  if (force || act != app.display.lastRightTextAct) {
    tft.fillRect(textX, Config::RTAB_Y + 20, 52, 16, ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setCursor(textX, Config::RTAB_Y + 20);
    tft.print(act);
    tft.print("%");
    app.display.lastRightTextAct = act;
  }

  if (panelStateChanged) {
    tft.fillRect(textX, Config::RTAB_Y + 42, 30, 10, ILI9341_BLACK);
    tft.setTextSize(1);
    if (showLimit) {
      const bool limitingNow = (app.actualPosR > Control::manualMaxAllowedExtension(app.boatSpeed) + Config::targetSettleBand);
      tft.setTextColor(limitingNow ? ILI9341_RED : ILI9341_WHITE, ILI9341_BLACK);
      tft.setCursor(textX, Config::RTAB_Y + 42);
      tft.print("LIM");
    } else {
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      tft.setCursor(textX, Config::RTAB_Y + 42);
      tft.print("TGT");
    }
  }

  if (force || valueLine != app.display.lastRightTextTgt) {
    tft.fillRect(textX, Config::RTAB_Y + 54, 52, 16, ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setCursor(textX, Config::RTAB_Y + 54);
    tft.print(valueLine);
    tft.print("%");
    app.display.lastRightTextTgt = valueLine;
  }

  if (force || fillH != app.display.lastRightFillH) {
    if (app.display.lastRightFillH < 0) {
      tft.fillRect(barX + 1, barY + 1, barW - 2, barH - 2, ILI9341_BLACK);
      if (fillH > 0) tft.fillRect(barX + 1, barY + 1, barW - 2, fillH, ILI9341_YELLOW);
    } else if (fillH > app.display.lastRightFillH) {
      tft.fillRect(barX + 1, barY + 1 + app.display.lastRightFillH, barW - 2, fillH - app.display.lastRightFillH, ILI9341_YELLOW);
    } else if (fillH < app.display.lastRightFillH) {
      tft.fillRect(barX + 1, barY + 1 + fillH, barW - 2, app.display.lastRightFillH - fillH, ILI9341_BLACK);
    }
    app.display.lastRightFillH = fillH;
  }

  if (force || lineY != app.display.lastRightLineY || showLimit != app.display.lastRightShowLimit) {
    if (app.display.lastRightLineY >= 0) {
      tft.drawFastHLine(barX - 4, app.display.lastRightLineY, barW + 8, ILI9341_BLACK);
    }
    if (showLimit) tft.drawFastHLine(barX - 4, lineY, barW + 8, ILI9341_RED);
    else tft.drawFastHLine(barX - 4, lineY, barW + 8, ILI9341_CYAN);
    app.display.lastRightLineY = lineY;
  }

  app.display.lastRightShowLimit = showLimit;
  app.display.lastRightActive = active;
}

void drawDashboard() {
  drawModePanel();
  drawSpeedPanel();
  drawStatusPanel();
  drawTabPanelLeft();
  drawTabPanelRight();
}

}  // namespace Display

// ============================================================
// Arduino Setup / Loop
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Tab Controller loading...");

  for (int p : {Config::pinL_EXT, Config::pinL_RET, Config::pinR_EXT, Config::pinR_RET,
                Config::pinLAUNCH, Config::pinSURF_L, Config::pinSURF_R}) {
    pinMode(p, INPUT_PULLUP);
  }

  pinMode(Config::pinBOTH_EXT, INPUT);
  pinMode(Config::pinBOTH_RET, INPUT);

  for (int p : {Config::ledL_EXT, Config::ledL_RET, Config::ledR_EXT, Config::ledR_RET}) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }

  Hardware::prefs.begin("tabsys", false);
  app.prefsInitialized = true;
  Persistence::loadState();

  Hardware::tft.begin();
  Hardware::tft.setRotation(0);
  Hardware::tft.fillScreen(ILI9341_BLACK);

  Display::drawStaticLayout();
  Display::drawDashboard();

  app.lastUpdate = millis();

  if (app.bootSeatL) {
    Control::armZeroSeat(app.driveL, app.actualPosL);
    app.bootZeroingActive = true;
  }
  if (app.bootSeatR) {
    Control::armZeroSeat(app.driveR, app.actualPosR);
    app.bootZeroingActive = true;
  }

  Serial.println("Setup complete");
}

void loop() {
  const unsigned long now = millis();
  const float dt = (now - app.lastUpdate) / 1000.0f;
  app.lastUpdate = now;

  Hardware::buttons.Loop();

  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      Simulation::processSerialLine(line);
    }
  }

  Control::updateFaultState(dt);

  if (app.faultLatched) {
    Persistence::maybeSaveState();
    Display::drawDashboard();
    return;
  }

  if (app.bootZeroingActive) {
    Control::updateBootZeroing(dt);
    Persistence::maybeSaveState();
    Display::drawDashboard();
    return;
  }

  Control::updateTapTrackers();
  Control::updateSurfSpeedState();
  Control::updateLaunchState();
  Control::updateSurfOverspeedExit();

  if (app.manualEntryZeroing) {
    Control::updateManualEntryZeroing(dt);
  } else if (app.baseMode == MODE_MANUAL && !app.launchActive && !app.postLaunchManualRetract) {
    Control::updateManualMode(dt);
  } else {
    Control::updateSurfTargetAdjust(dt);
    Control::updateAutoMode(dt);
    Control::updateSurfTransferState();
  }

  Persistence::maybeSaveState();
  Display::drawDashboard();
}
