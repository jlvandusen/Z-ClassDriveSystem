
/*
 * JOE'S DRIVE - FINAL: S2S + PID BALANCE + 3-SEC CALIBRATION
 * ESP32 Primary v9.1 — with enhanced sound command system + LIVE TUNING MODE
 *
 * SOUND SYSTEM OVERVIEW
 * ---------------------
 * Tracks available: 1–30, 50, 99–105
 * Special (CURRENT CONFIG):
 *   SOUND_STARTUP = 1   (played once at setup)
 *   SOUND_TOGGLE  = 105 (played when toggling enableDrive / autoBalance / reverseDrive / DomeServoMode)
 *
 * Drive Controller (buttonsR = right controller)
 *   Single D-pad (edge):    Up=1, Right=2, Down=3, Left=4
 *   L1 + D-pad combos:      Up=21, Right=22, Down=23, Left=24
 *   R1 + D-pad combos:      Up=25, Right=26, Down=27, Left=28
 *   L2 threshold (>60):     50
 *   Circle (edge):          Random(1..30)
 *
 * Dome Controller (buttonsL = left controller)
 *   Single D-pad (edge):    Up=5, Right=6, Down=7, Left=8
 *   L1 + D-pad combos:      Up=11, Right=12, Down=13, Left=14
 *   L2 threshold (>60):     99
 *   Circle (edge):          Random(1..30)
 *
 * Priority: If both controllers emit a sound on the same tick,
 *   Drive wins (flip by swapping chosen=... order).
 *
 * Transmission:
 *   - Sounds are emitted via emitSound(track) and sent in sendDataTo32u4().
 *   - soundcmd is one-shot and auto-clears (no repeat while held).
 */

/* ---------------------- How Live Tuning Works (Summary)-------------------------
  
Entering Live Tuning

Pitch PID Tuning: Hold PS + UP (drive controller) for 3 seconds → starts Pitch KP stage.
Roll S2S PID Tuning: Hold PS + RIGHT (drive controller) for 3 seconds → starts Roll PK stage.

Pitch (Drive) PID Stages

Stage 1 – KP
Start with KP=45, KI=0, KD=0.
Use UP/DOWN (drive D-pad) to increase/decrease KP (step = 1.0).
Press X to store KP, reduce by 25% (safety), save to Preferences, and advance.

Stage 2 – KD
Enable autoBalance automatically so you can see damping.
Use UP/DOWN to adjust KD (step = 1.0).
Press X to store KD, save, advance.

Stage 3 – KI
Use UP/DOWN to adjust KI (step = 0.1).
Press X to store KI, save, exit tuning.

Roll (S2S) PID Stages

Stage 1 – Pk2
Start with Ik2=0, Dk2=0; Pk2 default 45.
Use UP/DOWN to adjust Pk2 (step = 1.0).
Press X to store Pk2, reduce by 25% (safety), save, advance.

Stage 2 – Dk2
Enable S2S roll autoBalance so you can see lateral rocking.
Use UP/DOWN to adjust Dk2 (step = 1.0).
Press X to store Dk2, save, advance.

Stage 3 – Ik2
Use UP/DOWN to adjust Ik2 (step = 0.1).
Press X to store Ik2, save, exit tuning.

While in tuning:
No joystick control (drive/S2S), no sounds, no dome spin/flywheel.
Only D-pad UP/DOWN and X on the drive controller are used to advance/adjust.
For KD/Dk2 stages only, respective PID auto-balance is enabled so you can observe damping.
*/

/* ==================== DEBUG SWITCHES ====================

  HOW TO USE:
  - Uncomment a #define to enable its log stream.
  - Logs are rate-limited to keep serial traffic readable.

  FLAGS:
    DEBUG_CALIBRATION
      • Prints live pot center and error (constrained to S2S_POT_MIN/MAX).
      • Prints IMU pitch/roll with offsets applied.
      • Useful when calibrating potOffset, pitchOffset, rollOffset, and verifying IMU connection.
      • Frequency: 1 Hz (via printDebugInfo()).

    DEBUG_JOYSTICK
      • Prints drive controller right stick X/Y after deadzone filtering.
      • Helps confirm controller inputs and deadzone behavior.
      • Frequency: 1 Hz.

    DEBUG_S2S_MODE
      • Prints current S2S mode: "JOY MANUAL", "PID BALANCE", or "POT RETURN".
      • Also shows AutoBalance and DriveEnabled toggles.
      • Frequency: 1 Hz.

    DEBUG_MOTOR_OUTPUT
      • Prints S2S PWM and direction pins (S2S_PIN_1, S2S_PIN_2) and a slot for Drive PWM.
      • Useful when validating motor control mapping and gating.
      • Frequency: 1 Hz.

    DEBUG_IMU_RAW
      • Prints raw pitch/roll coming from IMU (no offsets, no deadzone).
      • Use this to verify sensor noise characteristics and offsets separately.
      • Frequency: 1 Hz.

    DEBUG_ALL
      • Convenience macro to enable ALL of the above 1 Hz streams at once.
      • Does not affect DEBUG_COMPACT (see below).

    DEBUG_COMPACT
      • Enables a compact, high-cadence (default ~10 Hz) single-line status:
        T|JOY_X|POT_FILT|ROLL|PID_OUT|S2S_PWM|DIRA|DIRB|MODE [+BAL] [+SAVE?] [+RESET?]
      • Minimal overhead, good for live tuning and quick inspections.
      • Controlled inside mainLoop() and rate-limited by COMPACT_DEBUG_INTERVAL.

  RECOMMENDED COMBOS:
    - During calibration: DEBUG_CALIBRATION + DEBUG_COMPACT
    - During PID tuning:  DEBUG_S2S_MODE + DEBUG_MOTOR_OUTPUT + DEBUG_COMPACT
    - Controller sanity:  DEBUG_JOYSTICK + DEBUG_COMPACT
*/

// #define DEBUG_CALIBRATION
// #define DEBUG_JOYSTICK
// #define DEBUG_S2S_MODE
// #define DEBUG_MOTOR_OUTPUT
// #define DEBUG_IMU_RAW
#define DEBUG_ALL
// #define DEBUG_COMPACT

#include <Arduino.h>
#include <EasyTransfer.h>
#include <analogWrite.h>
#include <Preferences.h>
#include <PSController.h>
#include <Ticker.h>
#include <PID_v1.h>

/* ------------------- CONFIG ------------------- */
const bool ENABLE_ESPNOW = false;
const bool REVERSE_S2S   = true;
const int  MAX_S2S_TILT  = 300;
const int  JOYSTICK_DEADZONE = 25;
const int  S2S_POT_MIN   = 1300;
const int  S2S_POT_MAX   = 2150;
const int  POT_FILTER_SIZE = 10;
const char* MASTER_NAV   = "7c:9e:bd:d7:63:c6";

/* ----- RETURN-TO-CENTER TUNING ----- */
const int  RETURN_DEADBAND   = 25;
const int  MIN_RETURN_PWM    = 30;
const int  MAX_RETURN_PWM    = 180;
const bool ENABLE_ROLL_BIAS  = false;
const float ROLL_BIAS_GAIN   = 10.0;
const int defaultPOT         = 1708;

/* ----- PID GAINS (MUTABLE for tuning; persisted in Preferences) ----- */
// Pitch (Drive) PID — defaults; overridden by Preferences on boot
float KP_PITCH = 45.0f;
float KI_PITCH = 0.8f;
float KD_PITCH = 12.0f;

// Roll (S2S) PID — defaults; overridden by Preferences on boot
float Pk2 = 45.0f;
float Ik2 = 1.0f;
float Dk2 = 15.0f;

/* ----- IMU / Timing ----- */
const float IMU_DEADZONE   = 1.45;
const unsigned long IMU_TIMEOUT = 500;

/* ----- CALIBRATION HOLD ----- */
const unsigned long CALIB_HOLD_TIME = 3000;  // 3s
unsigned long calibHoldStart = 0;
bool calibSaveActive = false;
bool calibResetActive = false;

/* ------------------- PINS ------------------- */
const uint8_t S2S_PWM      = 33;
const uint8_t S2S_PIN_1    = 26;
const uint8_t S2S_PIN_2    = 25;
const uint8_t DRIVE_PWM    = 21;
const uint8_t DRIVE_PIN_1  = 4;
const uint8_t DRIVE_PIN_2  = 27;
const uint8_t S2S_POT_PIN  = 34;
const uint8_t FLYWHEEL_PWM = 15;
const uint8_t FLYWHEEL_PIN_A = 32;
const uint8_t FLYWHEEL_PIN_B = 14;

/* ------------------- STRUCTS ------------------- */
struct IMUData { float pitch, roll; };

struct Send32u4Data {
  bool   driveEnabled;
  int8_t domeSpin;
  bool   moveL3, moveR3;
  int8_t leftStickX, leftStickY;
  int8_t soundcmd, psiFlash;
  float  pitch, roll;
};

struct ControllerButtons {
  bool cross, circle, up, down, left, right, ps, l1, l3, r1;
  int8_t leftStickX, leftStickY, rightStickX, rightStickY;
  int8_t l2;
};

/* ------------------- SOUND SYSTEM ------------------- */
enum : uint16_t {
  SOUND_NONE     = 0,
  SOUND_RANDOM   = 9,    // request random 1..30
  SOUND_STARTUP  = 1,    // played once at setup (current config)
  SOUND_TOGGLE   = 60   // played on toggles (current config)
};
static inline uint16_t pickRandom1to30() { return (uint16_t) random(1, 31); }

uint16_t pendingSound = SOUND_NONE;
static inline void emitSound(uint16_t track) { if (track != SOUND_NONE) pendingSound = track; }

#define EDGE_PRESSED(cur, prev) ((prev) == false && (cur) == true)
struct DPad { bool up{false}, right{false}, down{false}, left{false}; };

/* ------------------- LIVE TUNING MODE ------------------- */
/* UX:
 *  - Hold PS+UP    3s (drive) → Pitch PID tuning: KP → KD → KI (store each with X)
 *  - Hold PS+RIGHT 3s (drive) → Roll PID tuning: Pk2 → Dk2 → Ik2 (store each with X)
 *  - Hold PS+DOWN  3s (drive) → CANCEL tuning (restore persisted gains)
 *  - While tuning: ignore joysticks, sounds, dome spin, flywheel; only D-pad UP/DOWN + X are processed.
 */
enum class TuningSession { NONE, PITCH_KP, PITCH_KD, PITCH_KI, ROLL_PK, ROLL_DK, ROLL_IK };
TuningSession tuning = TuningSession::NONE;

unsigned long tuningHoldStart = 0;
bool tuningHoldActive = false;

float  kpPitchWork = 45.0f, kdPitchWork = 0.0f, kiPitchWork = 0.0f;
float  pk2Work     = 45.0f, dk2Work     = 0.0f, ik2Work     = 0.0f;

const float KP_STEP = 1.0f, KD_STEP = 1.0f, KI_STEP = 0.1f;
const float PK2_STEP = 1.0f, DK2_STEP = 1.0f, IK2_STEP = 0.1f;
const float REDUCE_FACTOR = 0.25f;  // 25% reduction when storing KP/Pk2 (safety)

/* ------------------- GLOBALS ------------------- */
EasyTransfer recIMU, send32u4;
Preferences preferences;
PSController driveController(nullptr), domeController(nullptr);
Ticker mainLoopTicker;
IMUData receiveIMUData;
Send32u4Data sendTo32u4Data;
ControllerButtons buttonsL, buttonsR;
bool IMUconnected = false, controllerConnected = false;
bool drivecontrollerConnected = false, domecontrollerConnected = false;
bool DomeServoMode = false, enableDrive = false, reverseDrive = false, EnableFlywheel = false;
bool autoBalance = false;
float flywheel = 0;
unsigned long lastIMUMillis = 0;
int potOffset = 1708;
float pitchOffset = 0.0, rollOffset = 0.0;
int potValues[POT_FILTER_SIZE];
int potValueIndex = 0;

unsigned long lastS2STime = 0;
unsigned long lastCompactDebug = 0;
const unsigned long COMPACT_DEBUG_INTERVAL = 100;

/* S2S Debug */
int lastPwmS2S = 0;
enum class S2SMode { JOY, RTN, STOP };
S2SMode s2sMode = S2SMode::STOP;

/* PID internals */
float pitchIntegral = 0.0;
float prevPitchError = 0.0;
unsigned long lastPIDTime = 0;
const float INTEGRAL_LIMIT = 100.0;

double Setpoint2 = 0.0;     // target roll = 0°
double Input2 = 0.0;        // current roll
double Output2 = 0.0;       // PID output (-255..+255)
double Output2_S2S_pwm = 0;

PID PID2_S2S(&Input2, &Output2, &Setpoint2, Pk2, Ik2, Dk2, DIRECT);

/* ------------------- FORWARD DECLS ------------------- */
void mainLoop();
void handleButtonActions();
void receiveIMU();
void sendDataTo32u4();
void S2S_Movement();
void drive_Movement();
void spinFlywheel();
void printDebugInfo();
void handleCalibrationHold();
void saveCalibration();
void resetCalibration();
void updateControllerStates();
void applyDeadzone(int8_t& x, int8_t& y);
int  filterPotValue(int raw);

// Live tuning helpers
void handleTuningHold();     // detect PS+UP / PS+RIGHT (start), PS+DOWN (cancel)
void handleLiveTuning();     // process UP/DOWN/X during active tuning
void printTuningBanner();    // one-time banner for tuning start
void cancelLiveTuning(bool announce = true);

// Sound mapping
static uint16_t resolveDriveControllerSound(const ControllerButtons& cur, const ControllerButtons& prev);
static uint16_t resolveDomeControllerSound (const ControllerButtons& cur, const ControllerButtons& prev);

static void printActiveDebugFlagsOnce() {
  Serial.print("DEBUG FLAGS: ");
#ifdef DEBUG_ALL
  Serial.print("ALL ");
#endif
#ifdef DEBUG_COMPACT
  Serial.print("COMPACT ");
#endif
#ifdef DEBUG_CALIBRATION
  Serial.print("CALIBRATION ");
#endif
#ifdef DEBUG_JOYSTICK
  Serial.print("JOYSTICK ");
#endif
#ifdef DEBUG_S2S_MODE
  Serial.print("S2S_MODE ");
#endif
#ifdef DEBUG_MOTOR_OUTPUT
  Serial.print("MOTOR_OUTPUT ");
#endif
#ifdef DEBUG_IMU_RAW
  Serial.print("IMU_RAW ");
#endif
#if !defined(DEBUG_ALL) && !defined(DEBUG_COMPACT) && !defined(DEBUG_CALIBRATION) && \
    !defined(DEBUG_JOYSTICK) && !defined(DEBUG_S2S_MODE) && !defined(DEBUG_MOTOR_OUTPUT) && \
    !defined(DEBUG_IMU_RAW)
  Serial.print("(none)");
#endif
  Serial.println();
}

/* ------------------- SETUP ------------------- */
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);                    // IMU serial
  Serial2.begin(74880, SERIAL_8N1, 13, 12); // 32u4 serial
  preferences.begin("JoeDriveV2", false);

  randomSeed((uint32_t)micros());

  // Load persisted calibration offsets
  potOffset   = preferences.getInt("potOffset", defaultPOT);
  pitchOffset = preferences.getFloat("pitchOffset", 0.0f);
  rollOffset  = preferences.getFloat("rollOffset", 0.0f);

  // Load persisted PID gains (fallback to defaults if not present)
  KP_PITCH = preferences.getFloat("kp_pitch", KP_PITCH);
  KI_PITCH = preferences.getFloat("ki_pitch", KI_PITCH);
  KD_PITCH = preferences.getFloat("kd_pitch", KD_PITCH);
  Pk2      = preferences.getFloat("pk2", Pk2);
  Ik2      = preferences.getFloat("ik2", Ik2);
  Dk2      = preferences.getFloat("dk2", Dk2);
  PID2_S2S.SetTunings(Pk2, Ik2, Dk2);   // apply roll PID gains

  Serial.printf("T: %lu INFO: Loaded Cal - Pot:%d Pitch:%.2f Roll:%.2f | PID[P]=%.2f/%.2f/%.2f | S2S[Pk2]=%.2f/%.2f/%.2f\n",
                millis(), potOffset, pitchOffset, rollOffset,
                KP_PITCH, KI_PITCH, KD_PITCH, Pk2, Ik2, Dk2);

  // Prime pot moving-average
  for (int i = 0; i < POT_FILTER_SIZE; i++) potValues[i] = potOffset;

  // IO setup
  pinMode(S2S_PWM, OUTPUT);
  pinMode(S2S_PIN_1, OUTPUT);
  pinMode(S2S_PIN_2, OUTPUT);
  pinMode(DRIVE_PWM, OUTPUT);
  pinMode(DRIVE_PIN_1, OUTPUT);
  pinMode(DRIVE_PIN_2, OUTPUT);
  pinMode(FLYWHEEL_PWM, OUTPUT);
  pinMode(FLYWHEEL_PIN_A, OUTPUT);
  pinMode(FLYWHEEL_PIN_B, OUTPUT);
  pinMode(S2S_POT_PIN, INPUT);

  // PS controllers
  if (!PSController::startListening(MASTER_NAV))
    Serial.printf("T: %lu BT failed\n", millis());
  else
    Serial.printf("T: %lu BT MAC: %s\n", millis(), PSController::getDeviceAddress().c_str());

  // EasyTransfer
  recIMU.begin(details(receiveIMUData), &Serial1);
  send32u4.begin(details(sendTo32u4Data), &Serial2);

  // Quick IMU probe
  for (int i = 0; i < 10000; i++) {
    if (recIMU.receiveData()) { IMUconnected = true; lastIMUMillis = millis(); break; }
    delay(1);
  }

  // S2S PID
  PID2_S2S.SetMode(AUTOMATIC);
  PID2_S2S.SetOutputLimits(-255, 255);
  PID2_S2S.SetSampleTime(10);

  lastPIDTime = millis();

  // 10 ms heartbeat
  mainLoopTicker.attach_ms(10, mainLoop);

  // Startup sound once
  emitSound(SOUND_STARTUP);
  sendDataTo32u4(); // one immediate frame

  Serial.println("JOE'S DRIVE V8.9b — ESP32 MASTER — SER — FULLY READY");
  printActiveDebugFlagsOnce();
}

/* ------------------- MAIN LOOP (Ticker @10ms) ------------------- */
void mainLoop() {
  receiveIMU();
  updateControllerStates();

  // Detect live tuning session holds (PS+UP / PS+RIGHT to start, PS+DOWN to cancel)
  handleTuningHold();

  if (tuning != TuningSession::NONE) {
    // While tuning, process only UP/DOWN/X on drive controller; ignore all other features
    handleLiveTuning();

    // Neutralize outputs while tuning (no drive, dome, sounds)
    enableDrive = false;
    sendTo32u4Data.driveEnabled = false;
    sendTo32u4Data.domeSpin = 0;
    sendTo32u4Data.moveL3 = false;
    sendTo32u4Data.moveR3 = false;
    pendingSound = SOUND_NONE;
    sendTo32u4Data.soundcmd = 0;

    // Allow S2S roll PID movement only during ROLL_DK / ROLL_IK stages
    bool allowS2SForRollTuning = (tuning == TuningSession::ROLL_DK || tuning == TuningSession::ROLL_IK);
    if (allowS2SForRollTuning) {
      // Auto-balance roll so user can visually see damping while adjusting Dk2/Ik2
      autoBalance = true;
      S2S_Movement();
    } else {
      // Force S2S idle
      digitalWrite(S2S_PIN_1, LOW);
      digitalWrite(S2S_PIN_2, LOW);
      analogWrite(S2S_PWM, 0);
      lastPwmS2S = 0;
    }

    // Drive is disabled during tuning
    digitalWrite(DRIVE_PIN_1, LOW);
    digitalWrite(DRIVE_PIN_2, LOW);
    analogWrite(DRIVE_PWM, 0);

    // Still transmit IMU + neutral control state
    sendDataTo32u4();

  } else {
    // Normal operation
    handleCalibrationHold();
    handleButtonActions();
    S2S_Movement();
    drive_Movement();
    spinFlywheel();
    sendDataTo32u4();
    printDebugInfo();

#ifdef DEBUG_COMPACT
    if (millis() - lastCompactDebug >= COMPACT_DEBUG_INTERVAL) {
      lastCompactDebug = millis();
      int rawPot      = analogRead(S2S_POT_PIN);
      int filteredPot = filterPotValue(rawPot);
      filteredPot     = constrain(filteredPot, S2S_POT_MIN, S2S_POT_MAX);
      const char* modeStr = (s2sMode == S2SMode::JOY) ? "JOY" :
                            (s2sMode == S2SMode::RTN) ? "RTN" : "STOP";
      Serial.printf(
        "T:%lu | JOY_X:%4d | POT_FILT:%4d | ROLL:%6.2f | PID_OUT:%6.1f | S2S:%3d | DIR:%d%d | MODE:%s%s%s%s\r\n",
        millis(),
        (int)buttonsR.rightStickX,
        filteredPot,
        receiveIMUData.roll,
        Output2,
        lastPwmS2S,
        digitalRead(S2S_PIN_1), digitalRead(S2S_PIN_2),
        modeStr,
        autoBalance ? " [BAL]" : "",
        calibSaveActive ? " [SAVE?]" : "",
        calibResetActive ? " [RESET?]" : ""
      );
    }
#endif
  }
}

/* ------------------- LIVE TUNING: HOLD DETECTION ------------------- */
void handleTuningHold() {
  unsigned long now = millis();

  // ---- CANCEL current tuning: PS + DOWN (drive) held for 3s ----
  if (tuning != TuningSession::NONE && buttonsR.ps && buttonsR.down) {
    if (!tuningHoldActive) { tuningHoldActive = true; tuningHoldStart = now; }
    if (tuningHoldActive && now - tuningHoldStart >= CALIB_HOLD_TIME) {
      cancelLiveTuning(true);
      return; // done
    }
  } else if (tuning != TuningSession::NONE) {
    // tuning active but cancel chord not held → reset hold tracker
    tuningHoldActive = false;
  }

  // ---- Start new tuning sessions (only when NOT already tuning) ----
  if (tuning == TuningSession::NONE) {
    // Pitch session: PS+UP held 3s
    if (buttonsR.ps && buttonsR.up) {
      if (!tuningHoldActive) { tuningHoldActive = true; tuningHoldStart = now; }
      if (tuningHoldActive && now - tuningHoldStart >= CALIB_HOLD_TIME) {
        kpPitchWork = 45.0f; kdPitchWork = 0.0f; kiPitchWork = 0.0f;
        KP_PITCH = kpPitchWork; KD_PITCH = kdPitchWork; KI_PITCH = kiPitchWork;
        autoBalance = false;         // off to observe pure KP effects first
        tuning = TuningSession::PITCH_KP;
        printTuningBanner();
        Serial.println(F("[TUNE] Pitch: Stage 1 (KP) — UP/DOWN adjust, X to store (25% reduction)"));
        tuningHoldActive = false;
      }
    }
    // Roll session: PS+RIGHT held 3s
    else if (buttonsR.ps && buttonsR.right) {
      if (!tuningHoldActive) { tuningHoldActive = true; tuningHoldStart = now; }
      if (tuningHoldActive && now - tuningHoldStart >= CALIB_HOLD_TIME) {
        pk2Work = 45.0f; dk2Work = 0.0f; ik2Work = 0.0f;
        Pk2 = pk2Work; Dk2 = dk2Work; Ik2 = ik2Work;
        PID2_S2S.SetTunings(Pk2, Ik2, Dk2);
        tuning = TuningSession::ROLL_PK;
        printTuningBanner();
        Serial.println(F("[TUNE] Roll: Stage 1 (Pk2) — UP/DOWN adjust, X to store (25% reduction)"));
        tuningHoldActive = false;
      }
    } else {
      tuningHoldActive = false;
    }
  }
}

/* ------------------- LIVE TUNING: PROCESS BUTTONS ------------------- */
void handleLiveTuning() {
  // Edge detection — track previous drive buttons exclusively for tuning
  static ControllerButtons prevTuneR{};
  // Freeze joysticks during tuning
  buttonsR.rightStickX = 0;
  buttonsR.rightStickY = 0;

  // Short helpers
  bool upPressed   = EDGE_PRESSED(buttonsR.up,    prevTuneR.up);
  bool downPressed = EDGE_PRESSED(buttonsR.down,  prevTuneR.down);
  bool xPressed    = EDGE_PRESSED(buttonsR.cross, prevTuneR.cross);

  switch (tuning) {
    case TuningSession::PITCH_KP:
      if (upPressed)   { kpPitchWork += KP_STEP; KP_PITCH = kpPitchWork; Serial.printf("[TUNE] KP %.2f\n", KP_PITCH); }
      if (downPressed) { kpPitchWork = max(0.0f, kpPitchWork - KP_STEP); KP_PITCH = kpPitchWork; Serial.printf("[TUNE] KP %.2f\n", KP_PITCH); }
      if (xPressed) {
        KP_PITCH = kpPitchWork * (1.0f - REDUCE_FACTOR);
        preferences.putFloat("kp_pitch", KP_PITCH);
        Serial.printf("[TUNE] KP STORED = %.2f (reduced 25%% from %.2f)\n", KP_PITCH, kpPitchWork);
        // Next stage: KD with autoBalance ON
        kdPitchWork = 0.0f; KD_PITCH = kdPitchWork; preferences.putFloat("kd_pitch", KD_PITCH);
        autoBalance = true;
        Serial.println(F("[TUNE] Pitch: Stage 2 (KD) — UP/DOWN to adjust, X to store"));
        tuning = TuningSession::PITCH_KD;
      }
      break;

    case TuningSession::PITCH_KD:
      if (upPressed)   { kdPitchWork += KD_STEP; KD_PITCH = kdPitchWork; Serial.printf("[TUNE] KD %.2f\n", KD_PITCH); }
      if (downPressed) { kdPitchWork = max(0.0f, kdPitchWork - KD_STEP); KD_PITCH = kdPitchWork; Serial.printf("[TUNE] KD %.2f\n", KD_PITCH); }
      if (xPressed) {
        preferences.putFloat("kd_pitch", KD_PITCH);
        Serial.printf("[TUNE] KD STORED = %.2f\n", KD_PITCH);
        // Next stage: KI
        kiPitchWork = 0.0f; KI_PITCH = kiPitchWork; preferences.putFloat("ki_pitch", KI_PITCH);
        Serial.println(F("[TUNE] Pitch: Stage 3 (KI) — UP/DOWN to adjust (0.1), X to store & exit"));
        tuning = TuningSession::PITCH_KI;
      }
      break;

    case TuningSession::PITCH_KI:
      if (upPressed)   { kiPitchWork += KI_STEP; KI_PITCH = kiPitchWork; Serial.printf("[TUNE] KI %.2f\n", KI_PITCH); }
      if (downPressed) { kiPitchWork = max(0.0f, kiPitchWork - KI_STEP); KI_PITCH = kiPitchWork; Serial.printf("[TUNE] KI %.2f\n", KI_PITCH); }
      if (xPressed) {
        preferences.putFloat("ki_pitch", KI_PITCH);
        Serial.printf("[TUNE] KI STORED = %.2f\n", KI_PITCH);
        Serial.println(F("[TUNE] Pitch tuning complete — exiting live tuning."));
        tuning = TuningSession::NONE;
      }
      break;

    case TuningSession::ROLL_PK:
      if (upPressed)   { pk2Work += PK2_STEP; Pk2 = pk2Work; PID2_S2S.SetTunings(Pk2, Ik2, Dk2); Serial.printf("[TUNE] Pk2 %.2f\n", Pk2); }
      if (downPressed) { pk2Work = max(0.0f, pk2Work - PK2_STEP); Pk2 = pk2Work; PID2_S2S.SetTunings(Pk2, Ik2, Dk2); Serial.printf("[TUNE] Pk2 %.2f\n", Pk2); }
      if (xPressed) {
        Pk2 = pk2Work * (1.0f - REDUCE_FACTOR);
        preferences.putFloat("pk2", Pk2);
        PID2_S2S.SetTunings(Pk2, Ik2, Dk2);
        Serial.printf("[TUNE] Pk2 STORED = %.2f (reduced 25%% from %.2f)\n", Pk2, pk2Work);
        // Next stage: Dk2 with S2S auto-balance ON
        dk2Work = 0.0f; Dk2 = dk2Work; preferences.putFloat("dk2", Dk2);
        PID2_S2S.SetTunings(Pk2, Ik2, Dk2);
        Serial.println(F("[TUNE] Roll: Stage 2 (Dk2) — UP/DOWN to adjust, X to store"));
        tuning = TuningSession::ROLL_DK;
      }
      break;

    case TuningSession::ROLL_DK:
      if (upPressed)   { dk2Work += DK2_STEP; Dk2 = dk2Work; preferences.putFloat("dk2", Dk2); PID2_S2S.SetTunings(Pk2, Ik2, Dk2); Serial.printf("[TUNE] Dk2 %.2f\n", Dk2); }
      if (downPressed) { dk2Work = max(0.0f, dk2Work - DK2_STEP); Dk2 = dk2Work; preferences.putFloat("dk2", Dk2); PID2_S2S.SetTunings(Pk2, Ik2, Dk2); Serial.printf("[TUNE] Dk2 %.2f\n", Dk2); }
      if (xPressed) {
        preferences.putFloat("dk2", Dk2);
        PID2_S2S.SetTunings(Pk2, Ik2, Dk2);
        Serial.printf("[TUNE] Dk2 STORED = %.2f\n", Dk2);
        // Next: Ik2
        ik2Work = 0.0f; Ik2 = ik2Work; preferences.putFloat("ik2", Ik2);
        PID2_S2S.SetTunings(Pk2, Ik2, Dk2);
        Serial.println(F("[TUNE] Roll: Stage 3 (Ik2) — UP/DOWN to adjust (0.1), X to store & exit"));
        tuning = TuningSession::ROLL_IK;
      }
      break;

    case TuningSession::ROLL_IK:
      if (upPressed)   { ik2Work += IK2_STEP; Ik2 = ik2Work; preferences.putFloat("ik2", Ik2); PID2_S2S.SetTunings(Pk2, Ik2, Dk2); Serial.printf("[TUNE] Ik2 %.2f\n", Ik2); }
      if (downPressed) { ik2Work = max(0.0f, ik2Work - IK2_STEP); Ik2 = ik2Work; preferences.putFloat("ik2", Ik2); PID2_S2S.SetTunings(Pk2, Ik2, Dk2); Serial.printf("[TUNE] Ik2 %.2f\n", Ik2); }
      if (xPressed) {
        preferences.putFloat("ik2", Ik2);
        PID2_S2S.SetTunings(Pk2, Ik2, Dk2);
        Serial.printf("[TUNE] Ik2 STORED = %.2f\n", Ik2);
        Serial.println(F("[TUNE] Roll tuning complete — exiting live tuning."));
        tuning = TuningSession::NONE;
      }
      break;

    default: break;
  }

  // Update previous for edge detection
  prevTuneR = buttonsR;
}

void printTuningBanner() {
  Serial.println(F("\n================ LIVE TUNING MODE ================"));
  Serial.println(F("Controls (Drive controller):"));
  Serial.println(F("  • Hold PS+UP    3s → Pitch PID (KP → KD → KI)"));
  Serial.println(F("  • Hold PS+RIGHT 3s → Roll PID  (Pk2 → Dk2 → Ik2)"));
  Serial.println(F("  • UP/DOWN           → adjust current gain"));
  Serial.println(F("  • X                 → store & advance to next stage"));
  Serial.println(F("  • Hold PS+DOWN  3s  → CANCEL tuning (restore persisted gains)"));
  Serial.println(F("  • During tuning: joystick, sounds, dome, flywheel disabled"));
  Serial.println(F("=================================================\n"));
}

void cancelLiveTuning(bool announce) {
  // Reload persisted gains (fallback to current if missing)
  KP_PITCH = preferences.getFloat("kp_pitch", KP_PITCH);
  KD_PITCH = preferences.getFloat("kd_pitch", KD_PITCH);
  KI_PITCH = preferences.getFloat("ki_pitch", KI_PITCH);

  Pk2      = preferences.getFloat("pk2", Pk2);
  Dk2      = preferences.getFloat("dk2", Dk2);
  Ik2      = preferences.getFloat("ik2", Ik2);
  PID2_S2S.SetTunings(Pk2, Ik2, Dk2);  // apply roll PID

  // Exit tuning and disable auto-balance
  tuning = TuningSession::NONE;
  tuningHoldActive = false;
  autoBalance = false;

  // Neutralize outputs immediately (mainLoop also gates)
  digitalWrite(S2S_PIN_1, LOW);
  digitalWrite(S2S_PIN_2, LOW);
  analogWrite(S2S_PWM, 0);
  digitalWrite(DRIVE_PIN_1, LOW);
  digitalWrite(DRIVE_PIN_2, LOW);
  analogWrite(DRIVE_PWM, 0);

  if (announce) {
    Serial.println(F("[TUNE] CANCELED — restored persisted PID gains and exited live tuning."));
    Serial.printf("[TUNE] Restored Pitch PID: KP=%.2f KD=%.2f KI=%.2f | Roll PID: Pk2=%.2f Dk2=%.2f Ik2=%.2f\n",
                  KP_PITCH, KD_PITCH, KI_PITCH, Pk2, Dk2, Ik2);
  }
}

/* ------------------- 3-SEC CALIBRATION HOLD ------------------- */
void handleCalibrationHold() {
  bool bothUp   = buttonsL.up && buttonsR.up;
  bool bothDown = buttonsL.down && buttonsR.down;

  unsigned long now = millis();

  // SAVE (UP+UP)
  if (bothUp && !calibSaveActive) {
    calibHoldStart = now;
    calibSaveActive = true;
    calibResetActive = false;
  } else if (!bothUp && calibSaveActive) {
    calibSaveActive = false;
  }
  if (calibSaveActive && (now - calibHoldStart >= CALIB_HOLD_TIME)) {
    saveCalibration();
    calibSaveActive = false;
  }

  // RESET (DOWN+DOWN)
  if (bothDown && !calibResetActive) {
    calibHoldStart = now;
    calibResetActive = true;
    calibSaveActive = false;
  } else if (!bothDown && calibResetActive) {
    calibResetActive = false;
  }
  if (calibResetActive && (now - calibHoldStart >= CALIB_HOLD_TIME)) {
    resetCalibration();
    calibResetActive = false;
  }
}

/* ------------------- SAVE / RESET ------------------- */
void saveCalibration() {
  int pot = filterPotValue(analogRead(S2S_POT_PIN));
  potOffset = pot;
  pitchOffset = receiveIMUData.pitch;
  rollOffset  = receiveIMUData.roll;

  preferences.putInt("potOffset", potOffset);
  preferences.putFloat("pitchOffset", pitchOffset);
  preferences.putFloat("rollOffset", rollOffset);

  Serial.printf("T: %lu INFO: CAL SAVED → Pot:%d Pitch:%.2f Roll:%.2f\n",
                millis(), potOffset, pitchOffset, rollOffset);
}

void resetCalibration() {
  potOffset = 0;
  pitchOffset = 0.0f;
  rollOffset = 0.0f;

  preferences.putInt("potOffset", 0);
  preferences.putFloat("pitchOffset", 0.0f);
  preferences.putFloat("rollOffset", 0.0f);

  Serial.printf("T: %lu INFO: CAL RESET → All offsets set to 0\n", millis());
}

/* ------------------- CONTROLLER ------------------- */
void updateControllerStates() {
  drivecontrollerConnected = driveController.isConnected();
  domecontrollerConnected  = domeController.isConnected();
  controllerConnected      = drivecontrollerConnected || domecontrollerConnected;

  if (controllerConnected) {
    // Drive controller (right)
    buttonsR.l1 = driveController.state.button.l1;
    buttonsR.r1 = driveController.state.button.r1;
    buttonsR.l2 = constrain(map(driveController.state.analog.button.l2, 0, 255, 0, 100), 0, 100);
    buttonsR.l3 = driveController.state.button.l3;
    buttonsR.cross = driveController.state.button.cross;
    buttonsR.circle = driveController.state.button.circle;
    buttonsR.up = driveController.state.button.up;
    buttonsR.down = driveController.state.button.down;
    buttonsR.left = driveController.state.button.left;
    buttonsR.right = driveController.state.button.right;
    buttonsR.ps = driveController.state.button.ps;
    buttonsR.rightStickX = driveController.state.analog.stick.lx;
    buttonsR.rightStickY = driveController.state.analog.stick.ly;
    applyDeadzone(buttonsR.rightStickX, buttonsR.rightStickY);

    // Dome controller (left)
    buttonsL.l1 = domeController.state.button.l1;
    buttonsL.l2 = constrain(map(domeController.state.analog.button.l2, 0, 255, 0, 100), 0, 100);
    buttonsL.l3 = domeController.state.button.l3;
    buttonsL.cross = domeController.state.button.cross;
    buttonsL.circle = domeController.state.button.circle;
    buttonsL.up = domeController.state.button.up;
    buttonsL.down = domeController.state.button.down;
    buttonsL.left = domeController.state.button.left;
    buttonsL.right = domeController.state.button.right;
    buttonsL.ps = domeController.state.button.ps;
    buttonsL.leftStickX = domeController.state.analog.stick.lx;
    buttonsL.leftStickY = domeController.state.analog.stick.ly;
    applyDeadzone(buttonsL.leftStickX, buttonsL.leftStickY);
  }
}

void applyDeadzone(int8_t& x, int8_t& y) {
  x = (abs(x) > JOYSTICK_DEADZONE) ? x : 0;
  y = (abs(y) > JOYSTICK_DEADZONE) ? y : 0;
}

int filterPotValue(int raw) {
  potValues[potValueIndex] = raw;
  potValueIndex = (potValueIndex + 1) % POT_FILTER_SIZE;
  long sum = 0;
  for (int i = 0; i < POT_FILTER_SIZE; i++) sum += potValues[i];
  return sum / POT_FILTER_SIZE;
}

/* ------------------- SOUND MAPPING HELPERS ------------------- */
static uint16_t resolveDriveControllerSound(const ControllerButtons& cur, const ControllerButtons& prev) {
  DPad d; d.up = cur.up; d.right = cur.right; d.down = cur.down; d.left = cur.left;

  // L1 + D-pad combos (21..24)
  if (cur.l1) {
    if (EDGE_PRESSED(d.up,    prev.up))    return 21;
    if (EDGE_PRESSED(d.right, prev.right)) return 22;
    if (EDGE_PRESSED(d.down,  prev.down))  return 23;
    if (EDGE_PRESSED(d.left,  prev.left))  return 24;
  }
  // R1 + D-pad combos (25..28)
  if (cur.r1) {
    if (EDGE_PRESSED(d.up,    prev.up))    return 25;
    if (EDGE_PRESSED(d.right, prev.right)) return 26;
    if (EDGE_PRESSED(d.down,  prev.down))  return 27;
    if (EDGE_PRESSED(d.left,  prev.left))  return 28;
  }
  // Single D-pad (1..4)
  if (EDGE_PRESSED(cur.up,    prev.up))    return 1;
  if (EDGE_PRESSED(cur.right, prev.right)) return 2;
  if (EDGE_PRESSED(cur.down,  prev.down))  return 3;
  if (EDGE_PRESSED(cur.left,  prev.left))  return 4;

  // L2 analog threshold
  if (cur.l2 > 60 && prev.l2 <= 60)        return 50;

  // Circle → random 1..30
  if (EDGE_PRESSED(cur.circle, prev.circle)) return pickRandom1to30();

  return SOUND_NONE;
}

static uint16_t resolveDomeControllerSound(const ControllerButtons& cur, const ControllerButtons& prev) {
  DPad d; d.up = cur.up; d.right = cur.right; d.down = cur.down; d.left = cur.left;

  // L1 + D-pad combos (11..14)
  if (cur.l1) {
    if (EDGE_PRESSED(d.up,    prev.up))    return 11;
    if (EDGE_PRESSED(d.right, prev.right)) return 12;
    if (EDGE_PRESSED(d.down,  prev.down))  return 13;
    if (EDGE_PRESSED(d.left,  prev.left))  return 14;
  }
  // Single D-pad (5..8)
  if (EDGE_PRESSED(cur.up,    prev.up))    return 5;
  if (EDGE_PRESSED(cur.right, prev.right)) return 6;
  if (EDGE_PRESSED(cur.down,  prev.down))  return 7;
  if (EDGE_PRESSED(cur.left,  prev.left))  return 8;

  // L2 analog threshold
  if (cur.l2 > 60 && prev.l2 <= 60)        return 99;

  // Circle → random 1..30
  if (EDGE_PRESSED(cur.circle, prev.circle)) return pickRandom1to30();

  return SOUND_NONE;
}

/* ------------------- BUTTONS + SOUNDS (NORMAL MODE) ------------------- */
void handleButtonActions() {
  if (!controllerConnected) return;
  if (tuning != TuningSession::NONE) return; // ignore normal actions during tuning

  static ControllerButtons prevR{}, prevL{};

  // Dome spin vs. Flywheel L1 gating
  if (buttonsR.l1) { flywheel = 0; sendTo32u4Data.domeSpin = buttonsR.rightStickX; buttonsR.rightStickY = 0; }
  else if (buttonsL.l1) { sendTo32u4Data.domeSpin = 0; EnableFlywheel = true; }
  else { sendTo32u4Data.domeSpin = 0; EnableFlywheel = false; }

  // Sounds (edge-triggered)
  uint16_t soundR = resolveDriveControllerSound(buttonsR, prevR);
  uint16_t soundL = resolveDomeControllerSound(buttonsL, prevL);
  uint16_t chosen = (soundR != SOUND_NONE) ? soundR : soundL;
  if (chosen != SOUND_NONE) emitSound(chosen);

  // Toggles (emit toggle sound)
  if (EDGE_PRESSED(buttonsR.ps,  prevR.ps)) { enableDrive = !enableDrive; sendTo32u4Data.driveEnabled = enableDrive; emitSound(SOUND_TOGGLE); }
  if (EDGE_PRESSED(buttonsL.l3,  prevL.l3)) { DomeServoMode = !DomeServoMode; sendTo32u4Data.moveR3 = DomeServoMode; emitSound(SOUND_TOGGLE); }
  if (EDGE_PRESSED(buttonsR.l3,  prevR.l3)) { reverseDrive = !reverseDrive; sendTo32u4Data.moveL3 = reverseDrive; emitSound(SOUND_TOGGLE); }
  if (EDGE_PRESSED(buttonsR.cross, prevR.cross)) { autoBalance = !autoBalance; Serial.printf("T: %lu Auto Balance %s\n", millis(), autoBalance ? "ON" : "OFF"); emitSound(SOUND_TOGGLE); }

  // Dome tilt stick to 32u4
  sendTo32u4Data.leftStickX = buttonsL.leftStickX;
  sendTo32u4Data.leftStickY = buttonsL.leftStickY;

  prevR = buttonsR; prevL = buttonsL;
}

/* ------------------- IMU ------------------- */
void receiveIMU() {
  unsigned long now = millis();
  if (recIMU.receiveData()) {
    float rawPitch = receiveIMUData.pitch - pitchOffset;
    float rawRoll  = receiveIMUData.roll  - rollOffset;

    sendTo32u4Data.pitch = abs(rawPitch) > IMU_DEADZONE ? rawPitch : 0;
    sendTo32u4Data.roll  = abs(rawRoll)  > IMU_DEADZONE ? rawRoll  : 0;

    IMUconnected = true;
    lastIMUMillis = now;
  } else if (now - lastIMUMillis > IMU_TIMEOUT) {
    IMUconnected = false;
    if (autoBalance) { autoBalance = false; Serial.printf("T: %lu IMU Lost → Balance OFF\n", now); }
  }
}

/* ------------------- SEND TO 32u4 (one-shot sound) ------------------- */
void sendDataTo32u4() {
  sendTo32u4Data.soundcmd = (int8_t)pendingSound;
  send32u4.sendData();
  pendingSound = SOUND_NONE;
}

/* ------------------- S2S MOVEMENT ------------------- */
void S2S_Movement() {
  unsigned long now = millis();
  float dt = (lastS2STime > 0) ? (now - lastS2STime) / 1000.0f : 0.01f;
  lastS2STime = now;

  int rawPot   = analogRead(S2S_POT_PIN);
  int potValue = filterPotValue(rawPot);
  potValue     = constrain(potValue, S2S_POT_MIN, S2S_POT_MAX);

  int joyX = buttonsR.rightStickX;

  int pwm = 0;
  bool dir1 = false, dir2 = false;
  s2sMode = S2SMode::STOP;

  if (abs(joyX) > JOYSTICK_DEADZONE && tuning == TuningSession::NONE) {
    // Manual joystick S2S
    pwm = map(abs(joyX), 0, 127, 0, 255);
    dir1 = (joyX < 0) ? REVERSE_S2S : !REVERSE_S2S;
    dir2 = (joyX < 0) ? !REVERSE_S2S : REVERSE_S2S;
    s2sMode = S2SMode::JOY;
    PID2_S2S.SetMode(MANUAL);
    Output2 = 0;
  } else {
    if (autoBalance && IMUconnected) {
      // PID BALANCE MODE (roll)
      Input2 = receiveIMUData.roll - rollOffset;
      if (abs(Input2) > IMU_DEADZONE) {
        PID2_S2S.SetTunings(Pk2, Ik2, Dk2); // ensure latest gains
        PID2_S2S.SetMode(AUTOMATIC);
        PID2_S2S.Compute();

        Output2_S2S_pwm = constrain(abs(Output2), MIN_RETURN_PWM, MAX_RETURN_PWM);
        pwm = (int)Output2_S2S_pwm;

        dir1 = (Output2 > 0) ? REVERSE_S2S : !REVERSE_S2S;
        dir2 = (Output2 > 0) ? !REVERSE_S2S : REVERSE_S2S;
        s2sMode = S2SMode::RTN;
      } else {
        PID2_S2S.SetMode(MANUAL);
        Output2 = 0;
      }
    } else {
      // Pot-based return to center
      int target = potOffset;
      if (ENABLE_ROLL_BIAS) target += (int)(sendTo32u4Data.roll * ROLL_BIAS_GAIN);
      int error = potValue - target;
      int candidate = constrain(abs(error) * 2, 0, MAX_RETURN_PWM);
      if (candidate >= MIN_RETURN_PWM && abs(error) > RETURN_DEADBAND) {
        pwm = candidate;
        dir1 = (error < 0) ? REVERSE_S2S : !REVERSE_S2S;
        dir2 = (error < 0) ? !REVERSE_S2S : REVERSE_S2S;
        s2sMode = S2SMode::RTN;
      }
    }
  }

  // Safety / gating
  if (controllerConnected && (tuning == TuningSession::NONE) && enableDrive && !buttonsL.l1 && !buttonsR.l1) {
    digitalWrite(S2S_PIN_1, dir1 ? HIGH : LOW);
    digitalWrite(S2S_PIN_2, dir2 ? HIGH : LOW);
    analogWrite(S2S_PWM, pwm);
    lastPwmS2S = pwm;
  } else if (tuning == TuningSession::ROLL_DK || tuning == TuningSession::ROLL_IK) {
    // During these tuning stages we allow S2S PID movement (already computed above)
    digitalWrite(S2S_PIN_1, dir1 ? HIGH : LOW);
    digitalWrite(S2S_PIN_2, dir2 ? HIGH : LOW);
    analogWrite(S2S_PWM, pwm);
    lastPwmS2S = pwm;
  } else {
    digitalWrite(S2S_PIN_1, LOW);
    digitalWrite(S2S_PIN_2, LOW);
    analogWrite(S2S_PWM, 0);
    lastPwmS2S = 0;
  }
}

/* ------------------- DRIVE + PID ------------------- */
void drive_Movement() {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0f;
  lastPIDTime = now;

  int joyY = reverseDrive ? -buttonsR.rightStickY : buttonsR.rightStickY;
  int baseSpeed = map(joyY, -127, 127, -255, 255);
  int finalSpeed = baseSpeed;

  if (autoBalance && IMUconnected && abs(buttonsR.rightStickY) <= JOYSTICK_DEADZONE) {
    float pitchError = receiveIMUData.pitch - pitchOffset;
    if (abs(pitchError) > IMU_DEADZONE) {
      float P = KP_PITCH * pitchError;
      pitchIntegral += pitchError * dt;
      pitchIntegral = constrain(pitchIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
      float I = KI_PITCH * pitchIntegral;
      float D = KD_PITCH * (pitchError - prevPitchError) / dt;
      prevPitchError = pitchError;
      finalSpeed += (int)(P + I + D);
    } else {
      pitchIntegral *= 0.95f;
    }
  } else {
    pitchIntegral = 0.0f;
    prevPitchError = 0.0f;
  }

  finalSpeed = constrain(finalSpeed, -255, 255);

  if (controllerConnected && (tuning == TuningSession::NONE) && enableDrive && abs(finalSpeed) > 5) {
    digitalWrite(DRIVE_PIN_1, finalSpeed < 0 ? HIGH : LOW);
    digitalWrite(DRIVE_PIN_2, finalSpeed < 0 ? LOW : HIGH);
    analogWrite(DRIVE_PWM, abs(finalSpeed));
  } else {
    digitalWrite(DRIVE_PIN_1, LOW);
    digitalWrite(DRIVE_PIN_2, LOW);
    analogWrite(DRIVE_PWM, 0);
  }
}

/* ------------------- FLYWHEEL ------------------- */
void spinFlywheel() {
  if (EnableFlywheel && (tuning == TuningSession::NONE) && enableDrive && buttonsL.l1) {
    flywheel = constrain(map(buttonsR.rightStickX, -128, 128,
                reverseDrive ? 255 : -255,
                reverseDrive ? -255 : 255), -255, 255);
    if (abs(flywheel) > 10) {
      digitalWrite(FLYWHEEL_PIN_A, flywheel > 0 ? HIGH : LOW);
      digitalWrite(FLYWHEEL_PIN_B, flywheel > 0 ? LOW : HIGH);
      analogWrite(FLYWHEEL_PWM, abs(flywheel));
    } else {
      digitalWrite(FLYWHEEL_PIN_A, LOW);
      digitalWrite(FLYWHEEL_PIN_B, LOW);
      analogWrite(FLYWHEEL_PWM, 0);
    }
  } else {
    digitalWrite(FLYWHEEL_PIN_A, LOW);
    digitalWrite(FLYWHEEL_PIN_B, LOW);
    analogWrite(FLYWHEEL_PWM, 0);
  }
}

/* ------------------- DEBUG ------------------- */
void printDebugInfo() {
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug < 1000) return;  // 1 Hz
  lastDebug = millis();

#ifdef DEBUG_ALL
  #define DEBUG_CALIBRATION
  #define DEBUG_JOYSTICK
  #define DEBUG_S2S_MODE
  #define DEBUG_MOTOR_OUTPUT
  #define DEBUG_IMU_RAW
#endif

#if defined(DEBUG_CALIBRATION) || defined(DEBUG_JOYSTICK) || defined(DEBUG_S2S_MODE) || defined(DEBUG_MOTOR_OUTPUT) || defined(DEBUG_IMU_RAW)
  Serial.printf("\n=== DEBUG T:%lu ===\n", millis());
#endif

#ifdef DEBUG_CALIBRATION
  int currentPot = filterPotValue(analogRead(S2S_POT_PIN));
  currentPot = constrain(currentPot, S2S_POT_MIN, S2S_POT_MAX);
  float adjPitch = receiveIMUData.pitch - pitchOffset;
  float adjRoll  = receiveIMUData.roll  - rollOffset;

  Serial.printf("POT: Curr=%4d  SavedCenter=%4d  Err=%+4d\n", 
                currentPot, potOffset, currentPot - potOffset);
  Serial.printf("IMU: Pitch=%+6.2f (offset=%+6.2f)  Roll=%+6.2f (offset=%+6.2f)  Conn=%s\n",
                adjPitch, pitchOffset, adjRoll, rollOffset, IMUconnected?"YES":"NO");
#endif

#ifdef DEBUG_JOYSTICK
  Serial.printf("JOY: X=%+4d  Y=%+4d  (Deadzone=%d)\n",
                buttonsR.rightStickX, buttonsR.rightStickY, JOYSTICK_DEADZONE);
#endif

#ifdef DEBUG_S2S_MODE
  const char* modeStr = 
    (s2sMode == S2SMode::JOY) ? "JOY MANUAL" :
    (autoBalance && IMUconnected && abs(receiveIMUData.roll - rollOffset) > IMU_DEADZONE) ? "PID BALANCE" :
    "POT RETURN";
  Serial.printf("S2S MODE: %s  |  AutoBal=%s  DriveEn=%s\n",
                modeStr, autoBalance?"ON":"OFF", enableDrive?"ON":"OFF");
#endif

#ifdef DEBUG_MOTOR_OUTPUT
  Serial.printf("MOTORS → S2S: PWM=%3d  DIR=%d%d  |  DRIVE: PWM=%3d\n",
                lastPwmS2S,
                digitalRead(S2S_PIN_1), digitalRead(S2S_PIN_2),
                /* Add drive PWM here if you track it */ 0);
#endif

#ifdef DEBUG_IMU_RAW
  Serial.printf("IMU RAW: Pitch=%+6.2f  Roll=%+6.2f\n",
                receiveIMUData.pitch, receiveIMUData.roll);
#endif

#if defined(DEBUG_CALIBRATION) || defined(DEBUG_JOYSTICK) || defined(DEBUG_S2S_MODE) || defined(DEBUG_MOTOR_OUTPUT) || defined(DEBUG_IMU_RAW)
  Serial.println("=====================================");
#endif
}

/* ------------------- LOOP ------------------- */
void loop() { delay(1); }
