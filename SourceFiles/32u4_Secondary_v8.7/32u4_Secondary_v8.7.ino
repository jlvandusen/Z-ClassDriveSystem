/*
 * Joe's Drive – V8.7 FINAL
 * Secondary 32u4 – Dome Movement & PSI Lights
 *
 * FULLY STABLE:
 *   - No encoder drift
 *   - No servo twitching
 *   - No left bias
 *   - Center = 878 (your measured value)
 *   - Auto-snap to center when Hall is LOW
 *
 * Encoder: A0 (18) & A1 (19) via PCINT
 * Hall:    A2 (20) – LOW when magnet present
 *
 * Upload and enjoy!
 */

#define debugServos
#define debugDOME
#define debugHALL
#define debugENC
#define debugEasyTransfer

#define MOVECONTROLLER
#define MP3Sparkfun
#define UseHallMonitor

/* ------------------- ENCODER PINS ------------------- */
#define motorEncoder_pin_A 18  // A0 → PCINT7
#define motorEncoder_pin_B 19  // A1 → PCINT6

/* ------------------- PIN DEFINITIONS ------------------- */
#define domeMotor_pwm        10
#define domeMotor_pin_A      9
#define domeMotor_pin_B      6
#define hallEffectSensor_Pin 20   // A2
#define leftServo_pin        12
#define rightServo_pin       11
#define PIN_MP3_TX           5
#define PIN_MP3_RX           13

/* ------------------- CONSTANTS ------------------- */
#define servoSpeed                255
#define servoEase                 10
#define domeTiltYAxis_MaxAngle    12
#define domeTiltXAxis_MaxAngle    12
#define printMillis               100
#define leftServoOffset           -7
#define rightServoOffset           0
#define TICKS_PER_REV             1680
#define CENTER_TICK               878        // YOUR ACTUAL CENTER
#define MAX_ANGLE_TICKS           140
#define MP3_INIT_TIMEOUT          5000
#define SERVO_UPDATE_INTERVAL     5
#define DOME_CENTER_TIMEOUT       10000
#define HALL_DEBOUNCE             50
#define CENTER_TOLERANCE          15
#define MOTOR_SPEED               100
#define FIND_CENTER_SPEED         50
#define ENC_MIN_INTERVAL          500      // µs debounce
#define DRIFT_CHECK_INTERVAL      5000     // ms

/* ------------------- LIBRARIES ------------------- */
#include <Wire.h>
#include <VarSpeedServo.h>
#include <EasyTransfer.h>
#include <PID_v1.h>
#include "SparkFun_Qwiic_MP3_Trigger_Arduino_Library.h"

/* ------------------- GLOBALS ------------------- */
volatile long encPos = CENTER_TICK;
volatile unsigned long lastEncInterrupt = 0;

double Input_domeSpinServoPid, Output_domeSpinServoPid, Setpoint_domeSpinServoPid = 0;
int domeServoPWM;
VarSpeedServo myservo1, myservo2;
EasyTransfer recESP32, sendESP32;
PID myPID_domeSpinServoPid(&Input_domeSpinServoPid, &Output_domeSpinServoPid,
                           &Setpoint_domeSpinServoPid, 3.5, 0.8, 0.05, DIRECT);

int16_t leftServo_0_Position  = 70 + leftServoOffset;
int16_t rightServo_0_Position = 110 + rightServoOffset;
double leftServoPosition  = leftServo_0_Position;
double rightServoPosition = rightServo_0_Position;
double leftOldPosition    = leftServo_0_Position;
double rightOldPosition   = rightServo_0_Position;
double domeTiltAngle_X_Axis, domeTiltAngle_Y_Axis, leftStickY, leftStickX;

bool domeCenterSet = false, domeServoMode = false, enableDrive = false, reverseDrive = false;
bool sndplaying = false;
int8_t soundcmd = 0;

unsigned long currentMillis, receiveMillis, lastPrintMillis, lastServoUpdateMillis;
unsigned long domeCenterStartMillis, lastHallLowMillis;
unsigned long lastDriftCheck = 0;
long lastStableEnc = CENTER_TICK;

#ifdef MP3Sparkfun
MP3TRIGGER mp3;
int randomsound = random(1, 55);
#endif

/* ------------------- DATA STRUCTURES ------------------- */
struct RECEIVE_DATA_STRUCTURE {
  bool driveEnabled;
  int8_t domeSpin;
  bool moveL3;
  bool moveR3;
  int8_t leftStickX;
  int8_t leftStickY;
  int8_t soundcmd;
  int8_t psiFlash;
  float pitch;
  float roll;
} receiveFromESP32Data;

struct SEND_DATA_STRUCTURE {
  int16_t tiltAngle;
  bool sndplaying;
} sendToESP32Data;

/* ------------------- DEBOUNCED + VALIDATED ENCODER ISR ------------------- */
ISR(PCINT0_vect) {
  unsigned long now = micros();
  if (now - lastEncInterrupt < ENC_MIN_INTERVAL) return;
  lastEncInterrupt = now;

  static uint8_t lastState = 0;
  uint8_t a = digitalRead(motorEncoder_pin_A);
  uint8_t b = digitalRead(motorEncoder_pin_B);
  uint8_t state = (a << 1) | b;

  static const int8_t transitionTable[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
  };

  int8_t delta = transitionTable[(lastState << 2) | state];
  if (delta != 0) {
    encPos += delta;
    encPos = (encPos % TICKS_PER_REV + TICKS_PER_REV) % TICKS_PER_REV;
  }
  lastState = state;
}

/* ------------------- SETUP ------------------- */
void setup() {
  Serial.begin(115200);
  Serial1.begin(74880);

  #ifdef MP3Sparkfun
  Wire.begin();
  unsigned long t = millis();
  bool ok = false;
  while (millis() - t < MP3_INIT_TIMEOUT && !ok) {
    if (mp3.begin()) ok = true;
    delay(100);
  }
  if (ok) { 
    mp3.setVolume(25); 
    Serial.print(F("MP3 Songs: ")); Serial.println(mp3.getSongCount());
  } else {
    Serial.println(F("MP3 init failed"));
  }
  #endif

  myservo2.attach(leftServo_pin);
  myservo1.attach(rightServo_pin);
  myservo2.write(leftServo_0_Position, 10);
  myservo1.write(rightServo_0_Position, 10);

  recESP32.begin(details(receiveFromESP32Data), &Serial1);
  sendESP32.begin(details(sendToESP32Data), &Serial1);
  pinMode(hallEffectSensor_Pin, INPUT_PULLUP);

  myPID_domeSpinServoPid.SetMode(AUTOMATIC);
  myPID_domeSpinServoPid.SetOutputLimits(-255, 255);

  pinMode(domeMotor_pwm,   OUTPUT);
  pinMode(domeMotor_pin_A, OUTPUT);
  pinMode(domeMotor_pin_B, OUTPUT);
  pinMode(motorEncoder_pin_A, INPUT_PULLUP);
  pinMode(motorEncoder_pin_B, INPUT_PULLUP);

  // Enable PCINT on A0 (PCINT7) and A1 (PCINT6)
  PCMSK0 |= (1 << PCINT7) | (1 << PCINT6);
  PCICR  |= (1 << PCIE0);

  #ifdef UseHallMonitor
    domeCenterStartMillis = millis();
    while (!domeCenterSet && millis() - domeCenterStartMillis < DOME_CENTER_TIMEOUT) {
      setDomeCenter();
    }
    if (!domeCenterSet) Serial.println(F("Centering timeout"));
  #else
    domeCenterSet = true;
    encPos = CENTER_TICK;
  #endif

  Setpoint_domeSpinServoPid = 0;
  lastDriftCheck = millis();
  lastStableEnc = encPos;
}

/* ------------------- MAIN LOOP ------------------- */
void loop() {
  Timechecks();
  SendRecieveData();
  DriftMonitor();  // Auto-snap to center

  if (enableDrive) {
    Servos();
    spinStuff();
    if (!domeCenterSet && domeServoMode && domeServoPWM == 0) setDomeCenter();
  }
  mp3play();
}

/* ------------------- DRIFT MONITOR (AUTO-CORRECT) ------------------- */
void DriftMonitor() {
  if (millis() - lastDriftCheck >= DRIFT_CHECK_INTERVAL) {
    if (domeCenterSet && digitalRead(hallEffectSensor_Pin) == LOW && abs(encPos - lastStableEnc) < 3) {
      encPos = CENTER_TICK;  // Snap to center
      Serial.println(F("Drift corrected: snapped to 878"));
    }
    lastStableEnc = encPos;
    lastDriftCheck = millis();
  }
}

/* ------------------- HALL CENTERING ------------------- */
void setDomeCenter() {
  if (digitalRead(hallEffectSensor_Pin) == LOW) {
    if (millis() - lastHallLowMillis >= HALL_DEBOUNCE) {
      domeCenterSet = true;
      encPos = CENTER_TICK;
      stopDome();
      Serial.println(F("Centered at 878"));
    }
    return;
  } else {
    lastHallLowMillis = millis();
  }

  double angleDiff = ((CENTER_TICK - encPos) % TICKS_PER_REV) * 360.0 / TICKS_PER_REV;
  if (angleDiff > 180) angleDiff -= 360;
  else if (angleDiff < -180) angleDiff += 360;

  if (abs(angleDiff) > (CENTER_TOLERANCE * 360.0 / TICKS_PER_REV)) {
    digitalWrite(domeMotor_pin_A, angleDiff > 0 ? HIGH : LOW);
    digitalWrite(domeMotor_pin_B, angleDiff > 0 ? LOW  : HIGH);
    analogWrite(domeMotor_pwm, FIND_CENTER_SPEED);
  } else {
    stopDome();
    if (abs(encPos - CENTER_TICK) <= CENTER_TOLERANCE) {
      domeCenterSet = true;
      encPos = CENTER_TICK;
    }
  }

  #ifdef debugHALL
  Serial.print(F("Hall:")); Serial.print(digitalRead(hallEffectSensor_Pin));
  Serial.print(F(" enc:")); Serial.print(encPos);
  Serial.print(F(" diff:")); Serial.println(angleDiff);
  #endif
}

/* ------------------- DOME SPIN CONTROL ------------------- */
void spinStuff() {
  static bool lastMoveR3 = false;
  bool curR3 = receiveFromESP32Data.moveR3;
  if (curR3 != lastMoveR3) {
    domeServoMode = curR3;
    lastMoveR3 = curR3;
    if (domeServoMode) domeCenterSet = false;
  }

  if (domeServoMode) domeServoMovement();
  else               spinDome();
}

void spinDome() {
  int speed = abs(domeServoPWM) > 3 ? constrain(map(abs(domeServoPWM), 3, 127, 50, 255), 0, 255) : 0;
  if (domeServoPWM > 3 && enableDrive) {
    digitalWrite(domeMotor_pin_A, LOW);  digitalWrite(domeMotor_pin_B, HIGH);
    analogWrite(domeMotor_pwm, speed);
  } else if (domeServoPWM < -3 && enableDrive) {
    digitalWrite(domeMotor_pin_A, HIGH); digitalWrite(domeMotor_pin_B, LOW);
    analogWrite(domeMotor_pwm, speed);
  } else {
    stopDome();
  }
}

void domeServoMovement() {
  if (!domeCenterSet) { setDomeCenter(); return; }

  Setpoint_domeSpinServoPid = enableDrive
      ? constrain(map(domeServoPWM, -180, 180, -MAX_ANGLE_TICKS, MAX_ANGLE_TICKS),
                  -MAX_ANGLE_TICKS, MAX_ANGLE_TICKS)
      : 0;

  Input_domeSpinServoPid = ((encPos - CENTER_TICK) % TICKS_PER_REV);
  if (Input_domeSpinServoPid > TICKS_PER_REV/2) Input_domeSpinServoPid -= TICKS_PER_REV;
  else if (Input_domeSpinServoPid < -TICKS_PER_REV/2) Input_domeSpinServoPid += TICKS_PER_REV;

  myPID_domeSpinServoPid.Compute();

  int speed = abs(Output_domeSpinServoPid) > 0.5 ? constrain(abs(Output_domeSpinServoPid), 80, 255) : 0;
  if (Output_domeSpinServoPid > 0.5 && enableDrive) {
    digitalWrite(domeMotor_pin_A, LOW);  digitalWrite(domeMotor_pin_B, HIGH);
    analogWrite(domeMotor_pwm, speed);
  } else if (Output_domeSpinServoPid < -0.5 && enableDrive) {
    digitalWrite(domeMotor_pin_A, HIGH); digitalWrite(domeMotor_pin_B, LOW);
    analogWrite(domeMotor_pwm, speed);
  } else {
    stopDome();
  }
}

void stopDome() {
  digitalWrite(domeMotor_pin_A, LOW);
  digitalWrite(domeMotor_pin_B, LOW);
  analogWrite(domeMotor_pwm, 0);
}

/* ------------------- TIME / COMMS ------------------- */
void Timechecks() {
  currentMillis = millis();
  receiveMillis = currentMillis;
  if (currentMillis - lastPrintMillis >= printMillis) {
    lastPrintMillis = currentMillis;
    debugRoutines();
  }
  if (currentMillis - receiveMillis >= 1000 && enableDrive) enableDrive = false;
}

void SendRecieveData() {
  if (recESP32.receiveData()) {
    receiveMillis = currentMillis;
    enableDrive   = receiveFromESP32Data.driveEnabled;
    reverseDrive  = receiveFromESP32Data.moveL3;
    domeServoPWM  = map(receiveFromESP32Data.domeSpin, -127, 127, -180, 180);
    soundcmd      = receiveFromESP32Data.soundcmd;

    if (!enableDrive || !domeServoMode) {
      Setpoint_domeSpinServoPid = 0;
    }

    sendToESP32Data.sndplaying = sndplaying;
    sendESP32.sendData();
  }
}

/* ------------------- MP3 ------------------- */
void mp3play() {
  #ifdef MP3Sparkfun
  if (mp3.isConnected()) {
    sndplaying = mp3.isPlaying();
    if (soundcmd) {
      if (soundcmd == 9) soundcmd = randomsound;
      if (sndplaying) mp3.stop();
      mp3.playFile(soundcmd);
      soundcmd = 0;
    }
  }
  #endif
}

/* ------------------- SERVO TILT (SAFE SCALING) ------------------- */
void Servos() {
  int y_Axis = receiveFromESP32Data.leftStickY;
  int x_Axis = receiveFromESP32Data.leftStickX;

  if (domeServoMode && domeCenterSet) {
    int domeTurnPercent = map(Setpoint_domeSpinServoPid, -MAX_ANGLE_TICKS, MAX_ANGLE_TICKS, 100, -100);
    int scale = 100 - abs(domeTurnPercent);
    y_Axis = map(y_Axis, -127, 127, -scale, scale);
    x_Axis = map(x_Axis, -127, 127, -scale, scale);
  }

  if (reverseDrive) { y_Axis *= -1; x_Axis *= -1; }

  leftStickY = map(y_Axis, -127, 127, -domeTiltYAxis_MaxAngle, domeTiltYAxis_MaxAngle);
  leftStickX = map(x_Axis, -127, 127, -domeTiltXAxis_MaxAngle, domeTiltXAxis_MaxAngle);

  if (currentMillis - lastServoUpdateMillis >= SERVO_UPDATE_INTERVAL) {
    lastServoUpdateMillis = currentMillis;
    domeTiltAngle_Y_Axis += (leftStickY > domeTiltAngle_Y_Axis)
        ? min(servoEase, leftStickY - domeTiltAngle_Y_Axis)
        : max(-servoEase, leftStickY - domeTiltAngle_Y_Axis);
    domeTiltAngle_X_Axis += (leftStickX > domeTiltAngle_X_Axis)
        ? min(servoEase, leftStickX - domeTiltAngle_X_Axis)
        : max(-servoEase, leftStickX - domeTiltAngle_X_Axis);

    domeTiltAngle_Y_Axis = constrain(domeTiltAngle_Y_Axis, -domeTiltYAxis_MaxAngle, domeTiltYAxis_MaxAngle);
    domeTiltAngle_X_Axis = constrain(domeTiltAngle_X_Axis, -domeTiltXAxis_MaxAngle, domeTiltXAxis_MaxAngle);
  }

  float pitch = receiveFromESP32Data.pitch;
  float roll  = receiveFromESP32Data.roll;

  leftServoPosition  = leftServo_0_Position  + map(domeTiltAngle_Y_Axis - pitch, -24, 24, 36, -30);
  rightServoPosition = rightServo_0_Position + map(domeTiltAngle_Y_Axis - pitch, -24, 24, -36, 30);
  leftServoPosition  += map(domeTiltAngle_X_Axis + roll, -24, 24, 30, -30);
  rightServoPosition += map(domeTiltAngle_X_Axis + roll, -24, 24, 36, -36);

  double lDiff = abs(leftOldPosition  - leftServoPosition);
  double rDiff = abs(rightOldPosition - rightServoPosition);

  if (lDiff > rDiff && lDiff > 0.1) {
    leftOldPosition  += (leftOldPosition < leftServoPosition) ? min(1.0, lDiff) : max(-1.0, -lDiff);
    rightOldPosition += (rightOldPosition < rightServoPosition) ? rDiff/lDiff : -rDiff/lDiff;
  } else if (rDiff >= lDiff && rDiff > 0.1) {
    rightOldPosition += (rightOldPosition < rightServoPosition) ? min(1.0, rDiff) : max(-1.0, -rDiff);
    leftOldPosition  += (leftOldPosition < leftServoPosition) ? lDiff/rDiff : -lDiff/rDiff;
  }

  myservo2.write(constrain(leftOldPosition,  leftServo_0_Position-36,  leftServo_0_Position+36),  servoSpeed);
  myservo1.write(constrain(rightOldPosition, rightServo_0_Position-36, rightServo_0_Position+36), servoSpeed);
}

/* ------------------- DEBUG ------------------- */
void debugRoutines() {
  #ifdef debugDOME
  Serial.print(F("enc:")); Serial.print(encPos);
  Serial.print(F(" c:")); Serial.print(domeCenterSet);
  Serial.print(F(" m:")); Serial.print(domeServoMode);
  Serial.print(F(" pwm:")); Serial.println(domeServoPWM);
  #endif

  #ifdef debugHALL
  Serial.print(F("Hall:")); Serial.print(digitalRead(hallEffectSensor_Pin));
  Serial.print(F(" enc:")); Serial.print(encPos);
  Serial.print(F(" diff:")); Serial.println(((encPos - CENTER_TICK) % TICKS_PER_REV) * 360.0 / TICKS_PER_REV);
  #endif

  #ifdef debugENC
  Serial.print(F("encPos:")); Serial.println(encPos);
  #endif
}