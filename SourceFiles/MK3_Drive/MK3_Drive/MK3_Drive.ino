  // ===================================================================================================================================================================================================== 
  //                         Joe's Drive  - Main drive mechanism - Updated 5/2/18
  //
  //             ***         You are free to use, and share this code as long as it is not sold. There is no warranty, guarantee, or other tomfoolery. 
  //                         This entire project was masterminded by an average Joe, your mileage may vary. 
  // ===================================================================================================================================================================================================== 
  //                            Written by Joe Latiola - https://www.facebook.com/groups/JoesDrive/
  //
  //                            You will need libraries: EepromEX: https://github.com/thijse/Arduino-EEPROMEx
  //                                                     PIDLibrary: http://playground.arduino.cc/Code/PIDLibrary  
  //                                                     EasyTransfer: https://github.com/madsci1016/Arduino-EasyTransfer
  //                                                     VarSpeedServo: https://github.com/netlabtoolkit/VarSpeedServo
  //                                                     Modified Low PowerLab: https://travis-ci.org/LowPowerLab/RFM69  
  //  * https://github.com/netlabtoolkit/VarSpeedServo - Servo Controls for Feather (Non ESP32)
  //  * https://github.com/PaulStoffregen/Encoder - Encoder for the Dome Spin motor
  //  * https://github.com/reeltwo/PSController - Modified USBhost Controller for PS3 Move Nav/Xbox Controllers
  //  * https://github.com/ERROPiX/ESP32_AnalogWrite - Used for Proper PWM with ESP32 devices
  //  * https://github.com/br3ttb/Arduino-PID-Library/ - PID Library Official
  //  * http://www.billporter.info/easytransfer-arduino-library/ - Easy Transfer
  //  * https://github.com/espressif/arduino-esp32/tree/master/libraries/Preferences - Equivilent to the EEPROM Controls
  //  * https://www.andymark.com/products/neverest-classic-60-gearmotor?Power%20Connector=Anderson%20Powerpole%2015A%20(am-3103)&quantity=1> - Dome Rotation with Encoder
  //
  //   Joe's Drive V2 / Z-Drive from James VanDusen
  // * Primary CPU uses ESP32 HUZZAH32
  // * Written by James VanDusen - https://www.facebook.com/groups/799682090827096
  // * You will need ESP32 Hardware: 
  // * ESP32 HUZZAH From Adafruit: https://www.adafruit.com/product/3619 used for Dome and Primary Feather
  // * ESp32 Board Libraries: https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/pinouts?view=all#using-with-arduino-ide
  // * 
  // * Utilizes ESP32NOW technology over WiFi to talk between Dome and Body - need to capture the Wifi MAC during bootup.
  // * On Line 241 - replace this with the mac of dome uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  // * Modified ESP32 BT Host Library https://github.com/reeltwo/PSController from Mimir
  // * 
  // * Libraries Required
  // * https://github.com/netlabtoolkit/VarSpeedServo - Servo Controls for Feather (Non ESP32)
  // * https://github.com/madhephaestus/ESP32Encoder/  - Encoder Supporting ESP32 boards
  // * https://github.com/reeltwo/PSController - Modified USBhost Controller for PS3 Move Nav/Xbox Controllers
  // * https://github.com/ERROPiX/ESP32_AnalogWrite - Used for Proper PWM with ESP32 devices
  // * https://github.com/br3ttb/Arduino-PID-Library/ - PID Library Official
  // * http://www.billporter.info/easytransfer-arduino-library/ - Easy Transfer
  // * https://github.com/espressif/arduino-esp32/tree/master/libraries/Preferences - Equivilent to the EEPROM Controls
  // * https://www.andymark.com/products/neverest-classic-60-gearmotor?Power%20Connector=Anderson%20Powerpole%2015A%20(am-3103)&quantity=1> - Dome Rotation with Encoder
  // 
  // ===================================================================================================================================================================================================== 
  // =====================================================================================================================================================================================================
 
//**************************** Choose the options that match your setup ****************************
//  #define MK2_Dome
//  #define MK3_Dome
  #define V2_Drive              // For use with V2 ALLINONE ESP32 Board hardware including electronics and motor controllers etc
//  #define enableESPNOW          // Used for V2_Drive for communication to Dome ESP32 custom board
//  #define SerialSound           // ONLY ENABLE IF YOU HAVE THE SOUNDBOARD WIRED TO SERIAL3
  #define Seriali2c             // ONLY ENABLE IF YOU HAVE THE SOUNDBOARD WIRED TO I2C
  #define MP3Sparkfun  // Enable qwiic/i2c communications to MP3 trigger for Sparkfun
  
//  #define disablePSIflash       // Uncomment to remove the PSI flash.    
//  #define TiltDomeForwardWhenDriving      // uncomment this if you want to tilt the dome forward when driving. 

  
  
//   Update these as necessary to match your setup
  #define DEBUG_PRINTLN(s) Serial.println(s)
  #define DEBUG_PRINT(s) Serial.print(s)
  #define SerialDebug Serial  // Where you wish to send the output for debug (Define which SerialX)
  
  //*****************These only matter if you're NOT using Serial3 for sounds
  #define soundpin1 26          // Connected to sound pin 0
  #define soundpin2 28          // Connected to sound pin 1
  #define soundpin3 30          // Connected to sound pin 2
  #define soundpin4 32          // Connected to sound pin 3
  #define soundpin5 46          // Connected to sound pin 4
  #define soundpin6 44          // Connected to sound pin 5
  //#################################################################

  //*****************These only matter if you're using Serial3 for sounds
  #define numberOfVoice   50        // This is the number of 'voice' sound files NOT INCLUDING Music and such
  #define numberOfMusic   6         // This is the number of 'music' files
      // Below are used for the multipress button sounds. Pressing button 1 on the left or button 3 on the right once plays a speach track at random, pressing 2-6 times will play quickVoice1-5. 
  #define quickVoice1     6
  #define quickVoice2     8
  #define quickVoice3     20
  #define quickVoice4     22
  #define quickVoice5     1
      // Below are used for the multipress button sounds. Pressing button 2 on the left once plays a sound at random, pressing 2-6 times will play quickMusic1-5. 
  #define quickMusic1     33
  #define quickMusic2     34
  #define quickMusic3     35
  #define quickMusic4     36
  #define quickMusic5     38
  #define SFX_RST 37
  //#################################################################
  
  //************************ Dome Tilt MK2 Settings: *****************
  #ifdef MK2_Dome
    #define domeTiltPotPin A1     // Connected to Potentiometer on the dome tilt mast
    #define easeDomeTilt 1.2      // Lower number means more easing when moving forward and back a.k.a. slower
    #define domeTiltPWM1 5        // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
    #define domeTiltPWM2 6        // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
    #define MaxDomeTiltAngle 17   // Maximum angle in which the dome will tilt. **  Max is 25  
  #endif

  //#################################################################

  //************************ Dome Tilt MK3 Settings: *****************
  #ifdef MK3_Dome
    #define leftDomeTiltServo  4  //Signal pin for the left dome tilt servo 
    #define rightDomeTiltServo 5  //Signal pin for the right dome tilt servo
    #define MaxDomeTiltY  10      // Maximum angle to tilt the dome in the Y axis ** - MAX IS 20
    #define MaxDomeTiltX  12      // Maximum angle to tilt the dome in the X axis ** - MAX IS 18
    #define DomeYEase .4          // Spead of front to back dome movement, higher == faster
    #define DomeXEase .7          // Speed of side to side domemovement, higher == faster
    #define domeSpeed 60          // Speed that the servos will move
  #endif
  //#################################################################

  //************************ V2_Drive Settings: *****************
  #ifdef V2_Drive
    #define leftDomeTiltServo  13  //Signal pin for the left dome tilt servo 
    #define rightDomeTiltServo 12  //Signal pin for the right dome tilt servo
    #define MaxDomeTiltY  10      // Maximum angle to tilt the dome in the Y axis ** - MAX IS 20
    #define MaxDomeTiltX  12      // Maximum angle to tilt the dome in the X axis ** - MAX IS 18
    #define DomeYEase .4          // Spead of front to back dome movement, higher == faster
    #define DomeXEase .7          // Speed of side to side domemovement, higher == faster
    #define domeSpeed 60          // Speed that the servos will move
    #define motorEncoder_pin_A 19    //18
    #define motorEncoder_pin_B 16    //19
    #define domeMotor_pwm 10         // 10 - 32u4 Basic Proto/M0 Proto/32u4 RF
    #define domeMotor_pin_A 9        //  9 - 32u4 Basic Proto/M0 Proto/32u4 RF
    #define domeMotor_pin_B 6        //  6 - 32u4 Basic Proto/M0 Proto/32u4 RF
    #define hallEffectSensor_Pin 20  //19
    #define Drive_pwm 21
    #define Drive_pin_1 4
    #define Drive_pin_2 27
    #define S2SPot_pin A2 //GPIO 34
    #define NeoPixel_pin 27
    #define S2S_pwm 33
    #define S2S_pin_1 26
    #define S2S_pin_2 25
    #define flyWheelMotor_pwm 15  
    #define flyWheelMotor_pin_A 32
    #define flyWheelMotor_pin_B 14
  #endif
  //#################################################################
  
  #ifndef V2_Drive 

    #define S2SenablePin 29       // Pin that provides power to motor driver enable pins
    #define enablePin 31          // Pin that provides power to motor driver enable pins
    #define enablePinDome 33      // Pin that provides power to Dome motor driver enable pin
    #define ACTpin 17            // Pin connected to ACT on soundboard
    #define fadePin A2            // Connected to + of one channel on sound board(use resistor to ground)

    #define domeSpinPot A4        // Pin used to monitor dome spin potentiometer
    #define battMonitor A3        // Pin used to monitor battery voltage
    #define outputVoltage 5.2     // This is the output voltage from the Buck Converter powering the arduino
    #define drivePWM1 12          // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
    #define drivePWM2 13          // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
    #define s2sPWM1 6             // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
    #define s2sPWM2 7             // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
    #define domeSpinPWM1 10       // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
    #define domeSpinPWM2 11       // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
    #define flywheelSpinPWM1 8    // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed 
    #define flywheelSpinPWM2 9    // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed 
  #endif
  
  #define domeServoModeAngle 85 // Angle in which dome will look left/right when in servo mode   
  #define S2SpotPin A0          // A0 Pin connected to side tilt potentiometer 
  #define easeDomeUp 23         // Lower number means more easing when spinning
  #define easeDomeDown 35       // Lower number means more easing when spinning
  #define maxS2STilt 20         // max tilt using the joystick; max is 25
  #define voltageSensor_Pin 17 //GPIO A3
  #define resistor1 151000      // Larger resisitor used on voltage divider to read battery level
  #define resistor2 82000       // Smaller resisitor used on voltage divider to read battery level
  #define flywheelEase 3        // Speed in which flywheel will increase/decrease during gradual movements
  #define S2SEase 2.5           // speed in which side to side moves. Higher number equates to faster movement
  #define resetPin 40
  #define drivespeed1 50        // Speed of "Slow"
  #define drivespeed2 70        // Speed of "Med"
  #define drivespeed3 110       // Speed of "Fast" DO NOT EXCEED 110
  #define dataDelay   0
  #define recDelay    10
  #define sendDelay   40

//**************************** Use these to correct any reversals ****************************

// Controller settings:
//      #define reverseDrive          // uncomment if your drive joystick up/down is reversed
//      #define reverseDomeTilt       // uncomment if your dome tilt joystick is reversed
//      #define reverseS2S            // uncomment if your side to side joystick is reversed
//      #define reverseDomeSpin         // uncomment if your dome spin joystick is reversed
//      #define reverseDomeSpinServo    // uncomment if your dome spin joystick is reversed in servo mode
//      #define reverseFlywheel       // uncomment if your flywheel joystick is reversed

  //IMU settings:
//      #define reversePitch          // reverse Pitch. Test this by moving the drive by hand; the weight/stabilization should move WITH you, NOT AGAINST. This means the weight should always stay down. 
//      #define reverseRoll           // reverse Roll. Test this by moving the drive by hand; the weight/stabilization should move WITH you, NOT AGAINST. This means the weight should always stay down.

  //Potentiometer settings:
//      #define reverseDomeTiltPot      // uncomment to reverse this pot. MK2 ONLY This is needed if the dome tilt arm oscilates back/forth
//      #define reverseDomeSpinPot    // uncomment to reverse dome spin pot. If the dome spin acts strange when in servo mode, this is likely the culprit. 
//      #define reverseS2SPot           // uncomment to reverse the side tilt pot. If your side to side is going apesh*t, try this one. 

  //PWM settings:
      // If your drive takes off by itself as soon as you enable it, try reversing the Drive PWM pins. 
      // If the side tilt goes all the way to one side as soon as you enable it, try reversing the S2S PWM pins. 
      // If the dome tilts all the way forward/backward when you enable the drive, you guessed it, try reversing the Dome Tilt PWM pins. 
  



//**************************** Debug  ****************************
  
//#define printRemote              // Uncomment to see values passed from controller
//#define debugS2S                 // Uncomment to see Side tilt variables, PID values, ETC.
//#define debugDrive               // Uncomment to see main drive variables, PID values, ETC.
//#define debugDomeTilt            // Uncomment to see Dome tilt variables, PID values, ETC.
//#define debugdomeRotation        // Uncomment to see Dome rotation variables, PID values, ETC.
//#define debugPSI                 // Uncomment to see PSI values.
//#define printbodyBatt            // Uncomment to see battery level 
//#define printYPR                 // Uncomment to see Yaw, Pitch, and Roll
//#define printDome                // Uncomment to see the Dome's Yaw
//#define printOffsets             // Uncomment to see the offsets
//#define printOutputs
//#define printSoundPins
//#define debugFlywheelSpin
  
  
  // =====================================================================================================================================================================================================
  // =====================================================================================================================================================================================================
  // 
  
  #ifndef V2_Drive  
    #include <EEPROMex.h>
  #else
    #include <stdint.h>
    #include <esp_now.h>
    #include "WiFi.h"
    #include <PSController.h>
    #define DRIVE_CONTROLLER_MAC  nullptr
    #define DOME_CONTROLLER_MAC nullptr
    
    #include <PID_v1.h>  // https://github.com/br3ttb/Arduino-PID-Library/
    #include "wiring_private.h" // pinPeripheral() function
    #include <analogWrite.h>  // https://www.arduinolibraries.info/libraries/esp32-analog-write
    #include <ESP32Encoder.h>
    #include <Preferences.h>
    #include <nvs_flash.h> // for wiping NVram
    uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0xD7, 0x63, 0xC4}; // DOME ESP32
//    uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0xD7, 0x63, 0xC4};
    Preferences preferences;
    PSController driveController(DRIVE_CONTROLLER_MAC); //define the drive an dome Controller variable to be used against the Nav1 and Nav2 controllers.
    PSController domeController(DOME_CONTROLLER_MAC);
    ESP32Encoder encoder;
  #endif

  #ifdef enableESPNOW
    #include <stdint.h>
    #include <esp_now.h>
    #include "WiFi.h"
  #endif
  
  #include "Arduino.h"

  #ifdef SerialSound
    #include "Adafruit_Soundboard.h"  
    Adafruit_Soundboard sfx = Adafruit_Soundboard(&Serial3, NULL, SFX_RST);
  #endif
  #ifdef MP3Sparkfun
    #include <Wire.h>
    #include "SparkFun_Qwiic_MP3_Trigger_Arduino_Library.h"  // http://librarymanager/All#SparkFun_MP3_Trigger
    #include "Arduino.h"
    MP3TRIGGER mp3;
    byte mp3Address = 0x37;  // default address for Qwiic MP3
    byte adjustableNumber = 1;
    int randomsound = random(1, 55);
    int8_t sound;
    int8_t soundcmd;
  #endif
    
  #include <SPI.h>
  #include <PID_v1.h>

  #ifdef SerialSound
  struct RECEIVE_DATA_STRUCTURE_REMOTE{
        int Joy1Y=256; //main drive pitch
        int Joy1X=256; //tilt / steer roll
        int Joy2Y=256; //head pitch
        int Joy2X=256; //head roll
        int Joy3X=256; //Flywheel spin
        int Joy4X=256; //head spin
        byte lJoySelect; //Select on left joystick
        byte lBut1; //left 1
        byte lBut2; //left 2
        byte lBut3; //left3
        byte Fwd; //Select on right joystick = rJoySelect
        byte Speed;
        byte rBut2; //right 2
        byte rBut3; //right 3
        byte motorEnable=1; //toggle on top
        byte CalibID; 
        byte wireless=1;    
  }recFromRemote; 
  #endif
  
  #ifndef defined(MK3_Dome) || defined(V2_Drive)
  struct RECEIVE_DATA_STRUCTURE_REMOTE{
        int Joy1Y=256; //main drive
        int Joy1X=256; //tilt / steer
        int Joy2Y=256; //head tilt
        int Joy2X=256; //head spin
        int Joy3X=256; //spin Flywheel
        int Joy4X=256;
        byte lJoySelect=1; //Select on left joystick
        byte lBut1=1; //left 1
        byte lBut2=1; //left 2
        byte lBut3=1; //left3
        byte Fwd; //Select on right joystick = rJoySelect
        byte Speed;
        byte rBut2=1; //right 2
        byte rBut3=1; //right 3
        byte motorEnable=1; //toggle on top
        byte CalibID; 
        byte wireless;    
  }recFromRemote; 
  #endif
  
  #ifdef V2_Drive
    struct controllerButtonsL {
      bool cross,circle,up,down,left,right,ps,l1,l3;
      int8_t leftStickX,leftStickY,l2;
    } buttonsL;
    
    struct controllerButtonsR {
      bool cross,circle,up,down,left,right,ps,l1,l3;
      // int8_t rightStickX,rightStickY,l2;
      int rightStickX,rightStickY,l2; // Mov controller causes false positive negative reverse when pushing 100% forward need int
    } buttonsR;
    
    int8_t joystickDeadZoneRange = 50;  // For controllers that centering problems, use the lowest number with no drift

    int Joy1Y=0; //main drive pitch
    int Joy1X=0; //tilt / steer roll
    int Joy2Y=0; //head pitch
    int Joy2X=0; //head roll
    int Joy3X=0; //Flywheel spin
    int Joy4X=0; //head spin
    byte lJoySelect; //Select on left joystick
    byte lBut1; //left 1
    byte lBut2; //left 2
    byte lBut3; //left3
    byte Fwd; //Select on right joystick = rJoySelect
    byte Speed;
    byte rBut2; //right 2
    byte rBut3; //right 3
    byte motorEnable=1; //toggle on top
    byte CalibID; 
    byte wireless=1;
    bool domecontrollerConnected = false;
    bool drivecontrollerConnected = false;
    bool EnableFlywheel = false;
    bool enableDrive = false;
    bool IMUconnected = false; 
    bool controllerConnected = false;
    bool Send_Rec = false;
    bool DomeServoMode = false;
    int32_t encoderValue = 0;

  #endif
  
  
  //remote to body
  #ifndef V2_Drive
    struct SEND_DATA_STRUCTURE_REMOTE{
          int psi;
          byte lBut3;
          float bat;
          
    }sendTo;
  #else
     /*
      * Define variables to store readings to be sent
      */
      int sendPSI = 0;
      byte sendHP = 0;
      float sendBAT = 0;    
      int sendDIS = 0;
      
      /* 
       *  Define variables to store incoming readings
      */
      int incomingPSI = 0;
      byte incomingBTN = 0;
      float incomingBAT = 0;      
      
      /*
       * Variable to store if sending data was successful
      */
      String success;
//      int PSI;
//      float bodyBatt;
    /*
     * Create Send Data for ESPNOW
     * must match the receiver structure
     */
      typedef struct struct_message {
          int psi;
          byte btn;
          float bat;
          int dis;
      } struct_message;
      
      // Create a struct_message called outgoingESPNOW to hold sensor readings
      struct_message outgoingESPNOW;
      
      // Create a struct_message to hold incoming sensor readings
      struct_message incomingESPNOW;
  #endif

  // EasyTransfer for IMU Setup =========================
  #ifndef V2_Drive  
    #include <EasyTransfer.h>
    EasyTransfer RecIMU; 
    EasyTransfer RecRemote;
    EasyTransfer SendBody;
    
    struct RECEIVE_DATA_STRUCTURE_IMU{
      float IMUloop;
      float pitch;
      float roll;
    }recIMUData;
  #else
    #include <Adafruit_MPU6050.h>
    #include <Adafruit_Sensor.h>
//    #include <Adafruit_Sensor_Calibration.h>
    Adafruit_MPU6050 mpu;
  #endif
  
  // End EasyTransfer Setup =========================

  #ifdef defined(MK3_Dome) || defined(V2_Drive)
    #include <VarSpeedServo.h>
    VarSpeedServo leftServo;
    VarSpeedServo rightServo;
    int Joy2Ya, Joy2XLowOffset, Joy2XHighOffset, Joy2XLowOffsetA, Joy2XHighOffsetA, ServoLeft, ServoRight;
    double Joy2X, Joy2Y, LeftJoy2X, LeftJoy2Y, Joy2XEase, Joy2YEase,  Joy2XEaseMap;
  #endif 
  
  double Joy2YEaseMap;

  bool SendData;
  byte RightBut1Sound, RightBut2Sound;
  byte readPinState = 1; 
  byte playSound, soundState;
  byte randSoundPin;
  int soundPins[] = {soundpin1, soundpin2, soundpin3, soundpin4};
    
  byte bodyStatus = 0;
  byte autoDisableDoubleCheck, autoDisable, autoDisableState;
  byte IMUStatus, DataStatus, ControllerStatus;
  byte lastCalibID, sendID = 1;
  byte psiState, quitState;
  byte lastSpeed;
  
  int musicState, voiceNum, musicNum;
  int flywheelRotation;
  int ch4Servo;           
  int currentDomeSpeed;
  int domeRotation;
  int joystickS2S;
  int joystickDrive;
  int ch5PWM;
  int lastDirection;
  int speedDomeTilt = 0;
  int domeTiltPot;
  int domeSpinOffset;
  
  int S2Spot;
  int joystickDome;
  int driveSpeed = 55;
  int driveAccel;
  int potOffsetS2S;
  int domeTiltPotOffset;

  
  // the speedArray is used to create an S curve for the 'throttle' of bb8
  int speedArray[] = {0,1,1,1,1,1,1,2,2,2,2,2,2,3,3,3,3,4,4,4,5,5,5,5,6,6,7,7,8,8,9,
  9,10,10,11,12,12,13,13,14,15,16,16,17,18,19,19,20,21,22,23,24,25,26,26,27,28,29,30,
  31,32,33,33,34,35,36,37,37,38,39,40,40,41,42,42,43,44,44,45,45,46,46,47,47,48,48,49,
  49,50,50,50,51,51,51,52,52,52,52,53,53,53,53,54,54,54,54,54,55,55,55,55,55};
  
  
  double Joy2YPitch, Joy2YDirection, Joy2XDirection;
  double domeTiltOffset;
  
  float pitch, roll;
  float pitchOffset, rollOffset;
  float R1 = resistor1; 
  float R2 = resistor2;
  float countdown;
  unsigned long lastSendMillis; //, rBut2Millis;
  
  unsigned long soundMillis; 
  unsigned long autoDisableMotorsMillis, autoDisableDoubleCheckMillis;
  unsigned long lastLoopMillis;
  unsigned long IMUMillis, lastRecMillis, moveDataMillis, lastReceivedMillis;
  unsigned long lastBatteryUpdate;
  
  

  
  
  //PID1 is for the side to side tilt
  double Pk1 = 14;  
  double Ik1 = 0;
  double Dk1 = 0.0;
  double Setpoint1, Input1, Output1, Output1a;    
  
  PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);   
   
  //PID2 is for side to side stability
  double Pk2 = .5; 
  double Ik2 = 0;
  double Dk2 = .01;
  double Setpoint2, Input2, Output2, Output2a;    
  
  PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup - S2S stability   
  
  //PID3 is for the main drive
  double Pk3 = 5; 
  double Ik3 = 0;
  double Dk3 = 0;
  double Setpoint3, Input3, Output3, Output3a;    
  
  PID PID3(&Input3, &Output3, &Setpoint3, Pk3, Ik3 , Dk3, DIRECT);    // Main drive motor
  
  //PID4 is for dome tilt fwd/back
  double Pk4 = 6;  
  double Ik4 = 0;
  double Dk4 = 0.05;
  double Setpoint4, Input4, Output4, Output4a;    
  
  PID PID4(&Input4, &Output4, &Setpoint4, Pk4, Ik4 , Dk4, DIRECT);   
  
  double Setpoint5a;
  
  //PID5 is for the dome spin servo
  double Kp5=4, Ki5=0, Kd5=0;
  double Setpoint5, Input5, Output5, Output5a;
  
  PID PID5(&Input5, &Output5, &Setpoint5, Kp5, Ki5, Kd5, DIRECT);     
  

  

  
  
  // ================================================================
  // ===                      INITIAL SETUP                       ===
  // ================================================================
  
  void setup() {
      Serial.begin(115200);
      #ifndef V2_Drive
        Serial1.begin(57600);
        Serial2.begin(115200);
      #endif
      #ifdef SerialSound
        Serial3.begin(9600);
      #endif
      
  
      #ifdef defined(MK3_Dome) || defined(V2_Drive) // MK3_Dome || V2_drive
      leftServo.attach(leftDomeTiltServo);
      rightServo.attach(rightDomeTiltServo);
      leftServo.write(95,50, false); 
      rightServo.write(90, 50, false);
      #endif

      #ifndef V2_Drive
      RecIMU.begin(details(recIMUData), &Serial2);
      RecRemote.begin(details(recFromRemote), &Serial1);
      SendBody.begin(details(sendTo), &Serial1);
      #else
        // Try to initialize!
        if (!mpu.begin()) {
          Serial.println("Failed to find MPU6050 chip over i2c");
          IMUStatus = 1;
        } else IMUStatus = 0;
        Serial.println("MPU6050 Found on i2c!");
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        Serial.print("Accelerometer range set to: ");
        switch (mpu.getAccelerometerRange()) {
        case MPU6050_RANGE_2_G:
          Serial.println("+-2G");
          break;
        case MPU6050_RANGE_4_G:
          Serial.println("+-4G");
          break;
        case MPU6050_RANGE_8_G:
          Serial.println("+-8G");
          break;
        case MPU6050_RANGE_16_G:
          Serial.println("+-16G");
          break;
        }
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        Serial.print("Gyro range set to: ");
        switch (mpu.getGyroRange()) {
        case MPU6050_RANGE_250_DEG:
          Serial.println("+- 250 deg/s");
          break;
        case MPU6050_RANGE_500_DEG:
          Serial.println("+- 500 deg/s");
          break;
        case MPU6050_RANGE_1000_DEG:
          Serial.println("+- 1000 deg/s");
          break;
        case MPU6050_RANGE_2000_DEG:
          Serial.println("+- 2000 deg/s");
          break;
        }
      
        mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
        Serial.print("Filter bandwidth set to: ");
        switch (mpu.getFilterBandwidth()) {
        case MPU6050_BAND_260_HZ:
          Serial.println("260 Hz");
          break;
        case MPU6050_BAND_184_HZ:
          Serial.println("184 Hz");
          break;
        case MPU6050_BAND_94_HZ:
          Serial.println("94 Hz");
          break;
        case MPU6050_BAND_44_HZ:
          Serial.println("44 Hz");
          break;
        case MPU6050_BAND_21_HZ:
          Serial.println("21 Hz");
          break;
        case MPU6050_BAND_10_HZ:
          Serial.println("10 Hz");
          break;
        case MPU6050_BAND_5_HZ:
          Serial.println("5 Hz");
          break;
        }
      
        Serial.println("");
        delay(100);
        
        // Attach the encoder pins
        ESP32Encoder::useInternalWeakPullResistors = UP;
        encoder.attachSingleEdge(motorEncoder_pin_A, motorEncoder_pin_B);
        // Set the initial count to 0
        encoder.clearCount();
        
    #endif
      #ifndef V2_Drive
        pinMode(enablePin, OUTPUT);  // enable pin
        pinMode(S2SenablePin, OUTPUT); //enable pin for S2S
        pinMode(enablePinDome, OUTPUT);  // enable pin for dome spin
        pinMode(13, OUTPUT);
        digitalWrite(13, LOW);
        pinMode(ACTpin, INPUT_PULLUP); // read stat of Act on Soundboard
      #else
        pinMode(domeMotor_pwm, OUTPUT);    // Speed Of Motor1 on Motor Driver 1
        pinMode(domeMotor_pin_A, OUTPUT);  // Direction
        pinMode(domeMotor_pin_B, OUTPUT);
        pinMode (motorEncoder_pin_A, INPUT);    //18 A0 - 32u4 Basic Proto/32u4 RF, 14 - 32u4 Proto M0 Feather M0
        pinMode (motorEncoder_pin_B, INPUT);    //19 A1 - 32u4 Basic Proto/32u4 RF, 15 - 32u4 Proto M0 Feather M0
        pinMode(S2S_pwm, OUTPUT);  // Speed Of Motor2 on Motor Driver 1 
        pinMode(S2S_pin_1, OUTPUT);  // Direction
        pinMode(S2S_pin_2, OUTPUT);
        pinMode(Drive_pwm, OUTPUT);  // Speed of Motor2 on Motor Driver 2 
        pinMode(Drive_pin_1, OUTPUT);  // Direction
        pinMode(Drive_pin_2, OUTPUT);
        pinMode(flyWheelMotor_pwm, OUTPUT);  // Speed of Motor2 on Motor Driver 2 
        pinMode(flyWheelMotor_pin_A, OUTPUT);  // Direction
        pinMode(flyWheelMotor_pin_B, OUTPUT);

        WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station
      
        if (esp_now_init() != ESP_OK) { // Init ESP-NOW
          Serial.println("Error initializing ESP-NOW");
          return;
        } else Serial.println("Finished initializing ESP-NOW");
      
        // Once ESPNow is successfully Init, we will register for Send CB to
        // get the status of Trasnmitted packet
        esp_now_register_send_cb(OnDataSent);
      
        esp_now_peer_info_t peerInfo;   // Register peer ESPNOW chip
        memcpy(peerInfo.peer_addr, broadcastAddress, 6);
        peerInfo.channel = 0;  
        peerInfo.encrypt = false;
      
      
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {   // Add peer        
          Serial.println("Failed to add peer");
          return;
        }
        // Register for a callback function that will be called when data is received
        esp_now_register_recv_cb(OnDataRecv);
        
      #endif
      
      #ifdef serialSound
        digitalWrite(reset_pin, LOW);
        pinMode(reset_pin, OUTPUT);
        delay(10);
        pinMode(reset_pin, INPUT);
        delay(1000);
      #endif
      #ifndef serialSound || Seriali2c
        pinMode(soundpin1, OUTPUT); // play sound from pin 0 on Soundboard
        pinMode(soundpin2, OUTPUT); // play sound from pin 1 on Soundboard
        pinMode(soundpin3, OUTPUT); // play sound from pin 2 on Soundboard
        pinMode(soundpin4, OUTPUT); // play sound from pin 3 on Soundboard
        pinMode(soundpin5, OUTPUT); // play sound from pin 4 on Soundboard
        pinMode(soundpin6, OUTPUT); // play sound from pin 4 on Soundboard
        digitalWrite(soundpin6, HIGH);
        digitalWrite(soundpin5, HIGH);
        digitalWrite(soundpin4, HIGH);
        digitalWrite(soundpin3, HIGH);
        digitalWrite(soundpin2, HIGH);
        digitalWrite(soundpin1, HIGH);
      #endif
      
      // *********** PID setup ***********
  
      PID1.SetMode(AUTOMATIC);              // PID Setup -  S2S SERVO     
      PID1.SetOutputLimits(-255, 255);
      PID1.SetSampleTime(15);
  
      PID2.SetMode(AUTOMATIC);              // PID Setup -  S2S Stability
      PID2.SetOutputLimits(-255, 255);
      PID2.SetSampleTime(15);
  
      PID3.SetMode(AUTOMATIC);              // PID Setup - main drive motor
      PID3.SetOutputLimits(-255, 255);
      PID3.SetSampleTime(15);
  
      PID4.SetMode(AUTOMATIC);              // PID Setup - dome tilt
      PID4.SetOutputLimits(-255, 255);
      PID4.SetSampleTime(15);
  
      PID5.SetMode(AUTOMATIC);
      PID5.SetOutputLimits(-255, 255);      // PID Setup - dome spin 'servo'
      PID5.SetSampleTime(15);
  
      // *********  Read offsets from EEPROM  **********
      #ifndef V2_Drive
        pitchOffset = EEPROM.readFloat(0);
        rollOffset = EEPROM.readFloat(4);
        potOffsetS2S = EEPROM.readInt(8);
        domeTiltPotOffset = EEPROM.readInt(10);
        domeSpinOffset = EEPROM.readInt(12);
      #else
        pitchOffset = preferences.getFloat("pitchOffset", 0);
        rollOffset = preferences.getFloat("rollOffset", 0);
        potOffsetS2S = preferences.getInt("potOffsetS2S", 0);
        domeTiltPotOffset = preferences.getInt("domeTiltPotOffset", 0);
        domeSpinOffset = preferences.getInt("domeSpinOffset", 0);
      #endif
      
      if( isnan(pitchOffset)){
        pitchOffset = 0; 
      }
      if( isnan(rollOffset)){
        rollOffset = 0; 
      }    
      if( isnan(potOffsetS2S)){
        potOffsetS2S = 0; 
      }    
      if( isnan(domeTiltPotOffset)){
        domeTiltPotOffset = 0; 
      }    
      if( isnan(domeSpinOffset)){
        domeSpinOffset = 0; 
      }
      
    readVin();
  }
  
   //     =======================================================================================================================================================================================
   //     =======================================================================================================================================================================================
   //     =====                                                                                LOOP, bruh!                                                                                  =====
   //     =======================================================================================================================================================================================
   //     =======================================================================================================================================================================================
   
   void loop() {
    
     receiveRemote();
     
     if(millis() - lastRecMillis >= 10){
        lastRecMillis = millis();
        receiveIMUData();
        
     }
      
     if (millis() - lastLoopMillis >= 20){
        lastLoopMillis = millis();
        movement(); 
        readVin();
        setDriveSpeed();
        sounds();
        
     }

     sendDriveData();
  }
  
