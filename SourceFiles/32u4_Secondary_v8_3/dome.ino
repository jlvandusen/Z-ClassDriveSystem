
void controlMotor(int pwmValue, bool direction) {
    pwmValue = constrain(abs(pwmValue), 0, 255);  // Constrain pwmValue to valid range
    if (direction) {
        digitalWrite(domeMotor_pin_A, HIGH);
        digitalWrite(domeMotor_pin_B, LOW);
    } else {
        digitalWrite(domeMotor_pin_A, LOW);
        digitalWrite(domeMotor_pin_B, HIGH);
    }
    analogWrite(domeMotor_pwm, pwmValue);
}

void stopMotor() {
    digitalWrite(domeMotor_pin_A, LOW);
    digitalWrite(domeMotor_pin_B, LOW);
    analogWrite(domeMotor_pwm, 0);
}

void alignToForwardPosition() {
    const int forwardPositionValue = 1;  // Hall sensor value for forward position
    const int alignmentTimeout = 5000;  // Timeout in milliseconds
    const int alignmentPWM = 100;       // PWM speed for alignment
    unsigned long startTime = millis();

    while (digitalRead(hallEffectSensor_Pin) != forwardPositionValue) {
        controlMotor(alignmentPWM, true);  // Rotate motor

        // Check for timeout
        if (millis() - startTime > alignmentTimeout) {
            Serial.println("Failed to align to forward position. Timeout reached!");
            stopMotor();
            return;
        }
    }

    stopMotor();
    Serial.println("Dome aligned to forward position!");
}

void spinStuff() {
    // Map joystick input to domeSpeed
    int domeSpeed = map(joystickInput, -127, 127, -255, 255);

    // Driving or Free Turning Mode
    if (enableDrive || !domeServoMode) {
        if (domeSpeed != 0) {
            controlMotor(domeSpeed, domeSpeed > 0);  // Motor control based on speed/direction
        } else {
            stopMotor();  // Stop if joystick is neutral
        }
    }

    // Centering Logic Using Hall Sensor (only when joystick is neutral)
    #ifdef useHallSensor
    if (joystickInput == 0 && enableDrive) {  // Joystick centered in driving mode
        if (!domeCenterSet) {
            targetPosition = hallSensorValue;  // Set forward position
            domeCenterSet = true;  // Ensure forward position is set only once
        }

        int error = targetPosition - myEnc.read();  // Calculate position error
        int correction = constrain(error * Kp_domeSpinServoPid, -255, 255);  // PID correction
        if (correction != 0) {
            controlMotor(correction, correction > 0);  // Correct dome position
        } else {
            stopMotor();  // Stop motor if no correction needed
        }
    }
    #endif
}


//void spinStuff() {
//  if (enableDrive) {
//    int domeSpeed = map(joystickInput, -127, 127, -255, 255);
//    if (domeSpeed != 0) {
//      controlMotor(domeSpeed, domeSpeed > 0);
//    } else {
//      stopMotor();
//    }
//  }
//
//  if (joystickInput == 0 && useHallSensor) {
//    if (!domeCenterSet) {
//      targetPosition = hallSensorValue;  // Forward position
//      domeCenterSet = true;
//    }
//    int error = targetPosition - myEnc.read();
//    int correction = constrain(error * Kp_domeSpinServoPid, -255, 255);
//    if (correction != 0) {
//      controlMotor(correction, correction > 0);
//    } else {
//      stopMotor();
//    }
//  }
//
//  if (!domeServoMode) {
//    int domeSpeed = map(joystickInput, -127, 127, -255, 255);
//    if (domeSpeed != 0) {
//      controlMotor(domeSpeed, domeSpeed > 0);
//    } else {
//      stopMotor();
//    }
//  }
//}
