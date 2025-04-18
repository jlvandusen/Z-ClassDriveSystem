
int mapJoystickToAngle(int axisValue, int maxAngle) {
    // Maps joystick input (-127 to 127) to dome tilt angle range (-maxAngle to maxAngle)
    return map(axisValue, -127, 127, -maxAngle, maxAngle);
}

int easeAngle(int currentAngle, int targetAngle, int easeStep) {
    if (targetAngle < currentAngle - easeStep) {
        return currentAngle - easeStep;
    } else if (targetAngle > currentAngle + easeStep) {
        return currentAngle + easeStep;
    }
    return targetAngle;
}

void adjustPitch(int domeTiltAngle, int pitch) {
    if (domeTiltAngle < 0) {
        leftServoPosition = leftServo_0_Position - constrain(map((domeTiltAngle - pitch) * dome_pitch_modifier, -40, 0, domeTiltYAxis_Offset, 0), -domeTiltYAxis_Offset, domeTiltYAxis_Offset);
        rightServoPosition = rightServo_0_Position - constrain(map((domeTiltAngle - pitch) * dome_pitch_modifier, -40, 0, -domeTiltYAxis_Offset, 0), -domeTiltYAxis_Offset, domeTiltYAxis_Offset);
    } else if (domeTiltAngle > 0) {
        leftServoPosition = leftServo_0_Position - constrain(map((domeTiltAngle - pitch) * dome_pitch_modifier, 0, 35, 0, -domeTiltYAxis_Offset), -domeTiltYAxis_Offset, domeTiltYAxis_Offset);
        rightServoPosition = rightServo_0_Position - constrain(map((domeTiltAngle - pitch) * dome_pitch_modifier, 0, 35, 0, domeTiltYAxis_Offset), -domeTiltYAxis_Offset, domeTiltYAxis_Offset);
    } else if (abs(pitch) >= dome_pitch_threshold) {
        leftServoPosition = leftServo_0_Position + constrain(map(pitch * dome_pitch_modifier, -40, 35, -domeTiltYAxis_Offset, domeTiltYAxis_Offset), -domeTiltYAxis_Offset, domeTiltYAxis_Offset);
        rightServoPosition = rightServo_0_Position - constrain(map(pitch * dome_pitch_modifier, -40, 35, -domeTiltYAxis_Offset, domeTiltYAxis_Offset), -domeTiltYAxis_Offset, domeTiltYAxis_Offset);
    }
}

void adjustRoll(int domeTiltAngle, int roll) {
    int rollDirection = (roll <= 0) ? 1 : -1; // Determine direction of roll correction
    if (domeTiltAngle < 0) {
        leftServoPosition += map((domeTiltAngle + roll) * dome_roll_modifier, -29, 0, 30, 0);
        rightServoPosition += map((domeTiltAngle + roll) * dome_roll_modifier, -29, 0, 50, 0);
    } else if (domeTiltAngle > 0) {
        leftServoPosition += rollDirection * map((domeTiltAngle + roll) * dome_roll_modifier, 0, 29, 0, 50);
        rightServoPosition += rollDirection * map((domeTiltAngle + roll) * dome_roll_modifier, 0, 29, 0, 30);
    } else if (abs(roll) >= dome_roll_threshold) {
        leftServoPosition += rollDirection * map((domeTiltAngle + roll) * dome_roll_modifier, -29, 29, 30, -30);
        rightServoPosition += rollDirection * map((domeTiltAngle + roll) * dome_roll_modifier, -29, 29, 50, -50);
    }
}

void smoothServoMovement() {
    leftDifference = abs(leftOldPosition - leftServoPosition);
    rightDifference = abs(rightOldPosition - rightServoPosition);
    rightOldPosition += (leftDifference != 0) ? (rightDifference / leftDifference) : 0;
    leftOldPosition += (rightDifference != 0) ? (leftDifference / rightDifference) : 0;

    if (leftDifference > rightDifference) {
        leftOldPosition += (leftOldPosition < leftServoPosition) ? 1 : -1;
        rightOldPosition += (rightOldPosition < rightServoPosition) ? rightDifference / leftDifference : -(rightDifference / leftDifference);
    } else {
        rightOldPosition += (rightOldPosition < rightServoPosition) ? 1 : -1;
        leftOldPosition += (leftOldPosition < leftServoPosition) ? leftDifference / rightDifference : -(leftDifference / rightDifference);
    }
}

void applyReverseLogic(int &y_Axis, int &x_Axis, int &tiltY, int &tiltX) {
    if (reverseDrive) {
        y_Axis *= -1;
        x_Axis *= -1;
        tiltY *= -1;
        tiltX *= -1;
    }
}


void Servos(){
  int domeTurnPercent;
  int y_Axis, x_Axis;
  const int LEFT_SERVO_MIN = leftServo_0_Position - 45;
  const int LEFT_SERVO_MAX = leftServo_0_Position + 55;
  const int RIGHT_SERVO_MIN = rightServo_0_Position - 55;
  const int RIGHT_SERVO_MAX = rightServo_0_Position + 45;

  if (enableDrive) {
    // Read joystick inputs
    x_Axis = receiveFromESP32Data.leftStickX;
    y_Axis = receiveFromESP32Data.leftStickY;

    // Apply reverse logic
    if (reverseDrive) {
        applyReverseLogic(y_Axis, x_Axis, domeTiltAngle_Y_Axis, domeTiltAngle_X_Axis);
    }

    // Map joystick values to dome tilt angles
    leftStickY = mapJoystickToAngle(y_Axis, domeTiltYAxis_MaxAngle);
    leftStickX = mapJoystickToAngle(x_Axis, domeTiltXAxis_MaxAngle);

    // Ease dome tilt angles
    domeTiltAngle_Y_Axis = easeAngle(domeTiltAngle_Y_Axis, leftStickY, servoEase);
    domeTiltAngle_X_Axis = easeAngle(domeTiltAngle_X_Axis, leftStickX, servoEase);

    // Adjust servo positions for pitch
    adjustPitch(domeTiltAngle_Y_Axis, receiveFromESP32Data.pitch);

    // Adjust servo positions for roll
    adjustRoll(domeTiltAngle_X_Axis, receiveFromESP32Data.roll);

    // Smooth servo movement
    smoothServoMovement();

    // Write positions to servos
    myservo2.write(constrain(leftOldPosition, LEFT_SERVO_MIN, LEFT_SERVO_MAX), servoSpeed);
    myservo1.write(constrain(rightOldPosition, RIGHT_SERVO_MIN, RIGHT_SERVO_MAX), servoSpeed);
  }
}


