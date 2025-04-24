void Servos() {
    double y_Axis, x_Axis; // Use double for consistency
    const int SERVO_MIN_OFFSET = 90; // Safe operational range
    const int SERVO_MAX_OFFSET = 90;
    const float DEADZONE_THRESHOLD = 0.5;  // Base deadzone for minor fluctuations
    const float IMU_THRESHOLD = 1.5;      // Threshold to determine IMU-driven correction activation
    const float SOFT_RETURN_FACTOR = 0.3;  // Weak correction force to return to center

    if (enableDrive) {
        double joystickPitch = map(receiveFromESP32Data.leftStickX, -127, 127, -domeTiltYAxis_MaxAngle, domeTiltYAxis_MaxAngle); // forward/backward
        double joystickRoll = map(receiveFromESP32Data.leftStickY, 127, -127, domeTiltXAxis_MaxAngle, -domeTiltXAxis_MaxAngle);  // left/right

        // Detect if joystick is active (above deadzone)
        bool joystickActive = abs(receiveFromESP32Data.leftStickX) > 10 || abs(receiveFromESP32Data.leftStickY) > 10;

        if (joystickActive) {
            // Apply joystick-controlled servo positions
            #ifdef reverseLeftRight
                leftServoPosition = leftServo_0_Position + joystickPitch - joystickRoll; // Swap left/right input
            #else
                leftServoPosition = leftServo_0_Position + joystickPitch + joystickRoll;
            #endif

            #ifdef reverseForwardBackward
                rightServoPosition = rightServo_0_Position + joystickPitch + joystickRoll; // Swap back/forward input
            #else
                rightServoPosition = rightServo_0_Position + joystickPitch - joystickRoll;
            #endif
        }
        else {
            // Determine if IMU-based corrections should activate
            bool imuActive = (abs(receiveFromESP32Data.pitch) > IMU_THRESHOLD) || (abs(receiveFromESP32Data.roll) > IMU_THRESHOLD);

            float error_roll = pitch_setpoint - receiveFromESP32Data.pitch; // Roll uses pitch data
            float error_pitch = roll_setpoint - receiveFromESP32Data.roll; // Pitch uses roll data

            // Apply deadzone filtering, but add soft correction when below threshold
            if (abs(error_pitch) < DEADZONE_THRESHOLD) error_pitch *= SOFT_RETURN_FACTOR;  // Weak correction instead of zero
            if (abs(error_roll) < DEADZONE_THRESHOLD) error_roll *= SOFT_RETURN_FACTOR;  // Weak correction instead of zero

            if (imuActive || abs(error_pitch) > DEADZONE_THRESHOLD || abs(error_roll) > DEADZONE_THRESHOLD) {
                // Amplify correction factors for more responsive stabilization
                float pitchCompensationFactor = (error_pitch < 0) ? 4.0 : 2.0; // Stronger for negative corrections
                float rollCompensationFactor = (error_roll < 0) ? 4.0 : 2.0;   // Stronger for negative corrections

                // PID logic for pitch (forward/backward)
                error_sum_pitch = constrain(error_sum_pitch + error_pitch, -10, 10); // Prevent integral windup
                float error_diff_pitch = error_pitch - prev_error_pitch;
                float pid_pitch_output = ((kp_pitch * error_pitch) + (ki_pitch * error_sum_pitch) + (kd_pitch * error_diff_pitch)) * pitchCompensationFactor;
                prev_error_pitch = error_pitch;

                // PID logic for roll (left/right)
                error_sum_roll = constrain(error_sum_roll + error_roll, -10, 10); // Prevent integral windup
                float error_diff_roll = error_roll - prev_error_roll;
                float pid_roll_output = ((kp_roll * error_roll) + (ki_roll * error_sum_roll) + (kd_roll * error_diff_roll)) * rollCompensationFactor;
                prev_error_roll = error_roll;

                // Apply stabilization to servos
                leftServoPosition = leftServo_0_Position - pid_pitch_output + pid_roll_output;
                rightServoPosition = rightServo_0_Position - pid_pitch_output - pid_roll_output;
            }
        }

        // Constrain servo positions for safe operation
        leftServoPosition = constrain(leftServoPosition, leftServo_0_Position - SERVO_MIN_OFFSET, leftServo_0_Position + SERVO_MAX_OFFSET);
        rightServoPosition = constrain(rightServoPosition, rightServo_0_Position - SERVO_MIN_OFFSET, rightServo_0_Position + SERVO_MAX_OFFSET);

        // Write positions to servos
        myservo2.write(leftServoPosition, servoSpeed);
        myservo1.write(rightServoPosition, servoSpeed);
    }
}
