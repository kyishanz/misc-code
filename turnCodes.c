/**
 * This method allows the robot to turn to an exact heading at a specific power. It uses PID to achieve this, and
takes in an exact number of degrees and the power needed.
 * @param degrees [-3600, 3600]
 * @param power [-127, 127]
 */
void turnGyro(int degrees, int power) {
    uint32_t cur = millis();

    /*Initializes PID constants*/
    double curP = TDRIVEP;
    double curI = TDRIVEI;
    double curD = TDRIVED;

    /*Calculates the error*/
    int errorChange = 0;
    int error = degrees - current_heading;

    /*Calculates how much the error has changed from the previous iteration for the derivative*/
    int prevError = degrees - current_heading;

    /*Calculates the sum of errors for the integral*/
    int integral = 0;

    bool endSequence = false;
    int startEnd = 0;
    int endTime = 200; //500
    //1155, 955

    /*While loop for calculating how much power should be assigend to the drivetrain*/
    while ((abs(errorChange) > TDRIVEMAXVEL || (!endSequence || (millis() - startEnd < endTime))) && millis() - cur < 800) {
        if (abs(error) <= TDRIVEPOSTOL && !endSequence) {
            endSequence = true;
            startEnd = millis();
        }

        /*Calculates the integral*/
        integral += error;
        if (integral > TDRIVEINTEGRALLIMIT) {
            integral = TDRIVEINTEGRALLIMIT;
        } else if (integral < -TDRIVEINTEGRALLIMIT) {
            integral = -TDRIVEINTEGRALLIMIT;
        }

        /*Calculates the derivative*/
        errorChange = error - prevError;
        prevError = error;

        delay(5);

        /*Calculates the error*/
        error = degrees - current_heading;

        int imu_reading = imu_get_heading(IMU_PORT);


        /*Assigns power to the drivetrain based on the proportional, integral, and derivative terms*/
        int powerToAssign = min(power, max(-power, error * TDRIVEP + integral * TDRIVEI + errorChange * TDRIVED));
        assignLeftRightDriveMotorsVoltage(powerToAssign, -powerToAssign);
    }

    /*After PID is done, stop moving*/
    driveBrake();
}

/**
Turning to GPS absolute heading
*/
void turnGPSAbsoluteGyro(double targetAngle, double speed) {
    if (targetAngle < 0) {
        targetAngle += 360;
    }
    double currentAngle = gps_get_heading(GPSFRONT_PORT);
    double difference = targetAngle - currentAngle;
    if (difference > 180) {
        difference -= 360;
    } else if (difference < -180) {
        difference += 360;
    }
    bool left = false;
    if (difference > 0) {
        assignLeftRightDriveMotorsVoltage(-speed, speed);
    } else if (difference < 0) {
        // left
        left = true;
        assignLeftRightDriveMotorsVoltage(speed, -speed);
    } else return;

    double totalDifference = difference;
    int start = millis();
    while (fabs(difference) > 1 && ((millis() - start) < 3000)) {
        delay(5);
        currentAngle = gps_get_heading(GPSFRONT_PORT);
        difference = targetAngle - currentAngle;
        if (difference > 180) {
            difference -= 360;
        } else if (difference < -180) {
            difference += 360;
        }
        if (fabs(difference) < fabs(totalDifference) * 0.4) {
            if (left) {
                assignLeftRightDriveMotorsVoltage(20, -20);
            } else assignLeftRightDriveMotorsVoltage(-20, 20);

        }
    }
    if (left) {
        assignLeftRightDriveMotorsVoltage(-abs(speed)/2, abs(speed)/2);
    } else assignLeftRightDriveMotorsVoltage(abs(speed)/2, -abs(speed)/2);
    delay(50);
    assignDriveMotorsVoltage(0);
}
