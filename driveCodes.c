/**
 * Using physics to decelerate.
 * @param speed
 * @param acc
 * @param dist
 * @return
 */
float calculateSpeedWithConstantAcceleration(float speed, float acc, float dist) {
    return sqrt(2 * acc * dist);
}

void driveHeadingWithRampDown(float targetHeading, float speed, float totalDistToDrive, float distToStartRampDown, bool smartRamp) {
    clearDriveMotors();

    float error = totalDistToDrive;
    float currentPos, driveSpeed;
    float headingError, currentHeading, headingCompensation;
    float kP = 0.02;

    float acc = speed * speed / (2 * totalDistToDrive);

    while(1) {
        currentPos = readDistanceInches();
        error = totalDistToDrive - currentPos;
        if (error < 0.1) {
            assignDriveMotorsVoltage(-10);
            delay(100);
            assignDriveMotorsVoltage(0);
            return;
        }
        currentHeading = imu_get_heading(IMU_PORT);
        headingError = targetHeading - currentHeading;

        headingCompensation = fabs(kP * headingError);
        if (headingCompensation > 0.2){
            headingCompensation = 0.2;
        }
        if (error < distToStartRampDown && smartRamp) {
            driveSpeed = calculateSpeedWithConstantAcceleration(fabs(speed), acc, error) * fabs(speed)/speed;
        } else if (error < distToStartRampDown && !smartRamp) {
            //do a dummy slow down
            driveSpeed = 20;
        } else driveSpeed = speed;
        if (headingError > 0) {
            assignLeftRightDriveMotorsVoltage(driveSpeed, (1 - headingCompensation) * driveSpeed);
        } else if (headingError < 0) {
            assignLeftRightDriveMotorsVoltage((1 - headingCompensation) * driveSpeed, driveSpeed);
        } else {
            assignDriveMotorsVoltage(driveSpeed);
        }
        delay(20);
    }
}

/**
This method maintains the correct heading while the robot is moving a certain distance. The parameters are
ticks, which is the number of total ticks this movement should take, the power to assign to the drivetrain,
and the heading which it should maintain throughout this movement.

This function does not use PID for dist driven. Only heading is corrected.
*/
void coastHeadingWithoutDrivePID(int ticks, int power, int heading) {
    int error = 0;

    /*Tares the drive motors in order to read how much they have moved (below)*/
    clearDriveMotors();

    /*Assigns initial values to the drivetrain motors, which was calculated above*/
    assignLeftRightDriveMotorsVoltage(power, power);

    /*Makes sure that the total movement by the drive motors does not exceed the number of ticks it should be moving*/
    while (abs(motor_get_position(PORT_DRIVELEFTFRONT)) + abs(motor_get_position(PORT_DRIVELEFTBACK)) + abs(motor_get_position(PORT_DRIVELEFTMIDDLE)) +
           abs(motor_get_position(PORT_DRIVERIGHTFRONT)) + abs(motor_get_position(PORT_DRIVERIGHTBACK)) + abs(motor_get_position(PORT_DRIVERIGHTMIDDLE))<
           ticks * NUMDRIVEMOTORS) {
        /*Calculates the error of the heading: where it should be minus where it is*/
        error = heading - current_heading;

        /* This assigns the power to the motors. It uses the P term of PID: proportional. It multiplies
        the error with a constant and uses that to adjust the current power of the drivetrain, ensuring
        that it maintains the heading it is supposed to. */
        assignLeftRightDriveMotorsVoltage(power + error * CORRD, power - error * CORRD);

        delay(5);
    }
    /*Stops movement after number of ticks has been reached*/
    assignLeftRightDriveMotorsVoltage(0, 0);
}

/**
No correction whatsoever; fastest despite risks of drift. 
*/
void assignDriveMotorsDist(int leftSide, int rightSide, int power, bool clear, bool turn) {
    if (clear) {
        clearDriveMotors();
    }

    int currentPrecision = MOVEENCODERPRECISION;
    if (turn) {
        currentPrecision = TURNENCODERPRECISION;
    }

    motor_move(PORT_DRIVELEFTFRONT, leftSide);
    motor_move(PORT_DRIVERIGHTFRONT, rightSide);
    motor_move(PORT_DRIVERIGHTMIDDLE, rightSide);
    motor_move(PORT_DRIVELEFTMIDDLE, leftSide);
    motor_move(PORT_DRIVELEFTBACK, leftSide);
    motor_move(PORT_DRIVERIGHTBACK, rightSide);

    delay(100);
    uint32_t timeout = millis();
    while ((abs(motor_get_position(PORT_DRIVELEFTFRONT) - leftSide) + abs(motor_get_position(PORT_DRIVELEFTBACK) - leftSide) + abs(motor_get_position(PORT_DRIVELEFTMIDDLE) - leftSide) +
            (abs(motor_get_position(PORT_DRIVERIGHTFRONT) - rightSide) + abs(motor_get_position(PORT_DRIVERIGHTBACK) - rightSide)) + abs(motor_get_position(PORT_DRIVERIGHTMIDDLE) - rightSide)) >
           currentPrecision * NUMDRIVEMOTORS) {
        uint32_t ctime = millis();
        if (ctime - timeout > 1100){
            break;
        }
        delay(20);
    }

    delay(250);
    assignLeftRightDriveMotorsVoltage(0, 0);
}

