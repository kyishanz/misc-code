//
// Created by Niu on 7/6/22.
//

#include "subsystemHeaders/hal.h"
#include "subsystemHeaders/my-config.h"

#define MOVEENCODERPRECISION 7
#define TURNENCODERPRECISION 6
#define INTEGRALLIMIT 6
#define DRIVEP 0.100
#define DRIVEI 0.0018
#define DRIVED 0.055
#define DRIVEINTEGRALLIMIT 1000
#define DRIVEMAXVEL 1
#define DRIVEPOSTOL 50
#define CORRD 0.3 //0.75

#define TDRIVEP 0.145 //.130 0.145
#define TDRIVEI 0.0089
#define TDRIVED 0.07 //0.04
#define TDRIVEINTEGRALLIMIT 1000
#define TDRIVEMAXVEL 2
#define TDRIVEPOSTOL 50 //6


#define TIMEOUT_LIMIT 1000

#define VISIONCONST 0.8
#define VISIONTIMEOUT 1000

int start_heading = 0;
int current_heading = 0;
int timeTakenByFunction = 0; // rename when needed

//void oneSidedTurn(int) {
//
//}

void getHeading() {
    int wrap = 0;
    int imucur = start_heading;
    while(true) {
        int previousHeading = imucur;
        imucur = imu_get_heading(IMU_PORT) * 10 + start_heading;
        if (previousHeading - imucur > 1000) {
            wrap ++;
        } else if (imucur - previousHeading > 1000) {
            wrap --;
        }
        current_heading = -(imucur + 3600 * wrap);
        delay(5);
    }
}

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
//            driveBrake();
            assignDriveMotorsVoltage(-10);
            delay(100);
            assignDriveMotorsVoltage(0);
            return;
        }
        //checking if robot is off with P-control
        currentHeading = imu_get_heading(IMU_PORT);
        headingError = targetHeading - currentHeading;
        // std::cout << "+" << headingError << endl;

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

double headingDiff(double h1, double h2){ //angle between two headings
    const double diff = fmod(h1 - h2 +3600, 360);
    return diff <= 180 ? diff : 360 - diff;
}

bool isTurnCCW(double hdg, double newHDG){ //should new heading turn left ccw
    const double diff = newHDG - hdg;
    return diff > 0 ? diff > 180 : diff >= -180;
}

/** turn gyro without correction */
void turnHeading(double newHeading, int power) {
    double targetHeading = newHeading;
    double currentHeading = imu_get_heading(IMU_PORT);
    if (currentHeading > 359) currentHeading = 0;
    bool turningLeft = isTurnCCW(currentHeading, targetHeading);
    double difference = headingDiff(currentHeading, targetHeading);
    bool notcross = true;//current and target crossing
    double diff = 0.0;

    if(turningLeft) {
        while((difference > 1.0) && (notcross)) {
            currentHeading = imu_get_heading(IMU_PORT);
            difference = headingDiff(currentHeading, targetHeading);
            turnLeftNoPID(power);
            diff = fmod(currentHeading - targetHeading +3600, 360);
        }
    }
    else if(turningLeft == false) {
        while((difference > 1.0) && (notcross)) {
            turnRightNoPID(power);
            currentHeading = imu_get_heading(IMU_PORT);
            difference = headingDiff(currentHeading, targetHeading);
        }
    }

    driveBrake();
    delay(20);
}

/**
 * Turning with physics deceleration.
 * @param speed
 * @param totalDegrees [-360, 360]
 * @param degreesToStartRampDown
 */
void turnWithRampDown(float speed, float totalDegrees, float degreesToStartRampDown) {
    clearDriveMotors();
    double initialHeading = imu_get_heading(IMU_PORT);
//    logDouble(initialHeading);

    float error = fabs(totalDegrees);
    float currentHeading, currentSpeed, degreesTurnedSoFar;

    float acc = speed * speed / (2 * totalDegrees);

    float direction = totalDegrees / fabs(totalDegrees);

    while(true) {
        currentHeading = imu_get_heading(IMU_PORT);
        logDouble(currentHeading);
        if (totalDegrees > 0) {
            if (currentHeading - initialHeading < 0) {
                degreesTurnedSoFar = currentHeading - initialHeading + 360;
            } else degreesTurnedSoFar = currentHeading - initialHeading;
        }
//        logDouble(degreesTurnedSoFar);
        error = totalDegrees - degreesTurnedSoFar;

        if (error <= 10) {
            logDouble(error);
            driveBrake();
            return;
        }
        if (error < degreesToStartRampDown) {
            currentSpeed = calculateSpeedWithConstantAcceleration(speed, acc, error);
            if (currentSpeed < 10) currentSpeed = 10;
            assignLeftRightDriveMotorsVoltage(direction * currentSpeed, -direction * currentSpeed);
        } else {
            assignLeftRightDriveMotorsVoltage(direction * speed, -direction * speed);
        }
        delay(100);
    }
}

#define TDRIVEPSLOW 0.145 //.130 0.145
#define TDRIVEISLOW 0.0089
#define TDRIVEDSLOW 0.07 //0.04
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
 * This method allows the robot to turn to an exact heading at a specific power. It uses PID to achieve this, and
takes in an exact number of degrees and the power needed. It obtains the heading from the imu sensor without a global task.
 * @param degrees [-3600, 3600]
 * @param power [-127, 127]
 */
void turnGyroWithoutTask(int degrees, int power, bool clockwise) {
    uint32_t cur = millis();

    int direction;
    if (clockwise) {
        direction = 1;
    } else direction = -1;

    /*Initializes PID constants*/
    double curP = TDRIVEP;
    double curI = TDRIVEI;
    double curD = TDRIVED;

    /*Calculates the error*/
    int errorChange = 0;
    int cur_heading = imu_get_heading(IMU_PORT) * 10;


    int error = degrees - cur_heading;

    /*Calculates how much the error has changed from the previous iteration for the derivative*/
    int prevError = degrees - cur_heading;

    /*Calculates the sum of errors for the integral*/
    int integral = 0;

    bool endSequence = false;
    int startEnd = 0;
    int endTime = 200;

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

        cur_heading = imu_get_heading(IMU_PORT) * 10;
//        logInt(cur_heading);
        /*Calculates the error*/
        error = degrees - cur_heading;
        if (error < -1800) {
            error += 3600;
        }
//        logInt(error);

        /*Assigns power to the drivetrain based on the proportional, integral, and derivative terms*/
        int powerToAssign = min(power, max(-power, error * TDRIVEP + integral * TDRIVEI + errorChange * TDRIVED));
        assignLeftRightDriveMotorsVoltage(powerToAssign, -powerToAssign);
    }

    /*After PID is done, stop moving*/
    assignLeftRightDriveMotorsVoltage(0, 0);
}

/**
This method maintains the correct heading while the robot is moving a certain distance. The parameters are
ticks, which is the number of total ticks this movement should take, the power to assign to the drivetrain,
and the heading which it should maintain throughout this movement.

This function does not use PID for dist driven. Only heading is corrected.
*/
void coastHeadingWithoutDrivePID(int ticks, int power, int heading) {
    /*Initializes the ticks and power*/
//    ticks *= 200.0 / RPM;
//    power *= 200.0 / RPM;

    /*Initializes the error*/
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

void coastHeadingWithoutDrivePIDWithoutHeadingTask(int inches, int power, int heading) {
    int ticks = convertInchesToTicks(inches);

    /*Initializes the error*/
    int error = 0;

    /*Tares the drive motors in order to read how much they have moved (below)*/
    clearDriveMotors();

    /*Assigns initial values to the drivetrain motors, which was calculated above*/
    assignLeftRightDriveMotorsVoltage(power, power);

    /*Makes sure that the total movement by the drive motors does not exceed the number of ticks it should be moving*/
    while (abs(motor_get_position(PORT_DRIVELEFTFRONT)) + abs(motor_get_position(PORT_DRIVELEFTBACK)) + abs(motor_get_position(PORT_DRIVELEFTMIDDLE)) +
           abs(motor_get_position(PORT_DRIVERIGHTFRONT)) + abs(motor_get_position(PORT_DRIVERIGHTBACK)) + abs(motor_get_position(PORT_DRIVERIGHTMIDDLE)) <
           ticks * NUMDRIVEMOTORS) {
        /*Calculates the error of the heading: where it should be minus where it is*/
        error = heading - imu_get_heading(IMU_PORT);

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
This method maintains the correct heading while the robot is moving a certain distance. The parameters are
ticks, which is the number of total ticks this movement should take, the power to assign to the drivetrain,
the heading which it should maintain throughout this movement, and the timeout limit (i.e. time taken before
either the number of ticks is reached or the time limit is met and the while loop exits)
 @param heading [-3600, 3600]
*/
void coastHeadingWithTimeout(int ticks, int power, int heading, bool corr, int timeoutLimit) {
    /*Initializes the ticks and power*/
    ticks *= 200.0 / RPM;
    power *= 200.0 / RPM;

    /*Initializes the error*/
    int error = 0;

    int curTime = millis();

    /*Tares the drive motors in order to read how much they have moved (below)*/
    clearDriveMotors();

    /*Assigns initial values to the drivetrain motors, which was calculated above*/
    assignLeftRightDriveMotorsVoltage(power, power);

    /*Makes sure that the total movement by the drive motors does not exceed the number of ticks it should be moving*/
    while ((abs(motor_get_position(PORT_DRIVELEFTFRONT)) + abs(motor_get_position(PORT_DRIVELEFTBACK)) + abs(motor_get_position(PORT_DRIVELEFTMIDDLE)) +
            abs(motor_get_position(PORT_DRIVERIGHTFRONT)) + abs(motor_get_position(PORT_DRIVERIGHTBACK)) + abs(motor_get_position(PORT_DRIVERIGHTMIDDLE))<
            ticks * NUMDRIVEMOTORS) && (millis() - curTime < timeoutLimit)) {

        /*Calculates the error of the heading: where it should be minus where it is*/
        error = heading - current_heading;
        if (!corr) {
            error = 0;
        }

        /*
        This assigns the power to the motors. It uses the P term of PID: proportional. It multiplies
        the error with a constant and uses that to adjust the current power of the drivetrain, ensuring
        that it maintains the heading it is supposed to.
        */
        assignLeftRightDriveMotorsVoltage(power + error * CORRD, power - error * CORRD);

        delay(5);
    }

    /*Stops movement after number of ticks has been reached*/
    assignLeftRightDriveMotorsVoltage(0, 0);
}

/**
 * Drive forwards at a set heading (no PID distance correction).
 * @param ticks
 * @param power
 * @param heading
 */
void forwardCoastHeading(int ticks, int power, int heading) {
    coastHeadingWithoutDrivePID(ticks, power, heading);
}

/**
 * Drive forwards at a set heading for a set time limit (no PID distance correction)
 * @param ticks
 * @param power
 * @param heading
 */
void forwardCoastHeadingWithTimeout(int ticks, int power, int heading) {
    coastHeadingWithTimeout(ticks, power, heading, true, TIMEOUT_LIMIT);
}

/**
 * Drive backwards at a set heading (no PID distance correction).
 * @param ticks
 * @param power
 * @param heading
 */
void backwardCoastHeading(int ticks, int power, int heading) {
    coastHeadingWithoutDrivePID(ticks, -power, heading);
}

/**
 * Turning left with PID.
 * @param degrees [-3600, 3600]
 * @param power [-127, 127]
 */
void turnLeft(int degrees, int power) {
    turnGyro(degrees, power);
}

/**
 * Turning right with PID.
 * @param degrees [-3600, 3600]
 * @param power [-127, 127]
 */
void turnRight(int degrees, int power) {
    turnGyro(degrees, power);
}

/**
 * Turns right by number of ticks and without PID correction.
 * @param ticks [tick conversion?]
 * @param power [-127, 127]
 * @param clear (whether or not to clear the motor encoders)
 */
void turnRightWithTicksAndTimeout(int ticks, int power, bool clear) {
    assignDriveMotorsDist(-ticks, ticks, power, clear, true);
}

/**
 * Turns left by number of ticks and without PID correction.
 * @param ticks [tick conversion?]
 * @param power [-127, 127]
 * @param clear (whether or not to clear the motor encoders)
 */
void turnLeftWithTicksAndTimeout(int ticks, int power, bool clear) {
    assignDriveMotorsDist(ticks, -ticks, power, clear, true);
}

/**
 * Acts as a brake without brake mode. After called, motors run at parameter {power} for 150 ms
 * before they are assigned to 0, acting as a slight buffer before the motors are directly
 * halted (which should be better for the motors).
 * @param power [-127, 127]
 */
void brake(int power) {
    assignDriveMotorsVoltage(power);
    delay(150);
    assignDriveMotorsVoltage(0);
}

void turnBrake(int power, bool ccw) {
    if (ccw) {
        assignLeftRightDriveMotorsVoltage(-power, power);
    } else assignLeftRightDriveMotorsVoltage(power, -power);
}

/** FUNCTIONS WITH SENSORS
 * Sensors help provide the robot with a way to perceive (and thereby act accordingly to) their surroundings.
 */


/**
This method allows the robot to move backwards towards the wall until it is a certain distance
away from it. It uses the distance sensor on the back of the robot. The parameters are the distance
from the wall that the robot should be in millimeters, the number of maximum ticks it is allowed
to move, in case the sensor malfunctions, the power of the movement, and the heading it should be
facing while coasting. This function uses PID to perform this movement as it is important to have this
as accurate as possible.
 * @param distanceFromWall
 * @param allowedTicks
 * @param power [-127, 127]
 * @param heading
*/
void backwardCoastToWall(int distanceFromWall, int allowedTicks, int power, int heading) {
    /* This variable keeps track of the total ticks it has currently moved.*/
    int totalTicks = 0;

    /* While the ticks it has moved is less than the ticks it is allowed to move and the
    value gotten the distance sensor is larger than the distance it is supposed ot be from the wall, execute.
    */
    while((totalTicks < allowedTicks) && (distance_get(DISTANCE_OUT) > distanceFromWall)) {
        /* Moves backwards at the given power and heading for 10 ticks and updates the counter.*/
        backwardCoastHeading(10, power, heading);
        totalTicks += 10;
    }

    /* Once the movement is done, brake the drivetrain.*/
    brake(10);
}
/**
 * Go forwards until distance sensor reads @param distanceFromWall [in mm]. No PID is used for speed (tipping point goal rush).
 * @param allowedTime Time allowed to drive forwards
 * @param power [-127, 127]
 */
void forwardUntilDistFromWallNoHeadingCorrection(int distanceFromWall, int allowedTime, int power) {
    int curTime = millis();

    /* While the distance sensor on the arm reads a value larger than inputted and the time
    that has elapsed is less than the time allowed.
    */
    while(distance_get(DISTANCE_IN) < distanceFromWall && (millis() - curTime < allowedTime)) {
        /* Assigns the drivetrain the power inputted. PID is not used here because it is slower than
        directly assigning power: PID assigns different powers to either side of the drivetrain
        because it attempts to maintain the correct heading, making it slower.
        */
        assignDriveMotorsVoltage(power);
        delay(10);
    }
}

/**
 * Coast forwards at a particular heading until robot is {distanceFromWall} away from the wall. Does not brake after
 * while loop exits.
 * @param distanceFromWall [in mm]
 * @param allowedTicks
 * @param power [-127, 127]
 * @param heading [-3600, 3600]
 */
void forwardCoastHeadingUntilDistFromWallWithoutBrake(int distanceFromWall, int allowedTicks, int power, int heading) {
    int curticks = 0;
    while(distance_get(DISTANCE_OUT) < distanceFromWall && (curticks < allowedTicks)) {
        forwardCoastHeading(10, power, heading);
        curticks += 10;
    }
}

/**
 * Coast forwards at a particular heading until robot is {distanceFromWall} away from the wall. Brakes (with a -10 buffer
 * before setting motors to 0) after while loop exits.
 * @param distanceFromWall (mm)
 * @param allowedTicks
 * @param power [-127, 127]
 * @param heading [-3600, 3600]
 */
void forwardCoastHeadingUntilDistFromWallWithBrake(int distanceFromWall, int allowedTicks, int power, int heading) {
    int curticks = 0;
    while(distance_get(DISTANCE_OUT) < distanceFromWall && (curticks < allowedTicks)) {
        forwardCoastHeading(10, power, heading);
        curticks += 10;
    }
    /*Once the movement is done, brake. */
    brake(-10);
}

/**
 * Coast forwards at a particular heading until robot is {distanceFromWall} away from the wall or time limit {allowedTime}
 * is met. Brakes (with a -10 buffer before setting motors to 0) after while loop exits.
 * @param distanceFromWall (mm)
 * @param allowedTime
 * @param power [-127, 127]
 * @param heading [-3600, 3600]
 */
void forwardCoastHeadingUntilDistFromWallWithBrakeWithTime(int distanceFromWall, int allowedTime, int power, int heading) {
    int curTime = millis();
    while(((millis() - curTime) < allowedTime) && (distance_get(FRONTDISTANCE) > distanceFromWall)) {
        assignDriveMotorsVoltage(power);
        delay(10);
    }

    /* Once the movement is done, brake.*/
    brake(-10);
}

void driveInches(int inches, int voltage) {
    clearDriveMotors();
    int curDist = 0;
    assignDriveMotorsVoltage(voltage);
    while (curDist < inches) {
        delay(20);
        curDist = readDistanceInches();
    }
}

/**
 * Sample bumper sensor function. Rewrite if we use bumper sensors again.
 * @param allowedTime
 */
//void clampBumper(int allowedTime) {
//    int curTime = millis();
//    /* While a bumper press is not felt and the time passed has not
//     * exceeded the time allowed, execute. */
//    while(digitalRead(ARMBUMPER) == LOW && (millis() - curTime < allowedTime)) {
//        delay(10);
//    }
////    armClampIn();         clamp when ARMBUMPER is HIGH (no longer LOW) --> goal is pushing into sensor
//}

//VISION

/**
 * Aim via vision
 * @param color [0 = red; 1 = blue]
 */
void visionAim(int color, int power, int timeout) {
    int startTime = millis();
    vision_object_s_t goal;
    if (color == 0) {
        // red
        goal = vision_get_by_sig(VISION, 0, 2); // 0 is largest signature [1st in list]
    } else {
        // blue
        goal = vision_get_by_sig(VISION, 0, 1); // 0 is largest signature [1st in list]
    }
    int goalCenterX = goal.x_middle_coord;
    logInt(goalCenterX);
//    logStringWithFloat("starting goal x = ", goalCenterX*1.0);
    bool seeSomething = true;
    if (goalCenterX == 0) {
        seeSomething = false; // do nothing if you don't see anything
    }
    float centerOfVisionField = 0; // 316 pixels horizontally; coor. set center to 0,0
    bool ccw = false;
    if (goalCenterX < centerOfVisionField) {
        ccw = true;
        logStringWithFloat("ccw = ", 1);
    } // if goalCenterX > centerOfVisionField, ccw = false
    logStringWithFloat("ccw = ", 0);
    while (seeSomething && fabs(goalCenterX - centerOfVisionField) > 2 && ((millis() - startTime) < timeout)) {
        double adjustPower = goalCenterX * VISIONCONST;
        if (ccw) {
            assignLeftRightDriveMotorsVoltage(power + adjustPower, -power - adjustPower);
        } else {
            assignLeftRightDriveMotorsVoltage(-power - adjustPower, power + adjustPower);
        }
        delay(5);
        if (color == 0) goal = vision_get_by_sig(VISION, 0, 2); // 0 is largest signature [1st in list]
        else goal = vision_get_by_sig(VISION, 0, 1); // 0 is largest signature [1st in list]
        goalCenterX = goal.x_middle_coord;
    }
//    driveBrake();

    if (ccw) {
        assignLeftRightDriveMotorsVoltage(-abs(power)/2, abs(power)/2);
    } else assignLeftRightDriveMotorsVoltage(abs(power)/2, -abs(power)/2);
    printf("done");
    delay(50);
    assignDriveMotorsVoltage(0);
}

/**
 * Moving to goal with vision. Function leftover from tp, use as reference for moving to an object with vision.
 * @param timeout [VISIONTIMEOUT]
 */
void forwardToGoalWithVision(int timeout) {
    int startTime = millis();

    vision_object_s_t goal = vision_get_by_sig(VISION_FRONT, 0, 2);
    int visionValueX = goal.x_middle_coord;
    bool flag = true;
    if(visionValueX == 0) {
        flag = false; // flag: do nothing if you don't see anything
    }
    while (flag && distance_get(ARMDISTANCE) > 50 && ((millis() - startTime) < timeout)) {
        goal = vision_get_by_sig(VISION_FRONT, 0, 2);
        visionValueX = goal.x_middle_coord;
        double adjustPower = visionValueX * VISIONCONST;

        double leftSidePower = 110 + adjustPower;
        double rightSidePower = 110 - adjustPower;

        motor_move(PORT_DRIVELEFTFRONT, leftSidePower);
        motor_move(PORT_DRIVELEFTMIDDLE, leftSidePower);
        motor_move(PORT_DRIVELEFTBACK, leftSidePower);

        motor_move(PORT_DRIVERIGHTFRONT, rightSidePower);
        motor_move(PORT_DRIVERIGHTMIDDLE, rightSidePower);
        motor_move(PORT_DRIVERIGHTBACK, rightSidePower);

        delay(10);
    }
    if (flag) {
//        digitalWrite(ARMCLAMP, LOW); // activate pneumatics: low: push in, high: push out
    }
    motor_move(PORT_DRIVELEFTFRONT, -5);
    motor_move(PORT_DRIVELEFTMIDDLE, -5);
    motor_move(PORT_DRIVELEFTBACK, -5);

    motor_move(PORT_DRIVERIGHTFRONT, -5);
    motor_move(PORT_DRIVERIGHTMIDDLE, -5);
    motor_move(PORT_DRIVERIGHTBACK, -5);
    delay(50);
    motor_move(PORT_DRIVELEFTFRONT, 0);
    motor_move(PORT_DRIVELEFTMIDDLE, 0);
    motor_move(PORT_DRIVELEFTBACK, 0);

    motor_move(PORT_DRIVERIGHTFRONT, 0);
    motor_move(PORT_DRIVERIGHTMIDDLE, 0);
    motor_move(PORT_DRIVERIGHTBACK, 0);

    timeTakenByFunction = millis() - startTime;
}

/**
 * Going backwards to target with vision. Sample function from tp

void backToGoalWithVision() {
    int startTime = millis();

    vision_object_s_t goal = vision_get_by_sig(VISION_BACK, 0, 1);
    int visionValueX = goal.x_middle_coord;
    int currentSig = 1;
    bool flag = true;

    if(visionValueX == 0) {
        goal = vision_get_by_sig(VISION_BACK, 0, 3);

        visionValueX = goal.x_middle_coord;
        currentSig = 3; // what's currentSig?
        if(visionValueX == 0) {
            flag = false;
        }
    }
    while(flag && digitalRead(BACKBUMPER) == LOW) {
        goal = vision_get_by_sig(VISION_BACK, 0, currentSig);
        visionValueX = goal.x_middle_coord;
        double adjustPower = visionValueX * VISIONCONST;

        double leftSidePower = -90 + adjustPower;
        double rightSidePower = -90 - adjustPower;

        motor_move(PORT_DRIVELEFTFRONT, leftSidePower);
        motor_move(PORT_DRIVELEFTMIDDLE, leftSidePower);
        motor_move(PORT_DRIVELEFTBACK, leftSidePower);

        motor_move(PORT_DRIVERIGHTFRONT, rightSidePower);
        motor_move(PORT_DRIVERIGHTMIDDLE, rightSidePower);
        motor_move(PORT_DRIVERIGHTBACK, rightSidePower);

        delay(10);
    }
    if (flag) {
        digitalWrite(BACKCLAMP, LOW);
    }
    motor_move(PORT_DRIVELEFTFRONT, 5);
    motor_move(PORT_DRIVELEFTMIDDLE, 5);
    motor_move(PORT_DRIVELEFTBACK, 5);

    motor_move(PORT_DRIVERIGHTFRONT, 5);
    motor_move(PORT_DRIVERIGHTMIDDLE, 5);
    motor_move(PORT_DRIVERIGHTBACK, 5);

    delay(50);

    motor_move(PORT_DRIVELEFTFRONT, 0);
    motor_move(PORT_DRIVELEFTMIDDLE, 0);
    motor_move(PORT_DRIVELEFTBACK, 0);

    motor_move(PORT_DRIVERIGHTFRONT, 0);
    motor_move(PORT_DRIVERIGHTMIDDLE, 0);
    motor_move(PORT_DRIVERIGHTBACK, 0);

    timeTakenByFunction = millis() - startTime;
}
 **/

/**
 * Going backwards with vision to target at specified power.
 * @param inputPower

void backToGoalWithVisionPower(int inputPower) {
    int startTime = millis();

    vision_object_s_t goal = vision_get_by_sig(VISION_BACK, 0, 1);
    int visionValueX = goal.x_middle_coord;
    int currentSig = 1;
    bool flag = true;

    //what is this chunk doing
    if(visionValueX == 0) {
        goal = vision_get_by_sig(VISION_BACK, 0, 3);

        visionValueX = goal.x_middle_coord;
        currentSig = 3; //what's this
        if(visionValueX == 0) {
            flag = false;
        }
    }

    while (flag && digitalRead(BACKBUMPER) == LOW && ((millis() - startTime) < 1200)) {
        goal = vision_get_by_sig(VISION_BACK, 0, currentSig);
        visionValueX = goal.x_middle_coord;
        double adjustPower = visionValueX*0.4;

        double leftSidePower = -inputPower + adjustPower;
        double rightSidePower = -inputPower - adjustPower;

        motor_move(PORT_DRIVELEFTFRONT, leftSidePower);
        motor_move(PORT_DRIVELEFTMIDDLE, leftSidePower);
        motor_move(PORT_DRIVELEFTBACK, leftSidePower);

        motor_move(PORT_DRIVERIGHTFRONT, rightSidePower);
        motor_move(PORT_DRIVERIGHTMIDDLE, rightSidePower);
        motor_move(PORT_DRIVERIGHTBACK, rightSidePower);

        delay(10);
    }

    if (flag) {
        digitalWrite(BACKCLAMP, LOW);
    }
    motor_move(PORT_DRIVELEFTFRONT, 5);
    motor_move(PORT_DRIVELEFTMIDDLE, 5);
    motor_move(PORT_DRIVELEFTBACK, 5);

    motor_move(PORT_DRIVERIGHTFRONT, 5);
    motor_move(PORT_DRIVERIGHTMIDDLE, 5);
    motor_move(PORT_DRIVERIGHTBACK, 5);

    delay(50);

    motor_move(PORT_DRIVELEFTFRONT, 0);
    motor_move(PORT_DRIVELEFTMIDDLE, 0);
    motor_move(PORT_DRIVELEFTBACK, 0);

    motor_move(PORT_DRIVERIGHTFRONT, 0);
    motor_move(PORT_DRIVERIGHTMIDDLE, 0);
    motor_move(PORT_DRIVERIGHTBACK, 0);

    timeTakenByFunction = millis() - startTime;
}
 */

/** LVGL **/
lv_obj_t* drawRectangle( int x, int y, int width, int height, lv_color_t color ) {
    lv_obj_t * obj1 = lv_obj_create(lv_scr_act(), NULL);

    lv_style_t *style1 = (lv_style_t *)malloc( sizeof( lv_style_t ));
    lv_style_copy(style1, &lv_style_plain_color);    /*Copy a built-in style to initialize the new style*/
    style1->body.empty = 1;
    style1->body.border.color = color;
    style1->body.border.width = 1;
    style1->body.border.part = LV_BORDER_FULL;

    lv_obj_set_style(obj1, style1);
    lv_obj_set_pos(obj1, x, y);
    lv_obj_set_size(obj1, width, height);

    return obj1;
}
lv_obj_t* drawRectangleWithText (int x, int y, int width, int height, lv_color_t color, char text[]) {
    lv_obj_t * obj1 = lv_obj_create(lv_scr_act(), NULL);

    lv_style_t *style1 = (lv_style_t *)malloc( sizeof( lv_style_t ));
    lv_style_copy(style1, &lv_style_plain_color);    /*Copy a built-in style to initialize the new style*/
    style1->body.empty = 1;
    style1->body.border.color = color;
    style1->body.border.width = 1;
    style1->body.border.part = LV_BORDER_FULL;

    lv_obj_set_style(obj1, style1);
    lv_obj_set_pos(obj1, x, y);
    lv_obj_set_size(obj1, width, height);

    return obj1;
}

// OPTICAL SENSOR (COLOR)

/**
 *
 * @return 0 if red, 1 if blue
 */
 int printCount = 0;
int colorOfRoller() {
//    optical_set_led_pwm(OPTICAL, 50);
//    delay(50);
    printCount++;
    float hue = optical_get_hue(OPTICAL);
    if (hue < 25 || hue > 322 && hue < 360) {
        if (printCount % 10 == 0) {
            logStringWithFloat("red", hue);
        }
//        optical_set_led_pwm(OPTICAL, 0);
        return 0;
    } else if (hue > 204 && hue <= 260) {
        if (printCount % 10 == 0) {
            logStringWithFloat("blue", hue);
        }
//        optical_set_led_pwm(OPTICAL, 0);
        return 1;
    } else {
        if (printCount % 10 == 0) {
            logStringWithFloat("weird optical number", hue);
        }
        return -1;
    }
}

// GPS SENSOR
/**
 * Auto aiming with GPS sensor
 * @param goalX
 * @param goalY
 * @param frontGPSsensor: if true, use front GPS; if false, use back
 * @param threeTile: if true, we start on 3 tile side; if false, we start on 2 tile side
 *
 * @return
 */
float autonGPSAngle(double goalX, double goalY, bool frontGPSsensor, bool threeTile, bool color) {
    // accessing robot position
    gps_status_s_t status;
    if (frontGPSsensor == true) {
        status = gps_get_status(GPSFRONT_PORT);
    } else status = gps_get_status(GPSBACK_PORT);
    double robotX = status.x;
    double robotY = status.y;

    double xDiff = goalX - robotX;
    double yDiff = goalY - robotY; // if tan is negative, robot turn left; if tan is positive, robot turn right

    double idealAngle = atan(yDiff / xDiff) * 180 / PI; //-86

    idealAngle = -idealAngle; // 86

    if (idealAngle > 45) {
        idealAngle = -idealAngle;  //-86
    }
    logStringWithFloat("GPS ", idealAngle);
    turnGyro(-idealAngle * 10, 127);
    brake(-10);
}

#define MAXTURNLIMIT 3200

int errorDiff(int goal, int cur, bool ccw) {
    int diff;

    // calculate turn degree diff if going clockwise
    if (goal < cur)
        diff = goal + 3600 - cur;
    else
        diff = goal - cur;

    // correct turn degree if counter clockwise is set as true
    if (ccw)
        diff = 3600 - diff;

    // up to here diff should be positive value
    // check if turn degree is over maxlimit, indicating overshoot, set diff as negative diff
    if (diff > MAXTURNLIMIT)
        diff = diff - 3600;

    return diff;
}

/**
 *
 * @param angle = absolute angle to turn to based on GPS coor.
 */
void turnGPSAbsoluteGyro(double targetAngle, double speed) {
    if (targetAngle < 0) {
        targetAngle += 360;
    }
    double currentAngle = gps_get_heading(GPSFRONT_PORT);
//    logStringWithFloat("gps start angle = ", currentAngle);
    double difference = targetAngle - currentAngle;
    if (difference > 180) {
        difference -= 360;
    } else if (difference < -180) {
        difference += 360;
    }

    // converting so that [-180, 180] becomes [0, 360)
//    if (targetAngle < 0) {
//        targetAngle += 3600;
//    }
//    double currentAngle = gps_get_heading(GPSFRONT_PORT) * 10;
//    logStringWithFloat("gps start angle = ", currentAngle);
//    double difference = targetAngle - currentAngle;
//    if (difference > 1800) {
//        difference -= 3600;
//    } else if (difference < -1800) {
//        difference += 3600;
//    }
//
//    uint32_t cur = millis();
//    uint32_t now;
//    int tmp;
//    int errorChange = 0;
//    int prevError;
//    int integral = 0;
//
//    bool endSequence = false;
//    int startEnd = 0;
//    int endTime = 100;//200; //500
//    int powerToAssign;
//
//    // base on degree to turn, decide on staring power, and when to go into PID (last few degrees)
//    prevError = fabs(difference);
//    logStringWithFloat("fabs(difference) at start = ", prevError);
//    if (prevError >= 1500) {
//        speed = 127;
//        tmp = 500;
//    } else if (prevError >= 600) {
//        speed = 127;
//        tmp = 400;
//    } else if (prevError >= 200) {
//        speed = 63;
//        tmp = max(prevError/2, 15);
//    } else {
//        speed = 63;
//        tmp = 0;
//    }
//    bool left = false;
//    if (difference > 0) {
//        // clockwise
//        // left = false
//        powerToAssign = -speed;
//    } else { // counterclockwise
//        left = true;
//        powerToAssign = speed;
//    }
//
//    int count = 0;
//    if (tmp > 0) { // use start power to turn until reach degree to go into PID
//        now = millis();
//        assignLeftRightDriveMotorsVoltage(-powerToAssign, powerToAssign);
//        do {
//            task_delay_until(&now, 5);
//            now += 5;
//            currentAngle = gps_get_heading(GPSFRONT_PORT) * 10;
//            difference = targetAngle - currentAngle;
//            if (difference > 1800) {
//                difference -= 3600;
//            } else if (difference < -1800) {
//                difference += 3600;
//            }
//            if (count % 50 == 0 || count == 0) logStringWithFloat("difference = ", difference);
//            count ++;
//        } while (fabs(difference) > tmp);
//    }
//
//    now = millis();
//    prevError = difference;
//    logStringWithFloat("prev error before PID start = ", difference);
//    while ((fabs(errorChange) > TDRIVEMAXVEL || (!endSequence || (millis() - startEnd < endTime))) && millis() - cur < 900) {
//        if (fabs(difference) < TDRIVEPOSTOL && !endSequence) {
//            endSequence = true;
//            startEnd = millis();
//        }
//
//        integral += difference;
//        if (integral > TDRIVEINTEGRALLIMIT) {
//            integral = TDRIVEINTEGRALLIMIT;
//        } else if (integral < -TDRIVEINTEGRALLIMIT) {
//            integral = -TDRIVEINTEGRALLIMIT;
//        }
//
//        errorChange = difference - prevError;
//        prevError = difference;
//
//        task_delay_until(&now, 5);
//        now += 5;
//
//        currentAngle = gps_get_heading(GPSFRONT_PORT) * 10;
//        difference = targetAngle - currentAngle;
//        if (difference > 1800) {
//            difference -= 3600;
//        } else if (difference < -1800) {
//            difference += 3600;
//        }
//        powerToAssign = min(speed, max(-speed, difference * TDRIVEP + integral * TDRIVEI + errorChange * TDRIVED));
//        if (left)
//            powerToAssign = -powerToAssign;
//        assignLeftRightDriveMotorsVoltage(-powerToAssign, powerToAssign);
//    }
//    //brake:
//    if (left) {
//        assignLeftRightDriveMotorsVoltage(-abs(speed)/2, abs(speed)/2);
//    } else assignLeftRightDriveMotorsVoltage(abs(speed)/2, -abs(speed)/2);
//    delay(50);
//    assignDriveMotorsVoltage(0);

    bool left = false;
    if (difference > 0) {
        //right; left = false
        assignLeftRightDriveMotorsVoltage(-speed, speed);
    } else if (difference < 0) {
        // left
        left = true;
        assignLeftRightDriveMotorsVoltage(speed, -speed);
    } else return;

    double totalDifference = difference;

//    int count = 0;
    int start = millis();
    while (fabs(difference) > 1 && ((millis() - start) < 3000)) {
//           count++;
        delay(5);
        currentAngle = gps_get_heading(GPSFRONT_PORT);
        difference = targetAngle - currentAngle;
        if (difference > 180) {
            difference -= 360;
        } else if (difference < -180) {
            difference += 360;
        }
//        if (count % 50 == 0 || count == 0) {
//            logStringWithFloat("error = ", currentAngle);
//        }
        if (fabs(difference) < fabs(totalDifference) * 0.4) {
            if (left) {
                assignLeftRightDriveMotorsVoltage(20, -20);
            } else assignLeftRightDriveMotorsVoltage(-20, 20);

        }
    }
//    brake(-1);
    if (left) {
        assignLeftRightDriveMotorsVoltage(-abs(speed)/2, abs(speed)/2);
    } else assignLeftRightDriveMotorsVoltage(abs(speed)/2, -abs(speed)/2);
    delay(50);
    assignDriveMotorsVoltage(0);
//    currentAngle = gps_get_heading(GPSFRONT_PORT);
//    logStringWithFloat("XXXXXXXXX = ", fabs(currentAngle - targetAngle));
}


/**
 *
 * @param goalX
 * @param goalY
 * @param frontGPSsensor
 * @param red if true: red goal; if false: blue goal
 */
void skillsGPSAim(double goalX, double goalY, bool frontGPSsensor, bool red, bool bashCorrection) {
    // getting GPS position
    gps_status_s_t status;
    if (frontGPSsensor == true) {
        status = gps_get_status(GPSFRONT_PORT);
    } else status = gps_get_status(GPSBACK_PORT);
    double robotX = status.x;
    double robotY = status.y;
    //TURN TO GPS HEADING FUNCTION

    double xDiff = goalX - robotX;
    double yDiff = goalY - robotY; // if tan is negative, robot turn left; if tan is positive, robot turn right

    double idealAngleGPSCoor = 0; // angle to turn to in GPS global heading
    double alpha = 00; // random value
    if (red) {
        if (xDiff > 0 && yDiff < 0) {
            alpha = atan(yDiff / xDiff) * 180 / PI;
            idealAngleGPSCoor = 90 - alpha;
        } else if (yDiff == 0 && xDiff > 0) {
            idealAngleGPSCoor = 90;
        } else if (xDiff > 0 && yDiff > 0) {
            double alpha = atan(yDiff / xDiff) * 180 / PI;
            idealAngleGPSCoor = 90 - alpha;
        } else if (xDiff == 0) {
            idealAngleGPSCoor = 0;
        } else if (xDiff < 0 && yDiff > 0) {
            alpha = atan(yDiff / xDiff) * 180 / PI;
            idealAngleGPSCoor = -90 - alpha;
        }
    } else { // blue
        if (yDiff > 0 && xDiff < 0) {
            alpha = atan(yDiff / xDiff) * 180 / PI;
            idealAngleGPSCoor = -90 - alpha;
        } else if (xDiff < 0 && yDiff == 0) {
            idealAngleGPSCoor = -90;
        } else if (xDiff < 0 && yDiff < 0) {
            alpha = atan(yDiff / xDiff) * 180 / PI;
            idealAngleGPSCoor = -90 - alpha;
        } else if (xDiff == 0 && yDiff < 0) {
            idealAngleGPSCoor = -180; // should be same as 180
        } else if (xDiff > 0 && yDiff < 0) {
            alpha = atan(yDiff / xDiff) * 180 / PI;
            idealAngleGPSCoor = 90 - alpha;
        }
    }

    idealAngleGPSCoor = idealAngleGPSCoor / fabs(idealAngleGPSCoor) * (fabs(idealAngleGPSCoor) - bashCorrection);
    turnGPSAbsoluteGyro(idealAngleGPSCoor - bashCorrection, 30);
}

// GAME SPECIFIC FUNCTIONS
/**
 * Turn roller to specified color.
 * @param color [red: 0, blue: 1]
 */
void turnRoller(int color, int timeout) {
    int startTime = millis();
    optical_set_led_pwm(OPTICAL, 100);
    printf("%d\n", color);
    int currentColor = -1;
    assignDriveMotorsVoltage(-30); // drive forwards to press to roller
    int initialColor = colorOfRoller();
    logInt(initialColor);
    printf("HELLO");
    if (initialColor == color) {
        motor_move(PORT_INTAKE, convert100to127(65)); // 65
        currentColor = colorOfRoller();
        while ((currentColor == color || currentColor == -1) && (millis() - startTime < timeout)) {
            delay(5);
            currentColor = colorOfRoller();
        }
        logInt(currentColor);
        while ((currentColor != color || currentColor == -1) && (millis() - startTime < timeout)) {
            delay(5);
            currentColor = colorOfRoller();
        }
        motor_set_brake_mode(PORT_INTAKE, E_MOTOR_BRAKE_BRAKE);
        logInt(currentColor);
        motor_move(PORT_INTAKE, -60); //60
        delay(100);
        motor_move(PORT_INTAKE, 0);
//        motor_move_relative(PORT_INTAKE, 100, 100);
    } else {
        motor_move(PORT_INTAKE, convert100to127(65)); // 65
        currentColor = colorOfRoller();
        while ((currentColor != color || currentColor == -1) && (millis() - startTime < timeout)) {
            delay(5);
            currentColor = colorOfRoller();
        }
//        motor_move_relative(PORT_INTAKE, 100, 100);
        motor_move(PORT_INTAKE, -60); // 60
        delay(100);
        motor_move(PORT_INTAKE, 0);
    }
    assignDriveMotorsVoltage(0);
    optical_set_led_pwm(OPTICAL, 0);
}

void turnRollerReverse(int color, int timeout) {
    int startTime = millis();
    optical_set_led_pwm(OPTICAL, 100);
    printf("%d\n", color);
    int currentColor = -1;
    assignDriveMotorsVoltage(-20); // drive forwards to press to roller
    int initialColor = colorOfRoller();
    logInt(initialColor);
    printf("HELLO");
    if (initialColor == color) {
        motor_move(PORT_INTAKE, -convert100to127(70)); // 45
        currentColor = colorOfRoller();
        while ((currentColor == color || currentColor == -1) && (millis() - startTime < timeout)) {
            delay(5);
            currentColor = colorOfRoller();
        }
        logInt(currentColor);
        while ((currentColor != color || currentColor == -1) && (millis() - startTime < timeout)) {
            delay(5);
            currentColor = colorOfRoller();
        }
        motor_set_brake_mode(PORT_INTAKE, E_MOTOR_BRAKE_BRAKE);
        logInt(currentColor);
        motor_move(PORT_INTAKE, 70); //45
        delay(100);
        motor_move(PORT_INTAKE, 0);
//        motor_move_relative(PORT_INTAKE, 100, 100);
    } else {
        motor_move(PORT_INTAKE, -convert100to127(70)); // 45
        currentColor = colorOfRoller();
        while ((currentColor != color || currentColor == -1) && (millis() - startTime < timeout)) {
            delay(5);
            currentColor = colorOfRoller();
        }
//        motor_move_relative(PORT_INTAKE, 100, 100);
        motor_move(PORT_INTAKE, 70); // 45
        delay(100);
        motor_move(PORT_INTAKE, 0);
    }
    assignDriveMotorsVoltage(0);
    optical_set_led_pwm(OPTICAL, 0);
}

/** game specific functions **/

void flywheelSpinToSpeed(int speed) {
    // speed is cartridge speed
    motor_move(PORT_FLYWHEEL, 127);
    while (motor_get_actual_velocity(PORT_FLYWHEEL) < (speed * 600 / 127)) {
//        logInt(motor_get_actual_velocity(PORT_FLYWHEEL));
        delay(10);
    }
    motor_move(PORT_FLYWHEEL, speed);
}

void spinFlywheelAuton() {
    flywheelSpinToSpeed(127);
}

/**
 * Shoot disk
 * @param numDisk number of disks to shoot
 */
void motorIndexerShootDisk(int numDisk, int flywheelPowerAfterShooting, int curHeading) {
    int heading = imu_get_heading(IMU_PORT);
    if (numDisk == 1) {
        motor_move(PORT_FLYWHEEL, -127);
//        digitalWrite(INDEXER, HIGH);
        delay(200);
        motor_move(PORT_FLYWHEEL, flywheelPowerAfterShooting);
//        digitalWrite(INDEXER, LOW);
    } else if (numDisk == 2) {
        motor_move(PORT_FLYWHEEL, -127);
        int dist = distance_get(DISTANCE_OUT);
        uint32_t timeout = millis();
        while (dist > distanceNoDisk && (millis() - timeout < 600)) {
//                logInt(dist);
            delay(5);
            dist = distance_get(DISTANCE_OUT);
        }
        motor_move(PORT_FLYWHEEL, flywheelPowerAfterShooting + 20);
        delay(500);

//        assignLeftRightDriveMotorsVoltage(80, 75);
//        delay(150);
//        brake(-10);
        driveHeadingWithRampDown(curHeading, 40, 2, 0, false);
        delay(250);

        motor_move(PORT_FLYWHEEL, -127);
        timeout = millis();
        while (distance_get(DISTANCE_OUT) > distanceNoDisk && (millis() - timeout < 600)) {
            delay(5);
        }
        motor_move(PORT_FLYWHEEL, flywheelPowerAfterShooting);
    } else if (numDisk == 3) {
        motor_move(PORT_FLYWHEEL, -127);
        int dist = distance_get(DISTANCE_OUT);
        uint32_t timeout = millis();
        while (dist > distanceNoDisk && (millis() - timeout < 600)) {
//                logInt(dist);
            delay(5);
            dist = distance_get(DISTANCE_OUT);
        }
        motor_move(PORT_FLYWHEEL, min(flywheelPowerAfterShooting + 25, 127));
        delay(350); // was 400

        driveHeadingWithRampDown(curHeading, 40, 1, 0, false);
        delay(200);
        motor_move(PORT_FLYWHEEL, -127);
        timeout = millis();
        while (distance_get(DISTANCE_OUT) > distanceNoDisk && (millis() - timeout < 600)) {
            delay(5);
        }
        motor_move(PORT_FLYWHEEL, min(flywheelPowerAfterShooting + 25, 127));
        delay(300); // was 500 WE'RE DECREASING TO SAVE TIME

        driveHeadingWithRampDown(curHeading, 40, 2, 0, false);
        delay(200);

        motor_move(PORT_FLYWHEEL, -127);
        timeout = millis();
        while (distance_get(DISTANCE_OUT) > distanceNoDisk && (millis() - timeout < 600)) {
            delay(5);
        }
        motor_move(PORT_FLYWHEEL, flywheelPowerAfterShooting);
    }
}

/**
 *
 * @param numDisk
 * @param flywheelPowerAfterShooting
 * @param adjustHeading [if adjustHeading > 0, adjust left; if adjustHeading < 0, adjust right]
 */
void motorIndexerShootDiskTurnAdjust(int numDisk, int flywheelPowerAfterShooting, int adjustHeading) {
    int heading = imu_get_heading(IMU_PORT);
    if (numDisk == 1) {
        motor_move(PORT_FLYWHEEL, -127);
//        digitalWrite(INDEXER, HIGH);
        delay(200);
        motor_move(PORT_FLYWHEEL, flywheelPowerAfterShooting);
//        digitalWrite(INDEXER, LOW);
    } else if (numDisk == 2) {
        motor_move(PORT_FLYWHEEL, -127);
        int dist = distance_get(DISTANCE_OUT);
        uint32_t timeout = millis();
        while (dist > distanceNoDisk && (millis() - timeout < 600)) {
//                logInt(dist);
            delay(5);
            dist = distance_get(DISTANCE_OUT);
        }
        motor_move(PORT_FLYWHEEL, flywheelPowerAfterShooting + 20);
        delay(300);

//        assignLeftRightDriveMotorsVoltage(80, 75);
//        delay(150);
//        brake(-10);
        turnGyro(heading + adjustHeading, 50);
        brake(-10);
        delay(200);

        motor_move(PORT_FLYWHEEL, -127);
        timeout = millis();
        while (distance_get(DISTANCE_OUT) > distanceNoDisk && (millis() - timeout < 600)) {
            delay(5);
        }
        motor_move(PORT_FLYWHEEL, flywheelPowerAfterShooting);
    } else if (numDisk == 3) {
        motor_move(PORT_FLYWHEEL, -127);
        int dist = distance_get(DISTANCE_OUT);
        uint32_t timeout = millis();
        while (dist > distanceNoDisk && (millis() - timeout < 600)) {
//                logInt(dist);
            delay(5);
            dist = distance_get(DISTANCE_OUT);
        }
        motor_move(PORT_FLYWHEEL, min(flywheelPowerAfterShooting + 25, 127));
        delay(350); // was 400

        turnGyro(heading + adjustHeading, 50);
        brake(-10);
        delay(200);
        motor_move(PORT_FLYWHEEL, -127);
        timeout = millis();
        while (distance_get(DISTANCE_OUT) > distanceNoDisk && (millis() - timeout < 600)) {
            delay(5);
        }
        motor_move(PORT_FLYWHEEL, min(flywheelPowerAfterShooting + 25, 127));
        delay(300); // was 500 WE'RE DECREASING TO SAVE TIME

        turnGyro(heading + adjustHeading, 50);
        brake(-10);
        delay(200);

        motor_move(PORT_FLYWHEEL, -127);
        timeout = millis();
        while (distance_get(DISTANCE_OUT) > distanceNoDisk && (millis() - timeout < 600)) {
            delay(5);
        }
        motor_move(PORT_FLYWHEEL, flywheelPowerAfterShooting);
    }
}

void shoot3Skills(int finalPower) {
    motor_move(PORT_FLYWHEEL, -127);
    int dist = distance_get(DISTANCE_OUT);
    uint32_t timeout = millis();
    while (dist > distanceNoDisk && (millis() - timeout < 600)) {
//                logInt(dist);
        delay(5);
        dist = distance_get(DISTANCE_OUT);
    }

    motor_move(PORT_FLYWHEEL, 127);
    delay(400);

    motor_move(PORT_FLYWHEEL, -127);
    timeout = millis();
    while (distance_get(DISTANCE_OUT) > distanceNoDisk && (millis() - timeout < 600)) {
        delay(5);
    }
    motor_move(PORT_FLYWHEEL, 127);
    delay(700);
    motor_move(PORT_FLYWHEEL, -127);

    timeout = millis();
    while (distance_get(DISTANCE_OUT) > distanceNoDisk && (millis() - timeout < 600)) {
        delay(5);
    }
    motor_move(PORT_FLYWHEEL, finalPower);

    delay(10);
}

void shoot3SkillsUpdated(int finalPower) {
    uint32_t timeout = millis();
    int checkCount = 0;

    motor_move(PORT_FLYWHEEL, -127);

    int dist = distance_get(DISTANCE_OUT);

    //shooting first one
    while (dist > distanceNoDisk) {
        delay(5);
        dist = distance_get(DISTANCE_OUT);
        if (checkCount % 10 == 0 || checkCount == 0) {
            uint32_t ctime = millis();
            if (ctime - timeout > 1000) {
                motor_move(PORT_FLYWHEEL, finalPower);
                printf("exit while 1");
                break;
            }
        }
        checkCount++;
    }
    checkCount = 0; // resetting for next while

    motor_move(PORT_FLYWHEEL, 127);
    delay(200);

    motor_move(PORT_FLYWHEEL, -127);
    timeout = millis();
    while (distance_get(DISTANCE_OUT) > distanceNoDisk) {
        delay(5);
        uint32_t ctime = millis();
        if (ctime - timeout > 1000) {
            motor_move(PORT_FLYWHEEL, finalPower);
            printf("exit while 2");
            break;
        }
    }
    checkCount = 0; // resetting again

    motor_move(PORT_FLYWHEEL, 127);
    delay(200);
    motor_move(PORT_FLYWHEEL, -127);
    timeout = millis();
    while (distance_get(DISTANCE_OUT) > distanceNoDisk) {
        delay(5);
        uint32_t ctime = millis();
        if (checkCount % 10 == 0 || checkCount == 0) { // to check less, not really necessary
            if (ctime - timeout > 1000) {
                motor_move(PORT_FLYWHEEL, finalPower);
                printf("exit while 3");
                break;
            }
        }
        checkCount++;
    }
    motor_move(PORT_FLYWHEEL, finalPower);
}

void shoot2Skills(int finalPower) {
    motor_move(PORT_FLYWHEEL, -127);
    int dist = distance_get(DISTANCE_OUT);
    uint32_t timeout = millis();
    while (dist > distanceNoDisk && (millis() - timeout < 600)) {
//                logInt(dist);
        delay(5);
        dist = distance_get(DISTANCE_OUT);
    }

    motor_move(PORT_FLYWHEEL, 127);
    delay(600);

    motor_move(PORT_FLYWHEEL, -127);
    timeout = millis();
    while (distance_get(DISTANCE_OUT) > distanceNoDisk && (millis() - timeout < 600)) {
        delay(5);
    }
    motor_move(PORT_FLYWHEEL, finalPower);
//    delay(700);
//    motor_move(PORT_FLYWHEEL, -127);

//    timeout = millis();
//    while (distance_get(DISTANCE) > distanceNoDisk && (millis() - timeout < 600)) {
//        delay(5);
//    }
//    motor_move(PORT_FLYWHEEL, finalPower);

    delay(10);
}


void motorIndexerShootDiskSlow(int numDisk, int flywheelPowerAfterShooting, int curHeading) {
    int heading = imu_get_heading(IMU_PORT);
    if (numDisk == 1) {
        motor_move(PORT_FLYWHEEL, -127);
//        digitalWrite(INDEXER, HIGH);
        delay(200);
        motor_move(PORT_FLYWHEEL, flywheelPowerAfterShooting);
//        digitalWrite(INDEXER, LOW);
    } else if (numDisk == 2) {
        motor_move(PORT_FLYWHEEL, -127);
        int dist = distance_get(DISTANCE_OUT);
        uint32_t timeout = millis();
        while (dist > distanceNoDisk && (millis() - timeout < 600)) {
//                logInt(dist);
            delay(5);
            dist = distance_get(DISTANCE_OUT);
        }
        motor_move(PORT_FLYWHEEL, flywheelPowerAfterShooting + 20);
        delay(400);

//        assignLeftRightDriveMotorsVoltage(80, 75);
//        delay(150);
//        brake(-10);
        driveHeadingWithRampDown(heading + 5, 40, 2, 0, false);
        delay(200);

        motor_move(PORT_FLYWHEEL, -127);
        timeout = millis();
        while (distance_get(DISTANCE_OUT) > distanceNoDisk && (millis() - timeout < 600)) {
            delay(5);
        }
        motor_move(PORT_FLYWHEEL, flywheelPowerAfterShooting);
    } else if (numDisk == 3) {
        motor_move(PORT_FLYWHEEL, -127);
        int dist = distance_get(DISTANCE_OUT);
        uint32_t timeout = millis();
        while (dist > distanceNoDisk && (millis() - timeout < 600)) {
//                logInt(dist);
            delay(5);
            dist = distance_get(DISTANCE_OUT);
        }
        motor_move(PORT_FLYWHEEL, flywheelPowerAfterShooting + 20);
        delay(700);

        driveHeadingWithRampDown(heading + 5, 40, 2, 0, false);
        delay(200);
        motor_move(PORT_FLYWHEEL, -127);
        timeout = millis();
        while (distance_get(DISTANCE_OUT) > distanceNoDisk && (millis() - timeout < 600)) {
            delay(5);
        }
        motor_move(PORT_FLYWHEEL, flywheelPowerAfterShooting + 20);
        delay(700);

//        driveHeadingWithRampDown(heading + 7, 40, 2, 0, false);
//        turnRight()
//        delay(200);

        motor_move(PORT_FLYWHEEL, -127);
        timeout = millis();
        while (distance_get(DISTANCE_OUT) > distanceNoDisk && (millis() - timeout < 600)) {
            delay(5);
        }
        motor_move(PORT_FLYWHEEL, flywheelPowerAfterShooting);
    }
}

void turnRollerByTime() {
//    assignDriveMotorsVoltage(-60);
    motor_move(PORT_INTAKE, -127);
    delay(160);
    motor_move(PORT_INTAKE, 10);
    delay(50);
    motor_move(PORT_INTAKE, 0);
}
void turnRollerByTimeProgrammingSkills(int time) {
    assignDriveMotorsVoltage(-40);
    motor_move(PORT_INTAKE, -127);
    delay(time);
    motor_move(PORT_INTAKE, 10);
    delay(50);
    motor_move(PORT_INTAKE, 0);
}

bool checkIndexerFull() {
    int dist = distance_get(DISTANCE_IN);
//    logInt(dist);
    if (dist < DIST_3DISKS) {
        return true;
    }
    return false;
}

