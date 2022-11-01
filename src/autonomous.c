#include "subsystemHeaders/library.h"

void testGPS() {
//    gps_set_position(GPSFRONT_PORT, 0, 0, 0);
//    gps_set_position(GPSBACK_PORT, 0, 0, 180);

    gps_status_s_t front_status;
    gps_status_s_t back_status;

    while (true) {
        front_status = gps_get_status(GPSFRONT_PORT);
//        front_status.x = 100;
//        front_status.y = 100;
        printf("FRONT GPS VALUES: X, Y \n");
        printf("%f\n", front_status.x);
        printf("%f\n", front_status.y);

        back_status = gps_get_status(GPSBACK_PORT);
        printf("BACK GPS VALUES: X, Y \n");
        logDouble(back_status.x);
        logDouble(back_status.y);
        delay(500);
    }
}

void testAutoAim() {
    /** vision auto aiming **/
    visionAim(0, 30, 3000);

//    vision_object_s_t goal = vision_get_by_sig(VISION, 0, 1);
//    logInt(goal.x_middle_coord);

    /** skills absolute auto aiming **/
//    skillsGPSAim(blueGoalX, blueGOALY, false, false);
//    skillsGPSAim(redGoalX, redGoalY, false, true);

    /** auton not absolute auto aiming **/
//    getGPSAngle(blueGoalX, blueGOALY, true, true, true);
//    getGPSAngle(redGoalX, redGoalY, true, true, true);
}

void testCoast() {
    flywheelSpinToSpeed(112);
//    motorIndexerShootDisk(2, 112);
}

void rotationalTest() {

}

void test4Disks() {
    motor_move(PORT_INTAKE, 127);
    int count = 0;
    while (true) {
        if (checkIndexerFull()) {
            count++;
        } else if (count > 0) count = 0;
        delay(50);

        if (count >= 9) {
            motor_move(PORT_INTAKE, 0);
        }
    }

    for (int i = 0; i < 10; i++) {
        if (distance_get(DISTANCE_IN) < 85) {
            count++;
        }
        delay(50);
    }
    if (count >= 9) {
        motor_move(PORT_INTAKE, 0);
    }
}

/**
 * Gets -- disks; stays only on right half
 */
void newRight() {
    spinFlywheel(127);
    backwardCoastHeading(convertInchesToTicks(14), 100, 0);
    backwardCoastHeading(convertInchesToTicks(15), 100, -900);
//    turnRight(-900, 100);
//    backwardCoastHeading(convertInchesToTicks(6), 50, -900);
    assignDriveMotorsVoltage(-30);
    startIntake(-127);
    delay(160);
    startIntake(0);

    forwardCoastHeading(convertInchesToTicks(15), 50, -400);
    turnLeft(1250, 100);
    startIntake(127);
    backwardCoastHeading(convertInchesToTicks(13), 100, 1300);
    delay(100);
    turnLeft(2330, 100); // was 2360
//    brake(-10);
    turnBrake(-10, true);
    delay(200);
    spinFlywheel(107);
    flywheelSpinToSpeed(107);
    startIntake(0);
    motorIndexerShootDiskSlow(3, 103, 2300);
    delay(300);
    backwardCoastHeading(convertInchesToTicks(10), 60, 2300);
    backwardCoastHeading(convertInchesToTicks(15), 60, 1870);
    startIntake(127);
    backwardCoastHeading(convertInchesToTicks(5), 60, 1870);

//    forwardCoastHeading(convertInchesToTicks(10), 60, 2300);
    forwardCoastHeading(convertInchesToTicks(23), 60, 1870);
    turnRight(1250, 100);
    backwardCoastHeading(convertInchesToTicks(18), 60, 1300);
    turnLeft(2280, 100);
    turnBrake(-10, true);
    startIntake(0);
    flywheelSpinToSpeed(103);
    motorIndexerShootDiskSlow(2, 115, 2280);

//    forwardCoastHeading(convertInchesToTicks(24), 60, 1800);
//    turnRight(1800, 80);
//    backwardCoastHeading(convertInchesToTicks(9), 60, 1800);

}

void newLeft() {
    spinFlywheel(127);
    if (motor_get_temperature(PORT_FLYWHEEL) > 50) {
        printf("%s", "OVERHEATED");
    }

//    turnGyro(900, 50);
    assignDriveMotorsVoltage(-30);
    startIntake(-127);
    delay(130);
    startIntake(0);
//    assignDriveMotorsVoltage(50);
//    delay(150);
    forwardCoastHeading(convertInchesToTicks(23), 50, -400);
//    brake(-10);

//    turnRight(-600, 100); //turn to 230, TODO: turning counter vs. clockwise kinda weird
    assignLeftRightDriveMotorsVoltage(-25, 127); //0, 127
    delay(400);
    brake(-10);

    delay(100);
    turnLeft(80, 100); // 90 93 (MOST RECENT =90)
    brake(-10);
    flywheelSpinToSpeed(111); // was 108
    printf("current imu = %f", imu_get_heading(IMU_PORT));
//    shoot2Skills(111);
    motorIndexerShootDisk(2, 111, 360); // was 106

    delay(200); //was 200
    startIntake(127);
    spinFlywheel(111);
    turnLeft(1350, 127);

    // splitting up line of 3 to not get all 3 at once (gets stuck)
    backwardCoastHeading(convertInchesToTicks(73), 72, 1320);
}



/**--------*/



void right() {
    spinFlywheel(127);
    backwardCoastHeading(convertInchesToTicks(19), 100, 0);
    brake(10);
    turnRight(-900, 100);
    backwardCoastHeading(convertInchesToTicks(6), 50, -900);
    assignDriveMotorsVoltage(-60);
    delay(300);
    startIntake(-127);
    delay(160);
    startIntake(0);
    delay(100);

    forwardCoastHeading(convertInchesToTicks(15), 50, -400);
    turnLeft(1250, 100);
    startIntake(127);
    backwardCoastHeading(convertInchesToTicks(10), 100, 1300);
    delay(200);
    backwardCoastHeading(convertInchesToTicks(10), 50, 1300);
    delay(500);
    turnLeft(2360, 100);
    brake(-10);
    backwardCoastHeading(convertInchesToTicks(2), 50, 2380);
    spinFlywheel(105);
    flywheelSpinToSpeed(105);
    motorIndexerShootDiskSlow(3, 115, 2300);
    backwardCoastHeading(convertInchesToTicks(3), 50, 2360);

    // getting to expansion spot
    turnLeft(1350, 100);
    forwardCoastHeading(convertInchesToTicks(50), 100, 1350);
    turnLeft(1800, 100);
    forwardCoastHeading(convertInchesToTicks(20), 100, 1800);
    turnLeft(1350, 100);


    // turnRight(-4000, 100);
    // startIntake(127);
    // backwardCoastHeading()
}

void left() {
    spinFlywheel(127);

    //spin roller
    assignDriveMotorsVoltage(-50);
    startIntake(-127);
    delay(150);
    startIntake(0);


    forwardCoastHeading(convertInchesToTicks(5), 50, -400);
    spinFlywheel(113);
    forwardCoastHeading(convertInchesToTicks(19), 70, -400);

    

    backwardCoastHeading(convertInchesToTicks(3), 70, -400);
    brake(-10);

    turnLeft(1250, 100);
    startIntake(127);

    backwardCoastHeading(convertInchesToTicks(12), 50, 1250);
    backwardCoastHeading(convertInchesToTicks(7), 45, 1250);
    delay(500);

    turnRight(277, 90);
    brake(-10);
    
    backwardCoastHeading(convertInchesToTicks(1), 50, 277);

    forwardCoastHeading(convertInchesToTicks(0.5), 50, 277);
    
    delay(500);
    motorIndexerShootDiskSlow(3, 127, 330);
    delay(500);

    spinFlywheel(115);
    backwardCoastHeading(convertInchesToTicks(0.5), 50, 277);
    turnLeft(1250, 100);
    backwardCoastHeading(convertInchesToTicks(20), 40, 1250);

    backwardCoastHeading(convertInchesToTicks(18), 40, 0);
    turnLeft(275, 100);
    brake(10);

    motorIndexerShootDiskSlow(3, 127, 275);

    // startIntake(127);

    // backwardCoastHeading(1000, 50, )


}

void soloWP() {
    spinFlywheel(127);
    if (motor_get_temperature(PORT_FLYWHEEL) > 50) {
        printf("%s", "OVERHEATED");
    } 

//    turnGyro(900, 50);
    assignDriveMotorsVoltage(-30);
    startIntake(-127);
    delay(130);
    startIntake(0);
//    assignDriveMotorsVoltage(50);
//    delay(150);
    forwardCoastHeading(convertInchesToTicks(23), 50, -400);
//    brake(-10);

//    turnRight(-600, 100); //turn to 230, TODO: turning counter vs. clockwise kinda weird
    assignLeftRightDriveMotorsVoltage(-25, 127); //0, 127
    delay(400);
    brake(-10);

    delay(100);
    turnLeft(80, 100); // 90 93 (MOST RECENT =90)
    brake(-10);
    flywheelSpinToSpeed(109); // was 108
    printf("current imu = %f", imu_get_heading(IMU_PORT));
//    shoot2Skills(111);
    motorIndexerShootDisk(2, 109, 360); // was 106

    delay(200); //was 200
    startIntake(127);
    spinFlywheel(109);
    turnLeft(1350, 127);

    // splitting up line of 3 to not get all 3 at once (gets stuck)
    backwardCoastHeading(convertInchesToTicks(73), 72, 1320);
//    backwardCoastHeading(convertInchesToTicks(20), 40, 1320);
//    delay(200); // was 300
//    backwardCoastHeading(convertInchesToTicks(33), 40, 1320);
    delay(100);
    turnRight(725, 100); // was 760
    brake(-10);
    flywheelSpinToSpeed(109); // was 105
    printf("current imu = %f", imu_get_heading(IMU_PORT));
    motorIndexerShootDisk(2, 108, 350);
    delay(200);// was 200

    backwardCoastHeading(convertInchesToTicks(23), 127, 1320);
    startIntake(0);
    backwardCoastHeading(convertInchesToTicks(11), 127, 900);
    delay(10);
    assignDriveMotorsVoltage(-30);
    startIntake(-127);
    delay(140);
    startIntake(0);

//
//    backwardCoastHeading(convertInchesToTicks(6), 90, 1000);
//    startIntake(0);
//    backwardCoastHeading(convertInchesToTicks(11), 100, 900);
//    delay(10);
//    assignDriveMotorsVoltage(-50);
//    delay(300);
//    startIntake(-127);
//    delay(100);
//    startIntake(0);
//    assignDriveMotorsVoltage(0);

}

void soloWinPoint() {
    spinFlywheel(127);
    if (motor_get_temperature(PORT_FLYWHEEL) > 50) {
        printf("%s", "OVERHEATED");
    }
    assignDriveMotorsVoltage(-30);
    startIntake(-127);
    delay(130);
    startIntake(0);
    forwardCoastHeading(convertInchesToTicks(23), 50, -400);

    assignLeftRightDriveMotorsVoltage(-25, 127); //0, 127
    delay(400);
    brake(-10);

    delay(100);
    turnLeft(80, 100); // 90 93 (MOST RECENT =90)
    brake(-10);
    flywheelSpinToSpeed(109); // was 108
    printf("current imu = %f", imu_get_heading(IMU_PORT));

    motorIndexerShootDisk(2, 109, 360); // was 106

    delay(200); //was 200
    startIntake(127);
    spinFlywheel(109);
    turnLeft(1350, 127);

    backwardCoastHeading(convertInchesToTicks(73), 72, 1320);
    delay(100);
    turnRight(725, 100); // was 760
    brake(-10);
    flywheelSpinToSpeed(109); // was 105
    printf("current imu = %f", imu_get_heading(IMU_PORT));
    motorIndexerShootDisk(2, 108, 350);
    delay(200);// was 200

    backwardCoastHeading(convertInchesToTicks(23), 127, 1320);
    startIntake(0);
    backwardCoastHeading(convertInchesToTicks(11), 127, 900);
    delay(10);
    assignDriveMotorsVoltage(-30);
    startIntake(-127);
    delay(140);
    startIntake(0);
}
