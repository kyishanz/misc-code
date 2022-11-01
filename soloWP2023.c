void soloWP() {
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
