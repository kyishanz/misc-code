//
// Created by Niu on 7/6/22.
//

#include "subsystemHeaders/config.h"

//Clears the values of the drive motors
void clearDriveMotors(){
    motor_tare_position(PORT_DRIVELEFTFRONT);
    motor_tare_position(PORT_DRIVELEFTMIDDLE);
    motor_tare_position(PORT_DRIVELEFTBACK);
    motor_tare_position(PORT_DRIVERIGHTFRONT);
    motor_tare_position(PORT_DRIVERIGHTMIDDLE);
    motor_tare_position(PORT_DRIVERIGHTBACK);
}

//Assigns the drive motors power in auton
void assignDriveMotorsVoltage(int power){
    motor_move(PORT_DRIVELEFTFRONT, power);
    motor_move(PORT_DRIVERIGHTFRONT, power);
    motor_move(PORT_DRIVERIGHTMIDDLE, power);
    motor_move(PORT_DRIVELEFTMIDDLE, power);
    motor_move(PORT_DRIVELEFTBACK, power);
    motor_move(PORT_DRIVERIGHTBACK, power);
}

/**
 *
 * @param rightSide [-127, 127]
 * @param leftSide [-127, 127]
 */
void assignLeftRightDriveMotorsVoltage(int rightSide, int leftSide){
    motor_move(PORT_DRIVELEFTFRONT, leftSide);
    motor_move(PORT_DRIVERIGHTFRONT, rightSide);
    motor_move(PORT_DRIVERIGHTMIDDLE, rightSide);
    motor_move(PORT_DRIVELEFTMIDDLE, leftSide);
    motor_move(PORT_DRIVELEFTBACK, leftSide);
    motor_move(PORT_DRIVERIGHTBACK, rightSide);
}

void driveBrake() {
    motor_set_brake_mode(PORT_DRIVELEFTFRONT, E_MOTOR_BRAKE_HOLD);
    motor_set_brake_mode(PORT_DRIVELEFTBACK, E_MOTOR_BRAKE_HOLD);
    motor_set_brake_mode(PORT_DRIVERIGHTFRONT, E_MOTOR_BRAKE_HOLD);
    motor_set_brake_mode(PORT_DRIVERIGHTBACK, E_MOTOR_BRAKE_HOLD);
    motor_set_brake_mode(PORT_DRIVELEFTMIDDLE, E_MOTOR_BRAKE_HOLD);
    motor_set_brake_mode(PORT_DRIVERIGHTMIDDLE, E_MOTOR_BRAKE_HOLD);
}

double averageVelocity() {
    return (abs(motor_get_actual_velocity(PORT_DRIVELEFTFRONT)) + abs(motor_get_actual_velocity(PORT_DRIVELEFTBACK)) +
            abs(motor_get_actual_velocity(PORT_DRIVERIGHTFRONT)) + abs(motor_get_actual_velocity(PORT_DRIVERIGHTBACK)) +
            abs(motor_get_actual_velocity(PORT_DRIVELEFTMIDDLE)) + abs(motor_get_actual_velocity(PORT_DRIVERIGHTMIDDLE)))/
           NUMDRIVEMOTORS;
}

/**
 * Calibrating gyro.
 */
void resetGyro(){
    //adi_gyro_reset(gyro);
    imu_reset(IMU_PORT);
}

/**
 * Turn left without PID
 * @param power
 * @param time
 */
void turnLeftNoPID(int power) {
    motor_move(PORT_DRIVELEFTFRONT, power);
    motor_move(PORT_DRIVELEFTMIDDLE, power);
    motor_move(PORT_DRIVERIGHTFRONT, -power);
    motor_move(PORT_DRIVERIGHTMIDDLE, -power);
    motor_move(PORT_DRIVELEFTBACK, power);
    motor_move(PORT_DRIVERIGHTBACK, -power);
}

/**
 * Turn right without PID
 * @param power [-127, 127]
 */
void turnRightNoPID(int power) {
    motor_move(PORT_DRIVELEFTFRONT, -power);
    motor_move(PORT_DRIVELEFTMIDDLE, -power);
    motor_move(PORT_DRIVERIGHTFRONT, power);
    motor_move(PORT_DRIVELEFTBACK, -power);
    motor_move(PORT_DRIVERIGHTMIDDLE, power);
    motor_move(PORT_DRIVERIGHTBACK, power);
}

float convertTicksToInches(int ticks);

double readDistanceInches() {
    double averageTicks = motor_get_position(PORT_DRIVELEFTFRONT) + motor_get_position(PORT_DRIVELEFTMIDDLE) + motor_get_position(PORT_DRIVELEFTBACK) +
            motor_get_position(PORT_DRIVERIGHTFRONT) + motor_get_position(PORT_DRIVERIGHTMIDDLE) + motor_get_position(PORT_DRIVERIGHTBACK);
    averageTicks /= 6;
    return convertTicksToInches(averageTicks);
}

double wrapAngle(double heading) {
    if (heading > 180) {
        return heading - 360;
    }
}

///**
// * Move arm at specific power.
// * @param power [-127, 127]
// */
//void moveArm(int power) {
//    motor_move(PORT_ARM, power);
//}

/** Terminal functions **/

/**
 * Print int n to console with line break
 * @param n
 */
void logInt(int n) {
    printf("%d\n", n);
}

/**
 * Print string s to console with line break
 * @param s
 */
void logString(char s[]) {
    printf("%s\n", s);
}

/**
 * Print double n to console with line break
 * @param n
 */
void logDouble(double n) {
    printf("%f\n", n);
}

/**
 * Print string s and then float f with tab in between
 * @param s
 * @param f
 */
void logStringWithFloat(char s[], float f) {
    printf("%s\t", s);
    printf("%f\n", f);
}

/**
 * Convert [-100, 100] to [-127, 127]
 * @param power100
 * @return
 */
float convert100to127(int power100) {
    return power100 * 127 / 100;
}

/**
 * Convert ticks to inches
 * @param ticks
 * @return inches
 */
float convertTicksToInches(int ticks) {
    float inches_per_tick = 2 * PI * WHEEL_RADIUS / (MOTOR_TICKS_PER_REV * MOTOR_REV_PER_WHEEL_REV);
    return ticks * inches_per_tick;
}

/**
 * Convert inches to ticks
 * @param inches
 * @return ticks
 */
float convertInchesToTicks(int inches) {
    float ticks_per_inch = MOTOR_TICKS_PER_REV * MOTOR_REV_PER_WHEEL_REV / (2 * PI * WHEEL_RADIUS);
    return inches * ticks_per_inch;
}

/** SPIN UP SPECIFIC HELPER FUNCTIONS ***/

/**
 * Spin flywheel //TODO: replace with PID with rotation sensor later?
 * @param speed [-127, 127]
 */
void spinFlywheel(int speed) {
    motor_move(PORT_FLYWHEEL, speed);
    delay(100);
}

void startIntake(int power) {
    motor_move(PORT_INTAKE, power);
}

void outTake(int power) {
    motor_move(PORT_INTAKE, -power);
}





