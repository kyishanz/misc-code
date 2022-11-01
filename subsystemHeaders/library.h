//
// Created by Niu on 7/6/22.
//

#ifndef SPIN_UP_LIBRARY_H
#define SPIN_UP_LIBRARY_H

#include "subsystemHeaders/hal.h"

void assignDriveMotorsDist(int leftSide, int rightSide, int power, bool clear, bool turn);
void getHeading();
void straightPID(int leftSide, int rightSide, int power, int heading);
void driveStraightPID (int leftSide, int rightSide, int power, int heading);
void coastHeadingWithoutDrivePID(int ticks, int power, int heading);
void coastHeadingWithoutDrivePIDWithoutHeadingTask(int inches, int power, int heading);
void coastHeadingWithTimeout(int ticks, int power, int heading, bool corr, int timeoutLimit);
void driveHeadingWithRampDown(float targetHeading, float speed, float totalDistToDrive, float distToStartRampDown, bool smartRamp);
void driveInches(int inches, int voltage);

        void turnGyro(int degrees, int power);
void turnGyroWithoutTask(int degrees, int power);
void turnWithRampDown(float speed, float totalDegrees, float degreesToStartRampDown);
void turnHeading(double newHeading, int power);
void turnLeft(int degrees, int power);
void turnRight(int degrees, int power);
void turnLeftWithTicksAndTimeout(int ticks, int power, bool clear);
void turnRightWithTicksAndTimeout(int ticks, int power, bool clear);

void forward(int ticks, int power, int heading);
void backward(int ticks, int power, int heading);

void forwardCoastHeading(int ticks, int power, int heading);
void forwardCoastHeadingWithTimeout(int ticks, int power, int heading);
void backwardCoastHeading(int ticks, int power, int heading);

void brake(int power);
void turnBrake(int power, bool ccw);

/** sensors **/

/* distance sensor */
void backwardCoastToWall(int distanceFromWall, int allowedTicks, int power, int heading);
void forwardUntilDistFromWallNoHeadingCorrection(int distanceFromWall, int allowedTime, int power);
void forwardCoastHeadingUntilDistFromWallWithoutBrake(int distanceFromWall, int allowedTicks, int power, int heading);
void forwardCoastHeadingUntilDistFromWallWithBrake(int distanceFromWall, int allowedTicks, int power, int heading);
void forwardCoastHeadingUntilDistFromWallWithBrakeWithTime(int distanceFromWall, int allowedTime, int power, int heading);

/* bumpers */
void clampBumper(int allowedTime);

/** vision sensor */
void visionAim(int color, int power, int timeout);
void forwardToGoalWithVision(int timeout);
void backToGoalWithVision();
void backToGoalWithVisionPower(int inputPower);

void straightPIDInchesWithScaled100Power(int leftSide, int rightSide, float power, int heading);

/** GPS */
void turnGPSAbsoluteGyro(double targetAngle, double speed);
float getGPSAngle(double goalX, double goalY, bool frontGPSsensor, bool threeTile, bool red);
void skillsGPSAim(double goalX, double goalY, bool frontGPSsensor, bool red, bool bashCorrection);

/** optical sensor */
int colorOfRoller();

/** lvgl **/
lv_obj_t* drawRectangle( int x, int y, int width, int height, lv_color_t color);
lv_obj_t* drawRectangleWithText (int x, int y, int width, int height, lv_color_t color, char text[]);

/** game specific functions **/
void turnRoller(int color, int timeout);
void turnRollerReverse(int color, int timeout);
void turnRollerByTime();
void turnRollerByTimeProgrammingSkills(int time);
void flywheelSpinToSpeed(int speed);
void spinFlywheelAuton();
void motorIndexerShootDisk(int numDisk, int flywheelPowerAfterShooting, int curHeading);
void motorIndexerShootDiskSlow(int numDisk, int flywheelPowerAfterShooting, int curHeading);
void shoot3SkillsUpdated(int finalPower);
void shoot3Skills(int finalPower);
void shoot2Skills(int finalPower);
bool checkIndexerFull();

#endif //SPIN_UP_LIBRARY_H
