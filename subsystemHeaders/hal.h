//
// Created by Niu on 7/6/22.
//

#ifndef SPIN_UP_HAL_H
#define SPIN_UP_HAL_H
#include "config.h"

#define max(a, b) \
({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a > _b ? _a : _b; })

#define min(a, b) \
({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a < _b ? _a : _b; })

void clearDriveMotors();
void assignDriveMotorsVoltage(int power);
void assignLeftRightDriveMotorsVoltage(int rightSide, int leftSide);
void driveBrake();
double readDistanceInches();
double averageVelocity();
void resetGyro();
double wrapAngle(double heading);

void turnLeftNoPID(int power);
void turnRightNoPID(int power);

void moveArm(int power);

void logInt(int n);
void logString(char s[]);
void logDouble(double n);
void logStringWithFloat(char s[], float f);

float convert100to127(int power100);
float convertTicksToInches(int ticks);
float convertInchesToTicks(int inches);

/** spin up specific functions **/
void spinFlywheel(int speed);
void startIntake(int power);
void outTake(int power);

#endif //SPIN_UP_HAL_H
