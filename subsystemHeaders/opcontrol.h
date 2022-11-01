//
// Created by Niu on 7/6/22.
//

#ifndef SPIN_UP_OPCONTROL_H
#define SPIN_UP_OPCONTROL_H

#include "subsystemHeaders/library.h"

void drivebaseControl();
void pneumaticsControl();
void flywheelControl();
void updatePower(void* param);
void intakeControl();
void flexRollerControl();
void angleTilterControl();
void pneumaticIndexerControl();
void shoot3Control();
void slowDriveControl();
void displayDriveInfo();
void endgameControl();
void magazineDiskCountControl();

#endif //SPIN_UP_OPCONTROL_H
