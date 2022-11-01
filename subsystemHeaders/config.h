//
// Created by Niu on 7/6/22.
//

#ifndef SPIN_UP_CONFIG_H
#define SPIN_UP_CONFIG_H

#include "main.h"

#define robot 1
#if robot == 0 // TiP robot
#define PORT_DRIVELEFTFRONT 15
#define PORT_DRIVELEFTMIDDLE 14
#define PORT_DRIVELEFTBACK 13
#define PORT_DRIVERIGHTFRONT 18
#define PORT_DRIVERIGHTMIDDLE 12
#define PORT_DRIVERIGHTBACK 17
#define PORT_ARM 19
#define PORT_ROLLERS 16
#define IMU_PORT 1
#define DISTANCE 4
#define ARMDISTANCE 3
#define FRONTDISTANCE 20
#define VISION_FRONT 10
#define VISION_BACK 9

// brain three wire ports
#define ARMCLAMP 'A'
#define FRONTCLAMP 'B'
#define FRONTTILTER 'C'
#define BACKCLAMP 'D'
#define BACKTILTER 'E'
#define ARMBUMPER 'F'
#define BACKBUMPER 'G'
#define STICK 'H'

#define NUMDRIVEMOTORS 6
#define RPM 300
#define MOTOR_TICKS_PER_REV 300
/**
 * 1800 ticks/rev with 36:1 gears
 * 900 ticks/rev with 18:1 gears
 * 300 ticks/rev with 6:1 gears
 */
#define MOTOR_REV_PER_WHEEL_REV 2

#elif robot == 1 // spin robot
#define PORT_DRIVELEFTFRONT 17
#define PORT_DRIVELEFTMIDDLE 16
#define PORT_DRIVELEFTBACK 15
#define PORT_DRIVERIGHTFRONT 14
#define PORT_DRIVERIGHTMIDDLE 13
#define PORT_DRIVERIGHTBACK 12
#define PORT_INTAKE 1
#define PORT_FLYWHEEL 20
#define IMU_PORT 11

#define DISTANCE_OUT 8
#define DISTANCE_IN 7

/** redo these **/
#define ARMDISTANCE 3
#define FRONTDISTANCE 4
#define VISION_FRONT 5
#define VISION_BACK 6
/** end of redo these **/

#define OPTICAL 4
#define VISION 5

#define GPSFRONT_PORT 10
#define FRONT_X_OFFSET 0.129 // 0.067 // -0.11
#define FRONT_Y_OFFSET 0.155 //-0.15

#define GPSBACK_PORT 6
#define BACK_X_OFFSET 0.021 // 0.003 //
#define BACK_Y_OFFSET 0.146 //0.15
// other gps non-constant variables are underneath

// brain three wire ports
#define ANGLETILTER 'A'
#define INDEXER 'B'
#define ENDGAMEF 'C'
#define ENDGAMEB 'D'
//#define BACKCLAMP 'D'
//#define BACKTILTER 'E'
//#define ARMBUMPER 'F'
//#define BACKBUMPER 'G'
//#define STICK 'H'

#define NUMDRIVEMOTORS 6
#define RPM 300
#define MOTOR_TICKS_PER_REV 300
/**
 * 1800 ticks/rev with 36:1 gears
 * 900 ticks/rev with 18:1 gears
 * 300 ticks/rev with 6:1 gears
 */
#define MOTOR_REV_PER_WHEEL_REV 2

#endif

#define blueGoalX -1.3716
#define blueGOALY -1.3716
#define redGoalX 1.3716
#define redGoalY 1.3716

#define DIST_3DISKS 92

#define WHEEL_RADIUS 2 // inches
#define PI 3.141592653575823846

#endif