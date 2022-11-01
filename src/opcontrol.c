#include "subsystemHeaders/library.h"
#include "subsystemHeaders/my-config.h"
#include "subsystemHeaders/config.h"
#include <stdio.h>
#include <inttypes.h>

bool hold = false;
int flywheelPower = 102; // was 112
bool check = false;
bool indexerOut = false;
bool angleTilterUp = false;
float powerRatio = 1; // 1 is regular, less is powered down
bool disableDiskCounter = false;
bool skills = false;
int intakeState = 0; // -1 reverse, 0 pause, 1 forward


void drivebaseControl() {
    while (true) {
        int forward = controller_get_analog(CONTROLLER_MASTER, ANALOG_LEFT_Y);
        int turn = controller_get_analog(CONTROLLER_MASTER, ANALOG_RIGHT_X);

        motor_move(PORT_DRIVERIGHTMIDDLE, max(-127, min(127, forward - turn)) * powerRatio);
        motor_move(PORT_DRIVELEFTMIDDLE, max(-127, min(127, forward + turn)) * powerRatio);
        motor_move(PORT_DRIVELEFTBACK, max(-127, min(127, forward + turn)) * powerRatio);
        motor_move(PORT_DRIVERIGHTBACK, max(-127, min(127, forward - turn)) * powerRatio);
        motor_move(PORT_DRIVERIGHTFRONT, max(-127, min(127, forward - turn)) * powerRatio);
        motor_move(PORT_DRIVELEFTFRONT, max(-127, min(127, forward + turn)) * powerRatio);

        delay(20);

        if (!skills && (controller_get_digital(CONTROLLER_MASTER, DIGITAL_Y) ||
                        controller_get_digital(CONTROLLER_PARTNER, DIGITAL_Y))) { // if Y button is pressed
            if (!hold) { // if not currently holding, and hold button is pressed, hold
                motor_set_brake_mode(PORT_DRIVELEFTFRONT, E_MOTOR_BRAKE_HOLD);
                motor_set_brake_mode(PORT_DRIVELEFTMIDDLE, E_MOTOR_BRAKE_HOLD);
                motor_set_brake_mode(PORT_DRIVELEFTBACK, E_MOTOR_BRAKE_HOLD);
                motor_set_brake_mode(PORT_DRIVERIGHTFRONT, E_MOTOR_BRAKE_HOLD);
                motor_set_brake_mode(PORT_DRIVERIGHTMIDDLE, E_MOTOR_BRAKE_HOLD);
                motor_set_brake_mode(PORT_DRIVERIGHTBACK, E_MOTOR_BRAKE_HOLD);
                hold = true; // set holding to true
            } else { // currently holding, release hold to coast
                motor_set_brake_mode(PORT_DRIVELEFTFRONT, E_MOTOR_BRAKE_COAST);
                motor_set_brake_mode(PORT_DRIVELEFTMIDDLE, E_MOTOR_BRAKE_COAST);
                motor_set_brake_mode(PORT_DRIVELEFTBACK, E_MOTOR_BRAKE_COAST);
                motor_set_brake_mode(PORT_DRIVERIGHTFRONT, E_MOTOR_BRAKE_COAST);
                motor_set_brake_mode(PORT_DRIVERIGHTMIDDLE, E_MOTOR_BRAKE_COAST);
                motor_set_brake_mode(PORT_DRIVERIGHTBACK, E_MOTOR_BRAKE_COAST);
                hold = false; // no longer holding
            }
            delay(100); //TODO: WHY? after release, wait for 100 ms
        }

        /**
         * Vision aiming button control: in drivebase control function to avoid conflict between joystick control and visionAim control
         */
        if (!skills && (controller_get_digital(CONTROLLER_MASTER, DIGITAL_Y) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_Y))
             && !controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT) && !controller_get_digital(CONTROLLER_PARTNER, DIGITAL_LEFT)) {
            visionAim(autonColor, 30, 1500);
            delay(20);
        }

        /**
         * Go back for color roller to gain more contact
         */
         if (disableDiskCounter == true) {
             assignDriveMotorsVoltage(-30);
             delay(160);
         }

         /**
          * GPS aiming button control for skills
          */
          if (skills && (controller_get_digital(CONTROLLER_MASTER, DIGITAL_X) && !controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT))) {
              skillsGPSAim(redGoalX, redGoalY, true, true, 0);
              delay(100);
          } else if (skills && (controller_get_digital(CONTROLLER_MASTER, DIGITAL_B) && !controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT))) {
              skillsGPSAim(blueGoalX, blueGOALY, true, false, 0);
              delay(100);
          }
    }
}

void flywheelControl(void *param) {
    while(true){
        if (controller_get_digital(CONTROLLER_MASTER, DIGITAL_L1) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_L1) || check) {
            check = false;
            motor_move(PORT_FLYWHEEL, flywheelPower);
            delay(100);
        }
        if (controller_get_digital(CONTROLLER_MASTER, DIGITAL_L2) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_L2)){
            motor_move(PORT_FLYWHEEL, -127);
            while(controller_get_digital(CONTROLLER_MASTER, DIGITAL_L2) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_L2)){
                delay(20);
            }
            motor_move(PORT_FLYWHEEL, flywheelPower);
        }
        if (!skills && (controller_get_digital(CONTROLLER_MASTER, DIGITAL_B) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_B)) && !controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT) && !controller_get_digital(CONTROLLER_PARTNER, DIGITAL_LEFT)){
            motor_move(PORT_FLYWHEEL, 0);
            // flywheelPower = 0;
        }
        delay(50);
    }
}

void updatePower(void* param){
    while(true) {
        if ((controller_get_digital(CONTROLLER_MASTER, DIGITAL_UP) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_UP)) && !controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT) && !controller_get_digital(CONTROLLER_PARTNER, DIGITAL_LEFT)){
            flywheelPower = flywheelPower + 5;
            if(flywheelPower > 127) {
                flywheelPower = 127;
            }
            check = true;
        } else if ((controller_get_digital(CONTROLLER_MASTER, DIGITAL_DOWN) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_DOWN)) && !controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT) && !controller_get_digital(CONTROLLER_PARTNER, DIGITAL_LEFT)){
            flywheelPower = flywheelPower - 5;
            if(flywheelPower < -127) {
                flywheelPower = -127;
            }
            check = true;
        }
        if ((controller_get_digital(CONTROLLER_MASTER, DIGITAL_B) && controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT)) || (controller_get_digital(CONTROLLER_PARTNER, DIGITAL_B) && controller_get_digital(CONTROLLER_PARTNER, DIGITAL_LEFT))) {
            flywheelPower = 102;
            check = true;
        }
        if ((controller_get_digital(CONTROLLER_MASTER, DIGITAL_A) && controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT)) || (controller_get_digital(CONTROLLER_PARTNER, DIGITAL_A) && controller_get_digital(CONTROLLER_PARTNER, DIGITAL_LEFT))) {
            flywheelPower = 107;
            check = true;
        }
        if ((controller_get_digital(CONTROLLER_MASTER, DIGITAL_X) && controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT)) || (controller_get_digital(CONTROLLER_PARTNER, DIGITAL_X) && controller_get_digital(CONTROLLER_PARTNER, DIGITAL_LEFT))) {
            flywheelPower = 112;
            check = true;
        }
        if ((controller_get_digital(CONTROLLER_MASTER, DIGITAL_Y) && controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT)) || (controller_get_digital(CONTROLLER_PARTNER, DIGITAL_Y) && controller_get_digital(CONTROLLER_PARTNER, DIGITAL_LEFT))) {
            flywheelPower = 117;
            check = true;
        }
        delay(100);
    }
}

void intakeControl() {
    while(true) {
        // forward intake
        if ((controller_get_digital(CONTROLLER_MASTER, DIGITAL_R1) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_R1)) && !controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT) && !controller_get_digital(CONTROLLER_PARTNER, DIGITAL_LEFT)) {
            if (intakeState == 0 || intakeState == -1) {
                motor_move(PORT_INTAKE, 127);
                intakeState = 1;
            } else if (intakeState == 1) {
                motor_move(PORT_INTAKE, 0);
                intakeState = 0;
            }
        }
        if ((controller_get_digital(CONTROLLER_MASTER, DIGITAL_R2) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_R2)) && !controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT) && !controller_get_digital(CONTROLLER_PARTNER, DIGITAL_LEFT)) {
            if (intakeState == 0 || intakeState == 1) {
                motor_move(PORT_INTAKE, -127);
                intakeState = -1;
            } else if (intakeState == -1) {
                motor_move(PORT_INTAKE, 0);
                intakeState = 0;
            }
        }
        delay(150);
    }
}

/**
 * Turning color rollers
 */
void flexRollerControl() {
    while (true) {
        if ((controller_get_digital(CONTROLLER_MASTER, DIGITAL_A) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_A)) && !controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT) && !controller_get_digital(CONTROLLER_PARTNER, DIGITAL_LEFT)) {
            disableDiskCounter = true;
            if (!skills) turnRollerReverse(autonColor, 1000);
            else turnRollerReverse(autonColor, 2000);
            disableDiskCounter = false;
            delay(50);
        }
        delay(20);
    }
}

/**
 * Angle changer
 */
void angleTilterControl() {
    while (true) {
        if (!skills && (controller_get_digital(CONTROLLER_MASTER, DIGITAL_X) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_X)) &&
            !controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT) && !controller_get_digital(CONTROLLER_PARTNER, DIGITAL_LEFT)) {
            if (angleTilterUp == false) {
                digitalWrite(ANGLETILTER, HIGH);
                angleTilterUp = true;
            } else {
                digitalWrite(ANGLETILTER, LOW);
                angleTilterUp = false;
            }
            delay(100);
        }
        delay(100);
    }
}

/**
 * Pneumatic indexer control
 */
void pneumaticIndexerControl() {
    while (true) {
        if (!skills && (controller_get_digital(CONTROLLER_MASTER, DIGITAL_Y) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_Y)) && !controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT) && !controller_get_digital(CONTROLLER_PARTNER, DIGITAL_LEFT)) {
            digitalWrite(INDEXER, HIGH);
            delay(400);
            digitalWrite(INDEXER, LOW);
        } else if (skills && (controller_get_digital(CONTROLLER_MASTER, DIGITAL_Y) && !controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT))) {
            digitalWrite(INDEXER, HIGH);
            delay(400);
            digitalWrite(INDEXER, LOW);
        }
        delay(100);
    }
}

void shoot3Control() {
    bool checkCount = 0;
    while (true) {
        if ((controller_get_digital(CONTROLLER_MASTER, DIGITAL_RIGHT) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_RIGHT)) &&
                !controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT) && !controller_get_digital(CONTROLLER_PARTNER, DIGITAL_LEFT)) {
           if (!skills) {
               uint32_t timeout = millis();
               motor_move(PORT_FLYWHEEL, -127);
               int dist = distance_get(DISTANCE_OUT);
               while (dist > distanceNoDisk) {
//                logInt(dist);
                   delay(5);
                   dist = distance_get(DISTANCE_OUT);
                   if (checkCount % 10 == 0 || checkCount == 0) {
                       uint32_t ctime = millis();
                       if (ctime - timeout > 1000) {
                           motor_move(PORT_FLYWHEEL, flywheelPower);
                           printf("exit while 1");
                           break;
                       }
                   }
               }
               checkCount = 0; // resetting for next while
               logInt(100001);
//            flywheelSpinToSpeed(102);

               motor_move(PORT_FLYWHEEL, 127);
               delay(200);

               motor_move(PORT_FLYWHEEL, -127);
               timeout = millis();
               while (distance_get(DISTANCE_OUT) > distanceNoDisk) {
                   delay(5);
                   uint32_t ctime = millis();
                   if (ctime - timeout > 1000) {
                       motor_move(PORT_FLYWHEEL, flywheelPower);
                       printf("exit while 2");
                       break;
                   }
               }
               checkCount = 0; // resetting again
               logInt(100002);
//            flywheelSpinToSpeed(115);
               motor_move(PORT_FLYWHEEL, 127);
               delay(200);
               motor_move(PORT_FLYWHEEL, -127);
               timeout = millis();
               while (distance_get(DISTANCE_OUT) > distanceNoDisk) {
                   delay(5);
                   uint32_t ctime = millis();
                   if (ctime - timeout > 1000) {
                       motor_move(PORT_FLYWHEEL, flywheelPower);
                       printf("exit while 3");
                       break;
                   }
               }
               logInt(100003);
//            flywheelSpinToSpeed(115);
               motor_move(PORT_FLYWHEEL, flywheelPower);

               delay(10);
           } else {
               shoot3Skills(flywheelPower);
               delay(10);
           }
       }
       delay(100);
            // digitalWrite(INDEXER, HIGH);
            // delay(300);
            // digitalWrite(INDEXER, LOW);
            // delay(300);
            // digitalWrite(INDEXER, HIGH);
            // delay(300);
            // digitalWrite(INDEXER, LOW);
            // delay(300);
            // digitalWrite(INDEXER, HIGH);
            // delay(300);
            // digitalWrite(INDEXER, LOW);
        // }
    }
}

/**
 * For angling before shooting
 */
void slowDriveControl() {
    while (true) {
        if ((controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT) && controller_get_digital(CONTROLLER_MASTER, DIGITAL_DOWN)) ||
                (controller_get_digital(CONTROLLER_PARTNER, DIGITAL_LEFT) && controller_get_digital(CONTROLLER_PARTNER, DIGITAL_DOWN))) {
            if (powerRatio == 1) {
                powerRatio = 0.5;
            } else powerRatio = 1;
            delay(100);
        }
        delay(100);
    }
}

void endgameControl() {
    bool endgamePressed = false;
    while (true) {
        if ((controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT) && controller_get_digital(CONTROLLER_MASTER, DIGITAL_RIGHT)) ||
            (controller_get_digital(CONTROLLER_PARTNER, DIGITAL_LEFT) && controller_get_digital(CONTROLLER_PARTNER, DIGITAL_RIGHT))) {
            if (endgamePressed == false) {
                digitalWrite(ENDGAMEF, HIGH);
                digitalWrite(ENDGAMEB, HIGH);
                endgamePressed = true;
            }
            delay(300);
        }
    }
}
// 112 good for triple
void magazineDiskCountControl() {
    int count = 0;
    while (true) {
        if (checkIndexerFull() && !disableDiskCounter) {
            count++;
        } else if (count > 0) count = 0;
        delay(50);

        if (count >= 5 && !disableDiskCounter) {
            if (motor_get_direction(PORT_INTAKE) == 1) {
                motor_move(PORT_INTAKE, 0);
                intakeState = 0;
            }
            count = 0;
        }
    }
}

/**
 * Display motor temperatures + other relevant information
 */
void displayDriveInfo() {
    lv_obj_clean(lv_scr_act());
    lv_obj_t * table = lv_table_create(lv_scr_act(), NULL);
    lv_table_set_row_cnt(table, 5);
    lv_table_set_col_cnt(table, 2);
    lv_obj_align(table, NULL, LV_ALIGN_CENTER, 0, 0);
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 2; j++) {
            lv_table_set_cell_align(table, i, j, LV_LABEL_ALIGN_CENTER);
        }
    }

    char string[4];

    snprintf(string, 4, "LB\t%f", motor_get_temperature(PORT_DRIVELEFTBACK));
    lv_table_set_cell_value(table, 0, 0, string);
    snprintf(string, 4, "LM\t%f", motor_get_temperature(PORT_DRIVELEFTMIDDLE));
    lv_table_set_cell_value(table, 1, 0, string);
    snprintf(string, 4, "LF\t%f", motor_get_temperature(PORT_DRIVELEFTFRONT));
    lv_table_set_cell_value(table, 2, 0, string);
    snprintf(string, 4, "RB\t%f", motor_get_temperature(PORT_DRIVERIGHTBACK));
    lv_table_set_cell_value(table, 3, 0, string);
    snprintf(string, 4, "RM\t%f", motor_get_temperature(PORT_DRIVERIGHTMIDDLE));
    lv_table_set_cell_value(table, 4, 0, string);
    snprintf(string, 4, "RF\t%f", motor_get_temperature(PORT_DRIVERIGHTFRONT));
    lv_table_set_cell_value(table, 0, 1, string);
    snprintf(string, 4, "FLY\t%f", motor_get_temperature(PORT_FLYWHEEL));
    lv_table_set_cell_value(table, 0, 2, string);
    snprintf(string, 4, "INT\t%f", motor_get_temperature(PORT_INTAKE));
    lv_table_set_cell_value(table, 0, 3, string);
    snprintf(string, 4, "FLYSPEED\t%f", flywheelPower);
    lv_table_set_cell_value(table, 0, 4, string);



//    lv_obj_t* label1 = lv_label_create(lv_scr_act(), NULL);
//    lv_label_set_align(label1, LV_LABEL_ALIGN_CENTER);       /*Center aligned lines*/
//    lv_label_set_text(label1, "LB");
}
