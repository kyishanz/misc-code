//
// Created by Niu on 7/6/22.
//

#include "subsystemHeaders/autonomous.h"
#include "subsystemHeaders/skills.h"
#include "subsystemHeaders/my-config.h"

void test() {
    task_t headingTask = task_create(getHeading, "PROS", TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT, "heading task");
     soloWP();
//    programmingSkills();
//     left();
//    right();
//    newRight();
//    testGPS();
//    testAutoAim();
//    test4Disks();
}

void runAuton() {
    task_t headingTask = task_create(getHeading, "PROS", TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT, "heading task");

    switch (autonNumber) {
        // solo WP
        case 1:
//            if (autonSide == 0) { // left
//                soloWP();
//                break;
//            }
//            // right // do not have right solo wp, run right in case we accidentally select it
//            right();
            soloWP();
            break;
        // half WP code
        case 2:
           if (autonSide == 0) { // left
               left();
               break;
           }
//            right();
            newRight();
            break;
        case 3: // skills
            autonColor = 0; // everything in skills turns to red
            programmingSkills();
            break;
        case 4:
//            straightForMiddleRight();
            break;
        case 5:
            break;
        case 6:
            break;
            //solo
        case 7:
//            if (leftRightIndex == 1) {
//                rightSoloWP();
//                break;
//            }
//            leftSideWithArms();
            break;
        case 8:
            break;
        case 9:
//            testGyro();
            break;
        case 10:
            break;
    }
}

