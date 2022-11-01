#include "subsystemHeaders/library.h"

void programmingSkills() {
    spinFlywheel(93); // 95
    turnRollerByTimeProgrammingSkills(400);
    delay(10);

    forwardCoastHeading(convertInchesToTicks(6), 20, -200);
    forwardCoastHeading(convertInchesToTicks(3), 20, -400);
    forwardCoastHeading(convertInchesToTicks(6), 20, -900);
    brake(0);
    startIntake(127);
    backwardCoastHeading(convertInchesToTicks(24), 40, -1100);
    delay(300);
    startIntake(0);
    backwardCoastHeading(convertInchesToTicks(2), 40, -900);

    turnRollerByTimeProgrammingSkills(400);

    forwardCoastHeading(convertInchesToTicks(5), 40, -900);
    startIntake(127);
    forwardCoastHeading(convertInchesToTicks(45), 60, 0);
    brake(-10);

    turnRight(-80, 50); // was -45
    brake(-10);
    flywheelSpinToSpeed(93); // newest: was 95 | was 100
    shoot3Skills(91); // newest: was 93 | used to be 97 (For the next shot)
    delay(300);
    backwardCoastHeading(convertInchesToTicks(25), 40, 0);
    turnRight(1350, 50);
    backwardCoastHeading(convertInchesToTicks(65), 70, 1320); // was 67
    brake(-10);

    turnRight(610, 50); // used to be 640 630
    brake(-10);

    // shooting 2nd batch of 3
    startIntake(0);

    flywheelSpinToSpeed(91); // USED TO BE 93
    shoot3Skills(91); // final power is the power for the next shot USED TO BE 93
    startIntake(127);
    backwardCoastHeading(convertInchesToTicks(36), 60, 470);

    forwardCoastHeading(convertInchesToTicks(34), 60, 525); // was 39 inches
    delay(200);

    // shoot 2 disks after coming back from other side
    flywheelSpinToSpeed(91);
    shoot2Skills(93);
    delay(100);

    turnLeft(1800, 50);
    backwardCoastHeading(convertInchesToTicks(24), 50, 1800);
    backwardCoastHeading(convertInchesToTicks(50), 60, 900);// was power 40 and distance 55 in
    backwardCoastHeading(convertInchesToTicks(11), 40, 250); // was 10
    turnLeft(900, 50);
    backwardCoastHeading(convertInchesToTicks(14), 40, 900);
    //backing up a bit
    startIntake(0);
    delay(400);
    turnRollerByTimeProgrammingSkills(400);

    forwardCoastHeading(convertInchesToTicks(20), 40, 900);
    turnLeft(1800, 50);
    backwardCoastHeading(convertInchesToTicks(19), 40, 1800);
    delay(100);
//    turnRoller(0, 2000);
    turnRollerByTimeProgrammingSkills(400);
    delay(50);

    forwardCoastHeading(convertInchesToTicks(21), 50, 1800);
    turnLeft(1350, 50);
    startIntake(127);
    backwardCoastHeading(convertInchesToTicks(22.5), 50, 1350);
    turnRight(900, 50);
    forwardCoastHeading(convertInchesToTicks(55), 60, 900);
    startIntake(0);

    flywheelSpinToSpeed(93);
    shoot2Skills(93);

    turnRight(400, 50);
    backwardCoastHeading(convertInchesToTicks(20), 80, 400);
    delay(50);
    forwardCoastHeading(convertInchesToTicks(2), 40, 400);
    startIntake(127);
    delay(50);
    backwardCoastHeading(convertInchesToTicks(12), 30, 400);
    delay(200);

    turnLeft(710, 127);
    delay(100);

    flywheelSpinToSpeed(93);
    startIntake(0);
    shoot3Skills(93);

    startIntake(127);
    backwardCoastHeading(convertInchesToTicks(20), 30, 400);

    forwardCoastHeading(convertInchesToTicks(30), 50, 500);
    delay(200);
    turnGyro(750, 50);
    delay(100);

    flywheelSpinToSpeed(93);
    startIntake(0);
    shoot3Skills(93);

    delay(100);
    turnLeft(1350, 127);
    backwardCoastHeading(convertInchesToTicks(18), 127, 1350);
    turnRight(900, 127);
    backwardCoastHeading(convertInchesToTicks(22), 127, 900);
    turnGyro(-450, 127);
    turnBrake(-10, true);

    backwardCoastHeading(convertInchesToTicks(8), 127, -450);

    // expansion
    digitalWrite(ENDGAMEF, HIGH);
    digitalWrite(ENDGAMEB, HIGH);
}
