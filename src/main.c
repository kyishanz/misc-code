#include "subsystemHeaders/initialize.h"
#include "subsystemHeaders/runAuton.h"
#include "subsystemHeaders/opcontrol.h"
#include "subsystemHeaders/my-config.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    resetGyro();
    initializeMotorDirection();
    initializeMotorCartridge();
    initializePneumatics();
//    initializeBumpers();
    initializeVision();
    initializeGPS();
//    myPicker();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
    myPicker();
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
//    runAuton();
    test();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
//    spinFlywheel(102);
    int skillsHere = 0;
    if (skillsHere == 0) {
        task_t driveTask = task_create(drivebaseControl, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Drive Task");
        task_t flywheelTask = task_create(flywheelControl, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel Task");
        task_t flywheelUpdateTask = task_create(updatePower, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel update Task");
        task_t intakeTask = task_create(intakeControl, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "intake task");
        task_t angleChangerTask = task_create(angleTilterControl, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "angle changer task");
        task_t pistonIndexerTask = task_create(pneumaticIndexerControl, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "pneumatic indexer task");
        task_t shoot3Task = task_create(shoot3Control, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Shoot 3 Task");
        task_t colorRollerTask = task_create(flexRollerControl, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Color roller button task");
        task_t slowDrivebaseTask = task_create(slowDriveControl, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Slow drivebase task");
        task_t endgameTask = task_create(endgameControl, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "endgame task");
        task_t magazineDiskCounter = task_create(magazineDiskCountControl, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "magazine disk counter");
    } else { /** SKILLS */
        task_t driveTask = task_create(drivebaseControl, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Drive Task");
        task_t flywheelTask = task_create(flywheelControl, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel Task");
        task_t flywheelUpdateTask = task_create(updatePower, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel update Task");
        task_t intakeTask = task_create(intakeControl, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "intake task");
        task_t pistonIndexerTask = task_create(pneumaticIndexerControl, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "pneumatic indexer task");
        task_t shoot3Task = task_create(shoot3Control, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Shoot 3 Task");
        task_t colorRollerTask = task_create(flexRollerControl, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Color roller button task");
        task_t endgameTask = task_create(endgameControl, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "endgame task");
        task_t magazineDiskCounter = task_create(magazineDiskCountControl, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "magazine disk counter");
//        test();
    }
}
