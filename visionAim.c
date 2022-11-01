/**
 * Aim via vision
 * @param color [0 = red; 1 = blue]
 */
void visionAim(int color, int power, int timeout) {
    int startTime = millis();
    vision_object_s_t goal;
    if (color == 0) {
        // red
        goal = vision_get_by_sig(VISION, 0, 2); // 0 is largest signature [1st in list]
    } else {
        // blue
        goal = vision_get_by_sig(VISION, 0, 1); // 0 is largest signature [1st in list]
    }
    int goalCenterX = goal.x_middle_coord;
    logInt(goalCenterX);
    bool seeSomething = true;
    if (goalCenterX == 0) {
        seeSomething = false; // do nothing if you don't see anything
    }
    float centerOfVisionField = 0; // 316 pixels horizontally; coor. set center to 0,0
    bool ccw = false;
    if (goalCenterX < centerOfVisionField) {
        ccw = true;
        logStringWithFloat("ccw = ", 1);
    } // if goalCenterX > centerOfVisionField, ccw = false
    logStringWithFloat("ccw = ", 0);
    while (seeSomething && fabs(goalCenterX - centerOfVisionField) > 2 && ((millis() - startTime) < timeout)) {
        double adjustPower = goalCenterX * VISIONCONST;
        if (ccw) {
            assignLeftRightDriveMotorsVoltage(power + adjustPower, -power - adjustPower);
        } else {
            assignLeftRightDriveMotorsVoltage(-power - adjustPower, power + adjustPower);
        }
        delay(5);
        if (color == 0) goal = vision_get_by_sig(VISION, 0, 2); // 0 is largest signature [1st in list]
        else goal = vision_get_by_sig(VISION, 0, 1); // 0 is largest signature [1st in list]
        goalCenterX = goal.x_middle_coord;
    }

    if (ccw) {
        assignLeftRightDriveMotorsVoltage(-abs(power)/2, abs(power)/2);
    } else assignLeftRightDriveMotorsVoltage(abs(power)/2, -abs(power)/2);
    printf("done");
    delay(50);
    assignDriveMotorsVoltage(0);
}
