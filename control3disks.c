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
