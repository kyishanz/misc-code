/**
 * Turn roller to specified color.
 * @param color [red: 0, blue: 1]
 */
void turnRoller(int color, int timeout) {
    int startTime = millis();
    optical_set_led_pwm(OPTICAL, 100);
    printf("%d\n", color);
    int currentColor = -1;
    assignDriveMotorsVoltage(-30); // drive forwards to press to roller
    int initialColor = colorOfRoller();
    logInt(initialColor);
    printf("HELLO");
    if (initialColor == color) {
        motor_move(PORT_INTAKE, convert100to127(65)); // 65
        currentColor = colorOfRoller();
        while ((currentColor == color || currentColor == -1) && (millis() - startTime < timeout)) {
            delay(5);
            currentColor = colorOfRoller();
        }
        logInt(currentColor);
        while ((currentColor != color || currentColor == -1) && (millis() - startTime < timeout)) {
            delay(5);
            currentColor = colorOfRoller();
        }
        motor_set_brake_mode(PORT_INTAKE, E_MOTOR_BRAKE_BRAKE);
        logInt(currentColor);
        motor_move(PORT_INTAKE, -60); //60
        delay(100);
        motor_move(PORT_INTAKE, 0);
    } else {
        motor_move(PORT_INTAKE, convert100to127(65)); // 65
        currentColor = colorOfRoller();
        while ((currentColor != color || currentColor == -1) && (millis() - startTime < timeout)) {
            delay(5);
            currentColor = colorOfRoller();
        }
        motor_move(PORT_INTAKE, -60); // 60
        delay(100);
        motor_move(PORT_INTAKE, 0);
    }
    assignDriveMotorsVoltage(0);
    optical_set_led_pwm(OPTICAL, 0);
}
