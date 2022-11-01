/**
 * Auto aiming with GPS sensor
 * @param goalX
 * @param goalY
 * @param frontGPSsensor: if true, use front GPS; if false, use back
 * @param threeTile: if true, we start on 3 tile side; if false, we start on 2 tile side
 *
 * @return
 */
float autonGPSAngle(double goalX, double goalY, bool frontGPSsensor, bool threeTile, bool color) {
    // accessing robot position
    gps_status_s_t status;
    if (frontGPSsensor == true) {
        status = gps_get_status(GPSFRONT_PORT);
    } else status = gps_get_status(GPSBACK_PORT);
    double robotX = status.x;
    double robotY = status.y;

    double xDiff = goalX - robotX;
    double yDiff = goalY - robotY; // if tan is negative, robot turn left; if tan is positive, robot turn right

    double idealAngle = atan(yDiff / xDiff) * 180 / PI; //-86

    idealAngle = -idealAngle; // 86

    if (idealAngle > 45) {
        idealAngle = -idealAngle;  //-86
    }
    logStringWithFloat("GPS ", idealAngle);
    turnGyro(-idealAngle * 10, 127);
    brake(-10);
}

/**
 *
 * @param goalX
 * @param goalY
 * @param frontGPSsensor
 * @param red if true: red goal; if false: blue goal
 */
void skillsGPSAim(double goalX, double goalY, bool frontGPSsensor, bool red, bool bashCorrection) {
    // getting GPS position
    gps_status_s_t status;
    if (frontGPSsensor == true) {
        status = gps_get_status(GPSFRONT_PORT);
    } else status = gps_get_status(GPSBACK_PORT);
    double robotX = status.x;
    double robotY = status.y;
    //TURN TO GPS HEADING FUNCTION

    double xDiff = goalX - robotX;
    double yDiff = goalY - robotY; // if tan is negative, robot turn left; if tan is positive, robot turn right

    double idealAngleGPSCoor = 0; // angle to turn to in GPS global heading
    double alpha = 00; // random value
    if (red) {
        if (xDiff > 0 && yDiff < 0) {
            alpha = atan(yDiff / xDiff) * 180 / PI;
            idealAngleGPSCoor = 90 - alpha;
        } else if (yDiff == 0 && xDiff > 0) {
            idealAngleGPSCoor = 90;
        } else if (xDiff > 0 && yDiff > 0) {
            double alpha = atan(yDiff / xDiff) * 180 / PI;
            idealAngleGPSCoor = 90 - alpha;
        } else if (xDiff == 0) {
            idealAngleGPSCoor = 0;
        } else if (xDiff < 0 && yDiff > 0) {
            alpha = atan(yDiff / xDiff) * 180 / PI;
            idealAngleGPSCoor = -90 - alpha;
        }
    } else { // blue
        if (yDiff > 0 && xDiff < 0) {
            alpha = atan(yDiff / xDiff) * 180 / PI;
            idealAngleGPSCoor = -90 - alpha;
        } else if (xDiff < 0 && yDiff == 0) {
            idealAngleGPSCoor = -90;
        } else if (xDiff < 0 && yDiff < 0) {
            alpha = atan(yDiff / xDiff) * 180 / PI;
            idealAngleGPSCoor = -90 - alpha;
        } else if (xDiff == 0 && yDiff < 0) {
            idealAngleGPSCoor = -180; // should be same as 180
        } else if (xDiff > 0 && yDiff < 0) {
            alpha = atan(yDiff / xDiff) * 180 / PI;
            idealAngleGPSCoor = 90 - alpha;
        }
    }
    idealAngleGPSCoor = idealAngleGPSCoor / fabs(idealAngleGPSCoor) * (fabs(idealAngleGPSCoor) - bashCorrection);
    turnGPSAbsoluteGyro(idealAngleGPSCoor - bashCorrection, 30);
}

/**
 *
 * @param angle = absolute angle to turn to based on GPS coor.
 */
void turnGPSAbsoluteGyro(double targetAngle, double speed) {
//     converting so that [-180, 180] becomes [0, 360)
    if (targetAngle < 0) {
        targetAngle += 3600;
    }
    double currentAngle = gps_get_heading(GPSFRONT_PORT) * 10;
    logStringWithFloat("gps start angle = ", currentAngle);
    double difference = targetAngle - currentAngle;
    if (difference > 1800) {
        difference -= 3600;
    } else if (difference < -1800) {
        difference += 3600;
    }

    uint32_t cur = millis();
    uint32_t now;
    int tmp;
    int errorChange = 0;
    int prevError;
    int integral = 0;

    bool endSequence = false;
    int startEnd = 0;
    int endTime = 100;//200; //500
    int powerToAssign;

    // base on degree to turn, decide on staring power, and when to go into PID (last few degrees)
    prevError = fabs(difference);
    logStringWithFloat("fabs(difference) at start = ", prevError);
    if (prevError >= 1500) {
        speed = 127;
        tmp = 500;
    } else if (prevError >= 600) {
        speed = 127;
        tmp = 400;
    } else if (prevError >= 200) {
        speed = 63;
        tmp = max(prevError/2, 15);
    } else {
        speed = 63;
        tmp = 0;
    }
    bool left = false;
    if (difference > 0) {
        // clockwise
        // left = false
        powerToAssign = -speed;
    } else { // counterclockwise
        left = true;
        powerToAssign = speed;
    }

    int count = 0;
    if (tmp > 0) { // use start power to turn until reach degree to go into PID
        now = millis();
        assignLeftRightDriveMotorsVoltage(-powerToAssign, powerToAssign);
        do {
            task_delay_until(&now, 5);
            now += 5;
            currentAngle = gps_get_heading(GPSFRONT_PORT) * 10;
            difference = targetAngle - currentAngle;
            if (difference > 1800) {
                difference -= 3600;
            } else if (difference < -1800) {
                difference += 3600;
            }
            if (count % 50 == 0 || count == 0) logStringWithFloat("difference = ", difference);
            count ++;
        } while (fabs(difference) > tmp);
    }

    now = millis();
    prevError = difference;
    logStringWithFloat("prev error before PID start = ", difference);
    while ((fabs(errorChange) > TDRIVEMAXVEL || (!endSequence || (millis() - startEnd < endTime))) && millis() - cur < 900) {
        if (fabs(difference) < TDRIVEPOSTOL && !endSequence) {
            endSequence = true;
            startEnd = millis();
        }

        integral += difference;
        if (integral > TDRIVEINTEGRALLIMIT) {
            integral = TDRIVEINTEGRALLIMIT;
        } else if (integral < -TDRIVEINTEGRALLIMIT) {
            integral = -TDRIVEINTEGRALLIMIT;
        }

        errorChange = difference - prevError;
        prevError = difference;

        task_delay_until(&now, 5);
        now += 5;

        currentAngle = gps_get_heading(GPSFRONT_PORT) * 10;
        difference = targetAngle - currentAngle;
        if (difference > 1800) {
            difference -= 3600;
        } else if (difference < -1800) {
            difference += 3600;
        }
        powerToAssign = min(speed, max(-speed, difference * TDRIVEP + integral * TDRIVEI + errorChange * TDRIVED));
        if (left)
            powerToAssign = -powerToAssign;
        assignLeftRightDriveMotorsVoltage(-powerToAssign, powerToAssign);
    }
    //brake:
    if (left) {
        assignLeftRightDriveMotorsVoltage(-abs(speed)/2, abs(speed)/2);
    } else assignLeftRightDriveMotorsVoltage(abs(speed)/2, -abs(speed)/2);
    delay(50);
    assignDriveMotorsVoltage(0);
}
