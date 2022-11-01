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
               motor_move(PORT_FLYWHEEL, flywheelPower);

               delay(10);
           } else {
               shoot3Skills(flywheelPower);
               delay(10);
           }
       }
       delay(100);
    }
}
