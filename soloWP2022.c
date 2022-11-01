void rightSoloWP() {
  double currentDist = 0.0;
  float distance = 0.0;

  RightBackMotor.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  LeftBackMotor.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  RightMiddleMotor.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  LeftMiddleMotor.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  RightFrontMotor.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  LeftFrontMotor.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);

  frontFork.set(1); //set front fork down
  distance = 23/circumference/1.4; //84/60, DRIVE INCHES: 23 inches

  while (distance > abs(currentDist)) {
      currentDist = max(RightFrontMotor.rotation(vex::rotationUnits::rev), LeftFrontMotor.rotation(vex::rotationUnits::rev));
      wait(10, msec);
    }
    
  LeftFrontMotor.stop(vex::brakeType::brake);
  RightFrontMotor.stop(vex::brakeType::brake);
  LeftMiddleMotor.stop(vex::brakeType::brake);
  RightMiddleMotor.stop(vex::brakeType::brake);
  LeftBackMotor.stop(vex::brakeType::brake);
  RightBackMotor.stop(vex::brakeType::brake);

  task::sleep(5);   

  frontFork.set(0); //set front fork up
  
  RightBackMotor.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  LeftBackMotor.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  RightMiddleMotor.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  LeftMiddleMotor.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  RightFrontMotor.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  LeftFrontMotor.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);

  resetDistance();

  distance = 25/circumference/1.4; //84/60

  FrontClampPneu.set(1); //Front long up

  while (distance > abs(currentDist)) {
    currentDist = max(RightFrontMotor.rotation(vex::rotationUnits::rev), LeftFrontMotor.rotation(vex::rotationUnits::rev));
    wait(10, msec);
  }

  DriveInches(10, -30, fwd);

  frontFork.set(1);
  // wait(10, msec);

  driveDistance(70, 10, 8);
  wait(10, msec);
  FrontClampPneu.set(0);
  // FrontPassivePneu.set(0);
  wait(0.1, msec);

  smallArmUp();
  frontFork.set(0); 

  turnHeading(265, 15); 

  driveSonarToMG(2.2, 50, false, backsonar); //drives to pick up aliiance mobile goal

  BackTiltPn2.set(1);
  wait(50, msec);
  vex::task startrings(oneRing);

  DriveInches(5, 15, fwd);

  turnHeading(269, 15);

  std::cout << "HEADING" << Gyro.heading() << endl;
  wait(10, msec);

  vex::task dropMG(dropMGinSoloWP);

  driveHeadingWithRampDown(270, 100, 90, 8, true);

  turnHeading(0, 15);
  BackClampPneu.set(0);

  if (backsonar.distance(vex::distanceUnits::in) > 8) {
    driveSonar(12, 50, false, backsonar); // drive back toward wall using sonar if all good
  } else if (backsonar.distance(vex::distanceUnits::in) < 8) {
    DriveInches(12, -30, fwd); //otherwise, drive distance
  } 

  turnHeading(293, 15);

  driveSonarToMG(2.2, 50, false, backsonar);
  BackTiltPn2.set(1);
  wait(200, msec);
  moveIntake(35);
  
  DriveInches(15, 95, fwd);
  BackTiltPn2.set(0);
}
