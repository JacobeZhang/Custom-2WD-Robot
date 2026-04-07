long getLTicks() {
  return encoderL.getCount(); // positive value means wheel is moving forwards, negative means backwards
}

long getRTicks() {
  return encoderR.getCount(); // offset encoderR by just a little bit
  // Smaller than 1 means more to the left
}

float normalizeError(float target, float current) {
  float error = target - current;
  while (error > 180) error -= 360;
  while (error < -180) error += 360;
  return error;
}

void printTicks() {
  Serial.print("Left: ");
  Serial.print(encoderL.getCount());
  Serial.println(" ");
  Serial.print("Right: ");
  Serial.print(encoderR.getCount());
  Serial.println(" ");
  Serial.print("Distance: ");
  Serial.print(encoderL.getCount() / ticksPerCm);
  Serial.println(" ");
}

double deadzoneCorrection(double input, int min, int max) {
  if (input == 0) return 0.0;
  if (input > 0) return map(input, 1, max, min, max);
  else return map(input, -max, -1, -max, -min);
}

void move (int powerL, int powerR) {
  if (powerL >= 0) {
    digitalWrite(IN_1_L, LOW);
    digitalWrite(IN_2_L, HIGH);
  } else {
    digitalWrite(IN_1_L, HIGH);
    digitalWrite(IN_2_L, LOW);
  }

  if (powerR >= 0) {
    digitalWrite(IN_1_R, HIGH);
    digitalWrite(IN_2_R, LOW);
  } else {
    digitalWrite(IN_1_R, LOW);
    digitalWrite(IN_2_R, HIGH);
  }

  ledcWrite(EN_L, constrain(abs(powerL), 0, 255));
  ledcWrite(EN_R, constrain(abs(powerR), 0, 255));
}

void brake() {
  digitalWrite(IN_1_L, LOW);
  digitalWrite(IN_2_L, LOW);

  digitalWrite(IN_1_R, LOW);
  digitalWrite(IN_2_R, LOW);

  ledcWrite(EN_L, 0);
  ledcWrite(EN_R, 0);
}

void moveDist(double dist, double seconds) {

  encoderL.setCount(0);
  encoderR.setCount(0);
  unsigned long startTime = millis();

  unsigned long lastTime = startTime;
  double lastErr = 0;
  double errSum = 0;
  
  double Input = 0;
  double Setpoint = ticksPerCm * dist * 0.99; // Target
  double Output = 0;

  double initialYaw = heading.yaw;
  double errYaw = normalizeError(initialYaw, heading.yaw);
  double yawCorrection = errYaw * 5.0;

  bool printData = false;

  while (millis() - startTime < (seconds * 1000.0)) {

    Input = (getLTicks() + getRTicks()) / 2; // Average Ticks

    unsigned long now = millis();

    if (rvc.read(&heading)) {

      double timeChange = (double) ((now - lastTime) / 1000.0);
      
      double error = Setpoint - Input;
      errSum += (error * timeChange);
      double dErr = (error - lastErr) / timeChange;

      //Output = constrain(0.5 * error, -150, 150);
      
      Output = constrain(0.5 * error + dErr * 0.05, -150, 150);

      errYaw = normalizeError(initialYaw, heading.yaw);
      yawCorrection = errYaw * 8.0;
      if (abs(error) < 200) {
        yawCorrection = errYaw * 0.75;
      }
      
      Serial.print("errorRot:");
      Serial.print(errYaw);
      Serial.print(",");
      Serial.print("dErr:");
      Serial.print(dErr);
      Serial.print(",");
      Serial.print("Output:");
      Serial.print(Output);
      Serial.print(",");

      printData = true;

      lastErr = error;
      lastTime = now;
    }

    int powerL = deadzoneCorrection(Output + yawCorrection, 105, 200);
    int powerR = deadzoneCorrection(Output - yawCorrection, 115, 210);  

    if (printData) {
      Serial.print("powerL:");
      Serial.print(powerL);
      Serial.print(",");
      Serial.print("powerR:");
      Serial.print(powerR);
      Serial.println(",");
      printData = false;
    }

    move(powerL, powerR);
  }
  printTicks();
}

void turn(bool right, double seconds) {

  // FOR IMU, CLOCKWISE IS POSITIVE AND COUNTERCLOCKWISE IS NEGATIVE
  double initialYaw = heading.yaw;

  encoderL.setCount(0);
  encoderR.setCount(0);
  
  unsigned long startTime = millis();

  unsigned long lastTime = startTime;
  double lastErr = 0;
  double errSum = 0;
  double Output = 0;
  
  int reverseDirection = 1;
  if (right) reverseDirection = -1;

  double Setpoint = -90;
  if (right) Setpoint = 90; // Set Target

  while (millis() - startTime < (seconds * 1000.0)) {
    double errorDiff = getLTicks() + getRTicks();
    double diffCorrection = 1 * errorDiff;

    int powerL = deadzoneCorrection(Output + diffCorrection, 105, 255); // Initialize some value to powerL and powerR in case IMU hasn't updated its data yet
    int powerR = deadzoneCorrection(Output - diffCorrection, 115, 255);

    unsigned long now = millis();

    if (rvc.read(&heading)) {

      double timeChange = (double) ((now - lastTime) / 1000.0);

      double Input = heading.yaw - initialYaw; // Average Ticks
      
      double error = normalizeError(Setpoint, Input); // When error is positive, robot is on its way towards the target, when error is negative, robot has moved past the target
      errSum += (error * timeChange);
      double dErr = (error - lastErr) / timeChange;
      
      Output = constrain(1 * error, -200, 200);
      if (abs(error) < 10) Output = constrain(4.5 * error + dErr * 0.125, -200, 200);
      
      Serial.print("errorDiff:");
      Serial.print(errorDiff);
      Serial.print(",");
      Serial.print("error:");
      Serial.print(error);
      Serial.print(",");
      Serial.print("dErr:");
      Serial.print(dErr);
      Serial.print(",");
      Serial.print("Output:");
      Serial.print(Output);
      Serial.println(",");
      Serial.print("powerL:");
      Serial.print(powerL);
      Serial.print(",");
      Serial.print("powerR:");
      Serial.print(powerR);
      Serial.println(",");
      lastErr = error;
      lastTime = now;
    }

    powerL = deadzoneCorrection(Output - diffCorrection, 105, 255);
    powerR = deadzoneCorrection(Output + diffCorrection, 115, 255);  

    move(powerL, -powerR);
  }
  printTicks();
}

/*
------ FUNCTIONS BELOW USE ENCODERS ONLY -------------
void moveDist(double dist, double seconds) {
  int count = 0;
  int reverseDirection = 1;
  if (dist < 0) {
    reverseDirection = -1;
  }
  encoderL.setCount(0);
  encoderR.setCount(0);
  unsigned long startTime = millis();
  while (millis() - startTime < 500) {
    double rampUp = (map(millis()-startTime, 0, 1000, 100, 200) * reverseDirection);
    double errorRot = getLTicks() - getRTicks();
    int powerL = rampUp - errorRot * 6;
    int powerR = rampUp + errorRot * 6;
    move(powerL, powerR);
    if (count > 1000) {
      Serial.println(errorRot);
      count = 0;
    }
    count++;
  } 

  unsigned long lastTime = 0;
  double lastErr = 0;
  double errSum = 0;
  double Output = -1000000;

  while (millis() - startTime < ((seconds * 1000.0) - 500)) {
    double errorRot = getLTicks() - getRTicks();
    double rotCorrection = errorRot * 1;

    double Input = (getLTicks() + getRTicks()) / 2; // Average Ticks
    double Setpoint = ticksPerCm * dist * 0.99; // Target

    unsigned long now = millis();
    double timeChange = (double) (now - lastTime);

    if (Output == -1000000) { // Do some shenanigans to make sure that Output isn't 0 at the start
      Output = constrain(0.5 * (Setpoint - Input), -150, 150);
    }

    int powerL = deadzoneCorrection(Output - rotCorrection, 105, 200); // RIGHT AND LEFT MOTOR HAVE DIFFERENT DEAD ZONES
    int powerR = deadzoneCorrection(Output + rotCorrection, 115, 200);  

    if (timeChange >= 0.01) {
      
      double error = Setpoint - Input;
      errSum += (error * timeChange);
      double dErr = (error - lastErr) / timeChange;
      
      Output = constrain(0.5 * error + dErr * 50, -150, 150);
      
      Serial.print("errorRot:");
      Serial.print(errorRot);
      Serial.print(",");
      Serial.print("dErr:");
      Serial.print(dErr);
      Serial.print(",");
      Serial.print("Output:");
      Serial.print(Output);
      Serial.println(",");
      Serial.print("powerL:");
      Serial.print(powerL);
      Serial.print(",");
      Serial.print("powerR:");
      Serial.print(powerR);
      Serial.println(",");
      lastErr = error;
      lastTime = now;
    }

    powerL = deadzoneCorrection(Output - rotCorrection, 105, 200);
    powerR = deadzoneCorrection(Output + rotCorrection, 115, 200);  

    move(powerL, powerR);
  }
  printTicks();
}

void turn(bool right, double seconds) {
  encoderL.setCount(0);
  encoderR.setCount(0);
  
  unsigned long startTime = millis();

  unsigned long lastTime = 0;
  double lastErr = 0;
  double errSum = 0;
  double Output = -1000000;
  
  int reverseDirection = 1;
  if (right) reverseDirection = -1;

  while (millis() - startTime < (seconds * 1000.0)) {
    double errorRot = getLTicks() + getRTicks();
    double rotCorrection = reverseDirection * errorRot * 1;

    double Input = reverseDirection * (-getLTicks() + getRTicks()) / 2; // Average Ticks
    double Setpoint = ticksPerCm * 14.95 * M_PI / 4 * 0.99; // Target
    if (right) Setpoint = ticksPerCm * 15 * M_PI / 4 * 0.99;

    unsigned long now = millis();
    double timeChange = (double) (now - lastTime);

    if (Output == -1000000) { // Do some shenanigans to make sure that Output isn't 0 at the start
      Output = constrain(0.5 * (Setpoint - Input), -150, 150);
    }
    int powerL = deadzoneCorrection(Output + rotCorrection, 105, 200); // RIGHT AND LEFT MOTOR HAVE DIFFERENT DEAD ZONES
    int powerR = deadzoneCorrection(Output - rotCorrection, 115, 200);  

    if (timeChange >= 0.01) {
      
      double error = Setpoint - Input;
      errSum += (error * timeChange);
      double dErr = (error - lastErr) / timeChange;
      
      Output = constrain(0.5 * error + dErr * 50, -150, 150);
      
      Serial.print("errorRot:");
      Serial.print(errorRot);
      Serial.print(",");
      Serial.print("dErr:");
      Serial.print(dErr);
      Serial.print(",");
      Serial.print("Output:");
      Serial.print(Output);
      Serial.println(",");
      Serial.print("powerL:");
      Serial.print(powerL);
      Serial.print(",");
      Serial.print("powerR:");
      Serial.print(powerR);
      Serial.println(",");
      lastErr = error;
      lastTime = now;
    }

    powerL = deadzoneCorrection(Output + rotCorrection, 105, 200);
    powerR = deadzoneCorrection(Output - rotCorrection, 115, 200);  

    move(-powerL * reverseDirection, powerR * reverseDirection);
  }
  printTicks();
}
*/