//#include MazeSolver
//#include GridMovement.h
//#include TurnSensor.h
#include <Wire.h>
#include <Zumo32U4.h>

//Serial1 communicates over XBee
//Serial communicates over USB cable
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;

//
#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];
//

uint16_t lineSensorValues[5] = { 0, 0, 0, 0, 0 };

bool proxLeftActive;
bool proxFrontActive;
bool proxRightActive;

char path[100];
uint8_t pathLength = 0;

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600);
  Serial.begin(9600);
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);

  //sensors
  lineSensors.initFiveSensors();
  proxSensors.initFrontSensor();
   /* After setting up the proximity sensors with one of the
   * methods above, you can also customize their operation: */
  //proxSensors.setPeriod(420);
  //proxSensors.setPulseOnTimeUs(421);
  //proxSensors.setPulseOffTimeUs(578);
  //uint16_t levels[] = { 4, 15, 32, 55, 85, 120 };
  //proxSensors.setBrightnessLevels(levels, sizeof(levels)/2);
  lineSensors.calibrate();

  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  int incomingByte = 0; // for incoming serial data
  
  // send data only when you receive data:
 if (Serial1.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial1.read();

    switch(incomingByte){
        case 'w':
        forwardMoving();
        break;
        case 's':
        stopMoving();
        break;
        case 'r':
        rightMoving();
        break;
        case 'l':
        leftMoving();
        break;
        case 'b':
        backwardMoving();
        break;
    }
  }
  static uint16_t lastSampleTime = 0;
  if ((uint16_t)(millis() - lastSampleTime) >= 100)
  {
    lastSampleTime = millis();
      // Send IR pulses and read the proximity sensors.
      proxSensors.read();
  
      // Just read the proximity sensors without sending pulses.
      proxLeftActive = proxSensors.readBasicLeft();
      proxFrontActive = proxSensors.readBasicFront();
      proxRightActive = proxSensors.readBasicRight();
  
      // Read the line sensors.
      lineSensors.readCalibrated(lineSensorValues);
  
      // Send the results to the LCD and to the serial monitor.
      printReadingsToSerial();
  } 
}

void printReadingsToSerial()
{
  static char buffer[80];
  sprintf(buffer, "%d %d %d %d %d %d  %d %d %d  %4d %4d %4d %4d %4d\n",
    proxSensors.countsLeftWithLeftLeds(),
    proxSensors.countsLeftWithRightLeds(),
    proxSensors.countsFrontWithLeftLeds(),
    proxSensors.countsFrontWithRightLeds(),
    proxSensors.countsRightWithLeftLeds(),
    proxSensors.countsRightWithRightLeds(),
    proxLeftActive,
    proxFrontActive,
    proxRightActive,
    lineSensorValues[0],
    lineSensorValues[1],
    lineSensorValues[2],
    lineSensorValues[3],
    lineSensorValues[4]
  );
  Serial.println(buffer);
  Serial1.println(buffer);
}

void mazeSolve()
{
  // Start with an empty path.  We will add and remove entries to
  // the path as we solve the maze.
  pathLength = 0;
  delay(1000);

  while(1)
  {
    uint8_t a = readSensors();
    if(!aboveLineDark(a))
    {
      // We found the end of the maze, so we succeeded in solving
      // the maze.
      stopMoving();
      Serial1.println("what to do?");
      Serial.println("what to do?");
      break;
    }
  }
}

// Takes calibrated readings of the lines sensors and stores them
// in lineSensorValues.  Also returns an estimation of the line
// position.
uint16_t readSensors()
{
  return lineSensors.readLine(lineSensorValues);
}

// Returns true if the sensor is seeing a line.
// Make sure to call readSensors() before calling this.
bool aboveLine(uint8_t sensorIndex)
{
  return lineSensorValues[sensorIndex] > sensorThreshold;
}

// Returns true if the sensor is seeing a lot of darkness.
// Make sure to call readSensors() before calling this.
bool aboveLineDark(uint8_t sensorIndex)
{
  return lineSensorValues[sensorIndex] > sensorThresholdDark;
}

void stopMoving(){
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
  delay(2);
}

void forwardMoving(){
  motors.setLeftSpeed(200);
  motors.setRightSpeed(200);
  delay(2);
  uint8_t a = readSensors();
  if(!aboveLineDark(a)){
    stopMoving();
  }
}

void leftMoving(){
  motors.setRightSpeed(200);
  motors.setLeftSpeed(-200);
  delay(2);
    uint8_t a = readSensors();
  if(!aboveLineDark(a)){
    stopMoving();
  }
}

void rightMoving(){
  motors.setLeftSpeed(200);
  motors.setRightSpeed(-200);
  delay(2);
    uint8_t a = readSensors();
  if(!aboveLineDark(a)){
    stopMoving();
  }
}

void backwardMoving(){
  motors.setRightSpeed(-200);
  motors.setLeftSpeed(-200);
  delay(2);
    uint8_t a = readSensors();
  if(!aboveLineDark(a)){
    stopMoving();
  }
}

void backTrack(){
  }

void turnLeft90(){
  turnSensorReset();
  // Turn to the left 90 degrees.
  motors.setSpeeds(-calibrationSpeed, calibrationSpeed);
  while((int32_t)turnAngle < turnAngle45 * 2)
  {
    lineSensors.calibrate();
    turnSensorUpdate();
  }
 }

void turnRight90(){
    turnSensorReset();
  // Turn to the right 90 degrees.
  motors.setSpeeds(calibrationSpeed, -calibrationSpeed);
  while((int32_t)turnAngle > -turnAngle45 * 2)
  {
    lineSensors.calibrate();
    turnSensorUpdate();
  }
}

void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}

// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void turnSensorUpdate()
{
  // Read the measurements from the gyro.
  gyro.read();
  turnRate = gyro.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how
  // much the robot has turned since the last update.
  // (angular change = angular velocity * time)
  int32_t d = (int32_t)turnRate * dt;

  // The units of d are gyro digits times microseconds.  We need
  // to convert those to the units of turnAngle, where 2^29 units
  // represents 45 degrees.  The conversion from gyro digits to
  // degrees per second (dps) is determined by the sensitivity of
  // the gyro: 0.07 degrees per second per digit.
  //
  // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
  // = 14680064/17578125 unit/(digit*us)
  turnAngle += (int64_t)d * 14680064 / 17578125;
}
