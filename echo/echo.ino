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
  lineSensors.initThreeSensors();
  proxSensors.initThreeSensors();
   /* After setting up the proximity sensors with one of the
   * methods above, you can also customize their operation: */
  //proxSensors.setPeriod(420);
  //proxSensors.setPulseOnTimeUs(421);
  //proxSensors.setPulseOffTimeUs(578);
  //uint16_t levels[] = { 4, 15, 32, 55, 85, 120 };
  //proxSensors.setBrightnessLevels(levels, sizeof(levels)/2);
  
  calibrateLineSensors();

  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  int incomingByte = 0; // for incoming serial data
  
  // send data only when you receive data:
 /* if (Serial1.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial1.read();

    switch(incomingByte){
      case 'w':
      motors.setLeftSpeed(200);
      motors.setRightSpeed(200);
      delay(2);
      break;
      case 's':
      motors.setLeftSpeed(0);
      motors.setRightSpeed(0);
      delay(2);
      break;
      case 'r':
      motors.setLeftSpeed(200);
      motors.setRightSpeed(-200);
      delay(2);
      break;
      case 'l':
      motors.setRightSpeed(200);
      motors.setLeftSpeed(-200);
      delay(2);
      break;
      case 'b':
      motors.setRightSpeed(-200);
      motors.setLeftSpeed(-200);
      delay(2);
      break;
    }
    // say what you got:
    //Serial1.print("I received: ");
    //Serial1.println(incomingByte, DEC);
    //Serial.print("I received: ");
    //Serial.println(incomingByte, DEC);*/
      static uint16_t lastSampleTime = 0;

  if ((uint16_t)(millis() - lastSampleTime) >= 100)
  {
    lastSampleTime = millis();

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
}
void calibrateLineSensors()
{

   Serial1.println("Calibrating");
   Serial.println("Calibrating");
  for (uint16_t i = 0; i < 400; i++)
  {
    lineSensors.calibrate();
  }
   Serial1.println("Calibrated");
   Serial.println("Calibrated");
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
    if(aboveDarkSpot())
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
}

void leftMoving(){
  motors.setRightSpeed(200);
  motors.setLeftSpeed(-200);
  delay(2);
}

void rightMoving(){
  motors.setLeftSpeed(200);
  motors.setRightSpeed(-200);
  delay(2);
}

void backwardMoving(){
  motors.setRightSpeed(-200);
  motors.setLeftSpeed(-200);
  delay(2);
}
