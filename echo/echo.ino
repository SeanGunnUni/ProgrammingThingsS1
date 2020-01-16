
#include <Wire.h>
#include <Zumo32U4.h>
//Serial1 communicates over XBee
//Serial communicates over USB cable
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
LSM303 compass;
Zumo32U4Encoders encoders;
Zumo32U4Buzzer buzzer;
//
//#define sensorThreshold
#define sensorThresholdDark 100 //Temp need to find out what black is in sensor
#define CALIBRATION_SAMPLES 70  // Number of compass readings to take when calibrating
#define CRB_REG_M_2_5GAUSS 0x60 // CRB_REG_M value for magnetometer +/-2.5 gauss full scale
#define CRA_REG_M_220HZ    0x1C // CRA_REG_M value for magnetometer 220 Hz update rate
#define SPEED           200 // Maximum motor speed when going straight; variable speed when turning
#define TURN_SPEED 100 // Base speed when turning (added to variable speed)
#define NUM_SENSORS 5
#define StartX compass.m.x
#define StartY compass.m.y


uint16_t lineSensorValues[3] = { 0, 0, 0 };

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
// Initiate the Wire library and join the I2C bus as a master
  Wire.begin();
  // Initiate LSM303
  compass.init();
  // Enables accelerometer and magnetometer
  compass.enableDefault();
  compass.writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS); // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
  compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);    // 220 Hz compass update rate
//motors
//  motors.setLeftSpeed(SPEED);
//  motors.setRightSpeed(-SPEED);//RIGHT
//

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
  lineSensors.calibrate();
  encoders.init();
  path[pathLength] = "start";
  ++pathLength;
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
        case 'h':
        returnToStart();
        break;
        case 'c':
        clearPath();
        break;
    }
    foundAnyOne();
  }
}

void clearPath(){
  for(int i = 0; i<100;i++){
    path[i]="'\0";
  }
  pathLength=0;
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
    lineSensorValues[2]
  );
  Serial.println(buffer);
  Serial1.println(buffer);
}



// Takes calibrated readings of the lines sensors and stores them
// in lineSensorValues.  Also returns an estimation of the line
// position.
uint16_t readSensors()
{
  return lineSensors.readLine(lineSensorValues);
}


// Returns true if the sensor is seeing a lot of darkness.
// Make sure to call readSensors() before calling this.
bool aboveLineDark(uint8_t sensorIndex)
{
  return lineSensorValues[sensorIndex] > sensorThresholdDark;
}

void stopMoving(){
    if(path[0] != "start"){
    path[0] = "start";
    ++pathLength;
  }
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
  int endNumber = (encoders.getCountsLeft() + encoders.getCountsRight())/2;
  path[pathLength] = "S";
  ++pathLength;
  path[pathLength] = (endNumber);
  ++pathLength;
  delay(2);
}

void forwardMoving(){
  if(path[0] != "start"){
    path[0] = "start";
    ++pathLength;
  }
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  motors.setLeftSpeed(200);
  motors.setRightSpeed(200);
  int endNumber = (encoders.getCountsLeft() + encoders.getCountsRight())/2;
  path[pathLength] = "W";
  ++pathLength;
  path[pathLength] = (endNumber);
  ++pathLength;
  delay(2);
  uint8_t a = readSensors();
  if(!aboveLineDark(a)){
    stopMoving();
  }

}

void leftMoving(){
   if(path[0] != "start"){
    path[0] = "start";
    ++pathLength;
  }
  int32_t tempX = compass.m.x;
  int32_t tempY = compass.m.y;
  int32_t newCompassX = tempX - 90;
  int32_t newCompassY = tempY - 90;
  while((compass.m.x != newCompassX) && (compass.m.y != newCompassY)){
    Serial1.println("Compass X = " +compass.m.x);
    Serial.println("Compass X = " +compass.m.x);
    Serial1.println("Compass Y = " +compass.m.y);
    Serial.println("Compass Y = " +compass.m.y);
    motors.setRightSpeed(TURN_SPEED);
    motors.setLeftSpeed(-TURN_SPEED);
  }
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  int endNumber = (encoders.getCountsLeft() + encoders.getCountsRight())/2;
  ++pathLength;
  path[pathLength] = "L";
  ++pathLength;
  path[pathLength] = (endNumber);
  ++pathLength;
  delay(2);
}

void rightMoving(){
    if(path[0] != "start"){
    path[0] = "start";
    ++pathLength;
  }
  int32_t tempX = compass.m.x;
  int32_t tempY = compass.m.y;
  int32_t newCompassX = tempX + 90;
  int32_t newCompassY = tempY + 90;
  while((compass.m.x != newCompassX) && (compass.m.y != newCompassY)){
  Serial1.println("Compass X = " +compass.m.x);
  Serial.println("Compass X = " +compass.m.x);
  Serial1.println("Compass Y = " +compass.m.y);
  Serial.println("Compass Y = " +compass.m.y);
  motors.setRightSpeed(-TURN_SPEED);
  motors.setLeftSpeed(TURN_SPEED);
  
  }
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  int endNumber = (encoders.getCountsLeft() + encoders.getCountsRight())/2;
  ++pathLength;
  path[pathLength] = "R";
  ++pathLength;
  path[pathLength] = (endNumber);
  ++pathLength;
  delay(2);
}

void backwardMoving(){
      if(path[0] != "start"){
    path[0] = "start";
    ++pathLength;
  }
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  motors.setRightSpeed(-200);
  motors.setLeftSpeed(-200);
  int endNumber = (encoders.getCountsLeft() + encoders.getCountsRight())/2;
  delay(2);
  path[pathLength] = "B";
  ++pathLength;
  path[pathLength] = (endNumber);
  ++pathLength;
  uint8_t a = readSensors();
  if(!aboveLineDark(a)){
    stopMoving();
  }
}

void returnToStart(){
    buzzer.playFromProgramSpace(PSTR("!>c32"));
    delay(1000);

    for(uint16_t i = pathLength; i > 0; i--)
  {
    // Make a turn according to the instruction stored in
    // path[i].
    turn(path[i]);    // Follow a line segment until we get to the center of an
    // intersection.
    if( !(path[i-1] == "start"))
      followPath(path[i],path[i - 1]);
  }
}

void turn(char dir)
{
  int32_t tempX;
  int32_t tempY;
  int32_t newCompassX;
  int32_t newCompassY;
  switch(dir)
  {
  case 'S':
    return;
  case 'B':
    tempX = compass.m.x;
    tempY = compass.m.y;
    newCompassX = tempX + 180;
    newCompassY = tempY + 180;
    while((compass.m.x != newCompassX) && (compass.m.y != newCompassY)){
      Serial1.println("Compass X = " +compass.m.x);
      Serial.println("Compass X = " +compass.m.x);
      Serial1.println("Compass Y = " +compass.m.y);
      Serial.println("Compass Y = " +compass.m.y);
      motors.setRightSpeed(-TURN_SPEED);
      motors.setLeftSpeed(TURN_SPEED);
    }
    break;

  case 'L':
    tempX = compass.m.x;
    tempY = compass.m.y;
    newCompassX = tempX - 90;
    newCompassY = tempY - 90;
    while((compass.m.x != newCompassX) && (compass.m.y != newCompassY)){
      Serial1.println("Compass X = " +compass.m.x);
      Serial.println("Compass X = " +compass.m.x);
      Serial1.println("Compass Y = " +compass.m.y);
      Serial.println("Compass Y = " +compass.m.y);
      motors.setRightSpeed(TURN_SPEED);
      motors.setLeftSpeed(-TURN_SPEED);
      
    }
    break;

  case 'R':
    tempX = compass.m.x;
    tempY = compass.m.y;
    newCompassX = tempX + 90;
    newCompassY = tempY + 90;
    while((compass.m.x != newCompassX) && (compass.m.y != newCompassY)){
      Serial1.println("Compass X = " +compass.m.x);
      Serial.println("Compass X = " +compass.m.x);
      Serial1.println("Compass Y = " +compass.m.y);
      Serial.println("Compass Y = " +compass.m.y);
      motors.setRightSpeed(-TURN_SPEED);
      motors.setLeftSpeed(TURN_SPEED);
    }
    break;

  default:
    // This should not happen.
    return;
  }
}
void followPath(char dir, int lengthGone)
{
  int startLeft;
  int startRight;
  int endRight;
  int endLeft;
  int bothEncoders;
  switch(dir)
  {
  case 'R':
    return;
  case 'L':
    return;
  case 'B':
    startLeft = encoders.getCountsAndResetLeft();
    startRight = encoders.getCountsAndResetRight();
    endRight = (encoders.getCountsRight());
    endLeft = (encoders.getCountsRight());
    bothEncoders = (endRight + endLeft)/2;
    while(bothEncoders < lengthGone){
        motors.setRightSpeed(100);
        motors.setLeftSpeed(100);
        endRight = (encoders.getCountsRight());
        endLeft = (encoders.getCountsRight());
        bothEncoders = (endRight + endLeft)/2;
    }
    delay(2);
    break;
  case 'S':
   break;
  case 'W':
   startLeft = encoders.getCountsAndResetLeft();
    startRight = encoders.getCountsAndResetRight();
    endRight = (encoders.getCountsRight());
    endLeft = (encoders.getCountsRight());
    bothEncoders = (endRight + endLeft)/2;
    while(bothEncoders < lengthGone){
        motors.setRightSpeed(100);
        motors.setLeftSpeed(100);
        endRight = (encoders.getCountsRight());
        endLeft = (encoders.getCountsRight());
        bothEncoders = (endRight + endLeft)/2;
    }
    delay(2);
   break;
   case 'P':
      proxSensors.read();
    // Just read the proximity sensors without sending pulses.
    int psl = proxSensors.readBasicLeft();
    int psf = proxSensors.readBasicFront();
    int psr = proxSensors.readBasicRight();
    char psls[31];
    char psfs[31];
    char psrs[31];
    itoa(psl,psls,31);
      itoa(psf,psfs,31);
        itoa(psr,psrs,31);
    //proxLeftActive = proxSensors.readBasicLeft();
    //proxFrontActive = proxSensors.readBasicFront();
    //proxRightActive = proxSensors.readBasicRight();
   // Serial1.println("Proximity Sensor Left no IR = " << psls << " Proximity Sensor Front no IR = " << psfs << " Proximity Sensor Right no IR = " << psrs);
  default:
    // This should not happen.
    return;
  }
}





void foundAnyOne(){
  // Send IR pulses and read the proximity sensors.
  proxSensors.read();
  // Just read the proximity sensors without sending pulses.
  int psl = proxSensors.readBasicLeft();
  int psf = proxSensors.readBasicFront();
  int psr = proxSensors.readBasicRight();
  char psls[31];
  char psfs[31];
  char psrs[31];
  itoa(psl,psls,31);
    itoa(psf,psfs,31);
      itoa(psr,psrs,31);
  //proxLeftActive = proxSensors.readBasicLeft();
  //proxFrontActive = proxSensors.readBasicFront();
  //proxRightActive = proxSensors.readBasicRight();
 // Serial1.println("Proximity Sensor Left no IR = " << psls << " Proximity Sensor Front no IR = " << psfs << " Proximity Sensor Right no IR = " << psrs);
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  int endNumber = (encoders.getCountsLeft() + encoders.getCountsRight())/2;
  path[pathLength] = "P";
  ++pathLength;
  path[pathLength] = (endNumber);
  ++pathLength;
  //if((proxFrontActive  == ...) || (proxRightActive == ... )||(proxLeftActive == ...)){}
  printReadingsToSerial();
  
}
