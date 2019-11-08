#include <Zumo32U4.h>

//Serial1 communicates over XBee
//Serial communicates over USB cable
Zumo32U4Motors motors;
void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600);
  Serial.begin(9600);
    //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);

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
    //Serial.println(incomingByte, DEC);
  }
}
