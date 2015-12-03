/*


test4a.ino

Demonstrates .run() in a function - move()
move until a sensor value is seen - move_to_sensor1()

This removes sensor and processing irregularities from
stepper motor control 

*/

#include <AccelStepper.h>
#include <NewPing.h>

const int MAX=200; // max sonar range
const int MotorEnable=13; // pin to enable/disable easystepper

//
// function declarations 
//
void destL(int speed, long int pos, int dir);
void destR(int speed, long int pos, int dir); 
void move(void);
int move_to_sensor1(int stopdist, int movedist);
int pingAve1(int num); 
void motorsOff(void);
void motorsOn(void);

//
//  AccelStepper(# motor wires, pulse pin#, direction pin#)
//
int DirLpin = 6; 
int DirRpin = 4; 
AccelStepper stepperL(1, 7, DirLpin); 
AccelStepper stepperR(1, 5, DirRpin); 

//
// ultrasonic sensors
//
#define TRIGGER1 30
#define ECHO1 31
NewPing sonar1(TRIGGER1,ECHO1, MAX);

//
// values to control EasyDriver 
//
#define MS1 52  // Mega pin for step size
#define MS2 53  // Mega pin for step size

int pos = 400;
int maxspeed = 600;
int motoraccel = 500 ; 
float FullStepSize=2.65; // cm

int pingDist; // distance variable
int moveDist; // distance variable

// setup - things that happen only once at start
void setup()
{  
  Serial.begin(115200);
  stepperL.setMaxSpeed(maxspeed);
  stepperL.setAcceleration(motoraccel);
  stepperR.setMaxSpeed(maxspeed);
  stepperR.setAcceleration(motoraccel);
  pinMode(MotorEnable, OUTPUT);  //HIGH = freewheel, LOW=enable 
  pinMode(MS1, OUTPUT);  
  pinMode(MS2, OUTPUT); 
  digitalWrite(MS1, LOW); 
  digitalWrite(MS2, LOW); 

}

//
// loop forever 
//
void loop()
{
  stepperL.setCurrentPosition(0) ;  //wherever we are now is zero
  stepperR.setCurrentPosition(0) ;  //wherever we are now is zero

  motorsOff();

  pingDist=pingAve1(2);

    if (pingDist > 20)
    {
      moveDist = pingDist * FullStepSize;
    }
    else if (pingDist > 10 && pingDist < 20)
    {
      moveDist = 0;
    }
    else if (pingDist < 10 && pingDist >0)
    {
      moveDist = -10 * FullStepSize; // back up 10 cm
    }
    else if (pingDist < 0)
    {
      moveDist = 10;  // wall out of range
    }

  //moveDist = pingDist * FullStepSize;

  destL(maxspeed,moveDist,1);
  destR(maxspeed/5,moveDist,1);

  Serial.print("wall at: ");
  Serial.print(pingDist); Serial.print(" Moving: ");  Serial.println(moveDist);

  move();
//  move_to_sensor1(15, 100);
  delay(1000);

}

//
// set speed, destination, dir for left motor
//
void destL(int speed, long int pos, int dir) {
  stepperL.setMaxSpeed(speed);
  stepperL.moveTo(pos);
  //digitalWrite(DirLpin, dir);
}

//
// set speed, dest, dir for right motor
//
void destR(int speed, long int pos, int dir) {
  stepperR.setMaxSpeed(speed);
  stepperR.moveTo(pos);
  //digitalWrite(DirRpin, dir);

}

//
// move both motors a predefined distance
// 
void move(){
  
  motorsOn();

/*
  Serial.print("moving... dist  distR to go:  ");
  Serial.print(stepperL.distanceToGo());
  Serial.print(" , ");
  Serial.print(stepperR.distanceToGo()); Serial.println(".");
*/

  while (stepperL.distanceToGo() !=0 || stepperR.distanceToGo() != 0) {
    stepperL.run();
    stepperR.run();
  }
//  Serial.println("done, returning");
  motorsOff();
}

//
//  move until sensor 1 says to stop
//  if ping returns someting > stopdist, decellerate over movedist steps
//
int move_to_sensor1(int stopdist, int movedist){

int pingdist;
int togoL;
int togoR;

togoL = stepperL.distanceToGo();
togoR = stepperR.distanceToGo();

  while (togoL != 0 || togoR != 0) {

    pingdist=pingAve1(1);

    if (pingdist>stopdist) {
//      stepperL.moveTo(stepperL.currentPosition()+movedist);;
//      stepperR.moveTo(stepperR.currentPosition()+movedist);;
      stepperL.moveTo(stepperL.currentPosition()+1);;
      stepperR.moveTo(stepperR.currentPosition()+1);;
    }
    stepperL.run();
    stepperR.run();
  }
  return(0);
}

//
// pingAve1(int num)  - average num pings from sensor 1
//
int pingAve1(int num) {

  int dist=0;
  int distAve=0;
  int count=0;
  int ret;

  for (int x=0; x < num; x++) {
    dist = sonar1.ping()/US_ROUNDTRIP_CM;
    if (dist > 0) {
      distAve+=dist;
      count += 1;
    }
    delayMicroseconds(5000); // min time between pings or they jam each other
  }
  ret = distAve/count;
//  Serial.print("PingAve1 returns: ");
//  Serial.println(ret); 
  return(ret);
}

//
// disable all motors to save power.  Freewheels.
//
void motorsOff(void) {
  digitalWrite(MotorEnable, HIGH);
}

//
// enable all motors.  Uses power to sit still, won't roll
//
void motorsOn(void) {
  digitalWrite(MotorEnable, LOW);
}


