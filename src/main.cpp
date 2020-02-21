#include <Arduino.h>
#include <NewPing.h>
#include "drive.h"

#define TRIGGER_PINL  6  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINL     7  // Arduino pin tied to echo pin on ping sensor.
#define TRIGGER_PINF  4  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINF     5  // Arduino pin tied to echo pin on ping sensor.
#define TRIGGER_PINR  2  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINR     3  // Arduino pin tied to echo pin on ping sensor.
#define RMOTOR_1      9
#define RMOTOR_2      10
#define LMOTOR_1      11
#define LMOTOR_2      12
#define HBRIDGE_G     13
#define SWITCH_OUT    22
#define SWITCH_IN     23
#define TURN_TIME     400
#define WAIT_TIME     500

#define MAX_DISTANCE 100 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

int drivePinsR[2] = {RMOTOR_1, RMOTOR_2};
int drivePinsL[2] = {LMOTOR_1, LMOTOR_2};

NewPing sonarL(TRIGGER_PINL, ECHO_PINL, MAX_DISTANCE);
NewPing sonarF(TRIGGER_PINF, ECHO_PINF, MAX_DISTANCE);
NewPing sonarR(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE);

int sonarReadings[3] = {0, 0, 0};

void readSonars(int * sonarReadings) {
  sonarReadings[0] = sonarL.ping_in();
  delay(300);
  sonarReadings[1] = sonarF.ping_in();
  delay(300);
  sonarReadings[2] = sonarR.ping_in();

  Serial.print("Ping: ");
  Serial.print(sonarReadings[0]);
  Serial.println("in");

  Serial.print("Ping: ");
  Serial.print(sonarReadings[1]);
  Serial.println("in");

  Serial.print("Ping: ");
  Serial.print(sonarReadings[2]);
  Serial.println("in");
}

void turnLeft(int * drivePinsL, int * drivePinsR) {
  drive(drivePinsL, -125);
  drive(drivePinsR, 125);
  delay(TURN_TIME);
  brake(drivePinsL[0], drivePinsL[1]);
  brake(drivePinsR[0], drivePinsR[1]);
  delay(WAIT_TIME);
}

void turnRight(int * drivePinsL, int * drivePinsR) {
  drive(drivePinsL, 125);
  drive(drivePinsR, -125);
  delay(TURN_TIME);
  brake(drivePinsL[0], drivePinsL[1]);
  brake(drivePinsR[0], drivePinsR[1]);
  delay(WAIT_TIME);
}

void forwardOneFoot(int * drivePinsL, int * drivePinsR, int * sonarReadings) {
  int frontReading = sonarReadings[1];
  int currReading = sonarReadings[1];
  drive(drivePinsL, 125);
  drive(drivePinsR, 125);
  while (frontReading - currReading < 0.9 * 12) {
    currReading = sonarF.ping_in();
    delay(300);
  }
  brake(drivePinsL[0], drivePinsL[1]);
  brake(drivePinsR[0], drivePinsR[1]);
  delay(WAIT_TIME);
}

void reverseOneFoot(int * drivePinsL, int * drivePinsR, int * sonarReadings) {
  int frontReading = sonarReadings[1];
  int currReading = sonarReadings[1];
  drive(drivePinsL, -125);
  drive(drivePinsR, -125);
  while (currReading - frontReading < 0.9 * 30) {
    currReading = sonarF.ping_cm();
    delay(300);
  }
  brake(drivePinsL[0], drivePinsL[1]);
  brake(drivePinsR[0], drivePinsR[1]);
  delay(WAIT_TIME);
}

void setup() {
  Serial.begin(9600);
  pinMode(RMOTOR_1, OUTPUT);
  pinMode(RMOTOR_2, OUTPUT);
  pinMode(LMOTOR_1, OUTPUT);
  pinMode(LMOTOR_2, OUTPUT);
  pinMode(HBRIDGE_G, OUTPUT);
  pinMode(SWITCH_IN, INPUT);
  pinMode(SWITCH_OUT, OUTPUT);
  digitalWrite(HBRIDGE_G, LOW);
  digitalWrite(SWITCH_OUT, HIGH);
}

void loop() {
  if (digitalRead(SWITCH_IN) == LOW) {
    brake(RMOTOR_1, RMOTOR_2);
    brake(LMOTOR_1, LMOTOR_2);
    delay(10000000);
  } else {
    readSonars(sonarReadings);
    if (sonarReadings[0] > 8) { // if left is open, turn left
      turnLeft(drivePinsL, drivePinsR);
    } else if (sonarReadings[1] > 8) {
      forwardOneFoot(drivePinsL, drivePinsR, sonarReadings);
    } else if (sonarReadings[2] > 8) {
      turnRight(drivePinsL, drivePinsR);
    } else {
      //panic
    }
  }
}

/*

#include <Arduino.h>

// defines pins numbers
const int trigPin = 2;
const int echoPin = 3;
// defines variables
long duration;
int distance;
void setup() {
pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin, INPUT); // Sets the echoPin as an Input
Serial.begin(9600); // Starts the serial communication
}
void loop() {
// Clears the trigPin
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
duration = pulseIn(echoPin, HIGH);
// Calculating the distance
distance= duration*0.034/2;
// Prints the distance on the Serial Monitor
Serial.print("Distance: ");
Serial.println(distance);
}
/*

int dir;


#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4



float P = 0.7 ;
float D = 0.5 ;
float I = 0.4 ;
float oldErrorP ;
float totalError ;
int offset = 5 ;

int wall_threshold = 13 ;
//int left_threshold = 10 ;
//int right_threshold = 10 ;
int front_threshold = 7 ;

boolean frontwall ;
boolean leftwall ;
boolean rightwall ;
boolean first_turn ;
boolean rightWallFollow ;
boolean leftWallFollow ;





int en1 = 2 ;
int en2 = 3 ;

int en3 = 4 ;
int en4 = 5 ;

int enA = 10 ;
int enB = 11 ;

int baseSpeed = 70 ;

int RMS ;
int LMS ;

int LED = 13 ;
int led1 = 8 ;
int led2 = 9 ;

void ReadSensors();
void walls();
void pid_start();
void PID(bool left);
void setDirection(int dir);

NewPing sonarLeft(TRIGGER_PINL, ECHO_PINL, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarRight(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE);
NewPing sonarFront(TRIGGER_PINF, ECHO_PINF, MAX_DISTANCE);

unsigned int pingSpeed = 30; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.


float oldLeftSensor, oldRightSensor, leftSensor, rightSensor, frontSensor, oldFrontSensor, lSensor, rSensor, fSensor;

//int TestNUM = 1  ;



void setup() {

  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.


  for (int i = 2; i <= 13; i++) //For Motor Shield
    pinMode(i, OUTPUT);



  first_turn = false ;
  rightWallFollow = false ;
  leftWallFollow = false ;
 

}

void loop() {


  //========================================START========================================//


  ReadSensors();

  walls();


  if ( first_turn == false ) {

    pid_start();

  }
  else if (leftWallFollow == true ) {

    PID(true) ;

  }
  else if (rightWallFollow == true ) {
    PID(false) ;
  }


  if (leftwall == true && rightwall == false && frontwall == true ) {

    // turnright();
    PID(false) ;

    if ( first_turn == false ) {

      //      right_threshold = right_threshold - offset ;
      //      left_threshold = left_threshold + offset ;


      first_turn = true ;
      rightWallFollow = true ;
      
      digitalWrite(led2 , LOW );
      digitalWrite(led1 ,HIGH );
    }
  }
   if (leftwall == false && rightwall == true && frontwall == true ) {

    //  turnleft();
    PID(true) ;

    if ( first_turn == false ) {

      //      left_threshold = left_threshold - offset ;
      //      right_threshold = right_threshold + offset ;

      first_turn = true ;
      leftWallFollow = true ;
      digitalWrite(LED , HIGH);
       
    }
  }
   if ( leftSensor == 0 || leftSensor > 100 && rightSensor == 0 || rightSensor > 100 && frontSensor == 0 || frontSensor > 100 ) {

    setDirection(STOP);
  }



  // read sensors & print the result to the serial monitor //


  Serial.print(" Left : ");
  Serial.print(leftSensor);
  Serial.print(" cm ");
  Serial.print(" Right : ");
  Serial.print(rightSensor);
  Serial.print(" cm ");
  Serial.print(" Front : ");
  Serial.print(frontSensor);
  Serial.println(" cm ");

  //measure error & print the result to the serial monitor


  Serial.print("error=");
  Serial.println(totalError);


}








//--------------------------------- direction control ---------------------------------//

void setDirection(int dir) {

  if ( dir == FORWARD ) {
    digitalWrite(en1, LOW);   // Left wheel forward
    digitalWrite(en2, HIGH);
    digitalWrite(en3, LOW);  // Right wheel forward
    digitalWrite(en4, HIGH);
  }
  else if ( dir == LEFT ) {
    digitalWrite(en1, HIGH);   // Left wheel forward
    digitalWrite(en2, LOW );
    digitalWrite(en3, LOW );  // Right wheel forward
    digitalWrite(en4, HIGH);
  }
  else if ( dir == RIGHT ) {
    digitalWrite(en1, LOW);   // Left wheel forward
    digitalWrite(en2, HIGH);
    digitalWrite(en3, HIGH);  // Right wheel forward
    digitalWrite(en4, LOW);
  }
  else if ( dir == STOP ) {
    digitalWrite(en1, HIGH);   // Left wheel forward
    digitalWrite(en2, HIGH );
    digitalWrite(en3, HIGH );  // Right wheel forward
    digitalWrite(en4, HIGH);
  }
  else if ( dir == BACKWARD ) {
    digitalWrite(en1, HIGH );   // Left wheel forward
    digitalWrite(en2, LOW );
    digitalWrite(en3, HIGH );  // Right wheel forward
    digitalWrite(en4, LOW );
  }
}
//---------------------------------------------------------------------------//


//--------------------------------- Sensors ---------------------------------//

void ReadSensors() {

  //leftSensor = sonarLeft.ping_median(TestNUM);     //accurate but slow
  //rightSensor = sonarRight.ping_median(TestNUM);     //accurate but slow
  //frontSensor = sonarFront.ping_median(TestNUM);     //accurate but slow

  //leftSensor = sonarLeft.convert_cm(leftSensor);
  //rightSensor = sonarRight.convert_cm(rightSensor);
  //frontSensor = sonarFront.convert_cm(frontSensor);

  lSensor = sonarLeft.ping_cm(); //ping in cm
  rSensor = sonarRight.ping_cm();
  fSensor = sonarFront.ping_cm();


  leftSensor = (lSensor + oldLeftSensor) / 2; //average distance between old & new readings to make the change smoother
  rightSensor = (rSensor + oldRightSensor) / 2;
  frontSensor = (fSensor + oldFrontSensor) / 2;


  oldLeftSensor = leftSensor; // save old readings for movment
  oldRightSensor = rightSensor;
  oldFrontSensor = frontSensor;

}

//---------------------------------------------------------------------------//


//--------------------------------- control ---------------------------------//

void pid_start() {

  float errorP = leftSensor - rightSensor ;
  float errorD = errorP - oldErrorP;
  float errorI = (2.0 / 3.0) * errorI + errorP ;

  totalError = P * errorP + D * errorD + I * errorI ;
  
  oldErrorP = errorP ;

  RMS = baseSpeed + totalError ;
  LMS = baseSpeed - totalError ;

  if (RMS < 0) {

    RMS = map(RMS , 0 , -255, 0, 255);

    analogWrite(enA , RMS);
    analogWrite(enB , LMS);

    setDirection(RIGHT);

  }
  else if (LMS < 0) {
    LMS = map(LMS , 0 , -255, 0, 255);


    analogWrite(enA , RMS);
    analogWrite(enB , LMS);

    setDirection(LEFT);
  }
  else {

    analogWrite(enA , RMS);
    analogWrite(enB , LMS);

    setDirection(FORWARD);
  }

}


//----------------------------- wall follow  control -------------------------------//

void PID( boolean left ) {

  if (left == true ) {

    float errorP = leftSensor - rightSensor - offset ;
    float errorD = errorP - oldErrorP;
    float errorI = (2.0 / 3) * errorI + errorP ;


    totalError = P * errorP + D * errorD + I * errorI ;

    oldErrorP = errorP ;


    RMS = baseSpeed + totalError ;
    LMS = baseSpeed - totalError ;

    //  if(RMS < -255) RMS = -255; if(RMS > 255)RMS = 255 ;
    //  if(LMS < -255) LMS = -255;  if(LMS > 255)LMS = 255 ;


    if (RMS < 0) {

      RMS = map(RMS , 0 , -255, 0, 255);

      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(RIGHT);

    }
    else if (LMS < 0) {
      LMS = map(LMS , 0 , -255, 0, 255);


      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(LEFT);
    }
    else {

      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(FORWARD);
    }

  }
  else {

    float errorP = leftSensor - rightSensor + offset ;
    float errorD = errorP - oldErrorP;
    float errorI = (2.0 / 3) * errorI + errorP ;

    totalError = P * errorP + D * errorD + I * errorI ;

    oldErrorP = errorP ;


    RMS = baseSpeed + totalError ;
    LMS = baseSpeed - totalError ;

    //  if(RMS < -255) RMS = -255; if(RMS > 255)RMS = 255 ;
    //  if(LMS < -255) LMS = -255;  if(LMS > 255)LMS = 255 ;


    if (RMS < 0) {

      RMS = map(RMS , 0 , -255, 0, 255);

      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(RIGHT);

    }
    else if (LMS < 0) {
      LMS = map(LMS , 0 , -255, 0, 255);


      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(LEFT);
    }
    else {

      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(FORWARD);
    }

  }

}

//--------------------------- wall detection --------------------------------//

void walls() {


  if ( leftSensor < wall_threshold ) {
    leftwall = true ;
  }
  else {
    leftwall = false ;
  }


  if ( rightSensor < wall_threshold ) {
    rightwall = true ;
  }
  else {
    rightwall = false ;


  } if ( frontSensor < front_threshold ) {
    frontwall = true ;
  }
  else {
    frontwall = false ;
  }

}



//---------------------------------------------------------------------------//

void turnright() {


  LMS = baseSpeed ;

  RMS = LMS * rightSensor / ( rightSensor + 11 ) ;


}

//---------------------------------------------------------------------------//

void turnleft() {


  RMS = baseSpeed ;

  LMS = RMS * leftSensor / ( leftSensor + 11 ) ;

}


//---------------------------------------------------------------------------//




*/