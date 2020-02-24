#include <Arduino.h>
#include <NewPing.h>     // Arduino Library for working with ultrasonic sensors.
#include "drive.h"
//#include "StackArray.h"

#define TRIGGER_PINL  6  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINL     7  // Arduino pin tied to echo pin on ping sensor.
#define TRIGGER_PINF  4  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINF     5  // Arduino pin tied to echo pin on ping sensor.
#define TRIGGER_PINR  2  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINR     3  // Arduino pin tied to echo pin on ping sensor.
#define RMOTOR_1      9  // Right motor, pin 1
#define RMOTOR_2      10 // Right motor, pin 2
#define LMOTOR_1      11 // Left motor, pin 1
#define LMOTOR_2      12 // Left motor, pin 2
#define TURN_TIME     700 // Time used for guessing turn time for 90 degrees.
#define DRIVE_TIME    600 // Time used for guessing drive time for 1 foot.
#define WAIT_TIME     500 // Time spent between drive steps

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
  drive(drivePinsL, 0);
  drive(drivePinsR, 200);
  delay(TURN_TIME);
  brake(drivePinsL[0], drivePinsL[1]);
  brake(drivePinsR[0], drivePinsR[1]);
  delay(WAIT_TIME);
}

void turnRight(int * drivePinsL, int * drivePinsR) {
  drive(drivePinsL, 200);
  drive(drivePinsR, 0);
  delay(TURN_TIME);
  brake(drivePinsL[0], drivePinsL[1]);
  brake(drivePinsR[0], drivePinsR[1]);
  delay(WAIT_TIME);
}

void guessDrive(int * drivePinsL, int * drivePinsR) {
  both(drivePinsL, drivePinsR, 200);
  delay(DRIVE_TIME);
  brake(drivePinsL[0], drivePinsL[1]);
  brake(drivePinsR[0], drivePinsR[1]);
  delay(WAIT_TIME);
}

void guessDrive(int * drivePinsL, int * drivePinsR, int driveTime) {
  both(drivePinsL, drivePinsR, 200);
  delay(driveTime);
  brake(drivePinsL[0], drivePinsL[1]);
  brake(drivePinsR[0], drivePinsR[1]);
  delay(WAIT_TIME);
}

void forwardOneFoot(int * drivePinsL, int * drivePinsR, int * sonarReadings) {
  int frontReading = sonarReadings[1];
  int currReading = sonarReadings[1];
  drive(drivePinsL, -200);
  drive(drivePinsR, 200);
  while (frontReading - currReading < 0.9 * 12) {
    currReading = sonarF.ping_in();
    delay(300);
  }
  brake(drivePinsL[0], drivePinsL[1]);
  brake(drivePinsR[0], drivePinsR[1]);
  delay(WAIT_TIME);
}

void driveTillTwoInches(int * drivePinsL, int * drivePinsR, int * sonarReadings) {
  while(true) {
    sonarReadings[1] = sonarF.ping_in();
    if (sonarReadings[1] > 10) {
      both(drivePinsL, drivePinsR, 200);
    } else if (sonarReadings[1] > 5) {
      both(drivePinsL, drivePinsR, 125);
    } else if (sonarReadings[1] > 3) {
      both(drivePinsL, drivePinsR, 100);
    } else if (sonarReadings[1] <= 2) {
      brake(drivePinsL[0], drivePinsL[1]);
      brake(drivePinsR[0], drivePinsR[1]);
      break;
    }
    delay(100);
  }
  delay(WAIT_TIME);
}

void driveTillFourteenInches(int * drivePinsL, int * drivePinsR, int * sonarReadings) {
  while(true) {
    sonarReadings[1] = sonarF.ping_in();
    if (sonarReadings[1] > 22) {
      both(drivePinsL, drivePinsR, 200);
    } else if (sonarReadings[1] > 17) {
      both(drivePinsL, drivePinsR, 125);
    } else if (sonarReadings[1] > 15) {
      both(drivePinsL, drivePinsR, 100);
    } else if (sonarReadings[1] <= 14) {
      brake(drivePinsL[0], drivePinsL[1]);
      brake(drivePinsR[0], drivePinsR[1]);
      break;
    }
    delay(100);
  }
  delay(WAIT_TIME);
}

void reverseOneFoot(int * drivePinsL, int * drivePinsR, int * sonarReadings) {
  int frontReading = sonarReadings[1];
  int currReading = sonarReadings[1];
  drive(drivePinsL, -200);
  drive(drivePinsR, -200);
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
}

void loop() {
  Serial.print("A");
  delay(100);
  driveTillTwoInches(drivePinsL, drivePinsR, sonarReadings);

  Serial.print("B");
  delay(100);
  turnLeft(drivePinsL, drivePinsR);
  delay(100);
  driveTillTwoInches(drivePinsL, drivePinsR, sonarReadings);

  Serial.print("C");
  delay(100);
  turnLeft(drivePinsL, drivePinsR);
  delay(100);
  driveTillTwoInches(drivePinsL, drivePinsR, sonarReadings);

  Serial.print("D");
  delay(100);
  turnRight(drivePinsL, drivePinsR);
  delay(100);
  driveTillTwoInches(drivePinsL, drivePinsR, sonarReadings);

  Serial.print("E");
  delay(100);
  turnRight(drivePinsL, drivePinsR);
  delay(100);
  driveTillTwoInches(drivePinsL, drivePinsR, sonarReadings);
  delay(100);
  driveTillTwoInches(drivePinsL, drivePinsR, sonarReadings);

  Serial.print("F");
  delay(100);
  turnRight(drivePinsL, drivePinsR);
  delay(100);
  driveTillTwoInches(drivePinsL, drivePinsR, sonarReadings);
  delay(100);
  driveTillTwoInches(drivePinsL, drivePinsR, sonarReadings);

  Serial.print("G");
  delay(100);
  turnLeft(drivePinsL, drivePinsR);
  delay(100);
  driveTillTwoInches(drivePinsL, drivePinsR, sonarReadings);

  Serial.print("H");
  delay(100);
  turnLeft(drivePinsL, drivePinsR);
  delay(100);
  driveTillTwoInches(drivePinsL, drivePinsR, sonarReadings);
  delay(100);
  driveTillTwoInches(drivePinsL, drivePinsR, sonarReadings);

  Serial.print("I");
  delay(100);
  turnRight(drivePinsL, drivePinsR);
  delay(100);
  driveTillTwoInches(drivePinsL, drivePinsR, sonarReadings);

  Serial.print("J");
  delay(100);
  turnRight(drivePinsL, drivePinsR);
  delay(100);
  driveTillFourteenInches(drivePinsL, drivePinsR, sonarReadings);

  Serial.print("K");
  delay(100);
  turnLeft(drivePinsL, drivePinsR);
  delay(100);
  driveTillFourteenInches(drivePinsL, drivePinsR, sonarReadings);

  Serial.print("L");
  delay(100);
  turnLeft(drivePinsL, drivePinsR);
  delay(100);
  driveTillTwoInches(drivePinsL, drivePinsR, sonarReadings);

  Serial.print("M");
  delay(100);
  turnRight(drivePinsL, drivePinsR);
  delay(100);
  guessDrive(drivePinsL, drivePinsR);
  delay(500000);
}