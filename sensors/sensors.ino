#define STEERING_SERVO_PIN 10
#define SERVO_STRAIGHT 95
#define TURN_RADIUS_RIGHT 15
#define TURN_RADIUS_LEFT 20

#define RIGHT_BUMPER_PIN 7
#define LEFT_BUMPER_PIN 6
#define ONE_SECOND 1000
#define THREE_SECONDS 3000
#define MOVE_SPEED 165
#define MAX_TIMER_COUNT 5000

#include <Servo.h> 
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motor = AFMS.getMotor(1);

Servo steeringServo;
int rightBumperState = 0;
int leftBumperState = 0;
int timeCounter = 0;

String command; // Used to process commands from RaspberryPi

void setup() {
  Serial.begin(9600);
  
  steeringServo.attach(STEERING_SERVO_PIN);

  AFMS.begin();  // create with the default frequency 1.6KHz

  motor->setSpeed(0);
  steeringServo.write(90);

  pinMode(RIGHT_BUMPER_PIN, INPUT);
  pinMode(LEFT_BUMPER_PIN, INPUT);

  // if analog input pin 0 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  randomSeed(analogRead(0));    
}

void loop() {
  if(Serial.available() > 0) {
    command = Serial.readStringUntil('\n');
    processCommand();
  }

//  rightBumperState = digitalRead(RIGHT_BUMPER_PIN);
//  leftBumperState = digitalRead(LEFT_BUMPER_PIN);
//
//  if(rightBumperPressed() || leftBumperPressed()) {
//    timeCounter++;     
//  } else {
//    timeCounter = 0;
//  }
//
//  if(timeCounter == MAX_TIMER_COUNT) {
//    evasiveManeuvers();
//    timeCounter = 0;
//  }
}

void processCommand() {
//  Serial.println(command);
  if(command == "MoveForward") { moveForward(); } 
  else if(command == "MoveBackward") { moveBackward(); }
  else if(command == "MoveStop") { stopMoving(); }

  else if(command == "TurnLeft") { turnLeft(); }
  else if(command == "TurnRight") { turnRight(); }
  else if(command == "TurnStraight") { turnStraight(); }
}


// #################################
// Ben's Functions
// #################################
void ben_bothLeftRight() {
  Serial.println("ben_bothLeftRight");
  stopMoving();
  moveBackward();
  turnRight();
   delay(ONE_SECOND);
  turnLeft();
   delay(ONE_SECOND);
  turnStraight();
  moveForward();
}
// #################################


// ####################################
// Environment Checks
// ####################################
void evasiveManeuvers() {
  if (bothBumpersPressed()) {
    // bothBumpersHit();
    ben_bothLeftRight();
  } else if(rightBumperPressed()) {
    rightBumperHit();
  } else if (leftBumperPressed()) {
    leftBumperHit();
  } 
}

// ####################################
// Environment Responses
// ####################################
void rightBumperHit() {
  Serial.println("rightBumperHit");
  stopMoving();
  turnRight();
  moveBackward();
  randomDelay();
  stopMoving();
  turnStraight();
  moveForward();   
}

void leftBumperHit() {
  Serial.println("leftBumperHit");
  stopMoving();
  turnLeft();
  moveBackward();
  randomDelay();
  stopMoving();
  turnStraight();
  moveForward();       
}

void bothBumpersHit() {
  Serial.println("bothBumpersHit");
  stopMoving();
  randomTurn();
  moveBackward();
  randomDelay();
  stopMoving();
  turnStraight();
  moveForward();
}

// ####################################
// Primative methods - Things it can do
// ####################################
void moveForward() {
//  Serial.println("Moving Forward");
  motor->setSpeed(MOVE_SPEED);
  motor->run(FORWARD);
}

void moveBackward() {
//  Serial.println("Moving Backward");
  motor->setSpeed(MOVE_SPEED);
  motor->run(BACKWARD);
}

void stopMoving() {
//  Serial.println("Stopping");
  motor->setSpeed(0);
  turnStraight();
}

void turnRight() {
//  Serial.println("Turn Right");
  steeringServo.write(SERVO_STRAIGHT + TURN_RADIUS_RIGHT);
}

void turnLeft() {
//  Serial.println("Turn Left");
  steeringServo.write(SERVO_STRAIGHT - TURN_RADIUS_LEFT);
}

void turnStraight() {
//  Serial.println("Turn Straight");
  steeringServo.write(SERVO_STRAIGHT);
}

// ####################################
// Primative methods - Things it can sense
// ####################################
bool rightBumperPressed() {
  return rightBumperState == LOW;
}

bool leftBumperPressed() {
  return leftBumperState == LOW;
}

bool bothBumpersPressed() {
  return rightBumperPressed() && leftBumperPressed();
}

// ####################################
// Helper methods
// ####################################
void randomTurn() {
  if(random(1, 3) == 1) {
    turnRight();
  } else {
    turnLeft();
  }
}

void randomDelay() {
  int randomTime = random(ONE_SECOND, THREE_SECONDS);
  delay(randomTime);
}

// ###########################################
// Diagnostic functions
// ###########################################

void systemTest() {
  stopMoving();
  delay(ONE_SECOND);
  moveForward();
  delay(ONE_SECOND);
  moveBackward();
  delay(ONE_SECOND);
  stopMoving();
  delay(500);
  turnRight();
  delay(ONE_SECOND);
  turnLeft();
  delay(ONE_SECOND);
  stopMoving();
}
