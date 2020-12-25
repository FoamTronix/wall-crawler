#define ONE_SECOND 1000

// Lights
// ##########################################
#define DS_PIN 11
#define STCP_PIN 12
#define SHCP_PIN 13

#define FRONT_LEFT 0
#define FRONT_RIGHT 2
#define REAR_LEFT 4
#define REAR_RIGHT 7
#define REVERSE_LEFT 6
#define REVERSE_RIGHT 9
#define SIGNAL_FRONT_LEFT 1
#define SIGNAL_REAR_LEFT 5
#define SIGNAL_FRONT_RIGHT 3 
#define SIGNAL_REAR_RIGHT 8
#define INFRARED1 10
#define INFRARED2 11
#define INFRARED3 12
#define INFRARED4 13
#define INFRARED5 14
#define INFRARED6 15
#define LIGHT_COUNT 16
#define SIGNAL_DELAY 500 // milliseconds
unsigned long int lastSignalChange;
boolean signalLeft;
boolean signalRight;
boolean lights[LIGHT_COUNT];

// Motors
// ##########################################
#include <Servo.h> 
#define MOVE_A 5        // L9110 chip :: Input A
#define MOVE_B 4        // L9110 chip :: Input B
#define STEERING_PIN 6  // Pin 6 is a PWM pin for the servo.
Servo steeringServo; 

// Ping Sensors
// ##########################################
#include <NewPing.h>
#define SONAR_NUM     4  // Number or sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

// Using only one ping for both the Trigger and Ping.  The NewPing library takes care of 
// toggling the pin's input/output state.
#define PING_SENSOR_1_PIN 7 
#define PING_SENSOR_2_PIN 8 
#define PING_SENSOR_3_PIN 9 
#define PING_SENSOR_4_PIN 10 

unsigned long pingTimer[SONAR_NUM];   // Holds the times when the next ping should happen for each sensor.
unsigned int pingRangesCm[SONAR_NUM]; // Where the ping distances are stored.
uint8_t currentSensor = 0;            // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {                                   // Sensor object array.
  NewPing(PING_SENSOR_1_PIN, PING_SENSOR_1_PIN, MAX_DISTANCE), // TriggerPin, EchoPin, MaxDistance
  NewPing(PING_SENSOR_2_PIN, PING_SENSOR_2_PIN, MAX_DISTANCE),
  NewPing(PING_SENSOR_3_PIN, PING_SENSOR_3_PIN, MAX_DISTANCE),
  NewPing(PING_SENSOR_4_PIN, PING_SENSOR_4_PIN, MAX_DISTANCE)
};

// Temperature
// ##########################################
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#define ONE_WIRE_PIN 2
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature sensors(&oneWire);
float lastTempCValue = 0.0;
float lastTempFValue = 0.0;

// LDR
// ##########################################
#define LDR_SENSOR_PIN A0
int lastLDRValue;

// Commands from serial
// ##########################################
String command; // Used to process commands from RaspberryPi

void setup() {
  Serial.begin(9600);
  initLights();
  initMotors();
  initPingSensors();
  initTemperatureSensor();
  initLDRSensor();
  // performSystemTests();
}

void loop() {
  if(Serial.available() > 0) {
    command = Serial.readStringUntil('\n');
    processCommand();
  }
  signalCheck();
  readPingSensors();
}

void processCommand() {
  if(command == "LightsOn") { lightsOn(); } 
  else if(command == "LightsOff") { lightsOff(); }

  else if(command == "InfraredOn") { infraredOn(); }
  else if(command == "InfraredOff") { infraredOff(); }

  else if(command == "MoveForward") { moveForward(); }
  else if(command == "MoveBackward") { moveBackward(); }
  else if(command == "MoveStop") { stopMoving(); }

  else if(command == "TurnLeft") { turnLeft(); }
  else if(command == "TurnRight") { turnRight(); }
  else if(command == "TurnCenter") { turnCenter(); }

  else if(command == "Sensors") { sensorReadings(); }
}

void performSystemTests() {  
  testLights();
  testMotors();
}

void sensorReadings() {
  fetchTemperature();
  fetchLDRValue();
  String data = "{";
  data += "\"ping1\":" + String(pingRangesCm[0]) + ",";
  data += "\"ping2\":" + String(pingRangesCm[1]) + ",";
  data += "\"ping3\":" + String(pingRangesCm[2]) + ",";
  data += "\"ping4\":" + String(pingRangesCm[3]) + ",";
  data += "\"tempC\":" + String(lastTempCValue, 3) + ",";
  data += "\"tempF\":" + String(lastTempFValue, 3) + ",";
  data += "\"light\":" + String(lastLDRValue);
  data += "}";
  Serial.println(data);

//  Serial.print(pingRangesCm[0]);
//  Serial.print(" :: ");
//  Serial.println(String(pingRangesCm[0], 3));
}

//##########################################
// Lights
//##########################################
void initLights() {
  lastSignalChange = 0;
  signalLeft = false;
  signalRight = false;

  pinMode(DS_PIN, OUTPUT);
  pinMode(STCP_PIN, OUTPUT);
  pinMode(SHCP_PIN, OUTPUT);

  allLightsOff();
}

void testLights() {
  allLightsOff();
  delay(ONE_SECOND);
  lightsOn();
  delay(ONE_SECOND);
  lightsOff();
  delay(ONE_SECOND);
  reverseLightsOn();
  delay(ONE_SECOND);
  reverseLightsOff();
  delay(ONE_SECOND);
  leftSignalLightOn(); 
  delay(ONE_SECOND); 
  rightSignalLightOn();
  delay(ONE_SECOND);
}

void allLightsOn() {
  lights[FRONT_LEFT] = HIGH;
  lights[FRONT_RIGHT] = HIGH;
  lights[REAR_LEFT] = HIGH;
  lights[REAR_RIGHT] = HIGH;
  lights[REVERSE_LEFT] = HIGH;
  lights[REVERSE_RIGHT] = HIGH;
  lights[SIGNAL_FRONT_LEFT] = HIGH;
  lights[SIGNAL_REAR_LEFT] = HIGH;
  lights[SIGNAL_FRONT_RIGHT] = HIGH;
  lights[SIGNAL_REAR_RIGHT] = HIGH;
  lights[INFRARED1] = LOW;
  lights[INFRARED2] = LOW;
  lights[INFRARED3] = LOW;
  lights[INFRARED4] = LOW;
  lights[INFRARED5] = LOW;
  lights[INFRARED6] = LOW;
  updateLights(); 
}

void allLightsOff() {
  lights[FRONT_LEFT] = LOW;
  lights[FRONT_RIGHT] = LOW;
  lights[REAR_LEFT] = LOW;
  lights[REAR_RIGHT] = LOW;
  lights[REVERSE_LEFT] = LOW;
  lights[REVERSE_RIGHT] = LOW;
  lights[SIGNAL_FRONT_LEFT] = LOW;
  lights[SIGNAL_REAR_LEFT] = LOW;
  lights[SIGNAL_FRONT_RIGHT] = LOW;
  lights[SIGNAL_REAR_RIGHT] = LOW;
  lights[INFRARED1] = LOW;
  lights[INFRARED2] = LOW;
  lights[INFRARED3] = LOW;
  lights[INFRARED4] = LOW;
  lights[INFRARED5] = LOW;
  lights[INFRARED6] = LOW;
  updateLights(); 
}

void lightsOn() {  
  lights[FRONT_LEFT] = HIGH;
  lights[FRONT_RIGHT] = HIGH;
  lights[REAR_LEFT] = HIGH;
  lights[REAR_RIGHT] = HIGH;   
  updateLights();  
}

void lightsOff() {
  lights[FRONT_LEFT] = LOW;
  lights[FRONT_RIGHT] = LOW;
  lights[REAR_LEFT] = LOW;
  lights[REAR_RIGHT] = LOW; 
  updateLights();  
}

void reverseLightsOn() {
  lights[REVERSE_LEFT] = HIGH; 
  lights[REVERSE_RIGHT] = HIGH;
  updateLights();  
}

void reverseLightsOff() {
  lights[REVERSE_LEFT] = LOW; 
  lights[REVERSE_RIGHT] = LOW;
  updateLights();  
}

void signalCheck() {
  if(!signalLeft && !signalRight) { return; }
  if(lastSignalChange + SIGNAL_DELAY >= millis()) { return; }
  lastSignalChange = millis();
  if(signalLeft) { toggleLeftSignalLight(); }
  if(signalRight) { toggleRightSignalLight(); }
}

void leftSignalLightOn() {
  signalLeft = true;
}

void rightSignalLightOn() {
  signalRight = true;
}

void signalLightsOff() {
  signalLeft = false;
  signalRight = false;  
  lights[SIGNAL_FRONT_LEFT] = LOW; 
  lights[SIGNAL_REAR_LEFT] = LOW;
  lights[SIGNAL_FRONT_RIGHT] = LOW; 
  lights[SIGNAL_REAR_RIGHT] = LOW;  
  updateLights();    
}

void toggleLeftSignalLight() {
  lights[SIGNAL_FRONT_LEFT] = !lights[SIGNAL_FRONT_LEFT]; 
  lights[SIGNAL_REAR_LEFT] = !lights[SIGNAL_REAR_LEFT]; 
  updateLights();  
}

void toggleRightSignalLight() {
  lights[SIGNAL_FRONT_RIGHT] = !lights[SIGNAL_FRONT_RIGHT]; 
  lights[SIGNAL_REAR_RIGHT] = !lights[SIGNAL_REAR_RIGHT];   
  updateLights();  
}

void infraredOn() {
  lights[INFRARED1] = HIGH; 
  lights[INFRARED2] = HIGH;
  lights[INFRARED3] = HIGH; 
  lights[INFRARED4] = HIGH;    
  lights[INFRARED5] = HIGH; 
  lights[INFRARED6] = HIGH;     
  updateLights(); 
}

void infraredOff() {
  lights[INFRARED1] = LOW; 
  lights[INFRARED2] = LOW;
  lights[INFRARED3] = LOW; 
  lights[INFRARED4] = LOW;    
  lights[INFRARED5] = LOW; 
  lights[INFRARED6] = LOW;   
  updateLights(); 
}

void updateLights() {
  ////Pin connected to ST_CP of 74HC595
  //int latchPin = 8;
  ////Pin connected to SH_CP of 74HC595
  //int clockPin = 12;
  //////Pin connected to DS of 74HC595
  //int dataPin = 11;
  //
  ////  digitalWrite(STCP_PIN, LOW);
  ////  shiftOut(dataPin, clockPin, LSBFIRST, leds);
  ////  digitalWrite(STCP_PIN, HIGH);

  digitalWrite(STCP_PIN, LOW);
  for (int i = LIGHT_COUNT-1; i >= 0; i--) {
    digitalWrite(SHCP_PIN, LOW);
    digitalWrite(DS_PIN, lights[i]);
    digitalWrite(SHCP_PIN, HIGH);
  }
  digitalWrite(STCP_PIN, HIGH);   
}
//##########################################
//##########################################

//##########################################
// Motors
//##########################################
void initMotors() {
  steeringServo.attach(STEERING_PIN); 
  pinMode(MOVE_A, OUTPUT);
  pinMode(MOVE_B, OUTPUT);
}

void testMotors() {
  stopMoving();  
  delay(ONE_SECOND);
  moveForward();
  delay(ONE_SECOND);
  moveBackward();
  delay(ONE_SECOND);
  turnLeft();
  delay(ONE_SECOND);
  turnRight();
  delay(ONE_SECOND);
  turnCenter();
  stopMoving();  
}

void moveForward() {
  reverseLightsOff();
  digitalWrite(MOVE_A, HIGH);
  digitalWrite(MOVE_B, LOW);
}

void moveBackward() {  
  reverseLightsOn();
  digitalWrite(MOVE_A, LOW);
  digitalWrite(MOVE_B, HIGH);
}

void turnLeft() {
  signalLightsOff();
  leftSignalLightOn();
  steeringServo.write(180);
}

void turnRight() {
  signalLightsOff();
  rightSignalLightOn();
  steeringServo.write(0);
}

void turnCenter() {
  signalLightsOff();
  steeringServo.write(90);
}

void stopMoving() {
  reverseLightsOff();
  digitalWrite(MOVE_A, LOW);
  digitalWrite(MOVE_B, LOW);  
}
//##########################################
//##########################################

//##########################################
// Ping sensor
//##########################################
void initPingSensors() {
  pingTimer[0] = millis() + 75;                      // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) {          // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  }     
}

void readPingSensors() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {       // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {               // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.

//      if (i == 0 && currentSensor == SONAR_NUM - 1) {
//        oneSensorCycle();      
//      }
      
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      pingRangesCm[currentSensor] = 0;            // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  } 
}

void oneSensorCycle() { // Do something with the results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(pingRangesCm[i]);
    Serial.print("cm ");
  }
  Serial.println();
}

void echoCheck() { 
  if (sonar[currentSensor].check_timer()) { // If ping received, set the sensor distance to array.
    pingRangesCm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
  }
}
//##########################################
//##########################################

//##########################################
// Temperature Sensor
//##########################################
void initTemperatureSensor() {
  sensors.begin();  
}

void fetchTemperature() {
  sensors.requestTemperatures();  
  lastTempCValue = sensors.getTempCByIndex(0);
  lastTempFValue = sensors.getTempFByIndex(0);
}
//##########################################
//##########################################

//##########################################
// LDR (light dependent resistor) Sensor
//##########################################
void initLDRSensor() {
  lastLDRValue = 0;
}

void fetchLDRValue() {
  lastLDRValue = analogRead(LDR_SENSOR_PIN);  
}
//##########################################
//##########################################
