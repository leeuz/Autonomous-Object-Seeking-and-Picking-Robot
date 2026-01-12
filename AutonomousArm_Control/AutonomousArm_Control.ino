#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL6180X.h>

// PIN DEFINITIONS
#define BASE_PIN 6
#define SHOULDER_PIN 5
#define ELBOW_PIN 3
#define GRIPPER_PIN 9
#define FSR1_PIN A0
#define FSR2_PIN A1

// TUNING ANGLES
// change these values based on physical testing.
const int BASE_START_ANGLE = 0;
const int BASE_DROP_ANGLE  = 180;

const int SHOULDER_IDLE    = 90;  // upright
const int SHOULDER_PICKUP  = 45;  // lean forward

const int ELBOW_IDLE       = 90;  // straight
const int ELBOW_PICKUP     = 130; // bent down to floor

const int GRIPPER_OPEN     = 0;   // open state
const int GRIPPER_CLOSE    = 160; // fully closed state

// SENSOR THRESHOLDS
const int DISTANCE_THRESHOLD = 100; // mm (detect object within 10cm)
const int FSR_THRESHOLD      = 200; // react when ~20% is pressed. can adjust sensitivity.

// OBJECTS
Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo gripperServo;
Adafruit_VL6180X vl = Adafruit_VL6180X();

void setup() {
  Serial.begin(9600);

  // Attach Servos
  baseServo.attach(BASE_PIN);
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);
  gripperServo.attach(GRIPPER_PIN);

  // Initialize ToF Sensor
  if (!vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("Sensor found!");

  // Set Initial Positions
  resetArm();
  delay(2000); 
}

void loop() {
  // Scan for object
  bool objectFound = scanForObject();

  if (objectFound) {
    Serial.println("Object Detected! Initiating Pickup...");
    
    // Move Arm to Position
    approachObject();
    
    // Grip Object
    bool grabbed = grabObject();

    if (grabbed) {
      Serial.println("Object Grabbed securely.");
      
      // Drop Object
      dropObject();
    } else {
      Serial.println("Failed to grab / No pressure detected.");
      // Open gripper and reset if we missed
      gripperServo.write(GRIPPER_OPEN);
      resetArm();
    }
  }
  
  delay(1000);
}

// FUNCTIONS

// Rotates base until ToF detects an object
bool scanForObject() {
  Serial.println("Scanning...");
  
  // Sweep from Start Angle to Drop Angle (or less, depending on scan range)
  for (int pos = BASE_START_ANGLE; pos <= 160; pos++) { 
    baseServo.write(pos);
    delay(50); 

    uint8_t range = vl.readRange();
    uint8_t status = vl.readRangeStatus();

    if (status == VL6180X_ERROR_NONE && range < DISTANCE_THRESHOLD && range > 10) {
      Serial.print("Object found at distance: "); 
      Serial.println(range);
      return true; 
    }
  }
  
  Serial.println("No object found in scan range.");
  resetArm(); 
  return false;
}

void approachObject() {
  Serial.println("Moving Shoulder...");
  slowMove(shoulderServo, SHOULDER_IDLE, SHOULDER_PICKUP);
  delay(500);

  Serial.println("Moving Elbow...");
  slowMove(elbowServo, ELBOW_IDLE, ELBOW_PICKUP);
  delay(500);
}

bool grabObject() {
  Serial.println("Closing Gripper...");
  
  // Close gripper incrementally until FSR detects force
  for (int pos = GRIPPER_OPEN; pos <= GRIPPER_CLOSE; pos++) {
    gripperServo.write(pos);
    delay(20); 
    
    int fsr1 = analogRead(FSR1_PIN);
    int fsr2 = analogRead(FSR2_PIN);

    if (fsr1 > FSR_THRESHOLD || fsr2 > FSR_THRESHOLD) {
      Serial.print("Pressure Detected! FSR1: "); Serial.print(fsr1);
      Serial.print(" FSR2: "); Serial.println(fsr2);
      return true; // Object grabbed
    }
  }
  
  // check again if we reached full close without triggering threshold
  // sometimes the object is small and we just need to hold it tight
  int finalFSR1 = analogRead(FSR1_PIN);
  int finalFSR2 = analogRead(FSR2_PIN);
  if (finalFSR1 > 50 || finalFSR2 > 50) return true; // Lower threshold for "holding" check
  
  return false;
}

void dropObject() {
  Serial.println("Lifting...");
  // Lift up first to avoid hitting the floor while turning
  slowMove(elbowServo, ELBOW_PICKUP, ELBOW_IDLE);
  slowMove(shoulderServo, SHOULDER_PICKUP, SHOULDER_IDLE);
  delay(500);

  Serial.println("Rotating to Basket...");
  // Rotate base to 180 (The Drop Zone)
  baseServo.write(BASE_DROP_ANGLE); 
  delay(1500); 

  Serial.println("Dropping...");
  gripperServo.write(GRIPPER_OPEN);
  delay(1000);

  Serial.println("Resetting...");
  resetArm();
}

void resetArm() {
  // Move to safe idle positions
  gripperServo.write(GRIPPER_OPEN);
  delay(200);
  elbowServo.write(ELBOW_IDLE);
  delay(200);
  shoulderServo.write(SHOULDER_IDLE);
  delay(200);
  baseServo.write(BASE_START_ANGLE);
  delay(1000);
}

// Helper function for smooth movement
void slowMove(Servo &s, int start, int end) {
  if (start < end) {
    for (int i = start; i <= end; i++) {
      s.write(i);
      delay(15);
    }
  } else {
    for (int i = start; i >= end; i--) {
      s.write(i);
      delay(15);
    }
  }
}
