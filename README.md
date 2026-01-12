# Autonomous Object-Seeking and Picking Robot Arm

**Name:** Yuji Lee  
**Class:** Engineering Design and Development (EDD)  
**Status:** Prototype Complete (Phase 1)

![Final Robot Arm Assembly](PLACE_YOUR_FINAL_ROBOT_PHOTO_HERE.jpg)

## Project Overview

### Description
The Autonomous Object-Seeking and Picking Robot is a self-navigating system designed to locate objects, avoid obstacles, and pick up items using a compact robotic arm. While standard robot vacuums are excellent for debris, they cannot handle larger 3D objects (toys, socks, small tools) scattered on floors. This project bridges that gap by integrating sensor-based detection with mechanical actuation.

### Problem Statement
Students, home users, and workplaces often have small objects scattered on floors or surfaces, making cleanup time-consuming. Existing automated systems (robot vacuums) cannot detect or collect these items. There is a need for an autonomous robot that can detect objects, pick them up, and store them without human intervention.

### Target Audience
* **Home Users:** Hands-free collection of small items.
* **Labs/Offices:** Recovery of misplaced small tools or parts.

---

## Technical Specifications & Part Selection

### Bill of Materials (BOM)

| Component | Model | Function |
| :--- | :--- | :--- |
| **Microcontroller** | Arduino Nano | Central processing unit for sensor logic and servo control. |
| **Primary Actuators** | MG996R High Torque Servo (x3) | Controls Base, Shoulder, and Elbow joints. |
| **Gripper Actuator** | MG90S Micro Servo | Actuates the claw mechanism. |
| **Object Sensor** | Adafruit VL6180X (ToF) | High-precision distance measurement for object detection. |
| **Contact Sensor** | Force Sensitive Resistor (FSR) | Feedback loop to confirm object has been grabbed. |
| **Power Source** | 4x AA Battery Pack (6V) | External power specifically for high-torque servos. |

### Torque Calculations & Physics
Before manufacturing, I performed static torque calculations to ensure the MG996R servos (Stall Torque: ~9.4 kg-cm) could lift the arm assembly.

**Formula Used:** Torque = Force x Distance

**The Calculation:**
* **Shoulder Segment:** 19cm length, ~28.5g mass.
* **Elbow Segment:** 20cm length, ~30g mass.
* **Shoulder Joint Requirement:** Supports shoulder segment + elbow servo + forearm + gripper.
    * Calculated Torque: ~3.16 kg-cm.
    * Servo Capacity: 9.4 kg-cm.
    * **Safety Factor:** ~2.97 (Safe).

![Torque Calculation Notes](PLACE_IMAGE_OF_YOUR_CALCULATION_NOTES_HERE.jpg)

---

## Design & Modeling (CAD)

The mechanical structure was designed in Autodesk Fusion 360.

### 1. The Base Design
The base required a custom housing to stabilize the heavy MG996R servo while accommodating the Arduino Nano and wiring.
* **Design Choice:** I included a specific channel (15mm x 10mm) for the USB cable to allow for programming updates without disassembling the robot.
* **Iterative Design:** The base size was defined as a variable in Fusion 360, allowing me to resize the housing easily when I realized the perfboard and wiring required more volume than initially calculated.

![Fusion 360 Base Design](PLACE_SCREENSHOT_OF_BASE_CAD_HERE.png)

### 2. The Gripper
I adapted an open-source parallel gripper design to fit the MG90S servo.
* **Modification:** I modified the jaw surface to house a Force Sensitive Resistor (FSR). This allows the robot to "feel" when it has successfully gripped an object, preventing the motor from crushing delicate items or burning out.

![Fusion 360 Gripper Render](PLACE_SCREENSHOT_OF_GRIPPER_CAD_HERE.png)

---

## Electronics & Wiring

### Wiring Diagram
The system uses a mixed-voltage circuit. The logic level (Arduino) runs on 5V USB, while the inductive load (Servos) runs on an external 6V source.

![Final Wiring Diagram](PLACE_IMAGE_OF_WIRING_DIAGRAM_HERE.png)

### Critical Challenge: The Common Ground
During the build, I encountered a major issue where the code uploaded successfully, but the servos refused to move.
* **Diagnosis:** The external battery pack (6V) and the Arduino (USB) were on isolated circuits. The servo control wire (Signal) had no reference point.
* **Solution:** I soldered a jumper wire connecting the Battery Negative (-) to the Arduino GND. This completed the circuit and allowed the servos to interpret the PWM signals.

---

## Software & Logic

**Repository:** [Link to Code Folder](./AutonomousArm_Control)

The robot operates on a state-machine logic loop: Scan -> Approach -> Grip -> Drop.

### Sensor Logic (Why ToF?)
Initially, I implemented three Ultrasonic Sensors (HC-SR04). However, testing revealed significant noise and interference between the sensors, leading to erratic position data.

* **Solution:** I switched to the VL6180X Time-of-Flight (ToF) sensor.
* **Benefit:** ToF measures the time it takes for photons to bounce back, providing millimeter-level accuracy at close range (0-100mm) via I2C, which was far more stable for grabbing small objects.

### Code Highlight: Smooth Movement
To prevent the robot from tipping over due to the inertia of the moving arm, I wrote a custom function slowMove() to ramp the servo speed rather than jumping instantly to a target angle.

```cpp
// Helper function for smooth movement to prevent tipping
void slowMove(Servo &s, int start, int end) {
  int speedDelay = 50; // Higher number = Slower movement
  
  if (start < end) {
    for (int i = start; i <= end; i++) {
      s.write(i);
      delay(speedDelay);
    }
  } else {
    for (int i = start; i >= end; i--) {
      s.write(i);
      delay(speedDelay);
    }
  }
}
```

---

## **Testing & Validation**

### Sensor Calibration Table (VL6180X)
I performed a static test to verify the accuracy of the ToF sensor before integrating it into the arm. 

| Real Distance (mm) | Sensor Reading (mm) | Error (%) |
| :--- | :--- | :--- |
| 20 | 20 | 0% |
| 50 | 52 | 4% |
| 100 | 102 | 2% |
| 100 | 102 | 2% |
| 200 | 188.5 | 5.75% |

* **Observation:** The sensor is highly accurate up to 100mm. Beyond 200mm, it returns "Range Error 11".
* **Action:** I programmed the logic to only initiate a "Grab" sequence if the distance is less than 100mm.

### Power Supply Verification
* **Attempt 1:** 4.5V (????)
   * **Result:** Failure. Servos stalled under load.
* **Attempt 2:** 6.0V (???)
   * **Results:** Success. The MG996R and MG90S Micro servos require a minimum of 4.8V to operate efficiently.

---

## Media

### Project Demo Video:

---

## Reflections & Future Improvements

### What Went Well
* **Torque Calculations:** The math held up; the arm moves smoothly without stalling.
* **FSR Integration:** The gripper successfully detects when an object is held, allowing for autonomous stopping.
* **Troubleshooting:** Successfully diagnosed the mixed-voltage common ground issue.

### Improvements for V2
1. **Mobile Chassis:** The original plan included a wheeled base. Due to time constraints, I focused on the arm. The next step is mounting this arm to a mobile robot vacuum chassis.
2. **Vision Processing:** The ToF sensor is excellent for distance but blind to what the object is. I would implement a camera (e.g., OpenMV) to distinguish between trash and valuables.
3. **PID Control:** Implementing PID logic would make the arm movement even smoother and faster than the current linear ramping function.

---

## Credits & Resources

* **Libraries:** Adafruit_VL6180X, Servo.h
* 3D Models: MG996R Servo model from GrabCAD
