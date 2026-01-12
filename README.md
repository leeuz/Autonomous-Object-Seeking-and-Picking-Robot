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

## Part Selection

### Sensor Selection

**Adafruit VL6180X Time-of-Flight (ToF)**
The [VL6180X ToF sensor](https://www.adafruit.com/product/3316) was selected over ultrasonic sensors (HC-SR04) because of its high precision at short ranges (0-100mm). Unlike ultrasonic sensors, which created significant noise and interference during testing, the ToF sensor uses laser technology to provide millimeter-level accuracy via I2C communication. This is critical for the "Approach" phase where the gripper must position itself perfectly around a small object.
![Adafruit VL6180X Sensor](https://cdn-shop.adafruit.com/970x728/3316-10.jpg)


**Force Sensitive Resistor (FSR)**
A strip-shaped FSR was selected to act as the tactile feedback system for the gripper. By placing this on the inner jaw of the claw, the robot can detect physical pressure. This allows the code to stop the servo motor the moment a firm grip is achieved, preventing the robot from crushing delicate objects or burning out the servo motor by stalling against a hard object.
<img 
  src="https://m.media-amazon.com/images/I/41PG4jWzTGL._SX342_SY445_QL70_FMwebp_.jpg"
  alt="Force Sensitive Resistor"
  width="600"
/>


### Actuator Selection

**MG996R High Torque Servos**
MG996R servos were selected for the Base, Shoulder, and Elbow joints. Based on my static torque calculations, the shoulder joint required approximately 3.16 kg-cm of torque to lift the arm assembly. The MG996R provides a stall torque of 9.4 kg-cm (at 4.8V), providing a safety factor of nearly 3. Metal gears were a requirement to prevent stripping under the load of the 3D-printed arm.
![MG996R Servo](https://m.media-amazon.com/images/I/71PIDs0nXnL._AC_SX679_.jpg)


**MG90S Micro Servo**
The MG90S was selected specifically for the gripper mechanism. Since this motor sits at the very end of the arm, weight was a critical constraint. The MG90S is lightweight (13.4g) yet provides enough torque (1.8 kg-cm) to actuate the parallel gripper mechanism and hold small household objects.
![MG90S Servo](https://m.media-amazon.com/images/I/812KFZH-2-L._SX522_.jpg)


### Microcontroller Selection

**Arduino Nano**
The Arduino Nano was chosen as the central controller due to its compact form factor, which fits easily inside the custom-designed 3D-printed base. It features dedicated I2C pins (A4/A5) required for the ToF sensor and sufficient PWM digital pins to control all four servos simultaneously.
![Arduino Nano](https://m.media-amazon.com/images/I/61yomoOzTpL._AC_SX300_SY300_QL70_FMwebp_.jpg)


---

## Technical Specifications

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

[Detailed Torque Calculation Notes (12/4&5)](https://docs.google.com/document/d/14iYlZt8_KT-G-6Bip5Sd_sYR2SBkpgpfMPnpm2VzzZg/edit?usp=sharing)

---

## Design & Modeling (CAD)

The mechanical structure was designed in Autodesk Fusion 360.

### 1. The Base Design
The base required a custom housing to stabilize the heavy MG996R servo while accommodating the Arduino Nano and wiring.
* **Design Choice:** I included a specific channel (15mm x 10mm) for the USB cable to allow for programming updates without disassembling the robot.
* **Iterative Design:** The base size was defined as a variable in Fusion 360, allowing me to resize the housing easily when I realized the perfboard and wiring required more volume than initially calculated.

![Fusion 360 Base Design](https://github.com/leeuz/Autonomous-Object-Seeking-and-Picking-Robot/blob/main/Base_CAD_Design.png)

### 2. The Gripper
I adapted an open-source parallel gripper design to fit the MG90S servo.
* **Modification:** I modified the jaw surface to house a Force Sensitive Resistor (FSR). This allows the robot to "feel" when it has successfully gripped an object, preventing the motor from crushing delicate items or burning out.

![Fusion 360 Gripper Render](https://github.com/leeuz/Autonomous-Object-Seeking-and-Picking-Robot/blob/main/Gripper_CAD_Design.png)

---

## Electronics & Wiring

### Wiring Diagram
The system uses a mixed-voltage circuit. The logic level (Arduino) runs on 5V USB, while the inductive load (Servos) runs on an external 6V source.

![Wiring Diagram](https://github.com/leeuz/Autonomous-Object-Seeking-and-Picking-Robot/blob/main/WiringDiagram.png)

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
* **Attempt 1:** 4.5V (DC Regulated Power Supply)
   * **Result:** Failure. Servos stalled under load.
* **Attempt 2:** 6.0V (DC Regulated Power Supply)
   * **Results:** Success. The MG996R and MG90S Micro servos require a minimum of 4.8V to operate efficiently.

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
* **3D Models:** MG996R Servo model from GrabCAD

---

## Skills Demonstrated
* **Embedded Systesm Programming:** Applied C++ knowledge to program an Arduino Nano, utilizing state-machine logic to coordinate multiple sensors (I2C and Analog) with actuator outputs.
* **Circuit Analysis & Soldering:** Diagnosed and resolved mixed-voltage circuit isolation issues (Common Ground) and successfully soldered voltage divider circuits for FSR integration using flux.
* **Physics & Static Analysis:** Calculated moment arms and torque requirements to select appropriate motors, ensuring a Safety Factor > 2.
* **Computer-Aided Design (CAD):** Demonstrated proficiency in Autodesk Fusion 360 by designing a custom servo housing with interference fits and modifying existing gripper geometries to integrate sensors.
