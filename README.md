# IEEE SoutheastCon 2026 Hardware Competition

![Robot](https://github.com/user-attachments/assets/64c70ccc-1f52-492c-876b-669b51d93515)

##  Capstone Robotics Project

**Team Name:** Team 301

**Group Members:**
Alejandro Caro (CpE)
Henry Forero (CpE)
Alex Harris (CpE)
Tyler Morgan (CpE)
Nathan Gibbs (EcE)
Josue Villarreal (EcE)
Tommy Buretz (EcE)

---

##  Achievement

**1st Place Winner — IEEE SoutheastCon 2026 Hardware Competition**

---

##  Abstract

This project presents the design and implementation of a fully autonomous robotic system developed for the IEEE SoutheastCon 2026 Hardware Competition.

The system consists of a ground robot integrated with a drone-based sensing component. The robot autonomously navigates an arena, activates antennas to illuminate LED indicators, detects colored signals via a drone, and transmits data back to an “Earth” receiver for scoring. Additionally, the robot collects and transports objects (rubber ducks) to a designated scoring zone.

The system operates without human intervention, using onboard sensing, navigation, and control algorithms to execute mission objectives efficiently. A point-based scoring strategy prioritizes antenna activation before secondary objectives.

Through iterative design, testing, and integration, the system achieved reliable performance and secured first place in competition.

---

##  Project Description

This project is a senior design (capstone) project focused on building a fully autonomous robotic platform using a distributed ROS2 architecture.

The system integrates:

* LiDAR-based navigation
* Computer vision
* Wireless communication (BLE + IR)
* Real-time embedded control

The robot is capable of navigating complex environments, detecting mission targets, and executing multi-stage tasks autonomously.

---

##  System Architecture

The robot uses a layered distributed control system:

###  Raspberry Pi 4 (ROS2 Host)

* High-level decision making
* LiDAR processing and navigation
* Camera-based object and color detection
* Mission planning and task sequencing
* Wireless communication (Bluetooth + BLE)
* Serial communication with Arduino

###  Arduino Mega 2560

* Low-level motor control
* Actuator and mechanical task execution
* Deterministic real-time control

###  XIAO ESP32C3

* BLE-based communication interface
* IR signal transmission system

This architecture enables parallel execution of perception, planning, and control tasks.

---

##  Software Design (ROS2-Based)

The system is implemented as a modular ROS2 node network.

###  Launch System

* Central launch file initializes all subsystems:

  * Camera
  * LiDAR
  * Navigation
  * Control
  * Communication

---

###  LiDAR Processing & Virtual Sensors

* Converts raw LiDAR scans into structured virtual sensor regions
* Applies median filtering for noise reduction
* Provides real-time environmental awareness

---

###  Wall-Following Controller

* PID-based control system
* Maintains consistent distance from walls
* Outputs differential drive commands

---

###  Mission Controller

* Executes high-level mission logic
* Uses sensor feedback for state transitions
* Coordinates navigation, task execution, and communication
* Stores detected LED color data

---

###  Vision System (Camera Module)

* Detects ducks and antenna LED colors
* Uses HSV-based color segmentation
* Includes calibration tools for threshold tuning

---

###  Serial Bridge (Raspberry Pi ↔ Arduino)

* Sends real-time velocity commands
* Implements safety timeout for fail-safe stopping
* Handles actuator-level commands

---

###  IR Communication (BLE → XIAO)

* Encodes detected color data
* Transmits via BLE
* XIAO triggers IR emitters for external communication

---

##  Serial Communication Protocol

The Raspberry Pi communicates with the Arduino using a custom serial protocol:

**Command Format:**

```
C <left_velocity> <right_velocity>
```

**Examples:**

```
C 100 100   # Move forward
C 100 -100  # Turn
C 0 0       # Stop
```

**Additional Commands:**

```
M0 → Stop motors
M1 → Enable motors
C → crank
C → flag
C → keypad
```

---

Hardware
Core Components
Raspberry Pi 4
Arduino Mega 2560
XIAO ESP32C3
Sensors
RPLIDAR A1M8
Raspberry Pi Camera Module
IR Sensors
IMU
Actuation
DC Motors
Servo Motors
Motor Drivers
Power System
LiPo Battery
Voltage Regulators
Fuse Protection

 Full Bill of Materials available in:
/hardware/Robot_BOM.xlsx

 System Diagram




 Codebase Structure
/pi_controller        → ROS2 nodes (navigation, vision, control)
/arduino_firmware     → Motor control and actuator code
/launch               → System launch configuration
/hardware             → BOM and wiring diagrams
/docs                 → Reports and documentation
/images               → Robot photos and visuals
 System Behavior
LiDAR generates spatial awareness
Virtual sensors interpret environment
Wall-following controller computes motion
Camera detects objects and LED colors
Mission controller executes task sequence
Arduino performs motor and actuator control
Data transmitted via BLE → XIAO → IR
Results sent to scoring system
 Setup & Installation
Prerequisites
Ubuntu 22.04
ROS2 Humble
OpenCV
RPLIDAR SDK
Installation
git clone https://github.com/yourusername/your-repo.git
cd your-repo
colcon build
source install/setup.bash
Running the System
ros2 launch launch/main_launch.py
 Key Contributions (Alejandro Caro)
Designed distributed ROS2 system architecture
Implemented LiDAR-based virtual sensor processing
Developed PID wall-following controller
Built serial communication interface (Pi ↔ Arduino)
Integrated vision system for object and color detection
Contributed to full system integration and testing
 Results
 1st Place — IEEE SoutheastCon 2026 Hardware Competition
Reliable LiDAR-based navigation
Accurate vision-based detection
Robust multi-stage autonomous execution
Seamless integration of perception, control, and communication
 Author

Alejandro Caro
Computer Engineering, Florida State University

 License

This project is licensed under the MIT License.

* Rasp
