# IEEE SoutheastCon 2026 Hardware Competition
![IMG_7518](https://github.com/user-attachments/assets/64c70ccc-1f52-492c-876b-669b51d93515)


## Capstone Robotics Project

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

## Achievement

1st Place Winner — IEEE SoutheastCon 2026 Hardware Competition

---

## Abstract

The purpose of this project was to design and implement a fully autonomous robotic system for the IEEE SoutheastCon 2026 Hardware Competition. The competition required teams to develop a self-operating system capable of navigating an arena, completing physical tasks, and maximizing points within a strict time limit.

The system consisted of a ground robot integrated with a drone-based sensing component. The robot performed four primary tasks to activate antennas, which illuminated LED indicators. A drone system then observed and identified the LED colors, and the detected information was transmitted to an “Earth” receiver to score points. Additionally, the robot collected rubber ducks distributed throughout the arena and transported them to a designated scoring zone.

The robot operated entirely autonomously using onboard sensing, navigation, and control algorithms to determine its position and execute task sequences. A point-based scoring system prioritized accurate and efficient task completion, with antenna activation yielding the highest rewards. As a result, the system was designed to prioritize these tasks before collecting additional points through duck retrieval.

Team 301 collaborated with a mechanical engineering team responsible for the robot’s structural design, while the electrical and computer engineering team focused on control systems, sensing, and system integration. The system underwent extensive testing and iterative refinement to improve navigation accuracy, task reliability, and coordination between subsystems.

The final system successfully demonstrated reliable autonomous operation and effective task execution, resulting in a competition-winning performance.

---

## Project Description

This project is a computer/electrical engineering senior design (capstone) project developed for the IEEE SoutheastCon 2026 Hardware Competition.

The system is a fully autonomous robotic platform built using a distributed ROS2 architecture. It integrates LiDAR-based navigation, computer vision, wireless communication, and real-time control to navigate an arena, detect mission targets, and execute complex multi-stage tasks without human intervention.

---

## System Architecture

The robot uses a distributed control system:

* **Raspberry Pi 4 (ROS2 Host)**

  * High-level decision making
  * LiDAR processing and navigation
  * Camera-based detection
  * Task sequencing and mission logic
  * Wireless communication (Bluetooth + BLE)

* **Arduino Mega 2560**

  * Low-level motor control
  * Actuator control
  * Deterministic real-time execution

* **XIAO ESP32C3 Module**

  * BLE-based IR transmission system

This layered architecture allows simultaneous execution of perception, planning, and control.

---

## Software Design (ROS2-Based)

The system is implemented as a modular ROS2 node network:

### Launch System

The entire robot stack is orchestrated using a ROS2 launch file that initializes all subsystems in sequence (camera, LiDAR, navigation, control, IR).

---

### LiDAR Processing & Virtual Sensors

* Converts raw LiDAR scans into structured “virtual sensors” representing distances around the robot
* Uses median filtering and angular windows for robustness

---

### Wall-Following Controller

* Implements PID-based wall following
* Maintains consistent distance from walls
* Generates differential drive commands

---

### Program Controller (Mission Logic)

A high-level sequencing engine that:

* Executes a predefined mission plan
* Uses sensor feedback to trigger transitions
* Coordinates navigation, tasks, and communication
* Stores detected colors and manages task execution

---

### Vision System (Camera Module 2)

* Detects ducks and LED colors from antennas
* Uses HSV-based color segmentation
* Includes a custom calibration tool for tuning detection thresholds

---

### Serial Bridge (Pi ↔ Arduino)

* Sends real-time wheel commands to Arduino
* Implements safety timeout for fail-safe stopping
* Handles task-level commands

---

### IR Communication (BLE → XIAO)

* Encodes detected colors into compact payloads
* Transmits via BLE to XIAO ESP32C3
* XIAO triggers IR emitters for communication

---

## Electronics & Hardware

* Raspberry Pi 4
* Arduino Mega 2560
* RPLIDAR A1M8
* Camera Module 2
* XIAO ESP32C3 (BLE + IR bridge)
* DC motors with motor drivers
* IR transmitters/receivers
* Bluetooth communication system
* Power supply system

---

## Codebase Structure

```bash
/pi_controller        → ROS2 nodes (navigation, vision, control)
/arduino_firmware     → Motor control and actuator code
/launch               → System launch configuration
/hardware             → Wiring diagrams and layouts
/docs                 → Reports and documentation
/images               → Robot photos and visuals
```

---

## System Behavior

The robot operates using a layered autonomous pipeline:

1. LiDAR generates spatial awareness
2. Virtual sensors interpret environment
3. Wall-following controller computes motion
4. Camera detects ducks and antenna LED colors
5. Program controller executes mission sequence
6. Arduino executes motor and mechanical commands
7. Color data is transmitted via BLE → XIAO → IR
8. Data is sent back to Earth

---

## Results

* 1st Place — IEEE SoutheastCon 2026 Hardware Competition
* Reliable LiDAR-based wall-following navigation
* Accurate vision-based color detection
* Successful multi-stage mission execution
* Robust ROS2 distributed system performance
* Seamless integration of navigation, vision, and communication

---

## Author

Alejandro Caro
Computer Engineering, Florida State University

---

## License

This project is licensed under the MIT License.
