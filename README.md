# IEEE SoutheastCon 2026 Hardware Competition

![Robot](https://github.com/user-attachments/assets/64c70ccc-1f52-492c-876b-669b51d93515)

---

## Capstone Robotics Project

**Team Name:** Team 301  

### Group Members
- Alejandro Caro (CpE)  
- Henry Forero (CpE)  
- Alex Harris (CpE)  
- Tyler Morgan (CpE)  
- Nathan Gibbs (EE)  
- Josue Villarreal (EE)  
- Tommy Buretz (EE)  

---

## Achievement

**1st Place — IEEE SoutheastCon 2026 Hardware Competition**  
(79 university teams, Huntsville, Alabama)

---

## Abstract

This project presents the design and implementation of a fully autonomous robotic system developed for the IEEE SoutheastCon 2026 Hardware Competition.

The system consists of a ground robot integrated with a drone-based sensing component. The robot autonomously navigates an arena, activates antennas to illuminate LED indicators, detects colored signals via a drone, and transmits data back to an “Earth” receiver for scoring. Additionally, the robot collects and transports objects (rubber ducks) to a designated scoring zone.

The system operates fully autonomously, using onboard sensing, navigation, and control algorithms to execute mission objectives efficiently. A point-based scoring strategy prioritizes antenna activation before secondary objectives.

Through iterative design, testing, and integration, the system achieved reliable performance and secured first place in competition.

---

## Objective

The IEEE SoutheastCon 2026 Hardware Competition challenges teams to design a fully autonomous rescue system.

### Scenario

A meteor has disabled lunar communications, stranding astronauts with no way to navigate back to their lander.

### Mission

Build a robot and UAV system capable of:
- Restoring four communication antennas  
- Locating and rescuing six stranded Astro-Ducks  
- Re-establishing communication with Earth  
- Completing all tasks within 3 minutes autonomously  

---

<img width="845" height="515" alt="Arena Layout" src="https://github.com/user-attachments/assets/797ac99a-ea9c-4175-89f4-ea5e20621f98" />

---

## Software Architecture (ROS2-Based)

The system is implemented as a modular ROS2 node network, enabling distributed communication between subsystems.

### Simplified Architecture

<img width="799" height="392" alt="Simplified ROS2 Diagram" src="https://github.com/user-attachments/assets/ab62042a-21ac-47a6-9086-706d5cf424a6" />

### Full Architecture

<img width="1813" height="810" alt="Full ROS2 Diagram" src="https://github.com/user-attachments/assets/06777acf-afe5-42fa-804a-e80f5901257f" />

---

## Hardware Architecture

### System Overview

The robot is designed to sense, process, and control movement using a distributed system with a Raspberry Pi 4 and an Arduino Mega.

The system supports LiDAR-based perception and uses ROS2 for high-level control of operations.

---

### System Design

The architecture is divided into two main layers:

#### High-Level System
- Sensor data processing  
- Navigation and decision making  
- ROS2-based system coordination  

#### Low-Level System
- Motor control  
- Actuator execution  
- Real-time hardware interaction  

---

### Main Hardware Components

<img width="965" height="761" alt="image" src="https://github.com/user-attachments/assets/b2544ee6-1c6f-4d23-b55d-1f19e83965df" />
<img width="708" height="647" alt="image" src="https://github.com/user-attachments/assets/77041c7b-95d6-4e44-aab4-f0c503d07272" />

---

### Full System Diagram

<img width="1513" height="835" alt="Full System Diagram" src="https://github.com/user-attachments/assets/98397a1c-5b4d-4a8a-bc80-602aa64941f7" />

---
