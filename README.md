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

**1st Place Winner out of 79 teams(Universities) for the  IEEE SoutheastCon 2026 Hardware Competition in Alabama-Huntsville**

---

##  Abstract

This project presents the design and implementation of a fully autonomous robotic system developed for the IEEE SoutheastCon 2026 Hardware Competition.

The system consists of a ground robot integrated with a drone-based sensing component. The robot autonomously navigates an arena, activates antennas to illuminate LED indicators, detects colored signals via a drone, and transmits data back to an “Earth” receiver for scoring. Additionally, the robot collects and transports objects (rubber ducks) to a designated scoring zone.

The system operates without human intervention, using onboard sensing, navigation, and control algorithms to execute mission objectives efficiently. A point-based scoring strategy prioritizes antenna activation before secondary objectives.

Through iterative design, testing, and integration, the system achieved reliable performance and secured first place in competition.

---

##  Objective

The 2026 IEEE SoutheastCon Hardware Competition challenges teams
to design a fully autonomous rescue system. ​

​

Imagine a scenario where a meteor has disabled lunar
communications, stranding astronauts with no way to navigate back to
their lander. ​

Our mission: build a robot and UAV system that can restore four
communication antennas, locate and rescue six stranded Astro-Ducks,
and re-establish contact with Earth; all within 3 minutes, completely
autonomous.

<img width="845" height="515" alt="image" src="https://github.com/user-attachments/assets/797ac99a-ea9c-4175-89f4-ea5e20621f98" />


---



##  Software Architecture (ROS2-Based)

The system is implemented as a modular ROS2 node network.
Simplified Software diagram:
<img width="799" height="392" alt="image" src="https://github.com/user-attachments/assets/ab62042a-21ac-47a6-9086-706d5cf424a6" />

Full Software diagram:
<img width="1813" height="810" alt="image" src="https://github.com/user-attachments/assets/06777acf-afe5-42fa-804a-e80f5901257f" />


##  Hardware architecture (ROS2-Based)

System Overview​

 The robot is designed for reliably sense the environment,
 process data, and control movement using a distributed
 system with a Raspberry Pi 4 and an Arduino Mega.​
 
 The system supports LiDAR-based perception and uses
 ROS2 for high-level control of system operations.​
 
 The System is divided into two main components:​
 
 High-level system for processing and decision making ​
 
 Low-level system for motor control and hardware operation​

 Main hardware components:
 
 <img width="646" height="591" alt="image" src="https://github.com/user-attachments/assets/142728be-e225-42e2-b272-91ae573f6239" />

 
 Full System Diagram

 <img width="1513" height="835" alt="image" src="https://github.com/user-attachments/assets/98397a1c-5b4d-4a8a-bc80-602aa64941f7" />






