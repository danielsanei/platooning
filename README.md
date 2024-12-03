# Platooning Project - A Story of Persistence
## Table of Contents
1. Overview
2. Team Members
3. Project Structure
4. Build & Assembly
5. Software Setup
ToDo: 4. Codebase Details, 5. Media, 6. Object Detection, 7. Replication, 8. Documentation, 9. Acknowledgements, 10. ToDo: Special Thanks

## Overview
The Platooning project aims to develop an autonomous car system capable of safely deploying an automated convoy which implements self-driving, adaptive cruise control, and lane switching. This project was conducted under the guidance of Professor Jack Silberman and fellow staff for MAE/ECE 148 (Introduction to Autonomous Vehicles) during the Fall 2024 quarter.

## Goals
- Lane Switching with a RC car
- Lane Following 
- Adaptive Cruise Control

## Achievements
- Lane Following Model for RC cqar
- Lane Switching
  
## Team #13 Members
- Joel Jijo
  - Electrical Engineering
- Maximilian Komm
  - Industrial Engineering
- Ruochen Li
  - Mechanical Engineering
- Daniel Sanei
  - Computer Engineering
 
## Project Structure
- Hardware: Jetson Nano, VESC, OAK-D Camera, Lidar, GNSS
- Software: ROS2 Humble, OpenCV (CUDA), YOLOv5, PyTorch, TensorFlow
- Integration: Lane detection, object detection

## Build & Assembly
For this project, we were provided an RC car platform which served as the chassis of our car. This chassis came with a steering servo motor and a brushless DC motor for driving and steering the wheels. Our first task was to design and create a base plate for mounting our essential hardware, and to 3D print a case for the Jetson Nano, and a mount for the LiDar camera. We went through several sketches and iterations before getting our final measurements.

![image](https://github.com/user-attachments/assets/c2853210-d12a-4e02-a724-db075174592c)

We initially used Inkscape for designing the dimensions of our base plate, as well as the size and spacing for our holes. After much deliberation, we eventually settled on using M3 screws for mounting our hardware to the base plate, and ensured we had plenty of holes with sufficient spacing to work with.

![image](https://github.com/user-attachments/assets/aea6ca42-3be0-4bed-9c10-248b403b53ac)

After finalizing our design on AutoCAD, we were able to successfully laser cut our base plate. All that was left after this was to manually screw holes for M2 screws to mount this base plate onto the car chassis (2 holes at the front, 2 at the back).

For our 3D printed parts, our team found open-source designs which we adapted to suit the limitations of our custom physical parts. At the earliest stage of progress, here's what our car looked like:

![image](https://github.com/user-attachments/assets/1a255cfb-1c6e-4066-9738-6d101d440d94)

Next, we needed to wire our electrical components together. To power our Jetson Nano, we connected a 5V barrel connector. We utilized an antispark module which connects to our battery for safe power distribution, and a buck converter to step down the voltage from 12V to the Jetson Nano's desired 5V supply. For our motor control, we connected the brushless DC motor to a VESC for speed and position control, and wired the steering servo motor to a servo PDB for steering control. The rest of our hardware consisted of sensors, which we connected to our Jetson Nano via a USB hub.

![image](https://github.com/user-attachments/assets/5e6d4ad7-364a-4679-ab85-e10c08bb0224)

Placing our car on a stand to safely test our final build, we conducted some baseline tests to ensure the car was safely operating, and then tested the car's response to our manual controller input:

https://github.com/user-attachments/assets/89ffddb0-0794-4232-b8b8-c4eddb98c565

After confirming correct operation of our car, we were ready to drive on the ground!

https://github.com/user-attachments/assets/2421baa7-b0c0-423c-a46f-772f3b66ef8e

We made further refinements to our design, resulting in a more organized look:

![image](https://github.com/user-attachments/assets/53b93547-c1c7-49de-bc9e-a965cbaa3220)

## Software Setup
