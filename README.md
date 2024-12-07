# Platooning Project - A Story of Persistence
## Table of Contents
1. Overview
2. Goals/Achievements
3. Team Members
4. Project Structure
5. Build & Assembly
6. Software Setup
ToDo: 7. Codebase Details, 8. Media, 9. Object Detection, 10. Replication, 11. Documentation, 12. Acknowledgements, 13. ToDo: Special Thanks

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
- Hardware: Jetson Nano, VESC, OAK-D Camera, Lidar, GNSS, Logitech F710
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

Next, we needed to wire our electrical components together. To power our Jetson Nano, we connected a 5V barrel connector. We utilized an antispark module which connects to our battery for safe power distribution, and a buck converter to step down the voltage from 12V to the Jetson Nano's desired 5V supply. For our motor control, we connected the brushless DC motor to a VESC for speed and position control by soldering the wires and connectors, and wired the steering servo motor to a servo PDB for steering control. The rest of our hardware consisted of sensors, which we connected to our Jetson Nano via a USB hub.

![image](https://github.com/user-attachments/assets/5e6d4ad7-364a-4679-ab85-e10c08bb0224)

Placing our car on a stand to safely test our final build, we conducted some baseline tests to ensure the car was safely operating, and then tested the car's response to our manual controller input:

https://github.com/user-attachments/assets/89ffddb0-0794-4232-b8b8-c4eddb98c565

After confirming correct operation of our car, we were ready to drive on the ground!

https://github.com/user-attachments/assets/2421baa7-b0c0-423c-a46f-772f3b66ef8e

We made further refinements to our design, resulting in a more organized look:

![image](https://github.com/user-attachments/assets/53b93547-c1c7-49de-bc9e-a965cbaa3220)

## Software Setup

### Flashing Ubuntu

Our first step on the software side was to flash the Micro SD card that will hold the Ubuntu (version 18.04) operating system for our SBC. We obtained UCSD's Jetson Nano Developer Kit SD card image, though our initial attempts instantly failed as our provided microSD adapter was faulty. After getting ahold of a working adapter, we used Etcher to flash our SD card, which also failed twice after reaching 99% completion. Upon trying again with another team member's Mac computer (as opposed to Windows), we finally successfully flashed our image.

Using a 5V power supply with a barrel jack connector, we powered on the Jetson Nano using a wall outlet. As our system was fresh, we started with wired communication to configure the WiFi network and make note of the SBC's IP address. To secure our SBC access for the quarter, we changed the host name and password to unique credentials. Strangely, after rebooting our Jetson Nano, we were unable to log in despite using the exact same host name and password that we had set earlier. After debugging further, we concluded that the most time-effective solution was to simply re-flash our SD card. After doing so, we finalized our setup by performing some basic system health checks and cleanup operations.

### Installing OpenCV

To leverage the Jetson Nano's GPU, we needed to install OpenCV from source so that we could enable CUDA acceleration, which would optimize our performance in computer vision tasks such as lane and object detection. This installation process takes 4 hours, which we performed 5 times before eventually debugging what was a memory issue. Our reference document instructed us to run "sudo reboot" after creating our 4 GB swap file, however, this would reset and delete the created swap file. By simply omitting that step, our OpenCV installation was finally successful!

![image](https://github.com/user-attachments/assets/d825d1a2-4d30-42ad-8bf7-c640e62298c0)

### Autonomous Driving with Machine Learning

For our course deliverables, we were instructed to use the DonkeyCar framework. This open-source framework for training self-driving cars using Python, TensorFlow, and Keras, and supports deep learning and reinforcement learning. To set it up, we created a virtual environment, and installed DonkeyCar and all its necessary dependencies. 

Our first task was to train a model that was robust enough to successfully perform 3 laps at our Engineering Building Unit II (EBU2) course track. After configuring the DonkeyCar with our hardware settings, we ran the framework's training script to train our model. Given Professor Silberman's recommendation of collecting data for at least 20 laps, we performed 30 laps to ensure a robust dataset.

https://github.com/user-attachments/assets/e569d961-fb99-4c43-b340-ee6f2a534348

The training process was as follows: we transferred over data to the UC San Diego GPU cluster (8 CPU, 1 GPU, 16 GB RAM) to remotely train our model before transferring it back to the Jetson Nano, and configuring our DonkeyCar to run using the model rather than manual driving. Our first dataset did not produce a smooth-running model, so we continued to collect data and train new models iteratively, making corrections at each iteration. We learned a few lessons along the way:

1. Each lap must employ the same strategy as the previous. Our model was intended to follow the dotted yellow lines, so during our training laps, we made sure that the car was centered in the middle of the track so as not to lose sight of the yellow lines.
2. The slower our car's throttle speed, the more reliable the model. For faster speeds, the car must be trained on a much higher number of laps to produce a robust model. 
3. The camera angle is critical for effectively deploying a model, as the model could fail if the camera angle was even slightly altered in comparison to the images used in the dataset.
4. Lighting was of utmost importance, as data collected during the day would be highly ineffective at night. Essentially, the conditions in which the data is collected must match closely to those during testing.
5. Sometimes, a system needs to be reset to resolve certain issues, as a strange complication with a corrupted DonkeyCar was only solved by creating a new one.

Despite everything we encountered, we finally succeeded in deploying a smooth-running machine learning model!

https://github.com/user-attachments/assets/2dcaf328-d50d-4d93-a6de-34f2dec7c8e1
