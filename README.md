# Convoy Platooning
## Table of Contents
1. Overview
2. Goals/Achievements
3. Team Members
4. Project Structure
5. Build & Assembly
6. Software Setup
ToDo: 7. Codebase Details, 8. Media, 9. Object Detection, 10. Replication, 11. Documentation, 12. Acknowledgements, 13. ToDo: Special Thanks

## Overview
The Platooning project aims to develop an autonomous car system for safely deploying an automated convoy, including self-driving, adaptive cruise control, and lane switching. This project was conducted under the guidance of Professor Jack Silberman and fellow staff for MAE/ECE 148 (Introduction to Autonomous Vehicles) during Fall 2024.

## Original Goals
- Design an autonomous vehicle for lane-centered self-driving on a pre-defined track using computer vision.
- Create an object detection model for recognizing a lead car.
- Implement adaptive cruise control to dynamically adjust speed and distance while following a lead car.
- **Stretch Goal:** Develop lane switching logic to track a lead car changing lanes.
- 
## Revised Goals
Our initial focus was adaptive cruise control. However, noticing overlap with another team's project, we consulted with Professor Silberman during a sprint review. He recommended prioritizing convoy platooning and lane switching while integratinng the other team's adaptive cruise control development by the end of the quarter. Our revised goals are as follows:
- Prioritize lane switching to track a lead car when it changes lanes.
- **Stretch Goal:** Integrate adaptive cruise control (from another team) into the platooning system.
- **Stretch Goal:** Achieve true convoy platooning, with our autonomous car between the lead car and another class car.

## Accomplishments
- Achieved lead car following using OpenCV-based object detection.
- Completed lane switching logic to follow a lead car changing lanes.
- Implemented an emergency-braking system with 2D LiDAR.

## Challenges
We faced significant challenges in deploying the lead car object detection model, as well as many other technical complications throughout the quarter. Despite our obstacles, we successfully delivered core project features, with an new emergency braking system as a new addition.

## Future Goals
- Refine lane switching for smoother steering when following a lead car.
- Develop a secondary object detection model to identify cars following the lead car and achieve true convoy platooning.
- Integrate adaptive cruise control to dynamically adjust speed and distance while following a lead car.
  
## Team #13 Members
- Joel Jijo
  - Electrical Engineering, Class of 2026
- Maximilian Komm
  - Industrial Engineering, Class of 2025
- Ruochen Li
  - Mechanical Engineering, Class of 2026
- Daniel Sanei
  - Computer Engineering, Class of 2026
 
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

Despite everything we encountered, we finally succeeded in training a robust model, and our car ran 3 autonomous laps!

https://github.com/user-attachments/assets/2dcaf328-d50d-4d93-a6de-34f2dec7c8e1

### Docker Container Setup

At this stage of our project, we needed to some additional set-up configurations in order to continue with using ROS2 (Humble). Since the Jetson Nano runs an outdated version of Ubuntu (18.04) which does not support ROS2, we pulled the course-created Docker container with ROS2 pre-installed as our new development environment. This container was configured with the necessary nodes and packages to complete our remaining course deliverables. We connected to the Jetson Nano remotely through SSH sessions, though this lacked a dedicated GUI. As a solution, we set up X-Forwarding to have any visualization on the Jetson Nano forward onto the computer that was connected. This sometimes resulted in one of our computers running a GUI, which would actually pop up on another teammate's computer as both connected to the Jetson Nano with X-Forwarding enabled. A humorous hiccup amidst the many more serious complications we've had so far.

### Lane Detection using OpenCV

For the next stage, our task was to fine-tune existing filters to detect the lane using OpenCV, and run 3 autonomous laps with live lane centering (rather than on a trained model). After ensuring our calibration node was set to true (its throttle parameters are automatically set to zero as a safety measure when calibrating), we ran the nodes to open up the windows below:

![nav_calibration](https://github.com/user-attachments/assets/67927b03-25d0-4458-89e0-68679b5b9f33)

Here, we adjusted our filters by tightening the low and high ranges as close as possible, while retaining a consistent visual of the yellow-dotted lines. Our biggest challenge was at the corners, where we needed to increase the size of the detected lines to account for the larger appearance given the shifted camera angle. We quickly learned that this process was similar to training the machine learning model, as our settings configured for daytime lighting were rendered ineffective during night times, and vice-versa. This required occassional re-tuning, though given our lessons learned from the previous deliverable, we were able to speed up our progress and achieve 3 autonomous laps with lane centering!

https://github.com/user-attachments/assets/60fb2b3e-9622-49fb-8f5d-4f1460c6682e

### GNSS Figure-8 Autonomous Laps

Our final deliverable was to use our GNSS to perform a figure 8 lap around the grass sections at Warren Mall, UCSD, in front of Engineering Building Unit I (EBU I). After setting up the GNSS< we manually drove the car in the intended figure-8 pattern to record GNSS waypoints. The u-blox GPS tracked the car's position, and recorded the drawn path. With some additional PID-tuning, the GNSS was intended to autonomously drive the car in a figure-8 pattern, running 3 laps total. However, despite our persistent efforts to fine-tune the PID parameters, our results were less than ideal:

https://github.com/user-attachments/assets/ce441187-495c-40d4-bf72-20efd4501a66

After hours of debugging, we eventually found that our u-blox data was too noisy to rely on. With the help of the course TAs, we tried replacing it with two backup GPS devices, with no different result. Eventually, we recevied the green light from the TAs to continue with our final project, as we were unable to get ahold of a more stable GPS due to limited class resources. 

## Applying Model Through ROS2 and FastAPI

In order to use model through API request, the first thing to do is to setup an API server.
Steps:
### Install Dependencies
Install fastapi to setup api server and ultralytics to use the model.

```bash
pip install fastapi
pip install ultralytics
'''
### Build API server
1. Make sure object_detection_api.py and "your_model".pt is in the same folder.
2. Open object_detection_api.py, modify model = YOLO("path/to/your/model.pt")
3. In commamd line, go to to folder where object_detection_api.py is, and use command

   '''cmd
   uvicorn object_detection_api:app --host your_ip_address --port port_number --reload
   '''
   which your_ip_address is the IPv4 address of the PC hosting api server, for example:

   '''cmd
   uvicorn object_detection_api:app --host 127.0.0.1 --port 8080 --reload
   '''

4. In a browser, go to your_ip_address:port_number/docs. If a page is shown, that means the API server has been established successfully.
Optional: Test your model in your_ip_address:port_number/docs.

### Sending API Request
1. In the machine that runs the object_detection_node_api, ping the ip address of API server to test if it's reachable.
2. Modify code of object_detection_node_api, change API_URL to the ip address of the API server, such as "http://127.0.0.1:8080/detects"
3. Run object_detection_node_api. If it recieves data from the camera topic, API server should be working and send prediction back. In the terminal that starts API server, it will display information when a request is made.
   
