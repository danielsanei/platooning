<div align="center">
<h1>Convoy Platooning</h1>
   
![image](https://github.com/user-attachments/assets/6e76ceaf-14fe-45d3-a85f-55117e9eaa97)

<h2>Autonomous Vehicles - Final Project<br>Team #13 - Fall 2024</h2>

![image](https://github.com/user-attachments/assets/ca2227ee-df92-4f81-9013-2533bbc3445a)
</div>

## Table of Contents
1. [Overview](#overview)
2. [Goals and Accomplishments](#goals-and-accomplishments)
   - [Original Goals](#original-goals)
   - [Revised Goals](#revised-goals)
   - [Accomplishments](#accomplishments)
3. [Video Demonstrations](#video-demonstrations)
4. [Challenges](#challenges)
   - [Object Detection Model Deployment](#object-detection-model-deployment)
   - [LiDAR Incremental Scanning](#lidar-incremental-scanning)
   - [Expanding SD Card Storage](#expanding-sd-card-storage)
   - [Car Crash](#car-crash)
5. [Team Members](#team-members)
6. [Project Structure](#project-structure)
7. [Build & Assembly](#build-assembly)
8. [Software Setup](#software-setup)
9. [Replication](#replication)
    - [Object Detection via FastAPI](#object-detection-via-fastapi)
    - [Emergency Braking System via LiDAR](#emergency-braking-system-via-lidar)
10. [Lessons Learned](#lessons-learned)
11. [Acknowledgements](#acknowledgements)

## Overview
The Platooning project aims to develop an autonomous car system for safely deploying an automated convoy, including self-driving, adaptive cruise control, and lane switching. This project was conducted under the guidance of Professor Jack Silberman and fellow staff for MAE/ECE 148 (Introduction to Autonomous Vehicles) during Fall 2024.

## Goals and Accomplishments

### Original Goals
- Design an autonomous vehicle for lane-centered self-driving on a pre-defined track using computer vision.
- Create an object detection model for recognizing a lead car.
- Implement adaptive cruise control to dynamically adjust speed and distance while following a lead car.
- **Stretch Goal:** Develop lane switching logic to track a lead car changing lanes.

### Revised Goals
Our initial focus was adaptive cruise control. However, noticing overlap with another team's project, we consulted with Professor Silberman during a sprint review. He recommended prioritizing convoy platooning and lane switching while integratinng the other team's adaptive cruise control development by the end of the quarter. Our revised goals are as follows:
- Prioritize lane switching to track a lead car when it changes lanes.
- **Stretch Goal:** Integrate adaptive cruise control (from another team) into the platooning system.
- **Stretch Goal:** Achieve true convoy platooning, with our autonomous car between the lead car and another class car.

### Accomplishments
- Achieved lead car following using OpenCV-based object detection.
- Completed lane switching logic to follow a lead car changing lanes.
- Implemented an emergency-braking system with 2D LiDAR.

## Video Demonstrations

### Object Detection Model
https://github.com/user-attachments/assets/413b1b09-951a-4c31-9895-cf87d444e294

### Static Lead Car Following
https://github.com/user-attachments/assets/0055cf35-a9c8-4b17-9804-4d625a7b41ca

### Emergency Braking System
https://github.com/user-attachments/assets/32447180-94ba-4e47-a1bf-0e56fe05ff11

### Lane Centering
https://github.com/user-attachments/assets/643741ac-5b76-4ac5-a23d-bc56d905cc1e

### Lead Car Lane-Switching
https://github.com/user-attachments/assets/f2674d6f-9fba-4aab-8485-085a45f5f64d

## Challenges
We faced significant challenges in deploying the lead car object detection model, as well as many other technical complications throughout the quarter. Despite our obstacles, we successfully delivered core project features, with an new emergency braking system as a new addition. Below are key insights to assist future teams working with similar hardware and project goals.

### Object Detection Model Deployment
Deploying the lead car object detection model from Roboflow presented several technical challenges.

1. **Edge Device Inference Issues**: Our initial approach was to utilize the Luxonis OAK-D camera for edge device inference, leveraging its built in neural network and computer vision capabilities. This method aimed to offload some computation from the Jetson Nano by running the object detection model directly on the camera. However, this implementation failed due to a persisting error that could not be resolved.
2. **Hardware Limitations:** Our initial 64 GB SD card did not have sufficient free memory space to locally deploy the Roboflow model.
3. **Software Limitations:** The Jetson Nano ran Ubuntu 18.04, which was incompatible with ROS2 Humble. As a workaround, the course provided a Docker container pre-configured with the updated Ubuntu 22.04 and ROS2 Humble. However, the container lacked GPU access, forcing the model to run using the CPU alone. This caused significant performance bottlenecks, with inference times reaching 6 times per second on the visual display and 3 seconds per frame internally, making local real-time inference impractical.

To overcome these challenges, we offloaded the object detection model to a FastAPI server hosted on a local computer. This allowed the local machine to take advantage of its more powerful CPU, enabling the object detection model to run effectively at real-time. Additionally, by sending API requests rather than run the model locally, the Jetson Nano was spared the intensive computation required by the model, which also achieves our initial goal.

On the side, we also eventually re-trained the model on RoboFlow. This resolved the error from earlier, and we discovered the root cause was simply a faulty model.

### LiDAR Incremental Scanning
Initially, our system was configured to take LiDAR distance measurements within a range of 355 to 5 degrees (a 10-degree span) to calcualte the average of all measurements. However, this setup led to unexpected behaviors: the car would sometimes brake unecessarily when no object was in front, and at other times, fail to stop when an obstacle was directly ahead.
Upon analysis, we identified two key inefficiencies with our initial approach:
1. **Averaging Measurements**: Calculating the average distance cuased issues when objects at farther distances skewed the final value. For example, if an obstacle was 3 meters away, but the average value was 4 meters, the car would continue driving and crash into the obstacle.
2. ** LiDAR Incremental Scanning:** The LiDAR did not perform a true 360-degree scan, but operated in increments rather than integer degrees. For instance, with an incremement of 0.2 degrees, a range between 0 and 360 degrees actually covered up to 360 x 0.2 = 72 degrees. As a result, the car may stop due to an obstacle 72 degrees to the side rather than directly in front, and the calculated average included values 72 degrees to the side. This finding greatly impacted our results, and explained the strange behaviors noted above.

### Expanding SD Card Storage
We started our project using a 64 GB SD card, and eventually ran out of free memory space. We copied an image of this SD card and flashed a new 512 GB SD card as our new replacement. However, the new card retained the original partition size of 64 GB rather than the available 512 GB memory storage.

To resolve this, we first used the `fdisk` partitioning tool to delete the primary partition without deleting its data, and recreating it to occupy the full SD card space before finally writing and exiting. Then, we resized the primary partition `/dev/mmcblk0p1` using the `sudo resize2fs` command. This command extended the filesystem to utilize the full storage of the new 512 GB card.

### Car Crash
The day before Thanksgiving, we mistakenly launched the `camera_nav` node instead of `camera_nav_calibration`. This activated the throttle, causing the car to drive off the table and crash into the ground. This resulted in significant damage, but thankfully, only to our 3D-printed parts as our electronics and hardware were intact.

With just two weeks left, we quickly redesigned, reprinted, and reassmbled the damaged parts. This mishap occurred at an already stressful time, as we were still troubleshooting issues with deploying our object detection model and completing our final project deliverables. Despite these challenges, we had a highly productive final two weeks, successfully developing and finalizing the core functionality of our platooning system.

## Future Goals
- Refine lane switching for smoother steering when following a lead car.
- Develop a secondary object detection model to identify cars following the lead car and achieve true convoy platooning.
- Integrate adaptive cruise control to dynamically adjust speed and distance while following a lead car.
  
## Team Members

![image](https://github.com/user-attachments/assets/57a6e454-b87f-44ef-ba27-bab961329b77)

- Joel Jijo
  - Electrical Engineering, Class of 2026
- Maximilian Komm
  - Industrial Engineering, Class of 2025
- Ruochen Li
  - Mechanical Engineering, Class of 2026
- Daniel Sanei
  - Computer Engineering, Class of 2026
 
## Project Structure

### Technology Stack
- Hardware: Jetson Nano, VESC, OAK-D Camera, Lidar, GNSS, Logitech F710
- Software: ROS2 Humble, OpenCV (CUDA), YOLOv5, PyTorch, TensorFlow
- Integration: Lane detection, object detection

### Code
1. `lane_detection.py`: Processes camera images to detect lanes, calculates centroid errors, and publishes these errors for vehicle steering adjustments.
2. `object_detection_api.py`: Hosts a FastAPI server to run the RoboFlow object detection model, and returns detection results.
3. `object_detection_node_api.py`: Subscribes to the camera feed, sends images to the FastAPI server for object detection, and publishes the detection results.
4. `turning_demo_node.py`: Implements a PID controller for steering and throttle adjustments based on input from centroid errors, object detection, and wall detection.
5. `wall_detector.py`: Uses LiDAR data to detect walls or obstacles in front of the vehicle and triggers an emergency stop if a wall is detected within a specified distance.

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

## Replication

### Object Detection via FastAPI
Our RoboFlow model was deployed on a local computer, hosting a FastAPI server to run object detection on incoming images. ROS2 nodes transmitted video feed data to the local server through an API request, where the images were processed to return real-time detection results. The steps below serve as a guide to configure this setup:

#### Install Dependencies

In order to use the model through API requests, the first thing to do is to setup an API server. Install `fastapi` to setup the server and `ultralytics` to use the model.

```bash
pip install fastapi
pip install ultralytics
```

#### Build API server

1. Ensure `object_detection_api.py` and `[YOUR_MODEL].pt` are in the same directory.
2. Edit the following line in `object_detection_api.py` to point to your model.
   ```cmd
   model = YOLO("path/to/your/model.pt")
   ```
4. In the terminal, navigate to the folder containing `object_detection_api.py`, and start the API server.
   ```cmd
   uvicorn object_detection_api:app --host [YOUR_IP_ADDRESS] --port [PORT_NUMBER] --reload
   ```
   Ensure that you use the IPv4 address for the computer hosting the API server.
   ```cmd
   uvicorn object_detection_api:app --host 127.0.0.1 --port 8080 --reload
   ```
5. Open a browser and navigate to `http://[YOUR_IP_ADDRESS]:[PORT_NUMBER]/docs`. If the page loads, the API server is running successfully. You may optionally test your model at this address.

#### Sending API Requests
1. On the machine running `object_detection_node_api`, test the connectivity to the API server by pinging its IP address to ensure it is reachable.
   ```cmd
   ping [YOUR_IP_ADDRESS]
   ```
3. Update the `API_URL` in `object_detection_node_api` to the API server address. Ensure that the ROS2 nodes and the API server are on the same network for communication.
   ```cmd
   http://[YOUR_IP_ADDRESS]:[PORT_NUMBER]/detects
   ```
6. Run `object_detection_node_api`. If it recieves data from the camera topic, the API server should send predictions back. Check the API server terminal for request logs.

#### Running Object Detection Using the Model

1. Open two terminals and access the same Docker container in both.
2. Ensure the API server is set up on the host machine (as described above).
3. Navigate to the project directory in the Docker container, and build `object_detection_pkg`,.
   ```cmd
   source install/setup.bash
   colcon build
   ros2 run object_detection_pkg object_detection_node_api
5. Verify that predictions from the API sever are displayed in the terminal running `object_detection_node_api`.
6. Create a new directory under `/home/projects/` for the `turning_node_pkg`.
    ```cmd
   source install/setup.bash
   colcon build
   ros2 run turning_node_pkg turning_demo_node.py
7. After these steps, the car should begin running autonomously.

### Emergency Braking System via LiDAR
Our emergency braking system used a 2D LiDAR sensor for its high accuracy in detecting distances within a 360-degree field of view. The LiDAR node operated independently from our object detection and lane-centering ROS2 nodes. This system monitored surroundings within a defined angle range and triggered braking when an obstacle was detected within the safety distance.

#### Creating the Wall Detector
1. Create the `wall_detector` package (here, our workspace is called `ros2_ws`).
   ```cmd
   cd ~/projects/ros2_ws/src
   ros2 pkg create wall_detector --build-type ament_python
   ```
2. Add the `wall_detector.py` script to the package.
    ```cmd
   ~/projects/ros2_ws/src/wall_detector/wall_detector.py
   ```
3. Modify `setup.py` to include the new `wall_detector` node.
   ```cmd
   entry_points={
    'console_scripts': [
        'wall_detector = wall_detector.wall_detector:main',
       ],
   },
   ```
4. Inside `wall_detector.py`, adjust the angle scan range to match your desired values. Recall that you may need to adjust these values based on the increment, as described in the `Achievements & Challenges` section (i.e. if the increment is set to 0.2, then 360 degrees is actually 360 * 0.2 = 72 degrees).
   ```cmd
   front_ranges = msg.ranges[0:int(360 / 0.2)]  # Adjust for a 0.2-degree increment
   ```
5. Build the package with the following commands.
   ```cmd
   cd ~/projects/ros2_ws
   colcon build
   source install/setup.bash
   ```

#### Running the Wall Detector
1. Launch two terminals and in both, enter your Docker container which includes the necessary dependencies for your system environment.
   ```cmd
   docker exec -it [YOUR_DOCKER_CONTAINER] bash
   ```
2. In one terminal, start the `lane_detection` node to activate the car's throttle, and begin autonomous driving using lane-centering and object detection.
   ```cmd
   ros2 launch ucsd_robocar_lane_detection2_pkg lane_detection.launch.py
   ```
3. In the second terminal, start the `wall_detector` node to activate the emergency braking system.
   ```cmd
   ros2 run wall_detector wall_detector
   ```
4. Verify the emergency braking system is correctly operating by checking the second terminal for LiDAR data outputs, which should display the calculated average distances (or minimum distance, depending on your configuration).

Note that if the `wall_detector` node detects an obstacle within the specified ranges, it will publish a stop signal that shuts down all active ROS2 nodes. This will stop your car entirely, and you will need to restart both the `lane_detection` and 'wall_detector` nodes to resume autonomous driving.


#### Setting the Angles to Scan
As mentioned in the challenges section, the LiDAR scans for angles in increments rather than integer degree scans . For accurately setting the angles, ensure that your integer degrees are converted to increments so that the obstacle detect operates as expected.

## Lessons Learned
Throughout the project, we encountered numerous technical challenges, including hardware failures, limitations, and software compatibility issues. At times, it felt as though every step forward required taking two steps back. However, these complications provided us with opportunities to develop patience, resilience, and determination as we worked tirelessly to achieve our goals and deliver functioning platooning prototype. Our experience not only highlighted the importance of iterative development, but also strengthened our communication skills as collaborated to overcome each obstacle as a team.

## Acknowledgements
We would like to express our sincere gratitude to Professor Jack Silberman for his guidance and support throughout this project. We also extend our deepest thanks to our teaching assistants, Winston Chou and Alexandar Haken, for their unwavering dedication and efforts in supporting our team through various challenges. Their expertise and encouragement made this project an enriching and rewarding experience!
