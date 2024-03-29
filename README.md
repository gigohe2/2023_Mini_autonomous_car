# 2023_Mini_autonomous_car

This repository is the result of a project to build small autonomous RC cars and validate autonomous driving algorithms.

I coducted SW development. 

## S/W
- Confirmation of small autonomous smart car operation and ROS integration
- Development of lateral path following algorithm using GPS location information and simulator verification
- Development of AI-based lane recognition algorithm and simulator verification
- Developed and verified AI-based ACC (adaptive cruise control) algorithm
- Developed a real-time autonomous smart car monitoring program

### 1. Lateral path following algorithm using GPS location information

![image](https://github.com/gigohe2/2023_Mini_autonomous_car/assets/59073888/b9d94927-98ea-470b-b889-e6618b90895a)

I used lateral path following algorithm which is updated based on <https://github.com/gigohe2/Stanley-method-lateral-controller>.
This algorithm considers 
 1) Heading error between the car heading and the slope of the path
 2) Cross-track error(cte) between the current position of the car and the target point
So, it can reduce cte while the car stays on the path.

![image](https://github.com/gigohe2/2023_Mini_autonomous_car/assets/59073888/6424e833-6ab4-4cb0-9bd3-743abb9cd7a0)

I ran the experiment on a real path and validated the error. The result is shown below.


![image](https://github.com/gigohe2/2023_Mini_autonomous_car/assets/59073888/f450187d-ea88-4c06-bbcd-db832ca2361c)
![image](https://github.com/gigohe2/2023_Mini_autonomous_car/assets/59073888/e9e811d2-a979-4417-a2cb-3b5a750c877b)

From the experimental results, we were able to verify that the lateral path following algorithm allows the small smart car to follow the given path well.


### 2. Deep learning based lane-detection algorithm
By using lane detection model which named 'LaneNet', I developed lane-detection algorithm in 'Carla', a vehicle simulator.
Moreover, I integrated this lane-detection node with ROS to drive a car while keeping lane.


![image](https://github.com/gigohe2/2023_Mini_autonomous_car/assets/59073888/5f865fab-bdaf-41f0-9c6c-9c7348431b07)

It was not easy to apply deep learing based lane-detection algorithm because of so many variables. Then, I applied an image processing algorithm to drive a small autonomous RC car in real environments. It worked well when the sun was out of the camera's FOV. But, when the camera sees the sun directly, It was hard to control the exposure of the camera.


![image](https://github.com/gigohe2/2023_Mini_autonomous_car/assets/59073888/4d40c0a9-ab5a-4ac8-96ee-0e1333dc1934)


### 3. Deep learning based ACC algorithm
I developed ACC(adaptive cruise control) to follow the lead vehicle.
To detect and track the lead vehicle, I used YOLO V8 object detection model. 
We collected a dataset of the lead vehicle and trained YOLO V8 model. An example dataset is shown below.

![image](https://github.com/gigohe2/2023_Mini_autonomous_car/assets/59073888/c87739cd-cebb-497e-a372-5c420384557b)
![image](https://github.com/gigohe2/2023_Mini_autonomous_car/assets/59073888/28d10763-80ab-4eb5-b859-b75a98d41ebd)


I used realsense depth camera to obtain the distance between mini car and the lead vehicle. 
When the yolo detects the lead vehicle, the output is the bounding box. In the bbox, there are so many outliers which are not the information of the lead vehicle. So, I clustered the depth image to obtain the correct distance of the lead vehicle.
And then, I used Kalman filter to predict the relative speed between two cars. So, the mini car can follow the lead vehicle regardless of the speed of the lead vehicle.
To follow the lead vehicle laterally, I caculated the lateral error between the center point of image and the bbox's center point. Considering FOV of the camera, I interpreted the lateral pixel error to the steering value.
Then, I implemented the validation of ACC algorithm in indoor environments.


### 4. ROS integration
To drive the mini autonomous car, I have to develop each algorithms and gather them.
I used ROS noetic to drive this car. Each nodes have their desired steering and speed value. Then, the main driver node determines the DRIVING_MODE and publishes the steering and motor speed topic to drive the car. 
Also, the car status information is published by the each nodes and subscribed by the autonomous vehicle monitoring node.

### 5. Real-time autonomous driving car monitoring program
To do experiment on the real environments, I developed real-time autonomous driving car monitoring program.
By this program, we can check the car status and the driving information in real time.
We can get current location, speed, battery voltage, RTK state of the mini autonomous car. 
We can also verify that YOLO is working properly. When the object is detected, you can see the class of that object in object detect box.
Besides, when the ACC nodes is on, the distance between two cars is shown in the distance box.


![image](https://github.com/gigohe2/2023_Mini_autonomous_car/assets/59073888/f687f09d-2b2b-4d35-aca1-a7ebb24c0de9)

