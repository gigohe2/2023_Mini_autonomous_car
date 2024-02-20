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



