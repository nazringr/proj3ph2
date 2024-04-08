# proj3ph2

# ENPM661 - Spring 2024
## Project 03 - Phase 2

### Group Members:
- Nazrin Gurbanova - Directory ID: nazrin, UID: 120426469

- Onkar Kher - Directory ID: okher, UID: 120407062  

### Overview:
This project aims to implement the A* algorithm on a Differential Drive (non-holonomic) TurtleBot3 robot to navigate it from a given start point to a given goal point. The implementation involves considering the differential drive constraints while planning the robot's motion. This README provides instructions on how to run the code for both Part 01 (2D implementation) and Part 02 (Gazebo visualization).

### How to Run:
#### Part 01 (2D Implementation):
1. Run the Python script for Part 01. 
2. Follow the terminal prompts to input start point coordinates, goal point coordinates, wheel RPMs, and clearance. Click y to run predefined inputs
3. View the generated optimal path on the map.

#### Part 02 (Gazebo Visualization):
1. Run the competition arena launch file in the turtlebot3_project3 package
2. In a new terminal run the publisher script 'vel_publish.py'
3. In the video, the following are the inputs for the gazebo world simulation: 
Clearance: 0.2
L_rpm: 2
R_rpm: 1
Start_x: 0.5
Start_y: 1
Start_orient: 0
Goal_x: 5.7
Goal_y: 1
All the distances are in meters.
4. After the optimal path has been shown close the window demonstrating the optimal path and then the publisher will start publishing on the topic cmd_vel, as shown in the video.

### GitHub Repository Link:
https://github.com/nazringr/proj3ph2.git

### Simulation Video Drive link:
-https://drive.google.com/drive/folders/1HeuHOmFE3AIkpdSBWb2SnWB01xnjVAiQ