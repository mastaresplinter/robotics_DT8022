# robotics_DT8022

This repository contains the final project for the course DT8022, focused on developing an autonomous robot capable of fetching two boxes labeled as "1" and placing them in a designated area within a time limit of 3 minutes.



## Tasks

The robot's functionality is divided into several tasks, including Task 1, Task 2, writeTask, and more. Each task operates within its own thread and performs specific functions. The tasks are described briefly as follows:

- Task 1: Updates the robot's state machine, global position, and uncertainty using odometry and the Kalman filter. It also sends a message to initiate the scan matching algorithm in Task 3 and runs the Kalman filter when the Cox algorithm finishes.
- Task 2: Runs the camera and object detection algorithm.
- Task 3: Runs the scan matching algorithm when the corresponding flag is set.
- Task 4: Fetches lidar messages from a TCP server and updates the global lidar message buffer.
- Task 5: Writes robot positions and uncertainties obtained from the odometry, scan match, and Kalman filter algorithms to three seperate files every 1 second.

## Authors
This project was developed by the following authors:
- [@mastaresplinter](https://github.com/mastaresplinter)
- [@Zorlolz](https://github.com/Zorlolz)
- [@Tim4ios](https://github.com/Tim4ios)