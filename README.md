

This repository contains a ROS 2 (Humble) workspace with multiple packages developed for an **Stair-Climbing Mobile Robot**. The project also includes a **design optimization notebook** used during the development phase to fine-tune system parameters and performance.

## Repository Structure

ROS 2 Packages are amr_description, amr_controller and amr_localization(Odometry)
notebook is not a ros package and is ignored by colcon

Launching the Simulation

colcon build

ros2 launch amr_description gazebo.launch.py

ros2 launch amr_controller controller.launch.py use_sim_time:=True

ros2 launch amr_localization local_localization.launch.py

rviz2

publish velocity message in /cmd/vel topic
