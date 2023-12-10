# Bruno_Quadruped_ROS2 


## Authors

|Name|ID|Email|
|:---:|:---:|:---:|
|Mayank Deshpande|120387333|msdeshp4@umd.edu|
|Tanmay Pancholi|120116711|tamy2909@umd.edu|


## Introduction

Bruno is a quadruped robot based on ROS2. The design of Bruno was created in Solidworks and its URDF was exported using `SW2urdf`. 

## Dependencies

- OS: Ubuntu Linux 22.04 Focal Fossa
- ROS Version: ROS2 Galactic
- Python

## Instruction to run the simulation

```bash
mkdir -p bruno_ws/src
# Copy the package here and move back to root of the workspace
cd ../
# source ROS2 and Build the workspace
source /opt/ros/galactic/setup.bash
colcon build
```
To run spawn the robot in Gazebo follow these steps:

```bash
# Go to the root of the workspace
source install/setup.bash
# Run the launch file
ros2 launch bruno gazebo.launch.py
```

To launch the RViz Visualization along with the Gazebo:

```bash
ros2 launch bruno debug.launch.py
```

## Inverse Kinematics

The bruno_ik file inside the package is the python script for computing the inverse kinematics of the quadruped robot. To run that script:

```bash
cd bruno_ws/src/bruno
python3 bruno_ik.py
```





