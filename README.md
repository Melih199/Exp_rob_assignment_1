# Aruco Navigation - Rosbot
This project is developed by:
1. *Natnael Berhanu Takele - s5446838*
2. *Mustafa Melih Toslak - s5431021*
3. *Ahmet Samet Kosum - s5635830*
4. *Abdelouadoud Guelmami - s5467288*

## Repository Organization
Within this repository, you will discover the "assignment_1" folder, meticulously organized to include all essential files, complemented by a comprehensive README.md file providing detailed guidance and information.

## Project Overview
This project is designed to facilitate Aruco navigation for a ROSbot robot, wherein it autonomously follows a specific Aruco Marker while maintaining a minimum safe distance. The implementation leverages ROS and Python as the primary software stacks, with the incorporation of OpenCV for Aruco detection.

An ArUco marker, utilized in this context, is a synthetic square marker characterized by a distinct black border and an internal binary matrix that determines its unique identifier (ID). The black border expedites rapid detection in the image, while the binary codification enables identification and the application of error detection and correction techniques. The size of the marker determines the dimensions of the internal matrix.

In our implementation, we have employed four markers with IDs 11, 12, 13, and 15. Each marker serves a specific purpose in the navigation sequence:

- Marker 11: Instructs the robot to rotate until Marker 12 is detected, then proceed to reach Marker 12.
- Marker 12: Directs the robot to rotate until Marker 13 is detected, then move towards reaching Marker 13.
- Marker 13: Guides the robot to rotate until Marker 15 is detected, and subsequently, reach Marker 15.
- Marker 15: Signifies the completion of the navigation task.
This structured approach ensures a systematic and autonomous navigation process based on the identification and sequence of ArUco markers.

## Pseudocode

Encapsulating the intricacies of our Aruco navigation system, the pseudocode provided below delves into the fundamental logic and sequential steps that govern the robot's autonomous movements. This representation serves as a comprehensive guide, offering a deeper insight into the algorithmic foundation that orchestrates the robot's responsive navigation behavior, specifically tailored to the identification and interpretation of Aruco markers.

## Installation

Prepare the repository:
cd ~
mkdir assignment_1
mkdir assignment_1/src
cd ~/ assignment_1/src
catkin_init_workspace
cd ~/ assignment_1
catkin_make

Clone this repository to your workspace:

```bash
cd ~/assignment_1/src
git clone https://github.com/husarion/rosbot_ros.git -b noetic
```
Install dependencies:

```bash
cd ~/assignment_1
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
cd ~/assignment_1
catkin_make
```

From this moment you can use rosbot simulations. Please remember that each time, when you open new terminal window, you will need to load system variables:

```bash
source ~/assignment_1/devel/setup.sh
```
### Clone the Aruco Navigation Package

To incorporate this repository into the "src" folder of your recently established Catkin workspace, execute the following command in your terminal:

```bash
git clone https://github.com/Melih199/Exp_rob_assignment_1.git
```

### Make the Package
We'll need to "make" everything in our catkin workspace so that the ROS environment knows about our new package. (This will also compile any necessary code in the package). Execute the given commands in your terminal.

```bash
cd ~/assignment_1
catkin_make
```
Fantastic! The installation process is complete, and now it's time to delve into the exciting world of robot exploration and experimentation. Let the robotics adventure begin! 🤖✨

## Launch

Upon execution of the provided command in the terminal, Gazebo simulation will be initiated, accompanied by the automatic launch of the ROS Master.

**For simulation**:
```bash
roslaunch assignment_1 aruco_navigation.launch
```
**For real robot**:
```bash
roslaunch tutorial_pkg all.launch
roslaunch assignment_1 aruco_navigation.launch
```

## Run the Node

Executing the provided command will activate the controller script responsible for orchestrating the robot's movements.

```bash
rosrun assignment_1 robot_control.py
```

## Conclusion and Future Improvements

