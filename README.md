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

## Flowchart

Encapsulating the intricacies of our Aruco navigation system, the flowchart provided below delves into the fundamental logic and sequential steps that govern the robot's autonomous movements. This representation serves as a comprehensive guide, offering a deeper insight into the algorithmic foundation that orchestrates the robot's responsive navigation behavior, specifically tailored to the identification and interpretation of Aruco markers.

 <img src="https://github.com/Melih199/Exp_rob_assignment_1/assets/58879182/5d27ada1-4e73-41b3-b62d-09fe41a9a5f3.type" width="1000" height="500">

## Installation

Prepare the repository:
cd ~
mkdir -r assignment_ws/src
cd ~/ assignment_ws
catkin_make

Clone this repository to your workspace:

```bash
cd ~/assignment_ws/src
git clone https://github.com/husarion/rosbot_ros.git -b noetic
```
Install dependencies:

```bash
cd ~/assignment_ws
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
cd ~/assignment_ws
catkin_make
```

From this moment you can use rosbot simulations. Please remember that each time, when you open new terminal window, you will need to load system variables:

```bash
source ~/assignment_ws/devel/setup.sh
```
### Clone the Aruco Navigation Package

To incorporate this repository into the "src" folder of your recently established Catkin workspace, execute the following command in your terminal:

```bash
git clone https://github.com/Melih199/Exp_rob_assignment_1.git
```

### Make the Package
We'll need to "make" everything in our catkin workspace so that the ROS environment knows about our new package. (This will also compile any necessary code in the package). Execute the given commands in your terminal.

```bash
cd ~/assignment_ws
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

<img src="https://github.com/Melih199/Exp_rob_assignment_1/blob/Rosbot_aruco_simulation/simulation_node.gif" width="1000" height="800"/>

<img src="https://github.com/Melih199/Exp_rob_assignment_1/blob/Rosbot_aruco_simulation/Rosbot%20Real%20World%20Scenario.gif" width="1000" height="800"/>





## Conclusion and Future Improvements

In concluding our Aruco Navigation project, our team has successfully developed a robust system for autonomous robot navigation using Aruco markers. The project's structure and implementation, detailed in the README, lay the foundation for future enhancements and applications.

### Achievements:

1. **Aruco Navigation:** The project achieves its primary goal of enabling a ROSbot robot to autonomously follow a predefined sequence of Aruco markers while maintaining a safe distance. The utilization of ROS, Python, and OpenCV contributes to the efficiency of the navigation system.

2. **Marker Logic:** The distinctive logic associated with each Aruco marker (11, 12, 13, and 15) provides a systematic approach, ensuring a seamless navigation sequence. The markers' roles in instructing the robot to rotate, detect subsequent markers, and reach specific destinations add a layer of intelligence to the navigation process.

3. **Flowchart:** The included flowchart offers a comprehensive and deep insight into the algorithmic underpinnings of the navigation system. This serves as a valuable resource for understanding the logical flow and procedural steps governing the robot's movements.

### Future Improvements:

1. **Dynamic Marker Handling:** Enhance the system to dynamically adapt to changes in the environment, allowing the robot to intelligently respond to the appearance or disappearance of Aruco markers during navigation.

2. **Obstacle Avoidance:** Integrate obstacle detection and avoidance mechanisms to further enhance the robot's ability to navigate in complex environments, ensuring safety and efficiency.

3. **Localization Enhancements:** Explore advanced localization techniques to improve the robot's accuracy in identifying and reaching Aruco markers, especially in scenarios with multiple markers in proximity.

4. **Human-Robot Interaction:** Implement features for human-robot interaction, allowing users to influence or modify the robot's navigation behavior based on real-time input or commands.

5. **Performance Optimization:** Conduct performance optimization to ensure efficient resource utilization and real-time responsiveness, particularly in resource-constrained environments.

By addressing these potential improvements, the Aruco Navigation project can evolve into a more versatile and adaptive robotic system, extending its applicability to various real-world scenarios.

