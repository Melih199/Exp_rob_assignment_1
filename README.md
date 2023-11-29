# Aruco Navigation - Rosbot_Simulation
This project is developed by:
1.Natnael Berhanu Takele - s5446838
2.Mustafa Melih Toslak - s5431021
3.Ahmet Samet Kosum - s5635830
4.Abdelouadoud Guelmami - s5467288

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
