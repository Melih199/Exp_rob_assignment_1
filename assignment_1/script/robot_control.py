#!/usr/bin/env python

import rospy
from assignment_1.msg import Aruco_info
from geometry_msgs.msg import Twist


class VisionController:
    def __init__(self):
        # Initialization
        
        self.marker_id_list = [11,12,13,15]
        self.target_size = rospy.get_param("aruco_navigation/target_size")
        self.linear_gain = rospy.get_param("aruco_navigation/linear_gain")
        self.angular_gain = rospy.get_param("aruco_navigation/angular_gain")
           
        self.angular_error = 0
        self.linear_error = 0
        self.vision_id = 0
        self.camera_center = []
        self.marker_center = []
        self.marker_size = []      

        # Publishers and Subscribers
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.aruco_info_sub = rospy.Subscriber('aruco_info', Aruco_info, self.aruco_callback)

    def aruco_callback(self, aruco_info_msg):
 
            self.vision_id = aruco_info_msg.id
            self.camera_center = aruco_info_msg.camera_center
            self.marker_center = aruco_info_msg.marker_center
            self.marker_size = aruco_info_msg.marker_size

   
    def control_loop(self):
        self.marker_turn=0
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():

            if self.marker_id_list[self.marker_turn] != self.vision_id:
                # Rotate to search for the target ArUco marker
                self.vel = Twist()          
                self.vel.angular.z = -0.4
                self.vel_pub.publish(self.vel)
        
            else:
                self.angular_error = self.camera_center[0] - self.marker_center[0]
                self.linear_error = self.target_size - self.marker_size[0]

                # Proportional control
                self.vel.linear.x = self.linear_gain * self.linear_error
                self.vel.angular.z = self.angular_gain * self.angular_error

                # Check for saturation in linear and angular velocities
                self.vel.linear.x = max(min(self.vel.linear.x, 1.0), -1.0)
                self.vel.angular.z = max(min(self.vel.angular.z, 1.0), -1.0)

                # Publish velocity command
                self.vel_pub.publish(self.vel)

                if self.marker_size[0] >= self.target_size:
            
                    if self.marker_id_list[self.marker_turn] != 15:
                        self.marker_turn = self.marker_turn+1
                        self.vel.angular.z = -0.4
                        self.vel_pub.publish(self.vel)
                    else:
                        rospy.signal_shutdown("exit")
            
  
            rate.sleep()
    


    
      
def main():
    # Main control loop
    obc = VisionController()
    rospy.init_node('robot_control')   
   
    obc.control_loop() 

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")       
        
           
if __name__ == '__main__':
    main()
