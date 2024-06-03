#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseActionResult
import random
import math

# Import PID library
from simple_pid import PID

class M2WRController:
    def __init__(self):
        rospy.init_node('m2wr_movement_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_subscriber = rospy.Subscriber('/m2wr/laser/scan', LaserScan, self.laser_callback)
        self.move_base_subscriber = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.move_base_callback)

        self.vel_msg = Twist()
        self.min_distance = 0.5
        self.rate = rospy.Rate(10)  # 10hz
        self.move_forward = True

        self.pid_linear = PID(1, 0.1, 0.01, setpoint=0)
        self.pid_angular = PID(1, 0.1, 0.01, setpoint=0)

        self.obstacle_front = False
        self.obstacle_left = False
        self.obstacle_right = False

        self.auto_mode = True

    def move_base_callback(self, data):
        # Switch to auto mode if there is an active goal
        if data.status.status == 3:  # 3 indicates a succeeded goal
            self.auto_mode = True
        else:
            self.auto_mode = False

    def laser_callback(self, data):
        num_points = len(data.ranges)
        front = data.ranges[num_points//4:num_points*3//4]
        left = data.ranges[:num_points//4]
        right = data.ranges[num_points*3//4:]
        
        safe_distance = 0.3

        self.move_forward = min(front) > safe_distance
        safe_left = min(left) > safe_distance
        safe_right = min(right) > safe_distance

        self.obstacle_front = not self.move_forward
        self.obstacle_left = not safe_left
        self.obstacle_right = not safe_right

        wall_in_front = not self.move_forward and not safe_left and not safe_right

        if wall_in_front or (self.obstacle_front and self.obstacle_left and self.obstacle_right):
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 1.0
        else:
            self.vel_msg.angular.z = 0.0
            if not self.move_forward or not safe_left or not safe_right:
                self.vel_msg.linear.x = 0.0  
                if not safe_left and safe_right:
                    self.vel_msg.angular.z = -0.5
                elif not safe_right and safe_left:
                    self.vel_msg.angular.z = 0.5
                elif not self.move_forward:
                    if safe_left:
                        self.vel_msg.angular.z = 0.5
                    elif safe_right:
                        self.vel_msg.angular.z = -0.5
                    else:
                        self.vel_msg.angular.z = random.choice([-0.5, 0.5])
            else:
                left_distance = min(left)
                right_distance = min(right)
                error = right_distance - left_distance
                angular_velocity = self.pid_angular(error)

                self.vel_msg.linear.x = 0.5
                self.vel_msg.angular.z = angular_velocity

    def move(self):
        while not rospy.is_shutdown():
            if not self.auto_mode:
                self.velocity_publisher.publish(self.vel_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        robot_controller = M2WRController()
        robot_controller.move()
    except rospy.ROSInterruptException:
        pass


	
