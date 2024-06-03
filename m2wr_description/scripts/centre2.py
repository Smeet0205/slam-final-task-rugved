#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random
import math

# Import PID library
from simple_pid import PID

class M2WRController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('m2wr_movement_controller', anonymous=True)
        
        # Create a publisher to send speed commands
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Create a subscriber to receive laser scan data
        self.laser_subscriber = rospy.Subscriber('/m2wr/laser/scan', LaserScan, self.laser_callback)
        
        # Create an instance of the Twist message to hold velocity commands
        self.vel_msg = Twist()

        # Set the minimum distance from an obstacle
        self.min_distance = 0.5

        # Set the rate of the loop
        self.rate = rospy.Rate(10) # 10hz

        # Initialize the movement flag
        self.move_forward = True

        # Initialize PID controllers for linear and angular velocities
        self.pid_linear = PID(1, 0.1, 0.01, setpoint=0)
        self.pid_angular = PID(1, 0.1, 0.01, setpoint=0)

        # Initialize variables to track obstacles on all front sides
        self.obstacle_front = False
        self.obstacle_left = False
        self.obstacle_right = False

    # Define the laser scan callback function
    def laser_callback(self, data):
        # Assuming 360-degree coverage and data.ranges is a list of all scan points
        num_points = len(data.ranges)
        front = data.ranges[num_points//4:num_points*3//4]
        left = data.ranges[:num_points//4]
        right = data.ranges[num_points*3//4:]
        
        # Define the safe distance from obstacles
        safe_distance = 0.3  # Change this value to 0.3

        # Check if there are obstacles within the safe distance in any direction
        self.move_forward = min(front) > safe_distance
        safe_left = min(left) > safe_distance
        safe_right = min(right) > safe_distance

        # Check if there are obstacles on all three front sides
        self.obstacle_front = not self.move_forward
        self.obstacle_left = not safe_left
        self.obstacle_right = not safe_right

        # Detect if there is a wall in front and no space to go on the sides
        wall_in_front = not self.move_forward and not safe_left and not safe_right

        # Adjust the robot's movement based on the detected obstacles
        if wall_in_front or (self.obstacle_front and self.obstacle_left and self.obstacle_right):
            # Perform a U-turn if a wall is detected in front or if obstacles are detected on all three front sides
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 1.0
        else:
            # Reset angular velocity when not performing a U-turn
            self.vel_msg.angular.z = 0.0

            # Adjust linear velocity based on obstacles
            if not self.move_forward or not safe_left or not safe_right:
                self.vel_msg.linear.x = 0.0  
                if not safe_left and safe_right:
                    self.vel_msg.angular.z = -0.5  # Turn right if too close to an obstacle on the left
                elif not safe_right and safe_left:
                    self.vel_msg.angular.z = 0.5  # Turn left if too close to an obstacle on the right
                elif not self.move_forward:
                    if safe_left:
                        self.vel_msg.angular.z = 0.5  # Prefer turning left by default
                    elif safe_right:
                        self.vel_msg.angular.z = -0.5
                    else:
                        # If stuck with obstacles on both sides, choose a direction randomly
                        self.vel_msg.angular.z = random.choice([-0.5, 0.5])
            else:
                # If safe, calculate control signals using PID controllers
                # Calculate the distance to the left and right walls
                left_distance = min(left)
                right_distance = min(right)

                # Calculate the error between the distances to the left and right walls
                error = right_distance - left_distance

                # Use PID to adjust angular velocity to center the robot between the walls
                angular_velocity = self.pid_angular(error)

                self.vel_msg.linear.x = 0.5  # Maintain constant forward velocity
                self.vel_msg.angular.z = angular_velocity

    # Define the main movement function
    def move(self):
        # Loop until the ROS node is shutdown
        while not rospy.is_shutdown():
            # Publish the velocity command
            self.velocity_publisher.publish(self.vel_msg)

            # Sleep to maintain the desired loop rate
            self.rate.sleep()

# Entry point of the script
if __name__ == '__main__':
    try:
        # Create an instance of the robot controller class
        robot_controller = M2WRController()
        # Start moving the robot
        robot_controller.move()
    except rospy.ROSInterruptException:
        pass

	
