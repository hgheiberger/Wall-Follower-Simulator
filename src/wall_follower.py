#!/usr/bin/env python2

import numpy as np

import rospy
import math
import sensor_msgs.point_cloud2 as pc2
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg
from std_msgs.msg import Header
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from visualization_tools import *

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    WALL_TOPIC = "/wall"
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    MIN_ANGLE = -2.35500001907
    MAX_ANGLE = 2.35500001907
    ANGLE_INCREMENT = 0.0475757569075


    def __init__(self):
        # Subscribes to LaserScan data
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.parse_laser_data)

        # Initializes drive command publisher
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped) 
        self.line_pub = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=1)
        #self.test_pub = rospy.Publisher("laserPointCloud", PointCloud2)

        # Initialize Helper objects
        self.lp = lg.LaserProjection()

        # Configurable Parameters
        self.previous_u = 0
        self.left_start_index = 45
        self.right_end_index = 55
        self.ignoreDistance = 3
        self.Kp = 625
        self.Kd = 4

    # Parses incoming laser scan data
    def parse_laser_data(self, laser_scan):
        # Selects points from correct size
        if self.SIDE == 1:
            laser_scan.ranges = laser_scan.ranges[self.left_start_index::]
            laser_scan.angle_min = self.MIN_ANGLE + self.ANGLE_INCREMENT*float(self.left_start_index)
        else:
            laser_scan.ranges = laser_scan.ranges[:self.right_end_index]
            laser_scan.angle_max = self.MIN_ANGLE + self.ANGLE_INCREMENT*float(self.right_end_index - 1)

        # Converts to list of points in world
        point_cloud = self.lp.projectLaser(laser_scan)
        point_list = pc2.read_points_list(point_cloud, field_names=['x', 'y'], skip_nans=True)
        x_point_list = []
        y_point_list = []

        # Removes noise from points
        for i in range(len(point_list)):
            if laser_scan.ranges[i] <= self.ignoreDistance:
                x_point_list.append(point_list[i].x)
                y_point_list.append(point_list[i].y)

        # Perform linear regression on points
        fit_m, fit_b = np.polyfit(x_point_list, y_point_list, 1) 
        self.visualizeLine(fit_m, fit_b)

        # Calculate robot's distance to line
        current_distance = self.distanceToLine(fit_m, fit_b)

        # Calculate control
        error = (self.DESIRED_DISTANCE - current_distance) * -self.SIDE
        u = self.Kp*error + self.Kd*self.VELOCITY*math.sin(self.previous_u)
        self.previous_u = u 

        # Drive Car
        self.controlRobot(self.VELOCITY, u)

        # For Debugging
        #rospy.loginfo("Distance: " + str(current_distance) + "u: " + str(u))
        #self.test_pub.publish(point_cloud)
        
    def visualizeLine(self, m, b):
        x = [-2.5, 2.5]
        y = [m*x[0] + b, m*x[1] + b]
        VisualizationTools.plot_line(x, y, self.line_pub, frame="/laser")

    def distanceToLine(self, m, b):
        return abs(b)/(math.sqrt(m*m + 1))

    # Sends control commands to the robot
    def controlRobot(self, speed, steering_angle, steering_angle_velocity=0, acceleration=0, jerk=0 ):
        drive_command_stamped = AckermannDriveStamped()
        
        # Builds command header
        drive_header = Header()
        drive_header.stamp = rospy.Time.now() 
        drive_header.frame_id = "base_link"
        drive_command_stamped.header = drive_header

        # Builds drive command
        drive_command = AckermannDrive()
        drive_command.speed = speed
        drive_command.steering_angle = steering_angle
        drive_command.steering_angle_velocity = steering_angle_velocity
        drive_command.acceleration = acceleration
        drive_command.jerk = jerk
        drive_command_stamped.drive = drive_command

        # Publishes message
        self.pub.publish(drive_command_stamped)

        
        




if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
