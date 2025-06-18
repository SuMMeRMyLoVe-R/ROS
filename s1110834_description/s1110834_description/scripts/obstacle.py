#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


class CmdVelObstacleFilter:
    def __init__(self):
        rospy.init_node('cmd_vel_obstacle_filter_node')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/cmd_vel_raw', Twist, self.cmd_callback)
        self.min_distance = float('inf')
        self.threshold = 1

    def scan_callback(self, scan):
        center_index = len(scan.ranges) // 1
        window = 120
        front_ranges = scan.ranges[center_index - window : center_index + window]
        valid = [r for r in front_ranges if not np.isnan(r) and not np.isinf(r)]
        self.min_distance = min(valid) if valid else float('inf')

    def cmd_callback(self, msg):
        cmd = Twist()
        cmd.angular.z = msg.angular.z

        if self.min_distance < self.threshold and msg.linear.x > 0:
            rospy.loginfo("Obstacle too close! Blocking forward movement.")
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = msg.linear.x

        self.pub.publish(cmd)


if __name__ == "__main__":
    CmdVelObstacleFilter()
    rospy.loginfo("Obstacle filter node started (keyboard safe mode).")
    rospy.spin()
