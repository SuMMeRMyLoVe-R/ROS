#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


def scan_callback(scan):
    center_index = len(scan.ranges) // 2
    window = 30  # scan角度區間 ±15°
    front_ranges = scan.ranges[center_index - window: center_index + window]

    valid_ranges = [r for r in front_ranges if not np.isnan(r) and not np.isinf(r)]
    min_distance = min(valid_ranges) if valid_ranges else float('inf')

    cmd = Twist()
    if min_distance < 0.5:
        rospy.loginfo(" Obstacle detected! Stopping. ")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5
    else:
        cmd.linear.x = 0.3
        cmd.angular.z = 0.0

    pub.publish(cmd)


if __name__ == "__main__":
    rospy.init_node('laser_avoidance_node')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.loginfo(" Laser-based obstacle avoidance node started.")
    rospy.spin()
