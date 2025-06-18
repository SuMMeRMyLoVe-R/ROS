#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def send_trajectory(joint_names, positions, duration):
    pub = rospy.Publisher('/arm_trajectory_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1.0)
    traj = JointTrajectory()
    traj.joint_names = joint_names
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(duration)
    traj.points.append(point)
    traj.header.stamp = rospy.Time.now()
    pub.publish(traj)


if __name__ == '__main__':
    rospy.init_node('arm_test_node')

    # 初始位置（保持）
    rospy.loginfo("Sending to initial position [0.0, 0.0]")
    send_trajectory(['revolute5', 'revolute6'], [0.0, 0.0], 2)
    rospy.sleep(3)

    # 上擺動 45 度（0.785 rad）
    rospy.loginfo("Lifting to 45 degrees [0.785, 0.785]")
    send_trajectory(['revolute5', 'revolute6'], [0.785, 0.785], 2)
    rospy.sleep(3)

    # 回原位
    rospy.loginfo("Returning to [0.0, 0.0]")
    send_trajectory(['revolute5', 'revolute6'], [0.0, 0.0], 2)
    rospy.sleep(2)

    rospy.loginfo("Motion complete.")
