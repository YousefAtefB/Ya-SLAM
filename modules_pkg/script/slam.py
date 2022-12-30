#!/usr/bin/env python3

from math import pi
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from modules_pkg.msg import LaserAndOdometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np

# SLAM using the EKF
# ROS
import rospy

# motion model
def g(state, action):
    # state: [x, y, theta]
    # action: [v, w]
    x, y, theta = state
    v, w = action
    #
    return np.array([x + v * np.cos(theta), y + v * np.sin(theta), theta + w])

# jacobian with respect to state
def jacobian_g_state(state, action):
    # state: [x, y, theta]
    # action: [v, w]
    x, y, theta = state
    v, w = action
    return np.array([[1, 0, -v * np.sin(theta)], [0, 1, v * np.cos(theta)], [0, 0, 1]])

# jacobian with respect to action
def jacobian_g_action(state, action):
    # state: [x, y, theta]
    # action: [v, w]
    x, y, theta = state
    v, w = action
    return np.array([[np.cos(theta), 0], [np.sin(theta), 0], [0, 1]])

# assign as publisher to slam_loc topioc
pub = rospy.Publisher('/slam_loc', PoseStamped, queue_size=10)

# x, y, theta
state = np.array([0, 0, 0])
# Covariance
# x, y, theta
state_cov = np.diag([0.1, 0.1, 0.1])

# SLAM
def slam_callback(sensors_msg):

    laser = sensors_msg.laser
    odom = sensors_msg.odom

    action = [odom.twist.twist.linear.x, odom.twist.twist.angular.z]

    state = g(state, action)

    # publish state
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'robot_map'
    pose.pose.position.x = state[0]
    pose.pose.position.y = state[1]
    q = quaternion_from_euler(0, 0, state[2])
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    #
    print(pose)
    #
    pub.publish(pose)


def main():
    state = np.array([0, 0, 0])
    # Initialize Node with node name
    rospy.init_node('slam')
    # Assign node as a subscriber to sensors topic
    sub = rospy.Subscriber('/sensors_topic', LaserAndOdometry, slam_callback)
    # Wait for messages
    rospy.spin()

if __name__ == '__main__':
    try:
        state = np.array([0, 0, 0])
        main()
    except rospy.ROSInterruptException:
        pass
