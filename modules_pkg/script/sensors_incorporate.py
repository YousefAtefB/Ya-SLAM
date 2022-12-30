#!/usr/bin/env python3

import rospy
import message_filters
from math import pi

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from modules_pkg.msg import LaserAndOdometry

import sys

# Assign node as a publisher to this topic
pub = rospy.Publisher('/sensors_topic', LaserAndOdometry, queue_size=10)

def sensors_callback(laser_msg, odom_msg):
    # print(laser_msg.angle_min, laser_msg.angle_max, laser_msg.angle_increment)
    # print(laser_msg.range_min, laser_msg.range_max)

    sensors = LaserAndOdometry()
    sensors.laser = laser_msg
    sensors.odom = odom_msg
    
    # print(sensors)
    pub.publish(sensors)

    # print('First topic: {} \n\n Second Topic: {} \n\n\n\n'.format(laser_msg.header,  odom_msg.header))
    # print(len(front_msg.ranges), len(rear_msg.ranges))    

def main():
    # Initialize Node with node name
    rospy.init_node('sensors_incorporate')

    # Make the node as a subscriber of multiple topics
    laser_sub = message_filters.Subscriber('/scan_multi', LaserScan)
    odom_sub = message_filters.Subscriber('/robot/robotnik_base_control/odom', Odometry)

    ts = message_filters.ApproximateTimeSynchronizer(
        [laser_sub, odom_sub],
        30,
        0.2,
        allow_headerless=False
    )
    
    ts.registerCallback(sensors_callback)

    # Wait for messages
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass