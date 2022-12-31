#!/usr/bin/env python3

import rospy
import message_filters
from math import pi

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from modules_pkg.msg import LaserAndOdometry

import sys

# Assign node as a publisher to this topic
pub = rospy.Publisher('/sensors_gamda', LaserAndOdometry, queue_size=10)

def sensors_callback(laser_msg, odom_msg):
    sensors = LaserAndOdometry()
    sensors.laser = laser_msg
    sensors.odom = odom_msg
    
    pub.publish(sensors)

def main():
    # Initialize Node with node name
    rospy.init_node('sensors_incorporate')

    # Listen to Laser and Odometry
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
        print('lazer goz weeeeeeeeee')
        print('/scan_multi + /robot/robotnik_base_control/odom --> /sensors_gamda')
        main()
    except rospy.ROSInterruptException:
        print('lazer iz ded')
        pass
