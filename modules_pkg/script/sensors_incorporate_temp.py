#!/usr/bin/env python3

import rospy
import message_filters
from math import pi

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import sys

# Assign node as a publisher to this topic
pub = rospy.Publisher('/sensors_topic', LaserScan, queue_size=10)

def laser_callback(front_msg, rear_msg, odom_msg):
    print(front_msg.angle_min, front_msg.angle_max, front_msg.angle_increment)
    print(front_msg.scan_time, front_msg.range_min, front_msg.range_max)
    # print(rear_msg.angle_min, rear_msg.angle_max, rear_msg.angle_increment)
    # print('First topic: {} \n\n Second Topic: {} \n\n Third Topic: {} \n\n\n\n'.format(front_msg.header,  rear_msg.header, 1))
    
    # print(len(front_msg.ranges), len(rear_msg.ranges))

    merged = []
    
    angle = front_msg.angle_min
    for d in front_msg.ranges:
        if angle >= -pi/2 and angle <= pi/2:
            merged.append(d)
        angle += front_msg.angle_increment 


    angle = rear_msg.angle_min
    for d in rear_msg.ranges:
        if angle > -pi/2 and angle < pi/2:
            merged.append(d)
        angle += rear_msg.angle_increment 

    print(len(merged))

    laser = LaserScan()
    laser.header = front_msg.header
    laser.header.frame_id = "robot_base_link"
    laser.angle_min = -pi/2
    laser.angle_max = -pi/2
    laser.angle_increment = front_msg.angle_increment 
    laser.time_increment = front_msg.time_increment
    laser.scan_time = front_msg.scan_time
    laser.range_min = front_msg.range_min
    laser.range_max = front_msg.range_max
    laser.ranges = merged
    # laser.intensities = front_msg.intensities

    pub.publish(laser)

def main():
    # Initialize Node with node name
    rospy.init_node('sensors_incorporate')

    # Make the node as a subscriber of multiple topics
    front_sub = message_filters.Subscriber('/robot/front_laser/scan', LaserScan)
    rear_sub = message_filters.Subscriber('/robot/rear_laser/scan', LaserScan)
    odom_sub = message_filters.Subscriber('/robot/robotnik_base_control/odom', Odometry)
    

    ts = message_filters.ApproximateTimeSynchronizer(
        [front_sub, rear_sub, odom_sub],
        30,
        0.2,
        allow_headerless=False
    )
    
    ts.registerCallback(laser_callback)

    # Wait for messages
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass