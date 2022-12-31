#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from modules_pkg.msg import LaserAndOdometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Bresenham's line drawing algorithm
def bresenham(x1, y1, x2, y2):
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
    # delta, sign
    dx, dy = np.abs(x2 - x1), np.abs(y2 - y1)
    # integers
    sx, sy = np.sign(x2 - x1),  np.sign(y2 - y1)
    if dy > dx:
        dx, dy = dy, dx
        swp = True
    else:
        swp = False
    #
    A = 2 * dy
    B = 2 * (dy - dx)
    E = 2 * dy - dx
    #
    # Output pixels
    xs, ys = [x1], [y1]
    #
    x, y = x1, y1
    # point (x2,y2) must not be included
    for i in range(1, dx):
        if E < 0:
            if swp: y += sy
            else: x += sx
            E = E + A
        else:
            y += sy
            x += sx
            E = E + B
        # mark output pixels
        xs.append(x)
        ys.append(y)
    #
    return zip(xs, ys)

# hit/miss
# p free -> 0.3 --> -0.8472978603872036 
# p occ  -> 0.9 --> 2.1972245773362196 
def log_odds(p):
    return np.log(p / (1 - p))

# ratio -> probabilty
def retrieve_p(l):
    return 1 - 1 / (1 + np.exp(l))

# Get theta from odometry
def get_odom_orientation(odom):
    # Docs say this is in quaternion form
    q = odom.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
    # limit
    while yaw < 0:
        yaw += 2 * np.pi
    while yaw >= 2 *np.pi:
        yaw -= 2 * np.pi
    #
    return yaw

# Odometry -> State
def get_odom_state(odom):
    x_odom, y_odom = odom.pose.pose.position.x, odom.pose.pose.position.y
    theta_odom = get_odom_orientation(odom)
    return [x_odom, y_odom, theta_odom]

# Hit/Miss parameters
P_PRIOR = 0.5       # Prior occupancy probability
P_OCC   = 0.9       # Probability that cell is occupied with total confidence
P_FREE  = 0.3       # Probability that cell is free with total confidence
WIDTH   = (-35, 35) # x axis limits
HEIGHT  = (-35, 35) # y axis limits
RES     = 0.5       # Grid resolution in [m]

# rasterize
def rasterize(x, y):
    return int((x-WIDTH[0])/RES), int((y-HEIGHT[0])/RES)

# Grid
x = np.arange(start = WIDTH[0], stop = WIDTH[1] + RES, step = RES)
y = np.arange(start = HEIGHT[0], stop = HEIGHT[1] + RES, step = RES)
# probability matrix in log-odds scale:
grid = np.full(shape = (len(x), len(y)), fill_value = log_odds(P_PRIOR))
# Pulbish a map
pub = rospy.Publisher('/map_gamda', OccupancyGrid, queue_size=10)

# CALLLLLLLLLLLLLLLLLLBACK
def map_callback(sensors_msg):
    #
    laser = sensors_msg.laser
    odom = sensors_msg.odom
    # Get distances and angles of laser scan
    dist_ang = [(d, laser.angle_min +i*laser.angle_increment)
            for i, d in enumerate(laser.ranges)
            if d < laser.range_max and d > laser.range_min]
    # get position and orientation from odometry
    x, y, th = get_odom_state(odom)
    # x, y, tot 
    dist = [(d * np.cos(theta +th), d * np.sin(theta +th), d)
            for d, theta in dist_ang]
    # absolute positions for obstacles, and total distance
    obst = [(x +dx, y +dy, d) for dx, dy, d in dist]
    # rasterize
    x1, y1 = rasterize(x, y)
    # Loop over obstacles
    for (px, py, d) in obst:
        #
        x2, y2 = rasterize(px, py)
        # Miss for cells along the path
        for (xi, yi) in bresenham(x1, y1, x2, y2):
            grid[xi][yi] += log_odds(P_FREE)
        # HIT
        if d < laser.range_max:
            grid[x2][y2] += log_odds(P_OCC)
    #
    # PUBLISH
    occ_grid = OccupancyGrid()
    occ_grid.header.stamp = rospy.Time.now()
    occ_grid.header.frame_id = 'robot_map'
    occ_grid.data = (retrieve_p(grid)*100).astype(int).ravel().tolist()
    occ_grid.info.resolution = RES
    occ_grid.info.width = grid.shape[0]
    occ_grid.info.height = grid.shape[1]
    occ_grid.info.origin.position.x = WIDTH[0]
    occ_grid.info.origin.position.y = HEIGHT[0]
    # Quaternion from euler
    q = quaternion_from_euler(np.pi, 0, np.pi/2)
    occ_grid.info.origin.orientation.x = q[0]
    occ_grid.info.origin.orientation.y = q[1]
    occ_grid.info.origin.orientation.z = q[2]
    occ_grid.info.origin.orientation.w = q[3]
    # print(max(occ_grid.data))
    pub.publish(occ_grid)


def main():
    # Initialize Node with node name
    rospy.init_node('map')
    # Read Sensors
    sub = rospy.Subscriber('/sensors_gamda', LaserAndOdometry, map_callback)
    # Wait for messages
    rospy.spin()

if __name__ == '__main__':
    try:
        print('arkab Giza mneen?')
        print('/sensors_gamda --> /map_gamda')
        main()
    except rospy.ROSInterruptException:
        print('howa alak feen?')
        pass

