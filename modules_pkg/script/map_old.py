#!/usr/bin/env python3

import numpy as np

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


def bresenham2(x1, y1, x2, y2):
	"""
	Bresenham's line drawing algorithm - working for all 4 quadrants!
	"""
	
	# Output pixels
	X_bres = []
	Y_bres = []

	x = x1
	y = y1
	
	delta_x = np.abs(x2 - x1)
	delta_y = np.abs(y2 - y1)
	
	s_x = np.sign(x2 - x1)
	s_y = np.sign(y2 - y1)

	if delta_y > delta_x:

		delta_x, delta_y = delta_y, delta_x
		interchange = True

	else:

		interchange = False

	A = 2 * delta_y
	B = 2 * (delta_y - delta_x)
	E = 2 * delta_y - delta_x

	# mark output pixels
	X_bres.append(x)
	Y_bres.append(y)

	# point (x2,y2) must not be included
	for i in range(1, delta_x):
		if E < 0:
			if interchange: y += s_y
			else: x += s_x
			E = E + A
		else:
			y += s_y
			x += s_x
			E = E + B

		# mark output pixels
		X_bres.append(x)
		Y_bres.append(y)

	return zip(X_bres, Y_bres)

import rospy


def log_odds(p):
	return np.log(p / (1 - p))

def retrieve_p(l):
	return 1 - 1 / (1 + np.exp(l))

# -----------------------------------------------------------------------

import numpy as np
from math import pi

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from modules_pkg.msg import LaserAndOdometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

P_PRIOR = 0.5	# Prior occupancy probability
P_OCC = 0.9	    # Probability that cell is occupied with total confidence
P_FREE = 0.3	# Probability that cell is free with total confidence 
WIDTH = (-35, 35) # x axis limits
HEIGHT = (-35, 35) # y axis limits
RES = 0.5      # Grid resolution in [m]

x = np.arange(start = WIDTH[0], stop = WIDTH[1] + RES, step = RES)
y = np.arange(start = HEIGHT[0], stop = HEIGHT[1] + RES, step = RES)
# probability matrix in log-odds scale:
grid = np.full(shape = (len(x), len(y)), fill_value = log_odds(P_PRIOR))

# Assign node as a publisher to this topic
pub = rospy.Publisher('/mapping_topic', OccupancyGrid, queue_size=10)

def get_odom_orientation(odom):
    q = odom.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
    if yaw < 0:
        yaw += 2 * np.pi  # 0->360 degrees >> 0->2pi
    while yaw >= 2 *np.pi:
        yaw -= 2 * np.pi
    return yaw

# def debug():
#     pub = rospy.Publisher('/map_topic', LaserScan, queue_size=10)
#     distances = []
#     ang = laser.angle_min
#     for i in range(50):
#         distances.append(laser.ranges[i])
#         ang += laser.angle_increment
#     ang -= laser.angle_increment
#     laser.angle_max = ang
#     laser.ranges = distances
#     pub.publish(laser) 

# rasterize
def rasterize(x, y):
    return np.round((x - WIDTH[0]) / RES).astype(int), np.round((y - HEIGHT[0]) / RES).astype(int)

def map_callback(sensors_msg):
    
    laser = sensors_msg.laser
    odom = sensors_msg.odom

    # get distances and angles of laser scan
    distances, angles = [], []
    ang = laser.angle_min
    for i in range(len(laser.ranges)):
        if laser.ranges[i] < laser.range_max and laser.ranges[i] > laser.range_min:
            d = laser.ranges[i]
            distances.append(d)
            angles.append(ang)
        ang += laser.angle_increment

    # print(max(distances))

    # get position and orientation from odometry
    x_odom, y_odom = odom.pose.pose.position.x, odom.pose.pose.position.y
    theta_odom = get_odom_orientation(odom)

    # debug()

    distances_x, distances_y = [], []
    for (d, ang) in zip(distances, angles):
        distances_x.append(x_odom + d * np.cos(ang + theta_odom))
        distances_y.append(y_odom + d * np.sin(ang + theta_odom))

    x1, y1  = rasterize(x_odom, y_odom)
    # x1, y1 = int((x_odom - WIDTH[0]) / RES), int((y_odom - HEIGHT[0]) / RES)

    for (d_x, d_y, d) in zip(distances_x, distances_y, distances):

        x2, y2 = int((d_x - WIDTH[0]) / RES), int((d_y - HEIGHT[0]) / RES)
        
        # draw a discrete line of free pixels, [robot position -> laser hit spot)
        for (x_bres, y_bres) in bresenham(x1, y1, x2, y2):
            grid[x_bres][y_bres] += log_odds(P_FREE)

        # mark laser hit spot as ocuppied (if exists)
        if d < laser.range_max:
            grid[x2][y2] += log_odds(P_OCC)

    occ_grid = OccupancyGrid()
    occ_grid.header.stamp = rospy.Time.now()
    occ_grid.header.frame_id = 'robot_map'
    occ_grid.data = (retrieve_p(grid)*100).astype(int).ravel().tolist()
    occ_grid.info.resolution = RES
    occ_grid.info.width = grid.shape[0]
    occ_grid.info.height = grid.shape[1]
    occ_grid.info.origin.position.x = WIDTH[0]
    occ_grid.info.origin.position.y = HEIGHT[0]
    # occ_grid.info.origin.orientation.w = 0.0 
    # occ_grid.info.origin.orientation.x, occ_grid.info.origin.orientation.y, occ_grid.info.origin.orientation.z, occ_grid.info.origin.orientation.w = 0, 0, 0, 0
    # occ_grid.info.origin.orientation.z = quaternion_from_euler()
    occ_grid.info.origin.orientation.x, occ_grid.info.origin.orientation.y, occ_grid.info.origin.orientation.z, occ_grid.info.origin.orientation.w = quaternion_from_euler(pi, 0, +pi/2)
    # print(occ_grid)
    print(max(occ_grid.data))
    pub.publish(occ_grid)



def main():
    # Initialize Node with node name
    rospy.init_node('map')

    # Assign node as a subscriber to hello topic
    # sub = rospy.Subscriber('/sensors_topic', LaserAndOdometry, map_callback)
    sub = rospy.Subscriber('/sensors_gamda', LaserAndOdometry, map_callback)

    # Wait for messages
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

