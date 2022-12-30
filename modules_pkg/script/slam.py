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

P_PRIOR = 0.5	# Prior occupancy probability
P_OCC = 0.9	    # Probability that cell is occupied with total confidence
P_FREE = 0.3	# Probability that cell is free with total confidence 
WIDTH = (-35, 35) # x axis limits
HEIGHT = (-35, 35) # y axis limits
RES = 0.5      # Grid resolution in [m]

def bresenham(x1, y1, x2, y2):
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

			if interchange:

				y += s_y
			
			else:

				x += s_x

			E = E + A

		else:

			y += s_y
			x += s_x
			E = E + B

		# mark output pixels
		X_bres.append(x)
		Y_bres.append(y)

	return zip(X_bres, Y_bres)

def log_odds(p):
	return np.log(p / (1 - p))

def retrieve_p(l):
	return 1 - 1 / (1 + np.exp(l))

x = np.arange(start = WIDTH[0], stop = WIDTH[1] + RES, step = RES)
y = np.arange(start = HEIGHT[0], stop = HEIGHT[1] + RES, step = RES)
# probability matrix in log-odds scale:
grid = np.full(shape = (len(x), len(y)), fill_value = log_odds(P_PRIOR))

def get_odom_orientation(odom):
    q = odom.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
    if yaw < 0:
        yaw += 2 * np.pi  # 0->360 degrees >> 0->2pi
    return yaw


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
pub_loc = rospy.Publisher('/slam_loc', PoseStamped, queue_size=10)
pub_map = rospy.Publisher('/slam_map', OccupancyGrid, queue_size=10)

#
tim = 0
delta = 0
# x, y, theta
state = np.array([0, 0, 0])
# Covariance
# x, y, theta
state_cov = np.diag([0.1, 0.1, 0.1])

# SLAM
def slam_callback(sensors_msg):
    #
    global state
    global state_cov
    global tim
    global delta

    laser = sensors_msg.laser
    odom = sensors_msg.odom

    ntim = laser.header.stamp.secs + laser.header.stamp.nsecs * 1e-9
    delta = ntim - tim
    tim = ntim
    # delta = 1
    # print(delta)

    action = [odom.twist.twist.linear.x, odom.twist.twist.angular.z]

    action[0] *= delta
    action[1] *= delta

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
    # print(pose)
    #
    pub_loc.publish(pose)

    # get position and orientation from odometry
    x_odom, y_odom, theta_odom = state

    # debug()

    # get distances and angles of laser scan
    distances, angles = [], []
    ang = laser.angle_min
    for i in range(len(laser.ranges)):
        if laser.ranges[i] < laser.range_max and laser.ranges[i] > laser.range_min:
            d = laser.ranges[i]
            distances.append(d)
            angles.append(ang)
        ang += laser.angle_increment

    # project the laser distances
    distances_x, distances_y = [], []
    for (d, ang) in zip(distances, angles):
        distances_x.append(x_odom + d * np.cos(ang + theta_odom))
        distances_y.append(y_odom + d * np.sin(ang + theta_odom))

    x1, y1 = int((x_odom - WIDTH[0]) / RES), int((y_odom - HEIGHT[0]) / RES)

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
    # print(max(occ_grid.data))
    pub_map.publish(occ_grid)


def main():
    # Initialize Node with node name
    rospy.init_node('slam')
    # Assign node as a subscriber to sensors topic
    sub = rospy.Subscriber('/sensors_topic', LaserAndOdometry, slam_callback)
    # Wait for messages
    rospy.spin()

if __name__ == '__main__':
    try:
        print('ya slaaaaaaaaaaaaaaaaaaaaaaaaaaaaam')
        main()
    except rospy.ROSInterruptException:
        pass
