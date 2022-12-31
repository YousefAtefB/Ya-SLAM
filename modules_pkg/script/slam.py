#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from modules_pkg.msg import LaserAndOdometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# SLAM using the EKF
# ROS

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
RES     = 0.3       # Grid resolution in [m]

# rasterize
def rasterize(x, y):
    return int((x-WIDTH[0])/RES), int((y-HEIGHT[0])/RES)

# Grid
x = np.arange(start = WIDTH[0], stop = WIDTH[1] + RES, step = RES)
y = np.arange(start = HEIGHT[0], stop = HEIGHT[1] + RES, step = RES)
# probability matrix in log-odds scale:
grid = np.full(shape = (len(x), len(y)), fill_value = log_odds(P_PRIOR))

# linear motion model
def g_linear(state, action):
    # state: [x, y, theta]
    # action: [v, w]
    x, y, theta = state
    v, w = action
    #
    return np.array([x + v * np.cos(theta), y + v * np.sin(theta), theta + w])

# motion model
def g(state, action):
    # state: [x, y, theta]
    # action: [v, w]
    x, y, theta = state
    v, w = action
    if w == 0:
        return g_linear(state, action)
    #
    xn = x + (-v/w * np.sin(theta) + v/w * np.sin(theta + w))
    yn = y + (v/w * np.cos(theta) - v/w * np.cos(theta + w))
    thetan = theta + w
    #
    while thetan < 0:
        thetan += 2 * np.pi
    while thetan >= 2 * np.pi:
        thetan -= 2 * np.pi
    #
    return np.array([xn, yn, thetan])

# jacobian with respect to state
def jacobian_g_state(state, action, delta):
    # state: [x, y, theta]
    # action: [v, w]
    x, y, theta = state
    v, w = action
    return np.array([
        [1, 0, -v *delta* np.sin(theta)],
        [0, 1, v *delta* np.cos(theta)],
        [0, 0, 1]])

# jacobian with respect to action
def jacobian_g_action(state, action):
    # state: [x, y, theta]
    # action: [v, w]
    x, y, theta = state
    v, w = action
    return np.array([
        [np.cos(theta), 0],
        [np.sin(theta), 0],
        [0, 1]])

# assign as publisher to slam_loc topioc
pub_loc = rospy.Publisher('/slam_loc', PoseStamped, queue_size=10)
pub_map = rospy.Publisher('/slam_map', OccupancyGrid, queue_size=10)

# previous time
tim = 0
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

    state = g_linear(state, action)
    #
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
    pub_loc.publish(pose)
    #
    #
    # MAPPING
    # get position and orientation from odometry
    x, y, th = state
    # Get distances and angles of laser scan
    dist_ang = [(d, laser.angle_min +i*laser.angle_increment)
            for i, d in enumerate(laser.ranges)
            if d < laser.range_max and d > laser.range_min]
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
    pub_map.publish(occ_grid)


def main():
    # Initialize Node with node name
    rospy.init_node('slam')
    # Assign node as a subscriber to sensors topic
    sub = rospy.Subscriber('/sensors_gamda', LaserAndOdometry, slam_callback)
    # Wait for messages
    rospy.spin()

if __name__ == '__main__':
    try:
        print('ya slaaaaaaaaaaaaaaaaaaaaaaaaaaaaam')
        main()
    except rospy.ROSInterruptException:
        pass
