#!/usr/bin/env python3
import rospy
import sys, select, termios, tty

from rospy.rostime import Time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Getting key input from terminal
# No Blocking
def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    # Initialize Node with node name
    rospy.init_node('robot_control')
    # Assign node as a publisher to this topic
    pub = rospy.Publisher('/robot/robotnik_base_control/cmd_vel', Twist, queue_size=10)

    # linear velocity parameters
    speed = 0.0
    acc = 0.5
    brk = 0.1
    max_speed = 2.0
    
    # angular velocity parameters
    ang = 0.0
    max_ang = 2.0

    # keep track of time to update linear/angular veclocity
    lst_speed = Time.now().to_sec()
    lst_ang = Time.now().to_sec()

    # main loop
    # simulation is so slow anyway hh
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        # get key input
        key = getKey(0)

        # forward
        if key == 'w' or key == 'W':
            lst_speed = Time.now().to_sec()
            speed += acc
            speed = min(speed, max_speed)
        
        # backward
        if key == 's' or key == 'S':
            lst_speed = Time.now().to_sec()
            speed -= acc
            speed = max(speed, -max_speed)
        
        # right 
        if key == 'd' or key == 'D':
            lst_ang = Time.now().to_sec()
            ang = -max_ang
        
        # left
        if key == 'a' or key == 'A':
            lst_ang = Time.now().to_sec()
            ang = max_ang
        
        # stop angular
        if Time.now().to_sec() - lst_ang >= 0.1:
            ang = 0.0

        # decrease linear velocity
        if Time.now().to_sec() - lst_speed >= 0.1:
            lst_speed = Time.now().to_sec()
            if speed > 0:
                speed -= brk 
                speed = max(speed, 0)
            else:
                speed += brk
                speed = min(speed, 0)
        
        # break if ctrl+c is pressed
        if (key == '\x03'):
            break

        # set and publish the message
        twist = Twist()
        twist.linear.x = speed
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = ang
        pub.publish(twist)

        # rate.sleep()

    # stop the robot from moving before ending
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub.publish(twist)
        
if __name__ == '__main__':
    try:
        print('robot goz vrooooooooom')
        print('wasd -> /robot/robotnik_base_control/cmd_vel')
        settings = termios.tcgetattr(sys.stdin)
        main()
    except rospy.ROSInterruptException:
        print('u brok jostik')
        pass
