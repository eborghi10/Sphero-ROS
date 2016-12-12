#! /usr/bin/env python

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from math import fmod, pi

# https://github.com/ilsemae/sphero_ball/blob/32854a8133fd55a15b4115b6bc45c0fd3ac66204/sphero_ball_real/src/leader_driver.py

sign = lambda x: math.copysign(1, x)

def odom_callback(msg):
    
    global x
    global y
    global theta
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    theta = fmod( msg.pose.orientation.w, 2*pi)

def go_to(pose_x, pose_y):
    # Go To Goal
    
    
    
if __name__ == '__main__':
    
    global pub
    
    pub = rospy.Publisher(
        'cmd_vel',
        Twist,
        queue_size = 1
    )
    
    sub = rospy.Subscriber(
        'odom',
        Odometry,
        odom_callback
    )
    
    go_to(0.5,-0.5)