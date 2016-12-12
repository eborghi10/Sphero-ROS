#! /usr/bin/env python

# To test independently: 
## rosrun my_sphero_topics publisher.py

## Imported libraries
import rospy
from geometry_msgs.msg import Twist

## Class definition
class CmdVelPub(object):
    def __init__(self):
        self._cmd_vel_pub = rospy.Publisher(
            '/cmd_vel',     # Topic to publish
            Twist,          # Topic message type
            queue_size = 1  # Size of the queue
        )
        # Underscore(_) after the global variable name
        self._twist_object = Twist()
        self.linearspeed = 0.2
        self.angularspeed = 0.5
        
    def move_robot(self, direction):
        '''
        if direction == "forwards":
            self._twist_object.linear.x = self.linearspeed
            self._twist_object.angular.z = 0.0
        elif direction == "right":
            self._twist_object.linear.x = 0.0
            self._twist_object.angular.z = self.angularspeed
        elif direction == "left":
            self._twist_object.linear.x = 0.0
            self._twist_object.angular.z = -self.angularspeed
        elif direction == "backwards":
            self._twist_object.linear.x = -self.linearspeed
            self._twist_object.angular.z = 0.0
        elif direction == "stop":
            self._twist_object.linear.x = 0.0
            self._twist_object.angular.z = 0.0
        else:
            pass
        '''
        if direction == "N" :
            self._twist_object.linear.x = self.linearspeed
            self._twist_object.angular.z = 0.0
        elif direction == "NE" :
            self._twist_object.linear.x = self.linearspeed
            self._twist_object.angular.z = self.angularspeed
        elif direction == "E" :
            self._twist_object.linear.x = 0.0
            self._twist_object.angular.z = self.angularspeed
        elif direction == "SE" :
            self._twist_object.linear.x = -self.linearspeed
            self._twist_object.angular.z = self.angularspeed
        elif direction == "S" :
            self._twist_object.linear.x = -self.linearspeed
            self._twist_object.angular.z = 0.0
        elif direction == "SW" :
            self._twist_object.linear.x = -self.linearspeed
            self._twist_object.angular.z = -self.angularspeed
        elif direction == "W" :
            self._twist_object.linear.x = 0.0
            self._twist_object.angular.z = -self.angularspeed
        elif direction == "NW" :
            self._twist_object.linear.x = self.linearspeed
            self._twist_object.angular.z = -self.angularspeed
        else :
            # Stop by default
            self._twist_object.linear.x = 0.0
            self._twist_object.angular.z = 0.0
        
        self._cmd_vel_pub.publish(self._twist_object)

## Main function
if __name__ == "__main__":
    rospy.init_node('cmd_vel__publisher_node')
    cmd_publisher_object = CmdVelPub()
    
    rate = rospy.Rate(1) # Once a time
    
    ctrl_c = False
    def shutdownhook():
        # WORKS BETTER THAN THE rospy.is_shut_down()
        global ctrl_c
        global twist_object
        global pub
        
        rospy.loginfo("shutdown time!")
        
        ctrl_c = True
        cmd_publisher_object.move_robot(direction="stop")
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        cmd_publisher_object.move_robot(direction="forwards")
        rate.sleep()