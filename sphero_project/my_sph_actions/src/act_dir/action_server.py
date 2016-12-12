#! /usr/bin/env python

import rospy
import actionlib
from my_sph_actions.msg import sphero_action_msgFeedback, sphero_action_msgResult, sphero_action_msgAction
from nav_msgs.msg import Odometry
from srv_dir.subscriber_odom import OdomTopicReader
from odometry_analysis import check_if_distance_reached


class RecordOdomClass(object):
    
    def __init__(self, goal_distance):
        """
        It starts an action Server. To test it was created correctly,
        just rostopic list and search for /rec_odom_as/...
        When launching, bare in mind that you should have :
        $catkin_make
        $source devel/setup.bash
        """
        # creates the action server
        self._as = actionlib.SimpleActionServer("/rec_odom_as", sphero_action_msgAction, self.goal_callback, False)
        self._as.start()
        
        # Create an object that reads from the topic Odom
        self._odom_reader_object = OdomTopicReader()
        
        # create messages that are used to publish result
        self._result   = sphero_action_msgResult()
        
        self._seconds_recording = 120 
        self._goal_distance = goal_distance
    
    
    
    def goal_callback(self, goal):
    
        success = True
        rate = rospy.Rate(4)    # Hz : times per second (higher is faster)
        
        for i in range(self._seconds_recording):
            rospy.loginfo("Recording Odom index=" + str(i))
            # check that preempt (cancelation) has not been requested by the action client
            if self._as.is_preempt_requested():
                rospy.logdebug('The goal has been cancelled/preempted')
                # the following line, sets the client in preempted state (goal cancelled)
                self._as.set_preempted()
                success = False
                # we end the action loop
                break
            
            else:   # builds the next feedback msg to be sent
                if not self.reached_distance_goal():
                    rospy.logdebug('Reading Odometry...')
                    self._result.result_odom_array.append(self._odom_reader_object.get_odomdata())
                else:
                    rospy.logwarn('Reached distance Goal')
                    # we end the action loop
                    break
            rate.sleep()
        
        # at this point, either the goal has been achieved (success==true)
        # or the client preempted the goal (success==false)
        # If success, then we publish the final result
        # If not success, we do not publish anything in the result
        if success:
            self._as.set_succeeded(self._result)
            # Clean the Result Variable
        
        self.clean_variables()
    
    
    
    def clean_variables(self):
        """
        Cleans variables for the next call
        """
        self._result   = sphero_action_msgResult()
    
    def reached_distance_goal(self):
        """
        Returns True if the distance moved from the first instance of recording till now has reached the self._goal_distance
        """
        return check_if_distance_reached(self._goal_distance, self._result.result_odom_array)
    
    
      
if __name__ == '__main__':
  rospy.init_node('record_odom_action_server_node')
  RecordOdomClass(goal_distance=2.0)
  rospy.spin()