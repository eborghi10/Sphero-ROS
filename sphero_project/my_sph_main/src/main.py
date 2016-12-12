#! /usr/bin/env python

# https://github.com/mcarth3/cs470/blob/6fec3c91d2a25e65084cc51cdc9b4b15e1d32820/Project%201/Team%20Lambda%20Code%20and%20Yutube/sphero_intrude_gui.py

import rospy
import actionlib
from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import Point
from my_sph_actions.msg import sphero_action_msgGoal, sphero_action_msgFeedback, sphero_action_msgResult, sphero_action_msgAction
from srv_dir.publisher import CmdVelPub
from pid_control import PID
from act_dir.odometry_analysis import OdometryAnalysis, check_if_distance_reached

class ControlSphero(object):
    def __init__(self, goal_distance):
        self._goal_distance = goal_distance
        self._machine_state = 0
        self.init_direction_service_client()
        self.init_rec_odom_action_client()
        self.init_move_sphero_publisher()
#        self.init_pid_controller()
        
    def init_direction_service_client(self, service_name = "/crash_direction_service"):
        rospy.loginfo('Waiting for Service Server')
        rospy.wait_for_service(service_name)
        rospy.loginfo('Service Server Found...')
        # create the connection to the service
        self._direction_service = rospy.ServiceProxy(service_name, Trigger)
        self._request_object = TriggerRequest()
        
    def init_pid_controller():
        pid_object = PID()
        
    def make_direction_request(self):
        # send through the connection the name of the object to be deleted by the SERVICE
        result = self._direction_service(self._request_object)
        
        if result.success == False :
            return result.message
            # If I didn't crash, continues in the same direction
        else :
            if result.message == "left" :
                return "right"
            elif result.message == "front" :
                return "back"
            elif result.message == "right" :
                return "left"
            elif result.message == "back" :
                return "front"
#            else :
#                rospy.logwarn('Crash detection error')
    
    def init_rec_odom_action_client(self):
        self._take_initial_odom_measurement = True
        self._odom_movement = Point(0.0, 0.0, 0.0)
        self._rec_odom_action_client = actionlib.SimpleActionClient('/rec_odom_as', sphero_action_msgAction)
        # waits until the action server is up and running
        rospy.loginfo('Waiting for action Server')
        self._rec_odom_action_client.wait_for_server()
        rospy.loginfo('Action Server Found...')
        self._rec_odom_action_goal = sphero_action_msgGoal()
    
    def send_goal_to_rec_odom_action_server(self):
        self._rec_odom_action_client.send_goal(self._rec_odom_action_goal, feedback_cb=self.rec_odom_feedback_callback)
        
    def rec_odom_feedback_callback(self,feedback):
        rospy.loginfo("Rec Odom Feedback feedback ==>" + str(feedback))
        '''
        if self._take_initial_odom_measurement == True :
            self._initial_odometry = feedback.pose.pose.position
            self._take_initial_odom_measurement = False
        else :
            self._odom_movement = abs(feedback.pose.pose.position - self._initial_odometry)
        '''
    
    def rec_odom_finished(self):
        
        DONE = 2
        
        has_finished = ( self._rec_odom_action_client.get_state() >= DONE )
        
        return has_finished
    
    def get_result_rec_odom(self):
        return self._rec_odom_action_client.get_result()
        
    def init_move_sphero_publisher(self):
        self._cmdvelpub_object = CmdVelPub()

    def move_sphero(self, direction):
        self._cmdvelpub_object.move_robot(direction)

    def distance_achieve(self, odom_result_array):
        return check_if_distance_reached(self._goal_distance, odom_result_array)
        
    def get_machine_state(self) :
        return self._machine_state
        
    def step_machine_state(self) :
        self._machine_state += 1
        # Should go to zero when == 4 ?
        
    def get_fsm_direction(self, state) :
        if state == 1 :
            rospy.loginfo('> State #1')
            return "NW"
        elif state == 2 :
            rospy.loginfo('> State #2')
            return "W"
        elif state == 3 :
            rospy.loginfo('> State #3')
            return "N"
        else :
#            rospy.logwarn('Forbidden FSM state')
            return "N"
'''            
    def reached_distance(self, odom_distance) :
        cond = self._odom_movement >= odom_distance
        if cond :
            _take_initial_odom_measurement = True
        return cond
'''
rospy.init_node("sphero_main_node", log_level=rospy.INFO)
controlsphero_object = ControlSphero(goal_distance=0.5)
rate = rospy.Rate(10)

'''
- VER QUE SE CUMPLA LA ODOMETRIA USANDO LA ACCION
- CADA VEZ QUE SE CUMPLA:
  * FRENAR EL ROBOT
  * CAMBIAR EL "GOAL" -> send_goal_to_rec_odom_action_server()
- REVISAR LA ESTRUCTURA DE LA ACCION. VER QUE FUNCIONEN LOS CAMBIOS DE "GOAL"
'''
while True == True :
    controlsphero_object.send_goal_to_rec_odom_action_server()
    
    state = controlsphero_object.get_machine_state()
    
    if state == 4 :
        # Stops the robot and exit
        controlsphero_object.move_sphero("stop")
        break   
    
    direction_to_go = controlsphero_object.get_fsm_direction(state)
    
    while not controlsphero_object.rec_odom_finished():
        controlsphero_object.move_sphero(direction_to_go)
        rate.sleep()
        
    odom_result = controlsphero_object.get_result_rec_odom()
    odom_result_array = odom_result.result_odom_array
    
    if controlsphero_object.distance_achieve(odom_result_array) :
        # Only change its state if the odometry is achieved
        controlsphero_object.step_machine_state()

'''
if controlsphero_object.got_out_maze(odom_result_array):
    rospy.loginfo("Out of Maze")
else:
    rospy.loginfo("In Maze")
'''
rospy.loginfo("Sphero Maze test Finished")