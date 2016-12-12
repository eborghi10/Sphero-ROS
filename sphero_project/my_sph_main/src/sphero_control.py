#! /usr/bin/env python

import rospy
#import time
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Vector3
from sensor_msgs.msg import JointState

## PID CONTROLLER TO USE WITH A PUBLISHER
# https://syrotek.felk.cvut.cz/course/ROS_CPP_INTRO/exercise/ROS_CPP_PID
# $ rosrun my_sph_main sphero_control.py
# -----
# https://syrotek.felk.cvut.cz/course/ROS_CPP_INTRO/exercise/ROS_CPP_BRAITENBERG
# https://syrotek.felk.cvut.cz/course/ROS_CPP_INTRO/exercise/ROS_CPP_WALLFOLLOWING
# https://syrotek.felk.cvut.cz/course/ROS_CPP_INTRO/exercise/ROS_CPP_DEAD
# https://syrotek.felk.cvut.cz/course/ROS_CPP_INTRO/exercise/ROS_CPP_TRAJ



class SpheroControl(object):
    
    def __init__(self, set_point, kv=1.0, max_vel=0.5):
        # Parameters
        self._K = kv
        self._set_point = set_point
        self._last_error = 0.0
        self._current_error = 0.0
        self._tol_error = 1e-1
        self._max_linear_velocity = max_vel
        self._direction_of_movement = ""
        self._sphero_velocity = 0.606 # rev/s
        
    def correct_distance(self, distance):
        if distance > 1.0:
            return 1.0
        elif distance < -1.0:
            return -1.0
        else:
            return distance
            
    def normalize_angle_positive(self, angle):
        return math.fmod(math.fmod(angle, 2.0*math.pi) + 2.0*math.pi, 2.0*math.pi);
        
        
    def input_error(self, current_value):
        # Distance from "_set_point"
    
        x_1 = set_points[step][0]+XX#self._set_point.position.x
        y_1 = set_points[step][1]+YY#self._set_point.position.y
 
        x = x_1 - x_0
        y = y_1 - y_0
        
        distance = math.sqrt(math.pow(x,2.0) + math.pow(y,2.0))
        self._current_error = self.correct_distance(distance)
        '''
        angle = math.atan2(y,x)
        
        if angle == 0:
            self._direction_of_movement = "forward"
        '''
        rospy.loginfo(
            '[' + str(x_0) + ',' + str(y_0) + ']' + '  ' +
            '[' + str(x_1) + ',' + str(y_1) + ']'
        )
        
        rospy.logwarn("> Current Error = " + \
                    str(self._current_error) + '  ' + \
                    str(self._direction_of_movement))
        

    def calculate_movement(self):
        
        linear_vel = self._max_linear_velocity / (abs(self._current_error) + 1)**0.5
        
        if self._last_error < self._current_error:
            angular_vel = self._last_error - self._current_error
        else:
            angular_vel = 0.0
        '''
        difference = self._current_error - self._last_error
        
        if self._direction_of_movement == "":
            angular_vel = 0.0
            self._direction_of_movement = "right"
            
        elif self._current_error <= self._last_error:
            # Mantengo la direccion
            if self._direction_of_movement == "right":
                angular_vel = 0.5#self._K * self._current_error
            else:
                angular_vel = -0.5#self._K * (-self._current_error)
        else:
            if self._direction_of_movement == "right":
                angular_vel = -0.5#self._K * (-self._current_error)
                self._direction_of_movement = "left"
            else:
                angular_vel = 0.5#self._K * self._current_error
                self._direction_of_movement = "right"
        '''
        self._last_error = self._current_error
        
        return linear_vel, angular_vel


    def update(self, current_value):
        
        self.input_error(current_value)
        
        return self.calculate_movement()
        
        
    def close_enough(self):
        
        if abs(self._current_error) < self._tol_error:
            return True
        
        return False
        
        
if __name__ == "__main__":
    
    global X,Y,XX,YY
    X = 0
    Y = 0
    
    def odom_callback(msg):
        global x_0, y_0, theta
        x_0 = msg.pose.pose.position.x
        y_0 = msg.pose.pose.position.y
    
    def joint_state_callback(msg):
        
        X = msg.position[0] * 0.1
        Y = msg.position[1] * 0.1
    
    # Test code
    rospy.init_node('pid_test_node', log_level = rospy.DEBUG)
    
    final_pose = Pose()
    
    global pub
    pub = rospy.Publisher(
        'cmd_vel',
        Twist,
        queue_size = 1
    )
    '''
    sub = rospy.Subscriber(
        '/odom', 
        Odometry, 
        odom_callback
    )
    '''
    sub = rospy.Subscriber(
        '/joint_states',
        JointState,
        joint_state_callback
    )
    odomdata = Odometry()   # Odometry object
    message = Twist()       # Twist object
    
    rate = rospy.Rate(8)
    
    ctrl_c = False
    def shutdownhook():
        global ctrl_c
        global message
        
        rospy.loginfo("> Stopping Robot")
        
        ctrl_c = True
        message.linear.x = 0.0
        message.angular.z = 0.0
        pub.publish(message)
    
    rospy.on_shutdown(shutdownhook)
    global set_points
    set_points = [[-0.35,0.0],[-0.35,-0.25],[-0.35,-0.45],[0.0,-0.25],[0.0,-0.5],[0.0,-1.0]]
    global step
    step = 0
    rospy.logerr('> Initial set point (' + str(step) + ')')
    final_pose.position.x = set_points[step][0]
    final_pose.position.y = set_points[step][1]
    control_object = SpheroControl(
        final_pose,     # Final position
        0.8,           # Gain constant parameter
        0.01            # Maximum velocity
    )
    tiempo_anterior = 0.0
    min_error = 1e6
    revolucion = False
    mover_robot = False
    _vel_giro = 0.5
    
    XX = X 
    YY = Y
    
    rospy.logerr(str(X) + '[]'  + str(Y))
    
    while not ctrl_c:
        ##
# TODO: SE NECESITA TIEMPO_ANTERIOR??
# LOS ANGULOS ESTAN MULTIPLICADOS POR 2PI?
        tiempo_actual = rospy.get_time()
        if revolucion == False:
            if tiempo_anterior == 0.0:
                tiempo_anterior = tiempo_actual
                tiempo_inicial = tiempo_actual
            
            pub.publish(Twist(
                Vector3(0.01,0.0,0.0), Vector3(0.0,0.0,_vel_giro)))
            
            dT = tiempo_actual - tiempo_inicial
            
            _x_0 = X#x_0
            _y_0 = Y#y_0
            _x_1 = final_pose.position.x
            _y_1 = final_pose.position.y
            
            actual_distance_error = math.sqrt(math.pow((_x_1-_x_0),2.0) + math.pow((_y_1-_y_0),2.0))
            angulo_actual = control_object._sphero_velocity * _vel_giro * dT
            rospy.logdebug('[err=' + str(actual_distance_error) + '  ang=' + str(angulo_actual*180/math.pi) + ']')
            if actual_distance_error < min_error:
                min_angulo = angulo_actual
                min_error = actual_distance_error
                dT_min = dT
                
            if angulo_actual >= 2.0*math.pi:
                revolucion = True
                tiempo_rev = dT
                tiempo_inf = dT_min * 0.8
                tiempo_sup = dT_min * 1.2
                rospy.logdebug('> REVOLUTION DONE [t_inf=' + str(tiempo_inf) + '- t_sup=' + str(tiempo_sup) + ']')
        else: 
        # revolucion == True
            otro_tiempo = tiempo_actual - tiempo_inicial - tiempo_rev
            #rospy.logdebug('OT : ' + str(otro_tiempo))
            if (otro_tiempo > tiempo_inf) and (otro_tiempo < tiempo_sup):
                # Se frena el robot
                pub.publish(Twist(
                    Vector3(0,0,0), Vector3(0,0,0)))
                mover_robot = True
                rospy.logdebug('Parando robot > ang_ideal=' + str(min_angulo*180/math.pi) + ' (ang_real=' + str(control_object._sphero_velocity*_vel_giro*dT_min*180/math.pi) + ')')
                _x_0 = X#odomdata.pose.pose.position.x
                _y_0 = Y#odomdata.pose.pose.position.y
                rospy.logdebug('err= ' + str(math.sqrt(math.pow((_x_1-_x_0),2.0) + math.pow((_y_1-_y_0),2.0))))
        
        tiempo_anterior = tiempo_actual
        
        if mover_robot == True:
            lin, ang = control_object.update(odomdata.pose.pose.position)
            message.linear.x = lin
            message.angular.z = ang
            
            rospy.logdebug("> Lineal Velocity = " + str(lin))
            rospy.logdebug("> Angular Velocity = " + str(ang))
            pub.publish(message)
            
            if control_object.close_enough():
                step += 1
                tiempo_anterior = 0.0
                min_error = 1e6
                revolucion = False
                mover_robot = False
                rospy.logerr('> Changing set point (' + str(step) + ')')
                final_pose.position.x = set_points[step][0] + XX
                final_pose.position.y = set_points[step][1] + YY
                control_object = SpheroControl(
                    final_pose, 
                    0.8, 
                    0.01
                )
                
            if step > 3:
                ctrl_c = True
        
        rate.sleep()
        
        