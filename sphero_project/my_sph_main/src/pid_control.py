#! /usr/bin/env python

import rospy
#import time
import math
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

## PID CONTROLLER TO USE WITH A PUBLISHER
# https://syrotek.felk.cvut.cz/course/ROS_CPP_INTRO/exercise/ROS_CPP_PID
# $ rosrun my_sph_main pid_control.py
# -----
# https://syrotek.felk.cvut.cz/course/ROS_CPP_INTRO/exercise/ROS_CPP_BRAITENBERG
# https://syrotek.felk.cvut.cz/course/ROS_CPP_INTRO/exercise/ROS_CPP_WALLFOLLOWING
# https://syrotek.felk.cvut.cz/course/ROS_CPP_INTRO/exercise/ROS_CPP_DEAD
# https://syrotek.felk.cvut.cz/course/ROS_CPP_INTRO/exercise/ROS_CPP_TRAJ

global x_0
global y_0

x_0 = 0
y_0 = 0

class PID(object):
    
    def __init__(self, set_point, kp=1.0, ki=0.0, kd=0.0):
        # Parameters
        self._Kp = kp
        self._Ki = ki
        self._Kd = kd
        self._set_point = set_point
        self._acum_pos_error = 0     # Error tracking
        self._acum_ang_error = 0
        self._prev_time = 0
        self._current_distance_error = 0
        self._current_heading_error = 0
        self._tol_distance_error = 1e-1
        self._tol_heading_error = 3.0 / 180.0 * math.pi
        self._current_heading = -3.0 * math.pi / 4.0   # Viewing the orientation in the simulation
        self._Vx = 0
        self._Vy = 0
        self._take_vel_samples = True
        self._num_samples = 0
        
    def setVel_x(self, value):
        self._Vx += value
        
    def setVel_y(self, value):
        self._Vy += value
        
    def add_samples(self, value_x, value_y):
        
        self.setVel_x(value_x)
        self.setVel_y(value_y)
        self._num_samples += 1
        
#        rospy.loginfo('[Vx, Vy]=[' + str(value_x) + ', ' + str(value_y) + ']')
        
    def get_samples(self):
        return self._take_vel_samples
        
    def clear_vel_variables(self):
        self._Vx = 0
        self._Vy = 0
        self._num_samples = 0
        self._take_vel_samples = True
        
    def correct_distance(self, distance):
        if distance > 1.0:
            return 1.0
        elif distance < -1.0:
            return -1.0
        else:
            return distance
    
    def correct_angle(self, angle):
        if angle < -math.pi:
            angle += 2*math.pi
        elif angle > math.pi:
            angle -= 2*math.pi
        return angle
        
    def close_enough(self):
        pos = abs(self._current_distance_error)
        ang = abs(self._current_heading_error)
        
        if pos < self._tol_distance_error:
            self._current_distance_error = 0.0
            
        if ang < self._tol_heading_error:
            self._current_heading_error = 0.0
            
    def heading(self, X_t, Y_t):
        
        x_V = x_0 - X_t
        y_V = y_0 - Y_t
        
        return math.atan2(y_V, x_V)
        
    def input_error(self):
        # Distance from "_set_point"
        
        current_time = rospy.get_time()
        
        if self._prev_time == 0:
            self._prev_time = current_time
            
        elapsed_time = current_time - self._prev_time
        rospy.loginfo('PID >Elapsed Time = ' + str(elapsed_time))
        
        # Update the variable with the actual time
        self._prev_time = current_time
        
        x_1 = self._set_point.position.x
        y_1 = self._set_point.position.y
 
        if self._num_samples != 0:
            Vx_med = self._Vx / self._num_samples
            Vy_med = self._Vy / self._num_samples
        else:
            Vx_med = 0
            Vy_med = 0
            
        self._take_vel_samples = False
        
        X_tilde = Vx_med * elapsed_time
        Y_tilde = Vy_med * elapsed_time
        
        phi = self.heading(X_tilde, Y_tilde)
 
#        phi = self._current_heading + (elapsed_time * message.angular.z)
#        phi = self.correct_angle(phi)
        
        x = x_1 - x_0
        y = y_1 - y_0
        
        distance = math.sqrt(math.pow(x,2.0) + math.pow(y,2.0))
        self._current_distance_error = self.correct_distance(distance)
        angle = math.atan2(y,x) - phi
        self._current_heading_error = self.correct_angle(angle)
        
#        rospy.logdebug('PID >Phi = ' + str(phi * 180 / math.pi))
#        rospy.logdebug('PID >Ang = ' + str(angle * 180 / math.pi))
        
        self.close_enough()
        
        rospy.loginfo(
            '[' + str(x_0) + ',' + str(y_0) + ']' + '  ' +
            '[' + str(x_1) + ',' + str(y_1) + ']' + '  ' +
            '[' + str(x) + ',' + str(y) + ']' + '  ' +
            '[' + str(math.atan2(y,x) * 180 / math.pi) + ']'
        )
        
        rospy.logwarn("PID >Current Error = " + \
            str(self._current_distance_error) + "\t" + \
            str(self._current_heading_error * 180.0 / math.pi))
        
        return self._current_distance_error, self._current_heading_error, elapsed_time
        
        
        
    def calculate_pid(self, _pos_error, _ang_error, elapsed_time):
        
        self._acum_pos_error += ( _pos_error * elapsed_time )
        self._acum_ang_error += ( _ang_error * elapsed_time )
        
        I_pos = self._acum_pos_error * self._Ki
        I_ang = self._acum_ang_error * self._Ki

        if elapsed_time == 0.0:
            D_pos = 0   # Avoid "division by zero" error
            D_ang = 0
        else:
            D_pos = (_pos_error / elapsed_time) * self._Kd
            D_ang = (_ang_error / elapsed_time) * self._Kd
        
        P_pos = _pos_error * self._Kp
        P_ang = _ang_error * self._Kp
        
        PID_pos = P_pos + I_pos + D_pos
        PID_ang = P_ang + I_ang + D_ang
        
        return PID_pos, PID_ang
    '''    
    def calculate_pid(self, _pos_error, _ang_error):
        
        PID_ang = _ang_error * self._Kp
        
        PID_pos = 0.05 / (abs(PID_ang) + 1)**0.5
        
        return PID_pos, PID_ang
    '''    
    def update(self):
        # PID: Kp*e + Kd*e*s + Ki*e/s
        e_pos, e_ang, elapsed_time = self.input_error()
        
        return self.calculate_pid(e_pos, e_ang, elapsed_time)
        
        
if __name__ == "__main__":
    
    def odom_callback(msg):
        x_0 = msg.pose.pose.position.x
        y_0 = msg.pose.pose.position.y
    
    def imu_callback(msg):
        
#        Phi = 2.0 * math.asin( msg.orientation.z )
#        Theta = math.fmod( msg.orientation.w, 2.0 * math.pi )
#        rospy.loginfo('Phi=' + str(Phi) + ', Theta=' + str(Theta))
        
        if pid_object.get_samples() == True:
            pid_object.add_samples(msg.angular_velocity.x, msg.angular_velocity.y)
        else :
            pid_object.clear_vel_variables()
    
    # Test code
    rospy.init_node('pid_test_node', log_level = rospy.DEBUG)
    
    final_pose = Pose()
    
    pub = rospy.Publisher(
        'cmd_vel',
        Twist,
        queue_size = 1
    )
    rospy.Subscriber(
        '/odom', 
        Odometry, 
        odom_callback
    )
    rospy.Subscriber(
        '/sphero/imu/data',
        Imu,
        imu_callback
    )
    
#    odomdata = Odometry()   # Odometry object
    message = Twist()       # Twist object
    
    rate = rospy.Rate(4)
    
    ctrl_c = False
    def shutdownhook():
        global ctrl_c
        global message
        global pub
        
        rospy.loginfo("PID >Stopping Robot")
        
        ctrl_c = True
        message.linear.x = 0.0
        message.angular.z = 0.0
        pub.publish(message)
    
    rospy.on_shutdown(shutdownhook)
    
    set_points = [[-0.35,-0.45],[0.0,-0.5],[0.0,1.0]]
    step = 0
    rospy.logerr('PID >Initial set point (' + str(step) + ')')
    final_pose.position.x = set_points[step][0]
    final_pose.position.y = set_points[step][1]
    pid_object = PID(
        final_pose, 
        0.05, 
        0.0, 
        0.0
    )
    
    while not ctrl_c:
        ##
        
        lin, ang = pid_object.update()
        message.linear.x = lin
        message.angular.z = ang
        
        rospy.logdebug("PID >Lineal Velocity = " + str(message.linear.x))
        rospy.logdebug("PID >Angular Velocity = " + str(message.angular.z))
        pub.publish(message)
        
        if message.linear.x == 0:
            step += 1
            rospy.logerr('PID >Changing set point (' + str(step) + ')')
            final_pose.position.x = set_points[step][0]
            final_pose.position.y = set_points[step][1]
            pid_object = PID(final_pose, 0.05, 0.0, 0.00)
            
        if step > 3:
            ctrl_c = True
        
        rate.sleep()
        
        