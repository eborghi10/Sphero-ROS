#! /usr/bin/env python

import rospy
import time
# The service server imports the IMU subscriber in order to use it
from subscriber_imu import ImuTopicReader
from std_srvs.srv import Trigger, TriggerResponse
'''
Trigger Service Message (Trigger.srv)
# request, Empty because no data is needed
---
#response
bool movement_successfull
string extra_data

'''

class CrashDirectionService(object):
    def __init__(self, srv_name='/crash_direction_service'):
        self._srv_name = srv_name
        self._imu_reader_object = ImuTopicReader() # Object of read data
        self.detection_dict = {"front":False, "left":False, "right":False, "back":False}
        self._my_service = rospy.Service(
            self._srv_name,                 # Service Name (Main <---/crash_direction_service---> Server)
            Trigger,                        # Message type
            self.srv_callback               # Handler
        )
        
# A DICTIONARY AND A FUNCION DON'T START WITH AN UNDERSCORE WHEN USED INSIDE A CLASS.

    def srv_callback(self, request):
        # four_sector_detection() returns a dictionary
        self.detection_dict = self._imu_reader_object.four_sector_detection()
        
        message = self.direction_to_move()  # Jumps to a class function
        
        rospy.logdebug("[LEFT=" + str(self.detection_dict["left"]) + \
                     ", FRONT=" + str(self.detection_dict["front"]) + \
                     ", RIGHT=" + str(self.detection_dict["right"]) + "]" + \
                     ", BACK=" + str(self.detection_dict["back"]) + "]")
        rospy.logdebug("DIRECTION ==>"+message)
        
        # "response" is the message sent by the service server
        # Here it creates the structure
        response = TriggerResponse()
        """
        ---                                                                                                 
        bool success   # indicate if crashed                                       
        string message # Direction
        """
        response.success = self.has_crashed() # Jumps to a class function
        response.message = message
        
        return response

    
    def has_crashed(self):
        for key, value in self.detection_dict.iteritems():
            if value:
                return True     # If any "value" exists, returns true
        
        return False
    
    def direction_to_move(self):

        if not self.detection_dict["front"]:
            message = "forwards"
        
        else:
            if not self.detection_dict["back"]:
                    message = "backwards"
            else:
                if not self.detection_dict["left"]:
                    message = "left"
                else:
                    if not self.detection_dict["right"]:
                        message = "right"
                    else:
                        message = "un_stuck"

        return message

if __name__ == "__main__":
    rospy.init_node('crash_direction_service_server', log_level=rospy.INFO) 
    dir_serv_object = CrashDirectionService()
    rospy.spin() # Mantain the service open.