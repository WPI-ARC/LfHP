import rospy
import math
import xml.dom.minidom
import subprocess
from pr2_robot_msgs.msg import *
from pr2_robot_msgs.srv import *
from sensor_msgs.msg import *
from visualization_msgs.msg import *
from trajectory_msgs.msg import *
from math import pi

from openravepy import *
from numpy import *
from misc_transform import *
from gazebo_msgs.srv import *

import time
import sys



if __name__=="__main__":
    rospy.init_node("Teleport_Handler")
    rospy.on_shutdown(close_server)
    
    global get_state_service, set_state_service, get_link_properties_service, set_link_properties_service
    populate_services()
    
    s = rospy.Service('TeleportService', TeleportService, teleport_handler)
    
    req = GetModelStateRequest()
    req.model_name = "nut_LF_1"
#     print(get_state_service(req))
    
    global listener
    listener = tf.TransformListener()
    
#     print("Initializing GUI comms service... ", end="")
#     s = rospy.Service('debris_fitting/info', DebrisInfoService, handle_info_service)
#     print("done")
    
#     global debris_publisher
#     print("Initializing publisher... ", end="")
#     debris_publisher = rospy.Publisher("/debris_update", DebrisUpdate)
#     print("done")
    
#     print("Instantiating TF Broadcaster... ", end="")
#     global tf_broadcaster
#     tf_broadcaster = tf.TransformBroadcaster()
#     print("done")

    
        
    rospy.spin()
