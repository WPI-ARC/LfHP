#!/usr/bin/python
#
#   Author: Artem Gritsenko

import rospy
import math
import xml.dom.minidom
import subprocess
from sensor_msgs.msg import *
from visualization_msgs.msg import *
from transformation_helper import *
from math import pi

from openravepy import *
from numpy import *
#from misc_transform import *

import time
import sys
import tf
import random
from threading import Timer
from tf.transformations import *

class ObjectStatePublisher:

    def __init__(self):
        self.object_pub = rospy.Publisher(rospy.get_namespace() + "visualization_marker_array", MarkerArray)
        self.in_collision = True
        self.point_pose = []
        self.timeout = 100.0
        self.dragon_head = True

    def timeout_th(self):
        self.timeout = True

    def check_col(self, p1, p2, eps = 0.3):
#        print "lengths %r %r" % (len(p1) , len(p2))
#        print "p1 is ", p1
#        print "p2 is ", p2

        if abs(p1[0] - p2[0]) < eps and abs(p1[1] - p2[1]) < eps:
            print "collision detected"
            self.in_collision = True

        

    def spawn_object(self):
        
        listener = tf.TransformListener()
        self.init_time = time.time()

        msg1 = Marker()
        msg1.header.frame_id = "mocap_world"
        msg1.header.stamp = rospy.Time.now()
        msg1.ns = "my_namespace"
        msg1.id = 1
        msg1.type = Marker.SPHERE
        msg1.action = Marker.ADD
        msg1.lifetime = rospy.Time(0.3)
        
        msg1.color.a = 1.0
        msg1.color.r = 0.8
        msg1.color.g = 0.1
        msg1.color.b = 0.5
      
        scale_factor_1 = .5

        msg3 = Marker()
        msg3.header.frame_id = "mocap_world"
        msg3.header.stamp = rospy.Time.now()
        msg3.ns = "my_namespace"
        msg3.id = 3
        msg3.type = Marker.MESH_RESOURCE
        msg3.action = Marker.ADD
        msg3.lifetime = rospy.Time(0.3)
        
        if self.dragon_head:
            msg3.mesh_resource = "package://heres_how/scripts/TT_Demo_2015/models_dae/dragon_head.dae"
        else:
            msg3.mesh_resource = "package://heres_how/scripts/TT_Demo_2015/models_dae/models/Rusted_Robot.dae" 

        msg3.color.a = 1.0
        msg3.color.r = 0.8
        msg3.color.g = 0.7
        msg3.color.b = 0.3
        
        if not self.dragon_head:
            scale_factor_3 = 0.15
        else:
            scale_factor_3 = 1.5

        while not rospy.is_shutdown():

            markerArray = MarkerArray()
            markerArray.markers = []

            time.time()

            now = rospy.Time.now()
            while True:
                try:
                    p3, q3 = listener.lookupTransform( '/mocap_world', "/mocap/Head0/Head0" , rospy.Time(0))
                    break
                except:
                    pass

            #print self.point_pose
            if len(self.point_pose) > 0:
                #print self.point_pose
                self.check_col(p3, self.point_pose)
            
            if time.time() - self.init_time > self.timeout or self.in_collision:
                #print "update the point"
                self.point_pose = [random.uniform(1.1, 3.0), random.uniform(0.2, 2.1), 1.3]
                msg1.pose.position.x =  self.point_pose[0]
                msg1.pose.position.y =  self.point_pose[1]
                msg1.pose.position.z =  self.point_pose[2]
                self.init_time = time.time()
                self.in_collision = False
                
            msg1.pose.orientation.x = 0.
            msg1.pose.orientation.y = 0.
            msg1.pose.orientation.z = 0.
            msg1.pose.orientation.w = 1.
            msg1.scale.x = 1. * scale_factor_1
            msg1.scale.y = 1. * scale_factor_1
            msg1.scale.z = 1. * scale_factor_1

            markerArray.markers.append( msg1 )


            if not self.dragon_head:
                eu = euler_from_quaternion(q3)
                q_new = quaternion_multiply(quaternion_from_euler(0., 0., pi/2), quaternion_from_euler(0., 0., eu[2]))

            msg3.pose.position.x = p3[0] #+ offset_x
            msg3.pose.position.y = p3[1] #+ offset_y
            msg3.pose.position.z = p3[2] #+ offset_z
            msg3.pose.orientation.x = q3[0]
            msg3.pose.orientation.y = q3[1]
            msg3.pose.orientation.z = q3[2]
            msg3.pose.orientation.w = q3[3]
            msg3.scale.x = 1. * scale_factor_3
            msg3.scale.y = 1. * scale_factor_3
            msg3.scale.z = 1. * scale_factor_3

            

            if not self.dragon_head:
                msg3.pose.position.z = 0.
                msg3.pose.orientation.x = q_new[0]
                msg3.pose.orientation.y = q_new[1]
                msg3.pose.orientation.z = q_new[2]
                msg3.pose.orientation.w = q_new[3]

            #msg3_yaw = math.atan2(2.0*(q3[1]*q3[2] + q3[3]*q3[0]), q3[3]*q3[3] - q3[0]*q3[0] - q3[1]*q3[1] + q3[2]*q3[2]) # x -dim
            #msg3_yaw = math.atan2(2.0*(q3[0]*q3[1] + q3[3]*q3[2]), q3[3]*q3[3] + q3[0]*q3[0] - q3[1]*q3[1] - q3[2]*q3[2])
            #msg3_yaw = math.asin(-2.0*(q3[0]*q3[2] - q3[3]*q3[1]))
            #msg3_yaw = math.atan2(2.0*(q3[0]*q3[3] + q3[1]*q3[2]), 1 - 2 * (q3[3]*q3[3] + q3[2]*q3[2]) ) # x -dim
            #msg3_yaw = math.asin( 2.0*(q3[0]*q3[2] - q3[3]*q3[1]) ) #y-dim
            #msg3_yaw = math.atan2(2.0*(q3[0]*q3[3] + q3[1]*q3[2]), 1 - 2 * (q3[2]*q3[2] + q3[3]*q3[3]) )


            #print msg3_yaw 

            # Offset in world
            pose = Pose()
            #pose.position.x =  -0.3 #+ offset_x -0.3 +
            #pose.position.y =  -11.3  #+ offset_y -11.3 +
            #pose.position.z =  -3.7 #+ offset_z -3.7 +

            if self.dragon_head:
                pose.position.x =  -0.3 #+ offset_x -0.3 +
                pose.position.y =  -11.3  #+ offset_y -11.3 +
                pose.position.z =  -3.7 #+ offset_z -3.7 +

            #msg3.pose = ComposePoses( msg3.pose, pose )

            msg3.pose = ComposePoses( msg3.pose, pose )

            markerArray.markers.append( msg3 )

            if len(markerArray.markers) > 0:
                self.object_pub.publish(markerArray)
            rospy.sleep(0.02)

if __name__ == '__main__':
    rospy.init_node('object_state_publisher',log_level=rospy.DEBUG)
    sp = ObjectStatePublisher()
    sp.spawn_object()

