#!/usr/bin/python
#
#   Author: Artem Gritsenko
#
#   Joints / Openrave indecies / Ros indecies 
#   r_shoulder_pan_joint                                     27           19
#   r_shoulder_lift_joint                                    28            3
#   r_upper_arm_roll_joint                                   29           40
#   r_elbow_flex_joint                                       30           24
#   r_forearm_roll_joint                                     31           34

import rospy
import math
import xml.dom.minidom
import subprocess
from pr2_robot_msgs.msg import *
from pr2_robot_msgs.srv import *
from sensor_msgs.msg import *
from visualization_msgs.msg import *
from math import pi

from openravepy import *
from numpy import *
from misc_transform import *

import time
import sys
import tf

class JointStatePublisher:

    def __init__(self, description_file):
        robot = xml.dom.minidom.parseString(description_file).getElementsByTagName('robot')[0]
        self.free_joints = {}
        self.latest_state = None
        self.last_time = rospy.get_time()
        self.listener = tf.TransformListener()
        # Create all the joints based off of the URDF and assign them joint limits
        # based on their properties
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed':
                    continue
                name = child.getAttribute('name')
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    limit = child.getElementsByTagName('limit')[0]
                    minval = float(limit.getAttribute('lower'))
                    maxval = float(limit.getAttribute('upper'))

                if minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min':minval, 'max':maxval, 'zero':zeroval, 'value':zeroval }
                self.free_joints[name] = joint
        #Setup the PR2State subscriber
        self.pr2_sub = rospy.Subscriber(rospy.get_namespace() + "pr2_state", JointCommandState, self.pr2_cb)
        #Setup the PR2 joint state publisher
        self.pr2_pub = rospy.Publisher(rospy.get_namespace() + "joint_states", JointState)
        #Setup the commands service
        self.pr2_srv = rospy.Service( rospy.get_namespace() + "PlayTrajectoryState", PlayTrajectoryState, self.commands_cb )
        #Setup the object publisher
        self.object_pub = rospy.Publisher(rospy.get_namespace() + "visualization_marker_array", MarkerArray)

    def commands_cb(self,msg):
        self.cmds = msg.commands
        self.play = msg.commands.play
        self.pause = msg.commands.pause
        self.slider_value = msg.commands.slider_value
        if (msg.commands.traj_id != ""):
            self.filename = rospy.get_param('traj_dir') + "/"  + msg.commands.traj_id
            self.traj_given = True
        #rospy.logdebug( "Commands : %s", self.cmds )
        rospy.logdebug( "Filename : %s", msg.commands.traj_id )
        
        return "NO ERROR"

    def pr2_cb(self, msg):
        new_state = {}
        try:
            assert(len(msg.joint_names) == len(msg.state))
            for index in range(len(msg.joint_names)):
                new_state[msg.joint_names[index]] = msg.state[index]
            self.latest_state = new_state
        except:
            rospy.logerr("*** Malformed PR2State! ***")
        self.last_time = rospy.get_time()

    def spawn_objects(self):
        now = rospy.Time.now()
        #self.listener.waitForTransform("/base_link", "/l_gripper_motor_slider_link", now, rospy.Duration(4.0) )
        #p1, q1 = self.listener.lookupTransform( '/base_link', "/l_gripper_motor_slider_link" , now)

        #print q1

        rate = rospy.Rate(40.0)
#stuff for experiment 1 and experiment 3

        # msg1 = Marker()
        # msg1.header.frame_id = "base_link"
        # msg1.header.stamp = rospy.Time.now()
        # msg1.ns = "my_namespace"
        # msg1.id = 1
        # msg1.type = Marker.MESH_RESOURCE
        # msg1.action = Marker.ADD
        #
        # msg1.color.a = 1.0
        # msg1.color.r = 0.8
        # msg1.color.g = 0.7
        # msg1.color.b = 0.3
        #
        # msg1.pose.position.x = 0.2 #1.
        # msg1.pose.position.y = 0.45 #0.
        # msg1.pose.position.z = 1.2 #1.2
        # msg1.pose.orientation.x = 0.0
        # msg1.pose.orientation.y = 0.0
        # msg1.pose.orientation.z = 0.5
        # msg1.pose.orientation.w = 0.0
        #
        # scale_factor_1 = 0.5
        #
        # msg1.scale.x = 1. * scale_factor_1
        # msg1.scale.y = 1. * scale_factor_1
        # msg1.scale.z = 1. * scale_factor_1
        #
        # msg1.mesh_resource = "package://heres_how/models/wheel_rotor.dae"
      
        scale_factor_1 = 1.0

        msg2 = Marker()
        msg2.header.frame_id = "r_gripper_motor_slider_link"
        msg2.header.stamp = rospy.Time.now()
        msg2.ns = "my_namespace"
        msg2.id = 2
        #msg2.type = Marker.MESH_RESOURCE
        msg2.type = Marker.CYLINDER
        msg2.action = Marker.ADD

        msg2.pose.position.x = 0.0
        msg2.pose.position.y = 0.0
        msg2.pose.position.z = 0.0
        msg2.pose.orientation.x = 0.707
        msg2.pose.orientation.y = 0.0
        msg2.pose.orientation.z = -0.707
        msg2.pose.orientation.w = 0.0
        
        msg2.color.a = 1.0
        msg2.color.r = 0.6
        msg2.color.g = 0.7
        msg2.color.b = 0.5

        scale_factor_2 = 1.0

        msg2.scale.x = 1. * 0.05
        msg2.scale.y = 1. * 0.05
        msg2.scale.z = 1. * 0.1

# stuff for nut

#        msg2.pose.position.x = 0.2
#        msg2.pose.position.y = 0.
#        msg2.pose.position.z = 0.2
#        msg2.pose.orientation.x = 0.65328
#        msg2.pose.orientation.y = 0.65328
#        msg2.pose.orientation.z = -0.2706
#        msg2.pose.orientation.w = -0.2706
#        
#        msg2.color.a = 1.0
#        msg2.color.r = 0.6
#        msg2.color.g = 0.7
#        msg2.color.b = 0.5

#        scale_factor_2 = 10.0

#        msg2.scale.x = 1. * scale_factor_2
#        msg2.scale.y = 1. * scale_factor_2
#        msg2.scale.z = 1. * scale_factor_2

#        msg2.mesh_resource = "package://heres_how/models/nut.dae"

# block for experiment 2
#
#         msg3 = Marker()
#         msg3.header.frame_id = "base_link"
#         msg3.header.stamp = rospy.Time.now()
#         msg3.ns = "my_namespace"
#         msg3.id = 3
#         #msg3.type = Marker.MESH_RESOURCE
#         msg3.type = Marker.CYLINDER
#         msg3.action = Marker.ADD
#
#         msg3.pose.position.x = 1.0
#         msg3.pose.position.y = 0.45
#         msg3.pose.position.z = 1.2
#         msg3.pose.orientation.x = 0.707
#         msg3.pose.orientation.y = 0.0
#         msg3.pose.orientation.z = -0.707
#         msg3.pose.orientation.w = 0.0
#
#         msg3.color.a = 1.0
#         msg3.color.r = 0.6
#         msg3.color.g = 0.7
#         msg3.color.b = 0.5
#
#         scale_factor_3 = 1.0
#
#         msg3.scale.x = 1. * 0.05
#         msg3.scale.y = 1. * 0.05
#         msg3.scale.z = 1. * 1.5

        #msg3.mesh_resource = "package://heres_how/models/Car/meshes/car.dae"

# block for experiment 2
        # msg4 = Marker()
        # msg4.header.frame_id = "base_link"
        # msg4.header.stamp = rospy.Time.now()
        # msg4.ns = "my_namespace"
        # msg4.id = 4
        # #msg3.type = Marker.MESH_RESOURCE
        # msg4.type = Marker.CUBE
        # msg4.action = Marker.ADD
        #
        # msg4.pose.position.x = 0.35
        # msg4.pose.position.y = 1.1
        # msg4.pose.position.z = 1.2
        # msg4.pose.orientation.x = 0.0
        # msg4.pose.orientation.y = 0.0
        # msg4.pose.orientation.z = 0.0
        # msg4.pose.orientation.w = 1.0
        #
        # msg4.color.a = 1.0
        # msg4.color.r = 0.6
        # msg4.color.g = 0.7
        # msg4.color.b = 0.5
        #
        # scale_factor_4 = 1.0
        #
        # msg4.scale.x = 1. * 0.3
        # msg4.scale.y = 1. * 0.6
        # msg4.scale.z = 1. * 0.4

        # back wall of the car
        msg5 = Marker()
        msg5.header.frame_id = "base_link"
        msg5.header.stamp = rospy.Time.now()
        msg5.ns = "my_namespace"
        msg5.id = 5
        msg5.type = Marker.CUBE
        msg5.action = Marker.ADD

        msg5.pose.position.x = 0.45
        msg5.pose.position.y = -1.0
        msg5.pose.position.z = 1.0
        msg5.pose.orientation.x = 0.0
        msg5.pose.orientation.y = 0.0
        msg5.pose.orientation.z = 0.0
        msg5.pose.orientation.w = 1.0

        msg5.color.a = 1.0
        msg5.color.r = 0.6
        msg5.color.g = 0.7
        msg5.color.b = 0.5

        msg5.scale.x = 1. * 0.5
        msg5.scale.y = 1. * 0.2
        msg5.scale.z = 1. * 0.8

        # floor of the car
        msg6 = Marker()
        msg6.header.frame_id = "base_link"
        msg6.header.stamp = rospy.Time.now()
        msg6.ns = "my_namespace"
        msg6.id = 6
        msg6.type = Marker.CUBE
        msg6.action = Marker.ADD

        msg6.pose.position.x = 0.5
        msg6.pose.position.y = 0.0
        msg6.pose.position.z = 0.65
        msg6.pose.orientation.x = 0.0
        msg6.pose.orientation.y = 0.0
        msg6.pose.orientation.z = 0.0
        msg6.pose.orientation.w = 1.0

        msg6.color.a = 1.0
        msg6.color.r = 0.6
        msg6.color.g = 0.7
        msg6.color.b = 0.5

        msg6.scale.x = 1. * 0.6
        msg6.scale.y = 1. * 2.5
        msg6.scale.z = 1. * 0.1

        # side wall of the car
        msg7 = Marker()
        msg7.header.frame_id = "base_link"
        msg7.header.stamp = rospy.Time.now()
        msg7.ns = "my_namespace"
        msg7.id = 7
        msg7.type = Marker.CUBE
        msg7.action = Marker.ADD

        msg7.pose.position.x = 0.2
        msg7.pose.position.y = 0.0
        msg7.pose.position.z = 0.7
        msg7.pose.orientation.x = 0.0
        msg7.pose.orientation.y = 0.0
        msg7.pose.orientation.z = 0.0
        msg7.pose.orientation.w = 1.0

        msg7.color.a = 1.0
        msg7.color.r = 0.6
        msg7.color.g = 0.7
        msg7.color.b = 0.5

        msg7.scale.x = 1. * 0.01
        msg7.scale.y = 1. * 2.5
        msg7.scale.z = 1. * 0.2

        # chair in the car 1
        msg8 = Marker()
        msg8.header.frame_id = "base_link"
        msg8.header.stamp = rospy.Time.now()
        msg8.ns = "my_namespace"
        msg8.id = 8
        msg8.type = Marker.CUBE
        msg8.action = Marker.ADD

        msg8.pose.position.x = 0.65
        msg8.pose.position.y = -0.4
        msg8.pose.position.z = 1.2
        msg8.pose.orientation.x = 0.0
        msg8.pose.orientation.y = 0.0
        msg8.pose.orientation.z = 0.0
        msg8.pose.orientation.w = 1.0

        msg8.color.a = 1.0
        msg8.color.r = 0.6
        msg8.color.g = 0.7
        msg8.color.b = 0.5

        msg8.scale.x = 1. * 0.6
        msg8.scale.y = 1. * 0.2
        msg8.scale.z = 1. * 0.5

        # chair in the car 2
        msg9 = Marker()
        msg9.header.frame_id = "base_link"
        msg9.header.stamp = rospy.Time.now()
        msg9.ns = "my_namespace"
        msg9.id = 9
        msg9.type = Marker.CUBE
        msg9.action = Marker.ADD

        msg9.pose.position.x = 0.65
        msg9.pose.position.y = -0.2
        msg9.pose.position.z = 1.0
        msg9.pose.orientation.x = 0.0
        msg9.pose.orientation.y = 0.0
        msg9.pose.orientation.z = 0.0
        msg9.pose.orientation.w = 1.0

        msg9.color.a = 1.0
        msg9.color.r = 0.6
        msg9.color.g = 0.7
        msg9.color.b = 0.5

        msg9.scale.x = 1. * 0.6
        msg9.scale.y = 1. * 0.2
        msg9.scale.z = 1. * 0.6

        markerArray = MarkerArray()
        markerArray.markers = []
        markerArray.markers.append( msg2 )
        markerArray.markers.append( msg5 )
        markerArray.markers.append( msg6 )
        markerArray.markers.append( msg7 )
        markerArray.markers.append( msg8 )
        markerArray.markers.append( msg9 )


        if len(markerArray.markers) > 0:
            self.object_pub.publish(markerArray)

    def loop(self, hz=20.):
        r = rospy.Rate(hz) 
        pub = read_traj()
        traj = RaveCreateTrajectory( pub.env, '' )

        joint_values = [0.]*45
        wpnt_ind = 0
        self.play =False
        self.pause = True
        self.traj_given = False
        self.slider_value = -1
        
       
        # Publish Joint States
        while not rospy.is_shutdown():
            if (not self.pause):
                if (self.traj_given):
                    # change loop parameters when the play button is pressed
                    if (self.play):
                        self.play = False
                        # relaunch the traj from filename
                        try:
                            f = open(self.filename,'r')
                        except:
                            rospy.logerr("File %s not found!" % self.filename) 
                        traj.deserialize( f.read() )
                        f.close()
                        #reinitalize all other values
                        wpnt_ind = 0

                    if (wpnt_ind < traj.GetNumWaypoints()):
                        # get the waypoint values, this holds velocites, time stamps, etc
                        data = traj.GetWaypoint(wpnt_ind)
                    wpnt_ind += 1

                    k = 0
                    for joint_name in self.free_joints:
                        index_ = pub.robot.GetJoint(joint_name).GetDOFIndex()
                        joint_values[k] = data[index_]
                        k += 1
            if (self.slider_value != -1):
                try:
                    f = open(self.filename,'r')
                except:
                    rospy.logerr("File %s not found!" % self.filename) 
                traj.deserialize( f.read() )
                f.close()
                #rospy.logdebug("number of waypoints: %f" % traj.GetNumWaypoints())
                wpnt_ind = traj.GetNumWaypoints()*self.slider_value/100
                if (wpnt_ind < traj.GetNumWaypoints()):
                    data = traj.GetWaypoint(wpnt_ind)
                    #rospy.logdebug("Waypoint index: ")
                    #rospy.logdebug(wpnt_ind)

                    k = 0
                    for joint_name in self.free_joints:
                        index_ = pub.robot.GetJoint(joint_name).GetDOFIndex()
                        joint_values[k] = data[index_]
                        k += 1
                else:
                    rospy.logdebug("Error! " )
            j = 0
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            for joint_name in self.free_joints:
                msg.name.append(joint_name)
                msg.position.append(joint_values[j])
                j += 1
            self.pr2_pub.publish(msg)
            self.spawn_objects()
            #Spin
            r.sleep()
            #rospy.logdebug("loop %f" % rospy.Time.now().to_sec() )

class read_traj():

    def __init__(self):

        self.env = Environment()
        # self.env.SetViewer('qtcoin')
        self.env.Reset()

        self.env.Load('robots/pr2-beta-static.zae')
        self.env.Load('data/shelf.kinbody.xml')

        shelf = None
        for b in self.env.GetBodies() :
            if  b.GetName() == "Shelf":
                shelf = b
        if shelf is not None:
            shelf.SetTransform( array( MakeTransform( rodrigues([-pi/2,0,0]), matrix([0.7,-0.5,0]) ) ) )

        self.robot = self.env.GetRobots()[0]

        self.robot.SetActiveManipulator(1)
        self.indices = self.robot.GetActiveManipulator().GetArmIndices()
        self.robot.SetActiveDOFs( self.indices )


if __name__ == '__main__':
    rospy.init_node('pr2_joint_state_publisher',log_level=rospy.DEBUG)
    #rospy.set_param('traj_file', '/home/artemgritsenko/catkin_ws/src/heres_how/scripts/trajectoryB.txt')
    description_file = rospy.get_param("robot_description")
    publish_rate = rospy.get_param("~rate", 20.0)
    jsp = JointStatePublisher(description_file)
    jsp.spawn_objects()
    #sys.stdin.readline()
    jsp.loop(publish_rate)

