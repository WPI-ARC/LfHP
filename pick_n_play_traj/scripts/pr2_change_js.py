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
from trajectory_msgs.msg import *
from math import pi

from openravepy import *
from numpy import *
from misc_transform import *
from gazebo_msgs.srv import *

import time
import sys

class JointStatePublisher:

    def __init__(self, description_file):
        robot = xml.dom.minidom.parseString(description_file).getElementsByTagName('robot')[0]
        self.free_joints = {}
        self.warnings = {}
        self.latest_state = None
        self.last_time = rospy.get_time()
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
        self.pr2_pub2 = rospy.Publisher("/r_arm_controller/joint_trajectory_action/goal", JointTrajectory)
        #Setup the commands service
        self.pr2_srv = rospy.Service( rospy.get_namespace() + "PlayTrajectoryState", PlayTrajectoryState, self.commands_cb )
        #Setup the wheel publisher
        self.wheel_pub = rospy.Publisher(rospy.get_namespace() + "visualization_marker", Marker)

        # initialize service
        #rospy.wait_for_service('gazebo_msgs/SetJointTrajectory', timeout=5)
        #set_j_traj = rospy.ServiceProxy('gazebo_msgs/SetJointTrajectory', SetJointTrajectory)

    def commands_cb(self,msg):
        self.cmds = msg.commands
        self.play = msg.commands.play
        self.pause = msg.commands.pause
        self.slider_value = msg.commands.slider_value
        if (msg.commands.traj_id != ""):
            self.filename = rospy.get_param('traj_dir') + "/"  + msg.commands.traj_id
            self.traj_given = True
        rospy.logdebug( "Commands : %s", self.cmds )
        rospy.logdebug( "Filename : %s", self.filename )
        
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
        #rospy.logdebug( "spawn objects" )
        msg = Marker()
        msg.header.frame_id = "base_link"
        msg.header.stamp = rospy.Time.now()
        msg.ns = "my_namespace"
        msg.id = 0
        msg.type = Marker.MESH_RESOURCE
        msg.action = Marker.ADD
        msg.pose.position.x = 1
        msg.pose.position.y = 1
        msg.pose.position.z = 1
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        msg.scale.x = 1
        msg.scale.y = 0.1
        msg.scale.z = 0.1
        msg.color.a = 1.0
        msg.color.r = 0.0
        msg.color.g = 1.0
        msg.color.b = 0.0
        msg.mesh_resource = "package://heres_how/models/table.dae"
        #rospy.logdebug( "Marker : %s", msg )
        self.wheel_pub.publish( msg )

        self.j_traj = []

    def loop2(self, hz=1.):
        r = rospy.Rate(hz)
        msg = JointTrajectory()
        #msg.header = Header()
        
        j_points = []
        j_point = JointTrajectoryPoint()
        j_point.positions = [0.6, 0.4, 0.5, -0.6, -1.5]
        j_point.velocities = [0., 0., 0., 0., 0.]
        j_point.accelerations = [0., 0., 0., 0., 0.]
        j_points.append(j_point)
        j_point.time_from_start = rospy.Duration.from_sec(90.)
        while not rospy.is_shutdown():
            j = 0
            j_names = []
            for joint_name in self.free_joints:
                if (j == 18) or (j == 2) or (j == 39) or (j == 23) or (j == 33): #TODO make general
                    j_names.append(joint_name)
                j += 1

            msg.joint_names = j_names
            msg.points = j_points
            self.pr2_pub2.publish(msg)
            r.sleep()
            

    def loop(self, hz=1.):
        r = rospy.Rate(hz) 
        pub = read_traj()
        traj = RaveCreateTrajectory( pub.env, '' )

        #self.filename = rospy.get_param('traj_file')
#        try:
#            f = open(self.filename,'r')
#        except:
#            rospy.logerr("File %s not found!" % self.filename) 
#        traj.deserialize( f.read() )
#        rospy.logdebug("traj read")
        joint_values = [0.]*45
        wpnt_ind = 0
        self.play =False
        self.pause = True
        self.traj_given = False
        self.slider_value = -1
        
        self.j_traj = []

        #rospy.logdebug("num of waypoints")

        #rospy.logdebug(traj.GetNumWaypoints())
        # Publish Joint States
        #rospy.logdebug("wait for the key")
        #sys.stdin.readline()
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

                    #rospy.logdebug("Num of waypoints:  ")
                    #rospy.logdebug(traj.GetNumWaypoints())
                    # while some waypoints are unpuplished
                    if (wpnt_ind < traj.GetNumWaypoints()):
                        # get the waypoint values, this holds velocites, time stamps, etc
                        data = traj.GetWaypoint(wpnt_ind)
                    wpnt_ind += 1

                    # rospy.logdebug("chunk of data")
                    # rospy.logdebug(data)

                    # rospy.logdebug("No valid message received from the PR2 yet")
                    
                    joint_values[18] = data[0]
                    joint_values[2] = data[1]
                    joint_values[39] = data[2]
                    joint_values[23] = data[3]
                    joint_values[33] = data[4]

                    

            if (self.slider_value != -1):
                try:
                    f = open(self.filename,'r')
                except:
                    rospy.logerr("File %s not found!" % self.filename) 
                traj.deserialize( f.read() )
                f.close()
                wpnt_ind = traj.GetNumWaypoints()/100*self.slider_value
                if (wpnt_ind < traj.GetNumWaypoints()):
                    data = traj.GetWaypoint(wpnt_ind)
                    rospy.logdebug("Waypoint index: ")
                    rospy.logdebug(wpnt_ind)

                    joint_values[18] = data[0]
                    joint_values[2] = data[1]
                    joint_values[39] = data[2]
                    joint_values[23] = data[3]
                    joint_values[33] = data[4]
            
            traj_point = JointTrajectoryPoint()
            traj_point.positions = [joint_values[18], joint_values[2], joint_values[39], joint_values[23], joint_values[33]]
            traj_point.velocities = [0., 0., 0., 0., 0.]
            traj_point.accelerations = [0., 0., 0., 0., 0.]
            traj_point.time_from_start = rospy.Duration.from_sec(traj.GetDuration()/traj.GetNumWaypoints()*wpnt_ind)
            j_traj.append(traj_point)

            j = 0
            msg = JointState()
            msg.header.stamp = rospy.Time.now()

            joint_names = []
            traj_points = JointTrajectoryPoint()

            for joint_name in self.free_joints:
                msg.name.append(joint_name)
                #msg.position.append(self.free_joints[joint_name]['zero'])
                msg.position.append(joint_values[j])

                if (j == 18) or (j == 2) or (j == 39) or (j == 23) or (j == 33): #TODO make general
                    joint_names.append(joint_name)

                j += 1

            self.pr2_pub.publish(msg)
            #self.spawn_objects()
            #Spin
            
            # TODO create request from joint values
            req = SetJointTrajectory()
            req.model_name = "PR2"
            req.joint_trajectory = JointTrajectory()
            req.joint_trajectory.header = Header()
            req.joint_trajectory.joint_names = joint_names
            req.joint_trajectory.points = j_traj
            
            #req.model_pose = 
            req.disable_physics_updates = True
            req.set_model_pose = False
            print wpnt_ind
            # sending the request
            if (wpnt_ind >= traj.GetNumWaypoints()):
                print "sending request", req
                self.set_j_traj(req)
    
            r.sleep()
            # rospy.logdebug("loop %f" % rospy.Time.now().to_sec() )

    def warn_user(self, warning_string):
        try:
            self.warnings[warning_string] += 1
        except:
            self.warnings[warning_string] = 1
            rospy.logwarn(warning_string + " - This message will only print once")

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
    rospy.init_node('pr2_change_js',log_level=rospy.DEBUG)
    #rospy.set_param('traj_file', '/home/artemgritsenko/catkin_ws/src/heres_how/scripts/trajectoryB.txt')
    try:
        description_file = rospy.get_param("robot_description")
    except:
        description_file  ="../launch/pr2.urdf"
    #publish_rate = rospy.get_param("~rate", 20.0)
    jsp = JointStatePublisher(description_file)
    #jsp.spawn_objects()
    #jsp.loop(publish_rate)
    jsp.loop2(20.)
    print "End play"
