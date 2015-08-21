import roslib
#roslib.load_manifest("lightning");
import rospy

#from BoxAdder import BoxAdder
#from tools.PathTools import InvalidSectionWrapper
#from pathlib.PathLibrary import PathLibrary
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelConfiguration, SetModelConfigurationRequest
#from arm_navigation_msgs.srv import GetMotionPlan, GetMotionPlanRequest
#from arm_navigation_msgs.msg import JointConstraint
from pr2_mechanism_msgs.srv import SwitchController, SwitchControllerRequest
#from kinematics_msgs.srv import GetKinematicSolverInfo, GetKinematicSolverInfoRequest, GetConstraintAwarePositionIK, GetConstraintAwarePositionIKRequest

from random import random
import time
import os
import sys
import subprocess

LIGHTNING_NAME = "/lightning/lightning_get_path";
RIGHT_ARM_JOINT_CONTROLLER = "r_arm_controller"
SET_MODEL_CONFIGURATION = "/gazebo/set_model_configuration"
SWITCH_CONTROLLER = "/pr2_controller_manager/switch_controller"

class LightningTester:

    def __init__(self):
        self.lightning_client = 1

    def switch_off_controller(self, name):
        switch_off_client = rospy.ServiceProxy(SWITCH_CONTROLLER, SwitchController)
        req = SwitchControllerRequest()
        req.stop_controllers.append(name)
        req.strictness = req.BEST_EFFORT
        rospy.wait_for_service(SWITCH_CONTROLLER)
        res = switch_off_client(req)

    def move_to_joint_configs(self, controller_name, angles):
#        set_model_config_client = rospy.ServiceProxy(SET_MODEL_CONFIGURATION, SetModelConfiguration)
#        req = SetModelConfigurationRequest()
#        req.model_name = "pr2"
#        req.urdf_param_name = "robot_description"
#        req.joint_names = rospy.get_param("/%s/joints" % (controller_name))
#        req.joint_positions = angles
#        print req
#        rospy.wait_for_service("/gazebo/set_model_configuration")
#        res = set_model_config_client(req)
#        print "Seting configuration is successfull"
#        print res

        req = SetModelConfigurationRequest()
        req.model_name = "pr2"
        req.urdf_param_name = "robot_description"
        req.joint_names = rospy.get_param("/%s/joints" % (controller_name))
        req.joint_positions = angles
        print req

        rospy.wait_for_service("/gazebo/set_model_configuration")
        try:
            set_model_config_client = rospy.ServiceProxy(SET_MODEL_CONFIGURATION, SetModelConfiguration)
            res = set_model_config_client(req)
            print "Seting configuration is successfull"
            print res
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


if __name__ == "__main__":
    try:
        rospy.init_node("run_test")
        rospy.loginfo("Lightning test: starting")
        tester = LightningTester()
        
        right_start = [-1.5777, 1.2081, -0.0126, -1.2829, -1.5667, -1.5655, 1.64557]
        rospy.loginfo("Switch off the controller")
        tester.switch_off_controller(RIGHT_ARM_JOINT_CONTROLLER)
        rospy.loginfo("Move to congiguration")
        tester.move_to_joint_configs(RIGHT_ARM_JOINT_CONTROLLER, right_start)
    except rospy.ROSInterruptException:
        pass;
