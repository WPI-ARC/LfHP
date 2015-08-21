#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')
import rospy
from geometry_msgs.msg import TransformStamped

import tf
from visualization_msgs.msg import *

def handle_object_pose(msg):
    
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        try:
            br.sendTransform((msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z), tf.transformations.quaternion_from_euler(msg.transform.rotation.x, msg.transform.rotation.x, msg.transform.rotation.z),
                             #tf.transformations.quaternion_from_euler(msg.transform.rotation.x, msg.transform.rotation.x, msg.transform.rotation.z, msg.transform.rotation.w),
                             rospy.Time.now(),
                             "/vicon/TouchTomorrow1/TouchTomorrow1",
                             "/world")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "exception"
        print "broadcast transform"

if __name__ == '__main__':
    rospy.init_node('object_tf_broadcaster')
    rospy.Subscriber('vicon/TouchTomorrow1/TouchTomorrow1', TransformStamped, handle_object_pose)
    rospy.spin()
