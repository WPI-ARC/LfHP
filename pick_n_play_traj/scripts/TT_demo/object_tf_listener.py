#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import sys

if __name__ == '__main__':
    rospy.init_node('tf_turtle')
    #rospy.logdebug("wait for the key")
    #sys.stdin.readline()
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            listener.waitForTransform('/touchtomorrow1', '/base_link', now, rospy.Duration(2.0))
            p = listener.lookupTransform('/touchtomorrow1','/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "exception"
            continue
        print "got transform"
   
    
    rospy.logdebug("transform is")
    rospy.logdebug(p)
    rospy.logdebug("wait for the key")
    sys.stdin.readline()
