#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()

    dock_vel = rospy.Publisher('/nav_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        cmd = geometry_msgs.msg.Twist()

        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/dock', rospy.Time(0))
            print(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        distance = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        angular = 1.5 * math.atan2(trans[1], trans[0])
        linear = 0.1 * distance
        
        cmd.linear.x = linear
        cmd.angular.z = angular
        print(distance)
        if distance>1.7:
            dock_vel.publish(cmd)
        else:
            cmd.linear.x = 0
            cmd.angular.z = 0
            dock_vel.publish(cmd)
            break
            
        rate.sleep()