#!/usr/bin/env python3
import rospy
import math
import tf
import geometry_msgs.msg
from docking import Docking



if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    docking = Docking()
    listener = tf.TransformListener()
    dock_vel = rospy.Publisher('/nav_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        cmd = geometry_msgs.msg.Twist()

        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/dock', rospy.Time(0))
            # print(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        distance = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        angular = 0.5 * math.atan2(trans[1], trans[0])
        linear = 0.1 * distance
        
        cmd.linear.x = linear
        cmd.angular.z = angular
        print(distance)
        if distance > 0.3 and docking.distance_no_change_count(distance) < docking.count_max:
            print(docking.count)
            dock_vel.publish(cmd)
        else:
            cmd.linear.x = 0
            cmd.angular.z = 0
            dock_vel.publish(cmd)
            break
            
        rate.sleep()