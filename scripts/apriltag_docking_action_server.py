#! /usr/bin/env python

import rospy

import actionlib
import tf
import math

import apriltag_docking_ros.msg
import geometry_msgs.msg

from docking import Docking

class DockingAction(object):
    # create messages that are used to publish feedback/result
    _feedback = apriltag_docking_ros.msg.DockingFeedback()
    _result = apriltag_docking_ros.msg.DockingResult()
    

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, apriltag_docking_ros.msg.DockingAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # append the seeds for the apriltag_docking sequence
        self._feedback.distance = 10.0
        
        # publish info to the console for the user
        rospy.loginfo('Executing, creating apriltag_docking, current distance: %f to goa: %s' % (self._feedback.distance, goal))
        
        # start executing the action
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            cmd = geometry_msgs.msg.Twist()

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            
            try:
                (trans,rot) = listener.lookupTransform('/base_link', goal.dock_tf_name, rospy.Time(0))
                # print(trans)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            self._feedback.distance = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            angular = 0.5 * math.atan2(trans[1], trans[0])
            linear = 0.1 * self._feedback.distance

            cmd.linear.x = linear
            cmd.angular.z = angular
            self._as.publish_feedback(self._feedback)
            # check if tag is still far away and is still detected
            if self._feedback.distance > 1.5 and docking.distance_no_change_count(self._feedback.distance) < docking.count_max: 
                # print(docking.count)
                dock_vel.publish(cmd)
            else:
                cmd.linear.x = 0
                cmd.angular.z = 0
                dock_vel.publish(cmd)
                self._result.success = True
                self._as.set_succeeded(True, text="Succeeded docking to %s" % goal.dock_tf_name)
                self._as.publish_feedback(self._feedback)
                break

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('apriltag_docking')

    listener = tf.TransformListener()
    docking = Docking()
    dock_vel = rospy.Publisher('/nav_vel', geometry_msgs.msg.Twist,queue_size=1)

    server = DockingAction(rospy.get_name())
    rospy.spin()