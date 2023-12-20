#! /usr/bin/env python3

import rospy

import actionlib
import tf
from tf.transformations import euler_from_quaternion
import math

import apriltag_docking_ros.msg
import geometry_msgs.msg

from docking import Docking

class DockingAction(object):
    # create messages that are used to publish feedback/result
    _feedback = apriltag_docking_ros.msg.DockingFeedback()
    _result = apriltag_docking_ros.msg.DockingResult()
    _cmd = geometry_msgs.msg.Twist()
    _br = tf.TransformBroadcaster()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, apriltag_docking_ros.msg.DockingAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    
    def stop_robot(self):
        self._cmd.linear.x = 0
        self._cmd.angular.z = 0
        dock_vel.publish(self._cmd)
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # parameters
        lookahead_dist_multiplier = 0.4
        rotational_gain = 0.5
        linear_gain = 0.1
        goal_distance = 0.33
        
        # publish info to the console for the user
        rospy.loginfo('Executing, creating apriltag_docking, to goa: %s' % (goal))
        
        # start executing the action
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            try:
                (trans,rot) = listener.lookupTransform('/base_link', goal.dock_tf_name, rospy.Time(0))
                (trans_tag,rot_tag) = listener.lookupTransform(goal.dock_tf_name, '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            self._feedback.distance = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            lookahead_dist = self._feedback.distance * lookahead_dist_multiplier
            
            self._br.sendTransform((0.0, 0.0, trans_tag[2] - math.sqrt(lookahead_dist**2 - trans_tag[0]**2)), 
                                   (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "goal", goal.dock_tf_name)

            try:
                (trans_goal,rot) = listener.lookupTransform('/base_link', "goal", rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue    

            angular = rotational_gain * math.atan2(trans_goal[1], trans_goal[0])
            # print("angular: %f" % angular)
            linear = linear_gain * self._feedback.distance
            self._cmd.linear.x = linear
            self._cmd.angular.z = angular
            
            # check if tag is still detected
            if docking.distance_no_change_count(self._feedback.distance) > docking.count_max: 
                self._as.set_aborted(text="can't see tag")
                self.stop_robot()
                break
            # check if goal reached
            if self._feedback.distance > goal_distance:
                dock_vel.publish(self._cmd)
            # succeeded
            else:
                self.stop_robot()
                self._result.success = True
                self._as.set_succeeded(True, text="Succeeded docking to %s" % goal.dock_tf_name)
                break
            
            self._as.publish_feedback(self._feedback)
            rate.sleep()

        self._as.publish_feedback(self._feedback)

if __name__ == '__main__':
    rospy.init_node('apriltag_docking')

    listener = tf.TransformListener()
    docking = Docking()
    dock_vel = rospy.Publisher('/nav_vel', geometry_msgs.msg.Twist,queue_size=1)

    server = DockingAction(rospy.get_name())
    rospy.spin()