#! /usr/bin/env python3

import rospy

import actionlib
import tf
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import apriltag_docking_ros.msg
import geometry_msgs.msg
from std_msgs.msg import Int16

from docking import Docking
from time import sleep

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
        self.collision_detections = 1

        # parameters
        self.lookahead_dist_multiplier = rospy.get_param("~lookahead_dist_multiplier")
        self.rotational_vel_gain = rospy.get_param("~rotational_vel_gain")
        self.linear_vel_gain = rospy.get_param("~linear_vel_gain")
        self.goal_tolerance = rospy.get_param("~goal_tolerance")
        self.trans_bias = rospy.get_param("~trans_bias")
        self.docking_timeout = rospy.get_param("~docking_timeout")
        self.tag_on_ceiling = rospy.get_param("~tag_on_ceiling")

    
    def stop_robot(self):
        self._cmd.linear.x = 0.0
        self._cmd.angular.z = 0.0
        dock_vel.publish(self._cmd)
    
    def go_straight(self):
        self._cmd.linear.x = 0.05
        self._cmd.angular.z = 0.0
        dock_vel.publish(self._cmd)
    
    def orient_to_goal(self, goal):
        rate = rospy.Rate(10.0)
        rospy.loginfo('Orient robot to goal: %s' % (goal))
        while not rospy.is_shutdown():
            try:
                (trans_tag,rot_tag) = listener.lookupTransform('/base_link', goal.dock_tf_name, rospy.Time(0))
                # print(rot_tag)
                (roll, pitch, yaw) = euler_from_quaternion (rot_tag)
                yaw = yaw + 1.52 # rot  frame by 90deg
                # print("roll: %f, pitch %f, yaw %f" % (roll, pitch, yaw))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            angular = .50 * yaw
            self._cmd.linear.x = 0.0
            self._cmd.angular.z = angular
            dock_vel.publish(self._cmd)

            if abs(yaw) < 0.03: # rotational error less then .5deg
                self.stop_robot()
                rospy.loginfo('rot tolerance reached')
                break

            rate.sleep()


    def obstacle_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        self.collision_detections = data.data

    def execute_cb(self, goal):
        # publish info to the console for the user
        rospy.loginfo('Executing, creating apriltag_docking, to goal: %s' % (goal))
        
        # start executing the action
        rate = rospy.Rate(10.0)
        start_time = rospy.Time.now()  # Get the current time

        while not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                break
            try:
                (trans_tag,rot_tag) = listener.lookupTransform(goal.dock_tf_name, '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
            if self.tag_on_ceiling:
                self._feedback.distance = math.sqrt((trans_tag[0] + self.trans_bias)** 2 + trans_tag[1] ** 2)
                # calc goal tf
                self._br.sendTransform((self.trans_bias, trans_tag[1] - self.lookahead_dist_multiplier * self._feedback.distance, 0.0), 
                                   (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "waypoint", goal.dock_tf_name)
            else:
                self._feedback.distance = trans_tag[2]
                # calc goal tf
                self._br.sendTransform((self.trans_bias, 0.0, trans_tag[2] - self.lookahead_dist_multiplier * self._feedback.distance), 
                                   (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "waypoint", goal.dock_tf_name)

            try:
                (trans_goal,rot) = listener.lookupTransform('/base_link', "waypoint", rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue    

            angular = self.rotational_vel_gain * math.atan2(trans_goal[1], trans_goal[0])
            # print("angular: %f" % angular)
            linear = self.linear_vel_gain * self._feedback.distance
            self._cmd.linear.x = linear
            self._cmd.angular.z = angular

            # Abort if docking takes too long
            current_time = rospy.Time.now()  # Get the current time
            if (current_time - start_time).to_sec() >= self.docking_timeout:
                self._as.set_aborted(text="docking timed out, aborted")
                self.stop_robot()
                break
            
            # check if tag is still detected
            if docking.distance_no_change_count(self._feedback.distance) > docking.tf_not_detected_counter: 
                self._as.set_aborted(text="can't see tag, aborted")
                self.stop_robot()
                break
            # check if goal reached
            if self._feedback.distance > self.goal_tolerance:
                # check if obstacles infront of robot, as long as we are further away as 0.25m.
                if self.collision_detections > 5 and self._feedback.distance > (0.33 + 0.25):
                    rospy.loginfo("Collision detected, aborted!")
                    self.stop_robot()
                    self._as.set_aborted(text="collision detected, aborted")
                    break
                dock_vel.publish(self._cmd)
            # succeeded
            else:
                # self.orient_to_goal(goal)
                # self.go_straight()
                # sleep(6.0)
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
    obstacle_sub = rospy.Subscriber("/collision/detections", Int16, server.obstacle_callback)
    rospy.spin()