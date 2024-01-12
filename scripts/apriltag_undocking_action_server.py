#! /usr/bin/env python

import rospy

import actionlib
import apriltag_docking_ros.msg
import geometry_msgs.msg
from std_msgs.msg import Int16

class UnDockingAction(object):
    # create messages that are used to publish feedback/result
    _feedback = apriltag_docking_ros.msg.DockingFeedback()
    _result = apriltag_docking_ros.msg.DockingResult()
    _cmd = geometry_msgs.msg.Twist()
    
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, apriltag_docking_ros.msg.UnDockingAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self.collision_detections = 1

        # parameters
        self.linear_vel = rospy.get_param("~linear_vel")
    
    def stop_robot(self):
        self._cmd.linear.x = 0.0
        self._cmd.angular.z = 0.0
        dock_vel.publish(self._cmd)
    
    def go_straight(self):
        self._cmd.linear.x = -self.linear_vel
        self._cmd.angular.z = 0.0
        dock_vel.publish(self._cmd)
    
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
            
            self.go_straight()
            current_time = rospy.Time.now()
            self._feedback.distance = abs(self._cmd.linear.x * (current_time - start_time).to_sec())

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                break
            # check if obstacles are detected
            if self.collision_detections > 5: 
                self._as.set_aborted(text="obstacle detected")
                self.stop_robot()
                break
            # check if goal reached
            if (self._feedback.distance > goal.backwards_distance):
                self.stop_robot()
                self._result.success = True
                self._as.set_succeeded(True, text="Succeeded undocking")
                break
            
            self._as.publish_feedback(self._feedback)
            rate.sleep()

        self._as.publish_feedback(self._feedback)

if __name__ == '__main__':
    rospy.init_node('apriltag_docking')

    dock_vel = rospy.Publisher('/nav_vel', geometry_msgs.msg.Twist,queue_size=1)
    server = UnDockingAction(rospy.get_name())
    obstacle_sub = rospy.Subscriber("/collision_back/detections", Int16, server.obstacle_callback)
    rospy.spin()