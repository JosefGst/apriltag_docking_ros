#!/usr/bin/env python3
# license removed for brevity
import rospy
import apriltag_docking_ros.msg

def apriltag_docking_publisher():
    pub = rospy.Publisher('apriltag_docking/goal', apriltag_docking_ros.msg.DockingActionGoal, queue_size=10)
    rospy.init_node('apriltag_docking_publisher', anonymous=True)
    while not rospy.is_shutdown():
        goal = apriltag_docking_ros.msg.DockingActionGoal()
        goal.goal.dock_tf_name = "CHARGER"
        pub.publish(goal)
        return True

if __name__ == '__main__':
    try:
        apriltag_docking_publisher()
    except rospy.ROSInterruptException:
        pass