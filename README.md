# Apriltag Docking ROS
![noetic badge](https://github.com/JosefGst/apriltag_docking_ros/actions/workflows/noetic.yaml/badge.svg)

Listens to tf published by e.g.apriltag node to get the position of the dock.

## Very simple aproach
Robot will steer directly to the docking tag.

    rosrun apriltag_docking_ros simple_apriltag_docking.py
    
## Use ROS Action and pure persuit controller 
Robot will aproach charger perpendicular.

    rosrun apriltag_docking_ros apriltag_docking_action_server.py

## Usage on TKO-robot
Pupblish the **/apriltag_docking/goal** topic and specify the "dock_tf_name". The robot will approach the named tf/tag.
Subscribe to **/apriltag_docking/status** and read the "status" value to check the operation.

## Topics
### /apriltag_docking/cancel
publish:
- id: '/apriltag_docking-4-657.69000000'"
The id need to match with the id form the feedpack topic.
Will cancle the docking action.
### /apriltag_docking/feedback
subscribe:
- distance: distance to dock
### /apriltag_docking/goal
publish:
- dock_tf_name: 'dock_name'"
    dock_name is the apriltags tf name
### /apriltag_docking/result
### /apriltag_docking/status
subscribe:
- distance: distance to dock
- status:
    - 1 processing
    - 2 preempted because new goal received
    - 3 goal reached
    - 4 aborted


TODO:
- [x] use pure persuit like controller
- [x] use ros action
- [ ] obstacle detection

