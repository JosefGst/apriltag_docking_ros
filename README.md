# Apriltag Docking ROS
![noetic badge](https://github.com/JosefGst/apriltag_docking_ros/actions/workflows/noetic.yaml/badge.svg)

Listens to tf published by e.g.apriltag node to get the position of the dock.

## Very simple aproach
Robot will steer directly to the docking tag.

    rosrun apriltag_docking_ros simple_apriltag_docking.py
    
## Use ROS Action and pure persuit controller 
Robot will aproach charger perpendicular.

    roslaunch apriltag_docking_ros apriltag_docking.launch
    roslaunch apriltag_docking_ros apriltag_undocking.launch

## Usage on TKO-robot
Pupblish the **/apriltag_docking/goal** topic and specify the "dock_tf_name". The robot will approach the named tf/tag.
Subscribe to **/apriltag_docking/status** and read the "status" value to check the operation.

## Parameters
- lookahead_dist_multiplier (double)
increasae the distance to the new waypoint
- rotational_vel_gain (double)
increase rotational velecity
- linear_vel_gain (double)
increase linear velecity
- goal_tolerance (double)
The tolerance in meters for the controller in the x & y distance when achieving a goal
- trans_bias
Can shift the goal to the right with positive values, negative to the left.
- docking_timeout
timeout in seconds. If docking takes longer then docking_timeout, abort.
- tag_on_ceiling (bool)
true if tag is mounted on the ceiling

## Docking
### Subscribed Topics
- /apriltag_docking/goal
    - dock_tf_name: 'dock_name'"

        dock_name is the apriltags tf name
### Published Topics
- /apriltag_docking/cancel
    - id: '/apriltag_docking-4-657.69000000'"

        The id need to match with the id form the feedpack topic.
        Will cancle the docking action.

- /apriltag_docking/feedback
    - distance
        
        distance to dock

- /apriltag_docking/result
- /apriltag_docking/status
    - status:
        - 1 processing
        - 2 preempted because new goal received
        - 3 goal reached
        - 4 aborted

## Unndocking
### Subscribed Topics
- /apriltag_undocking/goal
    - distance

        distance in meter the robot should move for undocking. Positive value makes the robot move forward, negative for backwards.
### Published Topics
- /apriltag_undocking/cancel
    - id: '/apriltag_docking-4-657.69000000'"

        The id need to match with the id form the feedpack topic.
        Will cancle the docking action.

- /apriltag_undocking/feedback
     - distance
        
        distance to dock
        
- /apriltag_docking/result
- /apriltag_docking/status
    - status:
        - 1 processing
        - 2 preempted because new goal received
        - 3 goal reached
        - 4 aborted

TODO:
- [ ] reusable for tags on ceiling

## Improvments

- [ ] use behavior tree
- [ ] make more generic
