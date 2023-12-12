# Apriltag Docking ROS
![noetic badge](https://github.com/JosefGst/apriltag_docking_ros/actions/workflows/noetic.yaml/badge.svg)

Listens to tf published by e.g.apriltag node to get the position of the dock.

# Very simple aproach

    rosrun apriltag_docking_ros simple_apriltag_docking.py
    
Robot will steer directly to the docking tag.

TODO:
- [ ] send move base goal infront of dock
- [ ] use ros action

