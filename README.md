# RRT Exploration

Given a dynamic global map and the locations of multiple robots within the map, this package issues navigation goals to each robot in order to collaboratively explore the environment. 

This is a ROS2 port of the RRT exploration algorithm in SMMR Explore repository https://github.com/efc-robot/SMMR-Explore. Several parts have been optimised for performance and memory handling. 

## Dependencies
* ROS2

>  This package has been tested on ROS2 Humble, Ubuntu 22.04
  
**Compiling** <br /> 
>      colcon build --packages-up-to rrt --symlink-install

**Running Exploration** <br /> 
>      ros2 launch rrt rrt.launch.py namespace:= <robot_namespace> 

#### Note
The rrt nodes should be run on all robots in the system. <robot_namespace> should be replaced by the namespace of the respective robot for each instance.

### Subscribed Topics
| Topic | msg Type | Description |
| --- | --- | --- |
| /rrt/enable | std_msgs/msg/Bool | Send Command to Start / Stop RRT Exploration
| /<robot_namespace>/map | nav_msgs/msg/OccupancyGrid | Merged map of all robot local maps (realtime) |
| /<robot_namespace>/global_costmap/costmap | nav_msgs/msg/OccupancyGrid | Global costmap generated by the navigation stack |
| /<robot_namespace>/tf <br />  /<robot_namespace>/tf_static <br /> | tf2_msgs/msg/TFMessage | TF topic of the robot. TF topic should include transformation between the merged_map frame and namespaced base_frames of all robots (eg. Transforms between world, robot1/base_link, robot2/base_link frames)|

  
### Published Topics
 Topic | msg Type | Description |
| --- | --- | --- |
| /<robot_namespace>/frontier_targets | visualization_msgs/msg/Marker | Detected frontier points by RRT
| /<robot_namespace>/rrt_segments | visualization_msgs/msg/Marker | RRT visualization (tree)
| /<robot_namespace>/end_detection_frontiers | visualization_msgs/msg/Marker | Remaining frontiers (Discovered via full map grid search. Only used for visualization and exploration completion detection.)
| /exploration_state | rrt_explore::msg::ExplorationState | Exploration state
### Parameters
* `map_topic` : Merged map topic (default: map)
* `costmap_topic` : Planner server costmap topic (default: global_costmap/costmap)
* `robot_base_frame` : Base frame of the robot, excluding the namespace (default: base_footprint). 
* `robot_frame_prefix` : Robot namespace prefix (default: ssugv). The namespaces of all the robots should comprise of a common prefix and an index number (i.e. robot1, robot2)
* `rate` : Exploration goal generation rate in Hz. Frontier calculation is also performed at this rate. (default: 0.3)
* `robot_count` : Number of robots in the system. Minimum 1.
* `info_radius` : (default: 1.0)
* `costmap_pixel_threshold` : (default: 30.0)
* `eta` : (default: 0.5)
* `hysteresis_radius` : (default: 3.0) 
* `hysteresis_radius` : (default: 2.0)
