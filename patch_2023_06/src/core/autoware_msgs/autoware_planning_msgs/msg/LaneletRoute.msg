std_msgs/Header header
geometry_msgs/Pose start_pose
geometry_msgs/Pose goal_pose
autoware_planning_msgs/LaneletSegment[] segments
unique_identifier_msgs/UUID uuid
bool allow_modification
# NAREN : added waypoint field for planning module to get the waypoints
geometry_msgs/Pose[] waypoints
