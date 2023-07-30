# This includes the current status of robot exploration

uint8 DONE=0
uint8 ACTIVE=1
uint8 ERROR=2
uint8 INACTIVE=3

std_msgs/Header header
int8 robot_id
uint8 status 
geometry_msgs/Pose location     # Robot location
