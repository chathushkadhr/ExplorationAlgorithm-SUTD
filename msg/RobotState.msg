#################################################################
## This message is used to transport a graph between two nodes.
#################################################################

std_msgs/Header header # Header for Timestamp and Id 
int8 id # RobotID
geometry_msgs/Point location # Location(in pixel)
