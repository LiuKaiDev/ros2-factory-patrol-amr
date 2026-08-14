string TARGET_CONFIRMED="TARGET_CONFIRMED"
string INSPECTION_REQUIRED="INSPECTION_REQUIRED"
string INSPECTION_COMPLETED="INSPECTION_COMPLETED"

uint8 SEVERITY_INFO=0
uint8 SEVERITY_WARNING=1

std_msgs/Header header
uint32 target_id
string event_type
string class_name
geometry_msgs/PoseStamped target_pose
float32 confidence
uint8 severity
