uint8 TENTATIVE=0
uint8 CONFIRMED=1
uint8 LOST=2
uint8 PROCESSED=3

std_msgs/Header header
uint32 target_id
string class_name
float32 confidence
geometry_msgs/Point position
bool depth_valid
uint8 tracking_state
