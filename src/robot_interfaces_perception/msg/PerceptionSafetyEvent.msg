string CLEAR="CLEAR"
string PERSON_NEAR="PERSON_NEAR"
string PERSON_TOO_CLOSE="PERSON_TOO_CLOSE"
string PERSON_IN_DANGER_ZONE="PERSON_IN_DANGER_ZONE"

uint8 STATE_CLEAR=0
uint8 STATE_SPEED_LIMITED=1
uint8 STATE_STOP=2

uint8 SEVERITY_INFO=0
uint8 SEVERITY_WARNING=1
uint8 SEVERITY_CRITICAL=2

std_msgs/Header header
uint32 target_id
string class_name
string event_type
uint8 safety_state
uint8 severity
geometry_msgs/Point target_position
float32 distance_m
string zone_id
string source
string reason
