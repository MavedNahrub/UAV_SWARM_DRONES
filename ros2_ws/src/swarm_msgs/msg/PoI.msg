string poi_id
geometry_msgs/Point position
uint8 priority
uint8 status
builtin_interfaces/Time created_stamp
builtin_interfaces/Time deadline_stamp
float32 survey_duration_s
string assigned_uav_id

# Priority constants
uint8 ROUTINE=1
uint8 URGENT=2
uint8 CRITICAL=3

# Status constants
uint8 PENDING=0
uint8 ASSIGNED=1
uint8 IN_PROGRESS=2
uint8 SURVEYED=3
uint8 EXPIRED=4
