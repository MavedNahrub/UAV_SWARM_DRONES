string uav_id
builtin_interfaces/Time stamp
geometry_msgs/Point position
float32 heading_rad
float32 speed_mps
float32 battery_frac
uint8 role
uint8 health
string assigned_poi_id
string[] bundle

# Role constants
uint8 SURVEYOR=0
uint8 RELAY=1
uint8 RTH=2
uint8 RESERVE=3
uint8 GROUNDED=4
uint8 IDLE=5

# Health constants
uint8 NOMINAL=0
uint8 DEGRADED=1
uint8 FAILED=2
