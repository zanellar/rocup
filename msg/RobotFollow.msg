geometry_msgs/Pose target_pose
geometry_msgs/Pose tool
float64[] target_joint 
int8 action
int8 target_type
int8 target_source

int8 ACTION_SETTARGET=10
int8 ACTION_RESET=11
int8 ACTION_SETTOOL=12
int8 ACTION_ALARM=20
int8 ACTION_ALARMRESET=22
int8 ACTION_SETSOURCE=23

int8 TARGET_IN_JOINT=31
int8 TARGET_IN_POSE=32

int8 TARGET_FROM_TRAJECTORY_PLANNER=101
int8 TARGET_FROM_EXTERNAL_FEEDBACK=102
