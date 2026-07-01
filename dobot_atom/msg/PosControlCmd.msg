uint8 SWITCH_MODE=0
uint8 MOVE_L=1
uint8 MOVE_J=2
uint8 RUN_TO=3
uint8 CART_JOG=4
uint8 STOP_MOTION=5

uint8 command_type

float64[<=7] left_q
float64[<=7] right_q
float64[<=3] hips_q

float64[<=6] left_pose
float64[<=6] right_pose
float64[<=1] hips_height

int32[<=6] left_cart_jog
int32[<=6] right_cart_jog
int32[<=1] hips_cart_jog

int32 stop
float64 speed
int32 target_mode

uint32 command_id
uint64 timestamp
