#topic:/robot/action
#相对于雷达坐标系，规定逆时针为正，顺时针为负，弧度制
float32 max_pitch_w #pitch最大仰角
float32 min_pitch_w #pitch最小俯角
float32 max_yaw_w #yaw最大角度，0～PI
float32 min_yaw_w #yaw最小角度，0～-PI
uint8   mode #0：跟雷达 1：巡逻
uint8   spin #0：不小陀螺 1：小陀螺