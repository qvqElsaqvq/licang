#0x0102 topic:/robot/supplyprojectile
uint8 reserved#保留
uint8 supply_robot_id#补弹机器人 ID：
#0当前无机器人补弹
#1红方英雄机器人补弹
#3/4/5：红方步兵机器人补弹
#101：蓝方英雄机器人补弹
#103/104/105：蓝方步兵机器人补弹
uint8 supply_projectile_step#出弹口开闭状态：
#0关闭
#1：弹丸准备中
#2：弹丸释放
uint8 supply_projectile_num#补弹数量：
#50：50 颗弹丸
#100：100 颗弹丸
#150：150 颗弹丸
#200：200 颗弹丸