#0x0301 topic:/robot/interaction
uint16 data_cmd_id#需为开放的子内容 ID
uint16 sender_id#需与自身 ID 匹配
uint16 receiver_id#仅限己方通信,需为规则允许的多机通讯接收者,若接收者为选手端，则仅可发送至发送者对应的选手端
uint8[112] user_data#内容数据段