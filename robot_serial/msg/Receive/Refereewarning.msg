#0x0104 #/robot/refereewarning
uint8 level#己方最后一次受到判罚的等级：
#1：双方黄牌
#2：黄牌
#3：红牌
#4：判负
uint8 offending_robot_id#己方最后一次受到判罚的违规机器人 ID。（如红 1 机器人 ID 为 1，蓝1 机器人 ID 为 101）,判负和双方黄牌时，该值为 0
uint8 count#己方最后一次受到判罚的违规机器人对应判罚等级的违规次数。（开局默认为 0）