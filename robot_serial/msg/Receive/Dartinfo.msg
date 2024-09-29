#0x0105 topic:/robot/dartinfo
uint8 dart_remaining_time#己方飞镖发射剩余时间，单位：秒
uint16 dart_aim_recently#最近一次己方飞镖击中的目标，开局默认为 0，1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机目标
uint16 dart_count_aim#对方最近被击中的目标累计被击中计数，开局默认为 0，至多为 4
uint16 dart_target_now#飞镖此时选定的击打目标，开局默认或未选定/选定前哨站时为 0，选中基地固定目标为 1，选中基地随机目标为 2