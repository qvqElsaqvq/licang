#0x0101 topic:/robot/event
#0：未占领/未激活
#1：已占领/已激活
uint32 front_recovery #己方补给站前补血点的占领状态，1 为已占领
uint32 inside_recovery#己方补给站内部补血点的占领状态，1 为已占领
uint32 recovery_rmul#己方补给区的占领状态，1 为已占领（仅 RMUL 适用）
uint32 energy_organ#己方能量机关激活点的占领状态，1 为已占领
uint32 small_energy_organ_status#己方小能量机关的激活状态，1 为已激活
uint32 big_energy_organ_status#己方大能量机关的激活状态，1 为已激活
uint32 circular_plateau#己方环形高地的占领状态，1 为被己方占领，2 为被对方占领
uint32 trapezoidal_heights#己方梯形高地的占领状态，1 为被己方占领，2 为被对方占领
uint32 trapezoidal_heights_2#己方梯形高地的占领状态，1 为被己方占领，2 为被对方占领
uint32 base_shield#己方基地虚拟护盾的剩余值百分比（四舍五入，保留整数）
uint32 time_of_dart_hit#飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为0）
uint32 target_of_dart_hit#飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为 0，1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机目标
uint32 centry_rmul#中心增益点的占领情况，0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领。（仅 RMUL 适用）