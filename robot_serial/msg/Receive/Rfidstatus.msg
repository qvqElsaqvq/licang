#0x0209 topic:/robot/rfidstatus
#bit 位值为 1/0 的含义：是否已检测到该增益点 RFID 卡
uint32 base#己方基地增益点
uint32 cricle_hill_me#己方环形高地增益点
uint32 cricle_hill_he#对方环形高地增益点
uint32 r3_hill_me#己方 R3/B3 梯形高地增益点
uint32 r3_hill_he#对方 R3/B3 梯形高地增益点
uint32 r4_hill_me#己方 R4/B4 梯形高地增益点
uint32 r4_hill_he#对方 R4/B4 梯形高地增益点
uint32 energy#己方能量机关激活点
uint32 feipo_front_me#己方飞坡增益点（靠近己方一侧飞坡前）
uint32 feipo_back_me#己方飞坡增益点（靠近己方一侧飞坡后）
uint32 feipo_front_he#对方飞坡增益点（靠近对方一侧飞坡前）
uint32 feipo_back_he#对方飞坡增益点（靠近对方一侧飞坡后）
uint32 outpost#己方前哨站增益点
uint32 recovery#己方补血点（检测到任一均视为激活）
uint32 patrol_me#己方哨兵巡逻区
uint32 patrol_he#对方哨兵巡逻区
uint32 big_resource_island_me#己方大资源岛增益点
uint32 big_resource_island_he#对方大资源岛增益点
uint32 exchange#己方兑换区
uint32 centry#中心增益点（仅 RMUL 适用）
