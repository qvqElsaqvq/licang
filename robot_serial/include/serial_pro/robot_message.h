//
// Created by mijiao on 23-11-20.
//

#ifndef ROBOT_SERIAL_ROBOT_MESSAGE_H
#define ROBOT_SERIAL_ROBOT_MESSAGE_H

#include "msg_serialize.h"

message_data Velocity{
    float v_x, v_y, v_w;
};

message_data Action{
    float max_pitch_w,min_pitch_w;
    float max_yaw_w,min_yaw_w;
    uint8_t mode;
    uint8_t spin;
};

message_data sentry_cmd_t{
    uint32_t sentry_cmd;
};

message_data custom_info_t{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
};
message_data map_data_t{
    uint8_t intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
    uint16_t sender_id;
};
message_data Whitelist{
    uint8_t robot[9];
};
message_data game_status_t{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
};
message_data game_result_t{
    uint8_t winner;
};
message_data game_robot_HP_t{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
};
message_data event_data_t{
    uint32_t front_recovery : 1;
    uint32_t inside_recovery : 1;
    uint32_t recovery_rmul : 1;
    uint32_t energy_organ : 1;
    uint32_t small_energy_organ_status : 1;
    uint32_t big_energy_organ_status : 1;
    uint32_t circular_plateau : 2;
    uint32_t trapezoidal_heights : 2;
    uint32_t trapezoidal_heights_2 : 2;
    uint32_t base_shield : 7;
    uint32_t time_of_dart_hit : 9;
    uint32_t target_of_dart_hit : 2;
    uint32_t centry_rmul : 2;
};
message_data ext_supply_projectile_action_t{
    uint8_t reserved;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
};
message_data referee_warning_t{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
};
message_data dart_info_t{
    uint8_t dart_remaining_time;
    uint16_t dart_aim_recently : 2;
    uint16_t dart_count_aim : 3;
    uint16_t dart_target_now : 2;
};
message_data robot_status_t{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
};
message_data power_heat_data_t{
    uint16_t chassis_voltage;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_17mm_2_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
};
message_data robot_pos_t{
    float x;
    float y;
    float angle;
};
message_data buff_t{
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
};
message_data hurt_data_t{
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;
};
message_data shoot_data_t{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
};
message_data projectile_allowance_t{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
};
message_data rfid_status_t{
    uint32_t base : 1;
    uint32_t cricle_hill_me : 1;
    uint32_t cricle_hill_he : 1;
    uint32_t r3_hill_me : 1;
    uint32_t r3_hill_he : 1;
    uint32_t r4_hill_me : 1;
    uint32_t r4_hill_he : 1;
    uint32_t energy : 1;
    uint32_t feipo_front_me : 1;
    uint32_t feipo_back_me : 1;
    uint32_t feipo_front_he : 1;
    uint32_t feipo_back_he : 1;
    uint32_t outpost : 1;
    uint32_t recovery : 1;
    uint32_t patrol_me : 1;
    uint32_t patrol_he : 1;
    uint32_t big_resource_island_me : 1;
    uint32_t big_resource_island_he : 1;
    uint32_t exchange : 1;
    uint32_t centry : 1;
    uint32_t reserve : 12;
};
message_data ground_robot_position_t{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float standard_5_x;
    float standard_5_y;
};
message_data sentry_info_t{
      uint32_t amount_of_get_ammunition : 11;
      uint32_t number_of_get_ammunition : 4;
      uint32_t number_of_get_blood : 4;
      uint32_t confirm_free_resurrection : 1;
      uint32_t confirm_resurrection_immediately : 1;
      uint32_t cost_of_resurrection : 10;
      uint32_t reserve : 1;
      uint16_t if_out_fight : 1;
      uint16_t projectile_allowance_17mm_of_team : 11;
      uint16_t reserve2 : 4;
};
message_data map_command_t{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint16_t cmd_source;
};
message_data robot_interaction_data_t{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[113];
};
message_data switch_t{
    uint8_t switch_1 : 1;
    uint8_t switch_2 : 1;
    uint8_t switch_3 : 1;
    uint8_t switch_4 : 1;
    uint8_t switch_5 : 1;
    uint8_t switch_6 : 1;
};
message_data sentry_decision_data_t{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint32_t sentry_cmd;
};
#endif //ROBOT_SERIAL_ROBOT_MESSAGE_H
