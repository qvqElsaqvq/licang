//
// Created by mijiao on 23-11-20.
//

#ifndef ROBOT_SERIAL_ROBOT_SERIAL_H
#define ROBOT_SERIAL_ROBOT_SERIAL_H

#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "sentry_msg.h"
#include "robot_message.h"
#include "robot_referee.h"
#include "robot_sentry.h"

class RobotSerial : public rclcpp::Node {
private:
    //uint16_t sender_id = 0x107;
    //referee::RefereeSerial refereeSerial;
    sentry::SentrySerial sentrySerial;
    rclcpp::Clock rosClock;

    rclcpp::Publisher<robot_serial::msg::Buff>::SharedPtr BuffPublisher;
    rclcpp::Publisher<robot_serial::msg::Dartinfo>::SharedPtr DartinfoPublisher;
    rclcpp::Publisher<robot_serial::msg::Event>::SharedPtr EventPublisher;
    rclcpp::Publisher<robot_serial::msg::Gameresult>::SharedPtr GameresultPublisher;
    rclcpp::Publisher<robot_serial::msg::Gamestatus>::SharedPtr GamestatusPublisher;
    rclcpp::Publisher<robot_serial::msg::Hp>::SharedPtr HpPublisher;
    rclcpp::Publisher<robot_serial::msg::Hurtdata>::SharedPtr HurtdataPublisher;
    rclcpp::Publisher<robot_serial::msg::Interaction>::SharedPtr InteractionPublisher;
    rclcpp::Publisher<robot_serial::msg::Mapcommand>::SharedPtr MapcommandPublisher;
    rclcpp::Publisher<robot_serial::msg::Projectileallowance>::SharedPtr ProjectileallowancePublisher;
    rclcpp::Publisher<robot_serial::msg::Refereewarning>::SharedPtr RefereewarningPublisher;
    rclcpp::Publisher<robot_serial::msg::Rfidstatus>::SharedPtr RfidstatusPublisher;
    rclcpp::Publisher<robot_serial::msg::Robotp>::SharedPtr RobotpPublisher;
    rclcpp::Publisher<robot_serial::msg::Robotposition>::SharedPtr RobotpositionPublisher;
    rclcpp::Publisher<robot_serial::msg::Robotstatus>::SharedPtr RobotstatusPublisher;
    rclcpp::Publisher<robot_serial::msg::Sentryinfo>::SharedPtr SentryinfoPublisher;
    rclcpp::Publisher<robot_serial::msg::Supplyprojectile>::SharedPtr SupplyprojectilePublisher;
    rclcpp::Publisher<robot_serial::msg::Switch>::SharedPtr SwitchPublisher;

    // rclcpp::Publisher<robot_serial::msg::Decision>::SharedPtr DecisionPublisher;
    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr VelocityPublisher;
    // rclcpp::Publisher<robot_serial::msg::Info>::SharedPtr InfoPublisher;
    // rclcpp::Publisher<robot_serial::msg::Map>::SharedPtr MapPublisher;
    // rclcpp::Publisher<robot_serial::msg::Whitelist>::SharedPtr WhitelistPublisher;
    // rclcpp::Publisher<robot_serial::msg::Action>::SharedPtr ActionPublisher;

    rclcpp::Subscription<robot_serial::msg::Action>::SharedPtr ActionSubscription;
    rclcpp::Subscription<robot_serial::msg::Decision>::SharedPtr DecisionSubscription;
    rclcpp::Subscription<robot_serial::msg::Map>::SharedPtr MapSubscription;
    rclcpp::Subscription<robot_serial::msg::Info>::SharedPtr InfoSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr VelocitySubscription;
    rclcpp::Subscription<robot_serial::msg::Whitelist>::SharedPtr WhitelistSubscription;
    
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
        static uint8_t SOF = 0x00;
        Velocity velocity{
                (float)(msg->linear.x),
                (float)(msg->linear.y),
                (float)(msg->angular.z),
        };
        SOF++;
        sentrySerial.write(0x0501, SOF,velocity);
        RCLCPP_INFO(this->get_logger(),"%f %f %f",msg->linear.x,msg->linear.y,msg->angular.z);
    }
    void actionCallback(const robot_serial::msg::Action::SharedPtr msg){
        static uint8_t SOF = 0x00;
        Action action{
            msg->max_pitch_w,
            msg->min_pitch_w,
            msg->max_yaw_w,
            msg->min_yaw_w,
            msg->mode,
            msg->spin,
        };
        SOF++;
        sentrySerial.write(0x0502, SOF,action);
    }
    void sentry_cmd_Callback(const robot_serial::msg::Decision::SharedPtr msg){
        static uint8_t SOF = 0x00;
        sentry_cmd_t sentry_cmd;
        sentry_decision_data_t sentry_decision;

        sentry_cmd.sentry_cmd = sentry_cmd.sentry_cmd & 0b00000000000000000000000000000000;
        sentry_cmd.sentry_cmd = sentry_cmd.sentry_cmd | (((msg->resurrection) &  0b00000000000000000000000000000001)<<0);
        sentry_cmd.sentry_cmd = sentry_cmd.sentry_cmd | (((msg->immediate_resurrection) &  0b00000000000000000000000000000001)<<1);
        sentry_cmd.sentry_cmd = sentry_cmd.sentry_cmd | (((msg->buy_bullet_at_recovery) &  0b00000000000000000000011111111111)<<2);
        sentry_cmd.sentry_cmd = sentry_cmd.sentry_cmd | (((msg->buy_bullet_outside) &  0b00000000000000000000000000001111)<<13);
        sentry_cmd.sentry_cmd = sentry_cmd.sentry_cmd | (((msg->buy_blood) &  0b00000000000000000000000000001111)<<17);

        sentry_decision.data_cmd_id = 0x0120;
        sentry_decision.sender_id = msg->sender_id;
        sentry_decision.receiver_id = 0x8080;
        sentry_decision.sentry_cmd = sentry_cmd.sentry_cmd;
        SOF++;
        //std::cout<<sentry_cmd.sentry_cmd<<std::endl;
        sentrySerial.write(0x0301, SOF,sentry_decision);
    }
    void custom_info_Callback(const robot_serial::msg::Info::SharedPtr msg){
        static uint8_t SOF = 0x00;
        custom_info_t custom_info;
        custom_info.sender_id = msg->sender_id;
        custom_info.receiver_id = msg->receiver_id;
        const auto& user_data_array = msg->user_data;
        for (size_t i = 0; i < user_data_array.size(); ++i) {
            custom_info.user_data[i] = user_data_array[i];
        }
        SOF++;
        sentrySerial.write(0x0308, SOF,custom_info);
    }
    void map_data_Callback(const robot_serial::msg::Map::SharedPtr msg){
        static uint8_t SOF = 0x00;
        map_data_t map_data;
        map_data.intention = msg->intention;
        map_data.start_position_x = msg->start_position_x;
        map_data.start_position_y = msg->start_position_y;
        const auto& delta_x_array = msg->delta_x;
        for (size_t i = 0; i < delta_x_array.size(); ++i) {
            map_data.delta_x[i] = delta_x_array[i];
        }
        const auto& delta_y_array = msg->delta_y;
        for (size_t i = 0; i < delta_y_array.size(); ++i) {
            map_data.delta_y[i] = delta_y_array[i];
        }
        map_data.sender_id = msg->sender_id;  
        SOF++;
        sentrySerial.write(0x0307, SOF,map_data);
    }
    void Whitelist_Callback(const robot_serial::msg::Whitelist::SharedPtr msg){
        static uint8_t SOF = 0x00;
        Whitelist whitelist;
        const auto& robot_array = msg->robot;
        whitelist.robot[1] = robot_array[1];
        whitelist.robot[2] = robot_array[2];
        whitelist.robot[3] = robot_array[3];
        whitelist.robot[4] = robot_array[4];
        whitelist.robot[5] = robot_array[5];
        whitelist.robot[6] = robot_array[7];
        whitelist.robot[7] = robot_array[10];
        whitelist.robot[8] = robot_array[11];
        SOF++;
        sentrySerial.write(0x0506, SOF,whitelist);
    }

public:
    explicit RobotSerial() : Node("robot_serial_node") {
        //declare_parameter("/serial_name_referee", "/dev/referee_serial");
        declare_parameter("/serial_name_sentry", "/dev/sentry_serial");
        
        //refereeSerial = std::move(referee::RefereeSerial(get_parameter("/serial_name_referee").as_string(), 115200));
        sentrySerial = std::move(sentry::SentrySerial(get_parameter("/serial_name_sentry").as_string(), 115200));

        RCLCPP_INFO(this->get_logger(),"robot_serial init success");

        if(0){
            sentrySerial.registerErrorHandle([this](int label, const std::string& text) {
                sentry::SentrySerial::error _label;
                _label = (sentry::SentrySerial::error) label;
                std::stringstream _str;
                for (auto c: text) {
                    _str << std::setw(2) << std::hex << (int) *(uint8_t*) &c << " ";
                }
                _str << std::endl;
                switch (_label) {
                    case sentry::SentrySerial::lengthNotMatch:
                        RCLCPP_ERROR_STREAM(get_logger(), "sentry_lengthNotMatch");
                        RCLCPP_ERROR_STREAM(get_logger(), _str.str());
                    case sentry::SentrySerial::rxLessThanLength:
                        RCLCPP_ERROR_STREAM(get_logger(), "sentry_rxLessThanLength");
                        break;
                    case sentry::SentrySerial::crcError:
                        RCLCPP_ERROR_STREAM(get_logger(), "sentry_crc8Error");
                        RCLCPP_ERROR_STREAM(get_logger(), _str.str());
                        break;
                    case sentry::SentrySerial::crc16Error:
                        RCLCPP_ERROR_STREAM(get_logger(), "sentry_crc16Error");
                        RCLCPP_ERROR_STREAM(get_logger(), _str.str());
                        break;
                    default:
                        return;
                }
            });

            // refereeSerial.registerErrorHandle([this](int label, const std::string& text) {
            //     referee::RefereeSerial::error _label;
            //     _label = (referee::RefereeSerial::error) label;
            //     std::stringstream _str;
            //     for (auto c: text) {
            //         _str << std::setw(2) << std::hex << (int) *(uint8_t*) &c << " ";
            //     }
            //     _str << std::endl;
            //     switch (_label) {
            //         case referee::RefereeSerial::lengthNotMatch:
            //             RCLCPP_ERROR_STREAM(get_logger(), "referee_lengthNotMatch");
            //             RCLCPP_ERROR_STREAM(get_logger(), _str.str());
            //         case referee::RefereeSerial::rxLessThanLength:
            //             RCLCPP_ERROR_STREAM(get_logger(), "referee_rxLessThanLength");
            //             break;
            //         case referee::RefereeSerial::crcError:
            //             RCLCPP_ERROR_STREAM(get_logger(), "referee_crc8Error");
            //             RCLCPP_ERROR_STREAM(get_logger(), _str.str());
            //             break;
            //         case referee::RefereeSerial::crc16Error:
            //             RCLCPP_ERROR_STREAM(get_logger(), "referee_crc16Error");
            //             RCLCPP_ERROR_STREAM(get_logger(), _str.str());
            //             break;
            //         default:
            //             return;
            //     }
            // });

        }
        
        sentrySerial.registerCallback(0x0601,[this](const switch_t& msg){
            robot_serial::msg::Switch _Switch;
            _Switch.switch_1 = msg.switch_1;
            _Switch.switch_2 = msg.switch_2;
            _Switch.switch_3 = msg.switch_3;
            _Switch.switch_4 = msg.switch_4;
            _Switch.switch_5 = msg.switch_5;
            _Switch.switch_6 = msg.switch_6;
            SwitchPublisher->publish(_Switch);
        });

        sentrySerial.registerCallback(0x0001,[this](const game_status_t& msg){
            robot_serial::msg::Gamestatus _Gamestatus;
            _Gamestatus.game_type = msg.game_type;
            _Gamestatus.game_progress = msg.game_progress;
            _Gamestatus.stage_remain_time = msg.stage_remain_time;
            _Gamestatus.synctimestamp = msg.SyncTimeStamp;
             GamestatusPublisher->publish(_Gamestatus);
        });
        sentrySerial.registerCallback(0x0002,[this](const game_result_t& msg){
            robot_serial::msg::Gameresult _Gameresult;
            _Gameresult.winner = msg.winner;
            GameresultPublisher->publish(_Gameresult);
        });
        sentrySerial.registerCallback(0x0003,[this](const game_robot_HP_t& msg){
            robot_serial::msg::Hp _Hp;
            _Hp.red_1_robot_hp = msg.red_1_robot_HP;
            _Hp.red_2_robot_hp = msg.red_2_robot_HP;
            _Hp.red_3_robot_hp = msg.red_3_robot_HP;
            _Hp.red_4_robot_hp = msg.red_4_robot_HP;
            _Hp.red_5_robot_hp = msg.red_5_robot_HP;
            _Hp.red_7_robot_hp = msg.red_7_robot_HP;
            _Hp.red_outpost_hp = msg.red_outpost_HP;
            _Hp.red_base_hp = msg.red_base_HP;
            _Hp.blue_1_robot_hp = msg.blue_1_robot_HP;
            _Hp.blue_2_robot_hp = msg.blue_2_robot_HP;
            _Hp.blue_3_robot_hp = msg.blue_3_robot_HP;
            _Hp.blue_4_robot_hp = msg.blue_4_robot_HP;
            _Hp.blue_5_robot_hp = msg.blue_5_robot_HP;
            _Hp.blue_7_robot_hp = msg.blue_7_robot_HP;
            _Hp.blue_outpost_hp = msg.blue_outpost_HP;
            _Hp.blue_base_hp = msg.blue_base_HP;
            HpPublisher->publish(_Hp);
        });
        sentrySerial.registerCallback(0x0101,[this](const event_data_t& msg){
            robot_serial::msg::Event _Event;
            _Event.front_recovery = msg.front_recovery;
            _Event.inside_recovery = msg.inside_recovery;
            _Event.recovery_rmul = msg.recovery_rmul;
            _Event.energy_organ = msg.energy_organ;
            _Event.small_energy_organ_status = msg.small_energy_organ_status;
            _Event.big_energy_organ_status = msg.big_energy_organ_status;
            _Event.circular_plateau = msg.circular_plateau;
            _Event.trapezoidal_heights = msg.trapezoidal_heights;
            _Event.trapezoidal_heights_2 = msg.trapezoidal_heights_2;
            _Event.base_shield = msg.base_shield;
            _Event.time_of_dart_hit = msg.time_of_dart_hit;
            _Event.target_of_dart_hit = msg.target_of_dart_hit;
            _Event.centry_rmul = msg.centry_rmul;
            EventPublisher->publish(_Event);
        });
            
        sentrySerial.registerCallback(0x0102,[this](const ext_supply_projectile_action_t& msg){
            robot_serial::msg::Supplyprojectile _Supplyprojectile;
            _Supplyprojectile.reserved = msg.reserved;
            _Supplyprojectile.supply_robot_id = msg.supply_robot_id;
            _Supplyprojectile.supply_projectile_step = msg.supply_projectile_step;
            _Supplyprojectile.supply_projectile_num = msg.supply_projectile_num;
            SupplyprojectilePublisher->publish(_Supplyprojectile);
        });
        sentrySerial.registerCallback(0x0104,[this](const referee_warning_t& msg){
            robot_serial::msg::Refereewarning _Refereewarning;
            _Refereewarning.level = msg.level;
            _Refereewarning.offending_robot_id = msg.offending_robot_id;
            _Refereewarning.count = msg.count;
            RefereewarningPublisher->publish(_Refereewarning);
        });
        sentrySerial.registerCallback(0x0105,[this](const dart_info_t& msg){
            robot_serial::msg::Dartinfo _Dartinfo;
            _Dartinfo.dart_remaining_time = msg.dart_remaining_time;
            _Dartinfo.dart_aim_recently = msg.dart_aim_recently;
            _Dartinfo.dart_count_aim = msg.dart_count_aim;
            _Dartinfo.dart_target_now = msg.dart_target_now;
            DartinfoPublisher->publish(_Dartinfo);
        });
        sentrySerial.registerCallback(0x0201,[this](const robot_status_t& msg){
            robot_serial::msg::Robotstatus _Robotstatus;
            _Robotstatus.robot_id = msg.robot_id;
            _Robotstatus.robot_level = msg.robot_level;
            _Robotstatus.current_hp = msg.current_HP;
            _Robotstatus.maximum_hp = msg.maximum_HP;
            _Robotstatus.shooter_barrel_cooling_value = msg.shooter_barrel_cooling_value;
            _Robotstatus.shooter_barrel_heat_limit = msg.shooter_barrel_heat_limit;
            _Robotstatus.chassis_power_limit = msg.chassis_power_limit;
            _Robotstatus.power_management_gimbal_output = msg.power_management_gimbal_output;
            _Robotstatus.power_management_chassis_output = msg.power_management_chassis_output;
            _Robotstatus.power_management_shooter_output = msg.power_management_shooter_output;
            RobotstatusPublisher->publish(_Robotstatus);
        });
        sentrySerial.registerCallback(0x0202,[this](const power_heat_data_t &msg){});
        sentrySerial.registerCallback(0x0203,[this](const robot_pos_t& msg){
            robot_serial::msg::Robotp _Robotp;
            _Robotp.x = msg.x;
            _Robotp.y = msg.y;
            _Robotp.angle = msg.angle;
            RobotpPublisher->publish(_Robotp);
         });
        sentrySerial.registerCallback(0x0204,[this](const buff_t& msg){
            robot_serial::msg::Buff _Buff;
            _Buff.recovery_buff = msg.recovery_buff;
            _Buff.cooling_buff = msg.cooling_buff;
            _Buff.defence_buff = msg.defence_buff;
            _Buff.vulnerability_buff = msg.vulnerability_buff;
            _Buff.attack_buff = msg.attack_buff;
            BuffPublisher->publish(_Buff);
        });
        sentrySerial.registerCallback(0x0206,[this](const hurt_data_t& msg){
            robot_serial::msg::Hurtdata _Hurtdata;
            _Hurtdata.armor_id = msg.armor_id;
            _Hurtdata.hp_deduction_reason = msg.HP_deduction_reason;
            HurtdataPublisher->publish(_Hurtdata);
        });
        sentrySerial.registerCallback(0x0208,[this](const projectile_allowance_t& msg){
            robot_serial::msg::Projectileallowance _Projectileallowance;
            _Projectileallowance.projectile_allowance_17mm = msg.projectile_allowance_17mm;
            _Projectileallowance.projectile_allowance_42mm = msg.projectile_allowance_42mm;
            _Projectileallowance.remaining_gold_coin = msg.remaining_gold_coin;
            ProjectileallowancePublisher->publish(_Projectileallowance);
        });
        sentrySerial.registerCallback(0x0209,[this](const rfid_status_t& msg){
            robot_serial::msg::Rfidstatus _Rfidstatus;
            _Rfidstatus.base = msg.base;
            _Rfidstatus.cricle_hill_me = msg.cricle_hill_me;
            _Rfidstatus.cricle_hill_he = msg.cricle_hill_he;
            _Rfidstatus.r3_hill_me = msg.r3_hill_me;
            _Rfidstatus.r3_hill_he = msg.r3_hill_he;
            _Rfidstatus.r4_hill_me = msg.r4_hill_me;
            _Rfidstatus.r4_hill_he = msg.r4_hill_he;
            _Rfidstatus.energy = msg.energy;
            _Rfidstatus.feipo_front_me = msg.feipo_front_me;
            _Rfidstatus.feipo_back_me = msg.feipo_back_me;
            _Rfidstatus.feipo_front_he = msg.feipo_front_he;
            _Rfidstatus.feipo_back_he = msg.feipo_back_he;
            _Rfidstatus.outpost = msg.outpost;
            _Rfidstatus.recovery = msg.recovery;
            _Rfidstatus.patrol_me = msg.patrol_me;
            _Rfidstatus.patrol_he = msg.patrol_he;
            _Rfidstatus.big_resource_island_me = msg.big_resource_island_me;
            _Rfidstatus.big_resource_island_he = msg.big_resource_island_he;
            _Rfidstatus.exchange = msg.exchange;
            _Rfidstatus.centry = msg.centry;
            RfidstatusPublisher->publish(_Rfidstatus);
        });
        sentrySerial.registerCallback(0x020B,[this](const ground_robot_position_t& msg){
            robot_serial::msg::Robotposition _Robotposition;
            _Robotposition.hero_x = msg.hero_x;
            _Robotposition.hero_y = msg.hero_y;
            _Robotposition.engineer_x = msg.engineer_x;
            _Robotposition.engineer_y = msg.engineer_y;
            _Robotposition.standard_3_x = msg.standard_3_x;
            _Robotposition.standard_3_y = msg.standard_3_y;
            _Robotposition.standard_4_x = msg.standard_4_x;
            _Robotposition.standard_4_y = msg.standard_4_y;
            _Robotposition.standard_5_x = msg.standard_5_x;
            _Robotposition.standard_5_y = msg.standard_5_y;
            RobotpositionPublisher->publish(_Robotposition);
        });
        sentrySerial.registerCallback(0x020D,[this](const sentry_info_t& msg){
            robot_serial::msg::Sentryinfo _Sentryinfo;
            _Sentryinfo.amount_of_get_ammunition = msg.amount_of_get_ammunition;
            _Sentryinfo.number_of_get_ammunition = msg.number_of_get_ammunition;
            _Sentryinfo.number_of_get_blood = msg.number_of_get_blood;
            _Sentryinfo.confirm_free_resurrection = msg.confirm_free_resurrection;
            _Sentryinfo.confirm_resurrection_immediately = msg.confirm_resurrection_immediately;
            _Sentryinfo.cost_of_resurrection = msg.cost_of_resurrection;
            _Sentryinfo.if_out_fight = msg.if_out_fight;
            _Sentryinfo.projectile_allowance_17mm_of_team = msg.projectile_allowance_17mm_of_team;
            SentryinfoPublisher->publish(_Sentryinfo);
        });
        sentrySerial.registerCallback(0x0301,[this](const robot_interaction_data_t& msg){
            robot_serial::msg::Interaction _Interaction;
            _Interaction.data_cmd_id = msg.data_cmd_id;
            _Interaction.sender_id = msg.sender_id;
            _Interaction.receiver_id = msg.receiver_id;
            for(int i = 0;i<112;i++)
                _Interaction.user_data[i] = msg.user_data[i];
            InteractionPublisher->publish(_Interaction);
        });
        sentrySerial.registerCallback(0x0303,[this](const map_command_t& msg){
            robot_serial::msg::Mapcommand _Mapcommand;
            _Mapcommand.target_position_x = msg.target_position_x;
            _Mapcommand.target_position_y = msg.target_position_y;
            _Mapcommand.cmd_keyboard = msg.cmd_keyboard;
            _Mapcommand.target_robot_id = msg.target_robot_id;
            _Mapcommand.cmd_source = msg.cmd_source;
            MapcommandPublisher->publish(_Mapcommand);
        });

        BuffPublisher = create_publisher<robot_serial::msg::Buff>("/robot/buff", 1);
        DartinfoPublisher = create_publisher<robot_serial::msg::Dartinfo>("/robot/dartinfo", 1);
        EventPublisher = create_publisher<robot_serial::msg::Event>("/robot/event", 1);
        GameresultPublisher = create_publisher<robot_serial::msg::Gameresult>("/robot/gameresult", 1);
        GamestatusPublisher = create_publisher<robot_serial::msg::Gamestatus>("/robot/gamestatus", 1);
        HpPublisher = create_publisher<robot_serial::msg::Hp>("/robot/hp", 1);
        HurtdataPublisher = create_publisher<robot_serial::msg::Hurtdata>("/robot/hurtdata", 1);
        InteractionPublisher = create_publisher<robot_serial::msg::Interaction>("/robot/interaction", 1);
        MapcommandPublisher = create_publisher<robot_serial::msg::Mapcommand>("/robot/mapcommand", 1);
        ProjectileallowancePublisher = create_publisher<robot_serial::msg::Projectileallowance>("/robot/projectileallowance", 1);
        RefereewarningPublisher = create_publisher<robot_serial::msg::Refereewarning>("/robot/refereewarning", 1);
        RfidstatusPublisher = create_publisher<robot_serial::msg::Rfidstatus>("/robot/rfidstatus", 1);
        RobotpPublisher = create_publisher<robot_serial::msg::Robotp>("/robot/robotp", 1);
        RobotpositionPublisher = create_publisher<robot_serial::msg::Robotposition>("/robot/robotposition", 1);
        RobotstatusPublisher = create_publisher<robot_serial::msg::Robotstatus>("/robot/robotstatus", 1);
        SentryinfoPublisher = create_publisher<robot_serial::msg::Sentryinfo>("/robot/sentryinfo", 1);
        SupplyprojectilePublisher = create_publisher<robot_serial::msg::Supplyprojectile>("/robot/supplyprojectile", 1);
        SwitchPublisher = create_publisher<robot_serial::msg::Switch>("/robot/switch", 1);

        // DecisionPublisher = create_publisher<robot_serial::msg::Decision>("/robot/decision", 1);
        // VelocityPublisher = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        // InfoPublisher = create_publisher<robot_serial::msg::Info>("/robot/info", 1);
        // MapPublisher = create_publisher<robot_serial::msg::Map>("/robot/map", 1);
        // WhitelistPublisher = create_publisher<robot_serial::msg::Whitelist>("/robot/whitelist", 1);
        // ActionPublisher = create_publisher<robot_serial::msg::Action>("/robot/action", 1);

        VelocitySubscription = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, 
                                                                          std::bind(&RobotSerial::velocityCallback, this,
                                                                                    std::placeholders::_1));
        DecisionSubscription = create_subscription<robot_serial::msg::Decision>("/robot/decision", 1, 
                                                                          std::bind(&RobotSerial::sentry_cmd_Callback, this,
                                                                                    std::placeholders::_1));
        ActionSubscription = create_subscription<robot_serial::msg::Action>("/robot/action", 1,
                                                                            std::bind(&RobotSerial::actionCallback, this,
                                                                                        std::placeholders::_1));
        InfoSubscription = create_subscription<robot_serial::msg::Info>("/robot/info", 1,
                                                                        std::bind(&RobotSerial::custom_info_Callback, this,
                                                                                    std::placeholders::_1));
        MapSubscription = create_subscription<robot_serial::msg::Map>("/robot/map", 1,
                                                                        std::bind(&RobotSerial::map_data_Callback, this,
                                                                                    std::placeholders::_1));
        WhitelistSubscription = create_subscription<robot_serial::msg::Whitelist>("/robot/whitelist", 1,
                                                                         std::bind(&RobotSerial::Whitelist_Callback, this,
                                                                                     std::placeholders::_1));
        //refereeSerial.spin(true);   

        sentrySerial.spin(true);
    }
};
 
#endif //ROBOT_SERIAL_ROBOT_SERIAL_H
