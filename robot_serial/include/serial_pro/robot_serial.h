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
#include "robot_warehouse.h"

class RobotSerial : public rclcpp::Node {
private:
    warehouse::WarehouseSerial warehouseSerial;
    rclcpp::Clock rosClock;

    rclcpp::Publisher<robot_serial::msg::Mapcommand>::SharedPtr MapcommandPublisher;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr VelocitySubscription;
    
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
        static uint8_t SOF = 0x00;
        Velocity velocity{
                (float)(msg->linear.x),
                (float)(msg->linear.y),
                (float)(msg->angular.z),
        };
        SOF++;
        warehouseSerial.write(0x0501, SOF,velocity);
        RCLCPP_INFO(this->get_logger(),"%f %f %f",msg->linear.x,msg->linear.y,msg->angular.z);
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
        warehouseSerial.write(0x0307, SOF,map_data);
    }

public:
    explicit RobotSerial() : Node("robot_serial_node") {
        declare_parameter("/serial_name_warehouse", "/dev/sentry_serial");

        warehouseSerial = std::move(warehouse::WarehouseSerial(get_parameter("/serial_name_warehouse").as_string(), 115200));

        RCLCPP_INFO(this->get_logger(),"robot_serial init success");

        if(0){
            warehouseSerial.registerErrorHandle([this](int label, const std::string& text) {
                warehouse::WarehouseSerial::error _label;
                _label = (warehouse::WarehouseSerial::error) label;
                std::stringstream _str;
                for (auto c: text) {
                    _str << std::setw(2) << std::hex << (int) *(uint8_t*) &c << " ";
                }
                _str << std::endl;
                switch (_label) {
                    case warehouse::WarehouseSerial::lengthNotMatch:
                        RCLCPP_ERROR_STREAM(get_logger(), "sentry_lengthNotMatch");
                        RCLCPP_ERROR_STREAM(get_logger(), _str.str());
                    case warehouse::WarehouseSerial::rxLessThanLength:
                        RCLCPP_ERROR_STREAM(get_logger(), "sentry_rxLessThanLength");
                        break;
                    case warehouse::WarehouseSerial::crcError:
                        RCLCPP_ERROR_STREAM(get_logger(), "sentry_crc8Error");
                        RCLCPP_ERROR_STREAM(get_logger(), _str.str());
                        break;
                    case warehouse::WarehouseSerial::crc16Error:
                        RCLCPP_ERROR_STREAM(get_logger(), "sentry_crc16Error");
                        RCLCPP_ERROR_STREAM(get_logger(), _str.str());
                        break;
                    default:
                        return;
                }
            });
        }

        warehouseSerial.registerCallback(0x0303,[this](const map_command_t& msg){
            robot_serial::msg::Mapcommand _Mapcommand;
            _Mapcommand.target_position_x = msg.target_position_x;
            _Mapcommand.target_position_y = msg.target_position_y;
            _Mapcommand.cmd_keyboard = msg.cmd_keyboard;
            _Mapcommand.target_robot_id = msg.target_robot_id;
            _Mapcommand.cmd_source = msg.cmd_source;
            MapcommandPublisher->publish(_Mapcommand);
        });

        MapcommandPublisher = create_publisher<robot_serial::msg::Mapcommand>("/robot/mapcommand", 1);

        VelocitySubscription = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, 
                                                                          std::bind(&RobotSerial::velocityCallback, this,
                                                                                    std::placeholders::_1));
        warehouseSerial.spin(true);
    }
};
 
#endif //ROBOT_SERIAL_ROBOT_SERIAL_H
