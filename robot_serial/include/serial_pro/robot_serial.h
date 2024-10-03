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

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr VelocitySubscription;
    rclcpp::Subscription<robot_serial::msg::Decision>::SharedPtr DecisionSubscription;
    
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
    void decisionCallback(const robot_serial::msg::Decision::SharedPtr msg){
        static uint8_t SOF = 0x00;
        decision_t decision{
            msg->if_navigation,
            msg->catch_decision,
            msg->qrcode_number,
        };
        SOF++;
        warehouseSerial.write(0x0502, SOF, decision);
        //RCLCPP_INFO(this->get_logger());
    }

public:
    explicit RobotSerial() : Node("robot_serial_node") {
        declare_parameter("/serial_name_warehouse", "/dev/c_serial");

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

        VelocitySubscription = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, 
                        std::bind(&RobotSerial::velocityCallback, this, std::placeholders::_1));
        DecisionSubscription = create_subscription<robot_serial::msg::Decision>("/robot/decision", 1,
                        std::bind(&RobotSerial::decisionCallback, this, std::placeholders::_1));

        warehouseSerial.spin(true);
    }
};
 
#endif //ROBOT_SERIAL_ROBOT_SERIAL_H
