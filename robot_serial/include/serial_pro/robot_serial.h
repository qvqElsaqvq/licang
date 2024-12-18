//
// Created by mijiao on 23-11-20.
//

#ifndef ROBOT_SERIAL_ROBOT_SERIAL_H
#define ROBOT_SERIAL_ROBOT_SERIAL_H

#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
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
    rclcpp::Subscription<robot_serial::msg::Location>::SharedPtr LocationErrorSubscription;
    rclcpp::Publisher<robot_serial::msg::Robotstatus>::SharedPtr RobotStatusPublisher;
    rclcpp::Subscription<robot_serial::msg::RotateVelocity>::SharedPtr RotateVelocitySubscription;
    
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
        static uint8_t SOF = 0x00;
        Velocity velocity{
                (float)(msg->linear.x),
                (float)(msg->linear.y),
                (float)(msg->angular.z),
        };
        SOF++;
        warehouseSerial.write(0x0501, SOF,velocity);
        //RCLCPP_INFO(this->get_logger(),"%f %f %f",msg->linear.x,msg->linear.y);
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
        RCLCPP_INFO(this->get_logger(), "if_navigation: %d,catch_decision: %d", msg->if_navigation, msg->catch_decision);
    }
    void locationerrorCallback(const robot_serial::msg::Location::SharedPtr msg)
    {
        static uint8_t SOF = 0x00;
        robot_position_t location_error{
            msg->x_err,
            msg->y_err,
            msg->angle_err,
        };
        SOF++;
        warehouseSerial.write(0x0503, SOF, location_error);
    }
    void rotatevelocityCallback(const robot_serial::msg::RotateVelocity::SharedPtr msg)
    {
        static uint8_t SOF = 0x00;
        rotate_velocity_t rotate_velocity{
            msg->rotate_velocity,
        };
        SOF++;
        warehouseSerial.write(0x0504, SOF, rotate_velocity);
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

        warehouseSerial.registerCallback(0x0303, [this](const robot_status_t& msg){
            robot_serial::msg::Robotstatus _Robotstatus;
            _Robotstatus.is_adjust = msg.is_adjust;
            _Robotstatus.is_finished = msg.is_finished;
            //RCLCPP_INFO(get_logger(), "is_adjust: %d is_finished: %d", msg.is_adjust, msg.is_finished);
            RobotStatusPublisher->publish(_Robotstatus);
        });

        VelocitySubscription = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, 
                        std::bind(&RobotSerial::velocityCallback, this, std::placeholders::_1));
        DecisionSubscription = create_subscription<robot_serial::msg::Decision>("/robot/decision", 1,
                        std::bind(&RobotSerial::decisionCallback, this, std::placeholders::_1));
        LocationErrorSubscription = create_subscription<robot_serial::msg::Location>("/robot/location_error",
                        1, std::bind(&RobotSerial::locationerrorCallback, this, std::placeholders::_1));
        RobotStatusPublisher = create_publisher<robot_serial::msg::Robotstatus>("/robot/robotstatus", 1);
        RotateVelocitySubscription = create_subscription<robot_serial::msg::RotateVelocity>("/robot/velocity",
                        1, std::bind(&RobotSerial::rotatevelocityCallback, this, std::placeholders::_1));

        warehouseSerial.spin(true);
    }
};
 
#endif //ROBOT_SERIAL_ROBOT_SERIAL_H
