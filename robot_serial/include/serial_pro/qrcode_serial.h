#ifndef ROBOT_SERIAL_QRCODE_SERIAL_H
#define ROBOT_SERIAL_QRCODE_SERIAL_H

#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "sentry_msg.h"
#include "robot_message.h"
#include "robot_referee.h"
#include "robot_warehouse.h"

class QrcodeSerial : public rclcpp::Node {
private:
    warehouse::WarehouseSerial warehouseSerial;
    rclcpp::Clock rosClock;

    rclcpp::Publisher<robot_serial::msg::Qrcodeinfo>::SharedPtr QrcodeinfoPublisher;

public:
    explicit QrcodeSerial() : Node("qrcode_serial_node") {
        declare_parameter("/serial_name_qrcode", "/dev/qrcode_serial");

        warehouseSerial = std::move(warehouse::WarehouseSerial(get_parameter("/serial_name_qrcode").as_string(), 115200));

        RCLCPP_INFO(this->get_logger(),"qrcode_serial init success");

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

        warehouseSerial.registerCallback(0x0302, [this](const qrcode_ball_t& msg){
            robot_serial::msg::Qrcodeinfo _Qrcodeinfo;
            _Qrcodeinfo.qrcode_ball = msg.qrcode_ball;
            QrcodeinfoPublisher->publish(_Qrcodeinfo);
        });

        QrcodeinfoPublisher = create_publisher<robot_serial::msg::Qrcodeinfo>("/robot/qrcodeinfo", 1);

        warehouseSerial.spin(true);
    }
};

#endif //ROBOT_SERIAL_QRCODE_SERIAL_H
