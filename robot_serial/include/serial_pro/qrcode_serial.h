#ifndef ROBOT_SERIAL_QRCODE_SERIAL_H
#define ROBOT_SERIAL_QRCODE_SERIAL_H

#include <rclcpp/rclcpp.hpp>
#include "sentry_msg.h"
#include "robot_message.h"
#include "robot_referee.h"
#include "robot_qrcode.h"

class QrcodeSerial : public rclcpp::Node {
private:
    qrcode::CodeSerial qrcodeSerial;

    rclcpp::Publisher<robot_serial::msg::Qrcodeinfo>::SharedPtr QrcodeinfoPublisher;

public:
    QrcodeSerial() : Node("qrcode_serial_node") {
        declare_parameter("/serial_name_qrcode", "/dev/qrcode_serial");

        qrcodeSerial = std::move(qrcode::CodeSerial(get_parameter("/serial_name_qrcode").as_string(), 115200));

        RCLCPP_INFO(this->get_logger(),"qrcode_serial init success");

        qrcodeSerial.registerCallback(0x00, [this](const qrcode_ball_t& msg){
            RCLCPP_INFO(get_logger(), "qrcode_serial received data: %d", msg.qrcode_ball);
            robot_serial::msg::Qrcodeinfo _Qrcodeinfo;
            _Qrcodeinfo.qrcode_ball = msg.qrcode_ball;
            QrcodeinfoPublisher->publish(_Qrcodeinfo);
        });

        QrcodeinfoPublisher = create_publisher<robot_serial::msg::Qrcodeinfo>("/robot/qrcodeinfo", 1);

        qrcodeSerial.spin(true);
    }
};

#endif //ROBOT_SERIAL_QRCODE_SERIAL_H
