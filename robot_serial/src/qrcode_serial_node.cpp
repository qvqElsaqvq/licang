//
// Created by mijiao on 23-11-20.
//

#include "serial_pro/qrcode_serial.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QrcodeSerial>());
    rclcpp::shutdown();
    return 0;
}
