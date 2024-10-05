//
// Created by mijiao on 23-11-20.
//

#ifndef ROBOT_SERIAL_ROBOT_MESSAGE_H
#define ROBOT_SERIAL_ROBOT_MESSAGE_H

#include "msg_serialize.h"

message_data Velocity{
    float v_x, v_y, v_w;
};
message_data image_location_t{
    uint8_t image_x;
    uint8_t image_y;
};
message_data decision_t{
    uint8_t if_navigation;
    uint8_t catch_decision;
    uint8_t qrcode_number;
};
message_data qrcode_ball_t{
    uint8_t qrcode_ball;
};

#endif //ROBOT_SERIAL_ROBOT_MESSAGE_H
