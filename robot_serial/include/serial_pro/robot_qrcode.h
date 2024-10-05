//
// Created by Elsa on 24-10-5.
//

#ifndef SERIALPRO_ROBOTQRCODE_H
#define SERIALPRO_ROBOTQRCODE_H

#include "serial_pro.h"

namespace qrcode{
    message_data Head{
        uint8_t SOF = 0x03;
        uint8_t cmd_id = 0x00;
        uint8_t length = 0;
    };
    message_data Tail {
        uint8_t crc16 = 0x0d;
    };
    class CodeSerial : public sp::serialPro<Head, Tail>{
    public:
        CodeSerial() = default;
        CodeSerial(std::string s,int band):sp::serialPro<Head,Tail>(s,band)
        {
            registerChecker([](const Head &head)->int {return head.SOF!=0x03;});  //返回0表示没有错误

            registerChecker([](const Tail &tail,const uint8_t *,const int &)->int{return tail.crc16!=0x0d;});

            setGetId([](const Head &head)->int{return head.cmd_id;});  //返回命令字

            setGetLength([](const Head &head)->int{return (int)head.length-1;});  //返回长度
        }
    };
}

#endif //SERIALPRO_ROBOTQRCODE_H
