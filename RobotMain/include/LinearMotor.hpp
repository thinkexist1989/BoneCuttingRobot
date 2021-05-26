// Copyright 2021, Yang Luo"
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// @Author
// Yang Luo, PHD
// Shenyang Institute of Automation, Chinese Academy of Sciences.
// email: luoyang@sia.cn

#ifndef BONECUTTINGROBOT_LINEARMOTOR_HPP
#define BONECUTTINGROBOT_LINEARMOTOR_HPP

#include "SerialPort.hpp"
#include <boost/assign.hpp>
#include <string>

using namespace boost::assign;

#define LINEAR_MOTOR_ID 0x01

#define CMD_RD    0x01
#define CMD_WR    0x02
#define CMD_WR_NR 0x03

#define TABLE_TARGET_POSITION 0x37
#define TABLE_OVER_CURRENT    0x20
#define TABLE_OVER_TEMP       0x62
#define TABLE_BACK_TEMP       0x64

#define POSITION_MIN    400
#define POSITION_MAX    2000

#define ACC_MIN         (2000 - POSITION_MAX)
#define ACC_MAX         (2000 - POSITION_MIN)

#define OFFSET_MIN      0
#define OFFSET_MAX      1000

class LinearMotor {
public:
    explicit LinearMotor(const std::string &portName = "/dev/ttyS0") : _sp(portName) {
        setBaudRate();
        setFlowControl();
        setStopBits();
        setParity();
        setDataBits();
    }

    ~LinearMotor() = default;

    //设置串口名
    inline void setPortName(const std::string &portName = "/dev/ttyS0") {
        _sp.setPortName(portName);
    }

    //设置串口波特率
    inline void setBaudRate(unsigned int rate = 115200) {
        _sp.setBaudRate(rate);
    }

    //设置流控制
    inline void setFlowControl(SerialPort::flow_control_type fc = SerialPort::flow_control::none) {
        _sp.setFlowControl(fc);
    }

    //设置停止位
    inline void setStopBits(SerialPort::stop_bits_type stopBits = SerialPort::stop_bits::one) {
        _sp.setStopBits(stopBits);
    }

    //设置奇偶校验位
    inline void setParity(SerialPort::parity_type parityBit = SerialPort::parity::none) {
        _sp.setParity(parityBit);
    }

    //设置数据位
    inline void setDataBits(unsigned int dataBits = 8) {
        _sp.setDataBits(dataBits);
    }

    //以无应答的方式设置直线电机位置 0~2000
    void setPositionNR(uint16_t pos) {

        _pos = (pos < 2000 && pos > 0) ? pos : (pos <= 0 ? 0 : 2000);

        std::vector<uint8_t> sendBuf;

        sendBuf += 0x55, 0xAA;       //帧头
        sendBuf += 0x04;             //帧长度 = 2+1+1
        sendBuf += LINEAR_MOTOR_ID;  //ID
        sendBuf += CMD_WR_NR;        //指令类型
        sendBuf += TABLE_TARGET_POSITION;
        sendBuf += getBinary(_pos, 0); // 低8位
        sendBuf += getBinary(_pos, 1); // 高8位

        uint8_t checksum = 0;
        for (int i = 2; i < sendBuf.size(); i++) {
            checksum += sendBuf[i];
        }
        sendBuf += checksum;

        _sp.write(sendBuf);

//        std::this_thread::sleep_for(std::chrono::milliseconds(1)); //等待1ms
    }

    void setPositionNR() {

        std::vector<uint8_t> sendBuf;

        sendBuf += 0x55, 0xAA;       //帧头
        sendBuf += 0x04;             //帧长度 = 2+1+1
        sendBuf += LINEAR_MOTOR_ID;  //ID
        sendBuf += CMD_WR_NR;        //指令类型
        sendBuf += TABLE_TARGET_POSITION;
        sendBuf += getBinary(_pos, 0); // 低8位
        sendBuf += getBinary(_pos, 1); // 高8位

        uint8_t checksum = 0;
        for (int i = 2; i < sendBuf.size(); i++) {
            checksum += sendBuf[i];
        }
        sendBuf += checksum;

        _sp.write(sendBuf);

    }

    // 设置油门开度，油门开度和电机位置正好相反，电机位置为0，油门开度最大（2000），电机位置2000，油门开度最小（0）
    void setAccelerator(int16_t acc) {
        _acc = (acc <= ACC_MAX && acc >= ACC_MIN) ? acc : (acc < ACC_MIN ? ACC_MIN : ACC_MAX);
    }

    void setOffset(int16_t offset) {
        _offset = (offset <= OFFSET_MAX && offset >= OFFSET_MIN) ? offset : (offset < OFFSET_MIN ? OFFSET_MIN
                                                                                                 : OFFSET_MAX);
    }

    void addAccelerator(int16_t step) {
        _acc += step;
        _acc = (_acc <= ACC_MAX && _acc >= ACC_MIN) ? _acc : (_acc < ACC_MIN ? ACC_MIN : ACC_MAX);
    }

    void addOffset(int16_t step) {
        _offset += step;
        _offset = (_offset <= OFFSET_MAX && _offset >= OFFSET_MIN) ? _offset : (_offset < OFFSET_MIN ? OFFSET_MIN
                                                                                                     : OFFSET_MAX);
    }

    void startProcessCyclicing() {
        while (isRunning) {
            int16_t temp = _acc + _offset;
            temp = (temp <= ACC_MAX && temp >= ACC_MIN) ? temp : (temp < ACC_MIN ? ACC_MIN : ACC_MAX);

            _pos = 2000 - temp;
            std::cout << "[DEBUG] Linear Motor MOVE: " << temp << "\t" << _pos << std::endl;
            setPositionNR();

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

    }

    void stopProcessCyclicing() {
        isRunning = false;
    }

protected:
    template<typename D>
    inline uint8_t getBinary(D t, size_t i) { //获取内存中存储数值
        return ((uint8_t *) &t)[i];
    }

private:
    SerialPort _sp;
    uint16_t _pos = POSITION_MAX;

    int16_t _acc = ACC_MIN;
    int16_t _offset = 0;

    bool isRunning = true;
};


#endif //BONECUTTINGROBOT_LINEARMOTOR_HPP
