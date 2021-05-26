/*
Copyright 2021, Yang Luo"
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

@Author
Yang Luo, PHD
Shenyang Institute of Automation, Chinese Academy of Sciences.
 email: luoyang@sia.cn

@Created on: 2021.05.18
*/
#ifndef BONECUTTINGROBOT_BONECUTTINGROBOT_HPP
#define BONECUTTINGROBOT_BONECUTTINGROBOT_HPP

#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <queue> // FIFO
#include <memory>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/format.hpp>
#include <boost/timer/timer.hpp>

#include "RobotInterface.h"
//#include "kinematicInterface.h"

//sri FT sensor
#include "sri/ftsensor.hpp"
#include "sri/commethernet.hpp"

#include "DigitalFilters.hpp"

//Betop Joystick
#include "Joystick.hpp"
#include "LinearMotor.hpp"


#define FTSENSOR_IP "192.168.1.108"
#define FT_CALI_NUM 200

#define SPEED_LIMIT       "V3012"   // V3012对应的速度限制
#define J0                "j0"      // 切骨起始点关节空间位置
#define P0                "p201005" // 切骨起始点笛卡尔空间位置

#define DELTA_T           0.005     // 力传感器采样频率
#define CUTOFF_FREQ       0.5       // 截止频率

#define LINEAR_MOTOR_DEV  "/dev/ttyS0"


// 力传感器状态，空闲，校准中，正常，滤波
enum FT_STATE {
    FS_IDLE = 0, // 传感器数据不做任何处理
    FS_CALIBRATION = 1,
    FS_NORMAL = 2, // 传感器数据正常返回
    FS_FILTER = 3  // 传感器数据滤波后返回
};

// 当前执行器状态，等待，开始切，切完成
enum CUT_STATE {
    CS_WAIT = 0,
    CS_START = 1,
    CS_FINISH = 2
};

// 在切骨过程中的状态，前进还是后退
enum STEP_STATE {
    SS_STOP = 0,
    SS_FORWARD = 1,
    SS_BACKWARD = 2
};

//终端显示颜色
enum Color {
    BLACK = 0,
    RED = 1,
    GREEN = 2,
    YELLOW = 3,
    BLUE = 4,
    MAGENTA = 5,
    CYAN = 6,
    WHITE = 7,
    DEFAULT = 9
};

constexpr size_t HASH_STRING_PIECE(const char *string_piece, size_t hashNum = 0) {
    return *string_piece ? HASH_STRING_PIECE(string_piece + 1, (hashNum * 131) + *string_piece) : hashNum;
}

constexpr size_t operator "" _HASH(const char *string_pice, size_t) {
    return HASH_STRING_PIECE(string_pice);
}

struct offsetpose {
    double x; // x方向偏移
    double y; // y方向偏移
    double a; // 角度偏移
    bool f; // 是否有力控
};

class BoneCuttingRobot {

public:
    BoneCuttingRobot() : _modes(10, 8),
                         _f("\033[1;3%1%m "),
                         _b("\033[1;4%1%m "),
                         _fb("\033[1;3%1%;4%2%m "),
                         _def("\033[0m "),
                         _fmt(" [%ts] "),
                         _axisPositions(3, 0.0),
                         _ftCaliSum(6, 0.0),
                         _ftGravComp(6, 0.0),
                         _ftValue(6, 0.0),
                         _ftFilterValue(6, 0.0),
                         _lp2(6, LowPassFilter2(DELTA_T, 1 / (2 * M_PI * CUTOFF_FREQ))),
                         _joystick(0),
                         _linearMotor(LINEAR_MOTOR_DEV) {


        init(); //初始化

    }

    ~BoneCuttingRobot() {
        _ftPtr->stopRealTimeDataRepeatedly();
    }

    /// @brief 开始工作线程
    void startWorkingThread() {
        isRunning = true;
        std::cout << _b % Color::BLUE << ">>>>>>>>>>> BONE CUTTING WORKING THREAD IS RUNNING >>>>>>>>>>>" << _def
                  << std::endl;
        boost::thread(&BoneCuttingRobot::boneCuttingHandler, this).detach();
    }

    /// @brief 停止工作线程
    void stopWorkingThread() {
        isRunning = false;
    }

    /// @brief 初始化机器人
    void init() {
        _robotName = get_name_robotEC_deviceHandle_c(getEC_deviceName(0, NULL), 0);
        if (_robotName == nullptr) {
            std::cout << _f % Color::RED << "Robot is not initialized" << _def << std::endl;
            return;
        }

        robot_setmode_c(_robotName, &_modes[0]); //设置运行模式

        _interval = get_BusyTs_s_c(getEC_deviceName(0, NULL)); //获取总线读取间隔 单位是秒？
        std::cout << _f % Color::GREEN << "[INFO]" << _timer.format(4, _fmt) << "EtherCAT bus interval is: "
                  << _interval << _def << std::endl;

        /***** 读取各种所需数据 *****/
        readJointSpacePoints("/hanbing/data/robjoint.POINT"); //读取关节示教点

        readCartesianSpacePoints("/hanbing/data/robpose.POINT"); // 读取笛卡尔空间示教点

        readSpeedLimits("/hanbing/data/speed.POINT"); // 读取速度限制

        readCuttingOffsets("/hanbing/data/offsetpose.POINT"); //读取切骨偏移量,

        /***** 初始化六维力传感器 *****/
        std::cout << _f % Color::GREEN << "[INFO]" << _timer.format(4, _fmt) << "FT Sensor is initializing ...." << _def
                  << std::flush;

        _cePtr = std::make_shared<SRI::CommEthernet>(FTSENSOR_IP, 4008);
        _ftPtr = std::make_shared<SRI::FTSensor>(_cePtr.get());

        std::cout << " finished" << std::endl;

        std::cout << _f % Color::GREEN << "[INFO]" << _timer.format(4, _fmt) << "FT Sensor IP Address: "
                  << _ftPtr->getIpAddress() << _def << std::endl;

        /***** 手柄遥控 *****/
        if (!_joystick.isFound()) {
            std::cout << _f % Color::YELLOW << "[WARNING]" << _timer.format(4, _fmt) << "Joystick is not found."
                      << _def << std::endl;
        } else {
            boost::thread(&BoneCuttingRobot::joystickHandler, this).detach();
        }

        boost::thread(&LinearMotor::startProcessCyclicing, &_linearMotor).detach();
        _linearMotor.setPositionNR();

    }

    /// @brief 机器人上电
    void robotPowerOn() {
        if (_robotName == nullptr) {
            std::cout << _f % Color::RED << "Robot is not initialized" << _def << std::endl;
            return;
        }

        robot_power_c(_robotName);
    }

    /// @brief 机器人下电
    void robotPowerOff() {
        if (_robotName == nullptr) {
            std::cout << _f % Color::RED << "Robot is not initialized" << _def << std::endl;
            return;
        }

        robot_poweroff_c(_robotName);
    }

    /// @brief 设置单轴位置
    /// @param axisId 轴ID
    /// @param position 期望位置
    void setAxisPosition(int axisId, double position) {
        if (_robotName == nullptr) {
            std::cout << _f % Color::RED << "Robot is not initialized" << _def << std::endl;
            return;
        }

        axis_setposition_angle(_robotName, position, axisId);
    }

    /// @brief 设置机器人关节位置
    /// @param angle 期望关节位置
    void setRobotPosition(std::vector<double> &angle) {
        if (_robotName == nullptr) {
            std::cout << _f % Color::RED << "Robot is not initialized" << _def << std::endl;
            return;
        }

        robot_setposition_angle(_robotName, &angle[0]);
    }

    /// @brief 读取机器人关节位置
    /// @return 返回关节位置的vector容器
    std::vector<double> getRobotPosition() {
        if (_robotName == nullptr) {
            std::cout << _f % Color::RED << "Robot is not initialized" << _def << std::endl;
            return std::vector<double>();
        }

        robot_getposition_angle(_robotName, &_axisPositions[0]);

        return _axisPositions;
    }

    /// @brief 从robjoint.POINT中读取关节空间示教点
    /// @param fileName 保存有关节空间示教点的文件
    void readJointSpacePoints(const std::string &fileName) {
        _jointSpacePoints.clear();

        boost::property_tree::ptree pt;
        boost::property_tree::read_ini(fileName, pt);

        for (auto &section : pt) {
            robjoint rj;
            int dof = 0;
            for (int i = 0; i < 10; i++) {
                if (section.second.count(std::to_string(i + 1)) != 0) {
                    rj.angle[i] = section.second.get<double>(std::to_string(i + 1));
                    dof++;
                }
            }
//            rj.angle[0] = section.second.get<double>("1");
//            rj.angle[1] = section.second.get<double>("2");
//            rj.angle[2] = section.second.get<double>("3");
//            rj.angle[3] = section.second.get<double>("4");
//            rj.angle[4] = section.second.get<double>("5");
//            rj.angle[5] = section.second.get<double>("6");
            rj.dof = dof;
//            std::cout << "dof: " << dof << std::endl;

            _jointSpacePoints[section.first] = rj;
        }

        std::cout << _f % Color::GREEN << "[INFO]" << _timer.format(4, _fmt) << "Loaded Joint Space Teach Points : "
                  << _jointSpacePoints.size() << _def << std::endl;
    }

    /// @brief 从robpose.POINT中读取笛卡尔空间示教点
    /// @param fileName 保存有笛卡尔空间示教点的文件
    void readCartesianSpacePoints(const std::string &fileName) {
        _cartesianSpacePoints.clear();

        boost::property_tree::ptree pt;
        boost::property_tree::read_ini(fileName, pt);

        for (auto &section : pt) {
            robpose rp;
            rp.xyz[0] = section.second.get<double>("x");
            rp.xyz[1] = section.second.get<double>("y");
            rp.xyz[2] = section.second.get<double>("z");

            rp.kps[0] = section.second.get<double>("k");
            rp.kps[1] = section.second.get<double>("p");
            rp.kps[2] = section.second.get<double>("s");

            _cartesianSpacePoints[section.first] = rp;
        }

        std::cout << _f % Color::GREEN << "[INFO]" << _timer.format(4, _fmt) << "Loaded Cartesian Space Teach Points : "
                  << _cartesianSpacePoints.size() << _def << std::endl;
    }

    /// @brief 从speed.POINT中读取速度限制
    /// @param fileName 保存有速度限制的文件
    void readSpeedLimits(const std::string &fileName) {
        _speedLimits.clear();

        boost::property_tree::ptree pt;
        boost::property_tree::read_ini(fileName, pt);

        for (auto &section : pt) {
            speed sp;
            int dof = 0;
            if (section.second.count("per1") != 0) {
                sp.per[0] = section.second.get<double>("per1");
                dof++;
            }
            if (section.second.count("per2") != 0) {
                sp.per[1] = section.second.get<double>("per2");
                dof++;
            }
            if (section.second.count("per3") != 0) {
                sp.per[2] = section.second.get<double>("per3");
                dof++;
            }
            if (section.second.count("per4") != 0) {
                sp.per[3] = section.second.get<double>("per4");
                dof++;
            }
            if (section.second.count("per5") != 0) {
                sp.per[4] = section.second.get<double>("per5");
                dof++;
            }
            if (section.second.count("per6") != 0) {
                sp.per[5] = section.second.get<double>("per6");
                dof++;
            }
            if (section.second.count("per_flag") != 0) {
                sp.per_flag = section.second.get<int>("per_flag");
            }
            if (section.second.count("tcp") != 0) {
                sp.per[0] = section.second.get<double>("tcp");
            }
            if (section.second.count("tcp_flag") != 0) {
                sp.tcp_flag = section.second.get<int>("tcp_flag");
            }
            if (section.second.count("orl") != 0) {
                sp.per[0] = section.second.get<double>("orl");
            }
            if (section.second.count("orl_flag") != 0) {
                sp.orl_flag = section.second.get<int>("orl_flag");
            }

            sp.dof = dof;

            _speedLimits[section.first] = sp;
        }

        std::cout << _f % Color::GREEN << "[INFO]" << _timer.format(4, _fmt) << "Loaded Speed Limits : "
                  << _speedLimits.size() << _def << std::endl;
    }

    /// @brief 从offset.POINT中读取切骨点偏移
    /// @param fileName 保存切骨点偏移的文件
    void readCuttingOffsets(const std::string &fileName) {
        _cuttingOffsets = std::queue<offsetpose>(); // 清空队列

        boost::property_tree::ptree pt;
        boost::property_tree::read_ini(fileName, pt);

        for (auto &section : pt) {
            offsetpose op;
            op.x = section.second.get<double>("x");
            op.y = section.second.get<double>("y");
            if (section.second.count("a") != 0)
                op.a = section.second.get<double>("a");
            else
                op.a = 0.0;
            op.f = section.second.get<bool>("f");

            _cuttingOffsets.push(op);
        }

        std::cout << _f % Color::GREEN << "[INFO]" << _timer.format(4, _fmt) << "Loaded Cutting Offset Points : "
                  << _cuttingOffsets.size() << _def << std::endl;
    }

    /// @brief 获取相应的关节空间示教点
    /// @param name 示教点名称
    /// @param rj   关节点信息
    bool getJointSpacePoint(const std::string &name, robjoint &rj) {
        if (_jointSpacePoints.count(name) != 0) {
            rj = _jointSpacePoints[name];
            return true;
        } else
            return false;
    }

    /// @brief 获取相应的笛卡尔空间示教点
    /// @param name 示教点名称
    /// @param rp   笛卡尔点信息
    bool getCartesianSpacePoint(const std::string &name, robpose &rp) {
        if (_cartesianSpacePoints.count(name) != 0) {
            rp = _cartesianSpacePoints[name];
            return true;
        } else
            return false;
    }

    /// @brief 获取相应的速度限制
    /// @param name 速度限制名称
    /// @param sp   速度限制信息
    bool getSpeedLimit(const std::string &name, speed &sp) {
        if (_speedLimits.count(name) != 0) {
            sp = _speedLimits[name];
            return true;
        } else
            return false;
    }

protected:
    char *_robotName = nullptr; //机器人名，用于上电、下电等操作
    std::vector<signed char> _modes; //运行模式 8为csp
    double _interval = 0.001; //dt，总线频率，应该是1 ms = 1000 us = 1000000 ns

    std::map<std::string, robjoint> _jointSpacePoints; //关节空间示教点 /hanbing/data/robjoint.POINT
    std::map<std::string, robpose> _cartesianSpacePoints; //笛卡尔空间示教点 /hanbing/data/robpose.POINT
    std::map<std::string, speed> _speedLimits; //笛卡尔空间速度 /hanbing/data/speed.POINT

    std::queue<offsetpose> _cuttingOffsets; //切骨路径规划点，相对于当前位置（先移动到j点）的偏移

    std::vector<double> _axisPositions;


    boost::format _f; //设置前景色
    boost::format _b; //设置背景色
    boost::format _fb; //前景背景都设置
    boost::format _def; //恢复默认

    const std::string _fmt; //显示时间戳格式
    boost::timer::cpu_timer _timer;

    std::shared_ptr<SRI::CommEthernet> _cePtr; //六维力传感器通信
    std::shared_ptr<SRI::FTSensor> _ftPtr; //六维力传感器指针

    FT_STATE _ftState = FS_IDLE;
    CUT_STATE _cutState = CS_WAIT;
    STEP_STATE _stepState = SS_STOP;

    int _ftCaliCount = 0; // 力传感器校准计数
    std::vector<double> _ftCaliSum; // 力传感器均值滤波校准加和
    std::vector<double> _ftGravComp; // 力传感器重力补偿
    std::vector<double> _ftValue;    // 力传感器实时数据
    std::vector<double> _ftFilterValue; // 力传感器滤波后的数据

    std::vector<LowPassFilter2> _lp2; // 保存2阶低通滤波器指针

    bool newFtDataComing = false;
    boost::mutex _ftMutex;          // 用于力传感器读取的互斥量

    bool isRunning = true;

    Joystick _joystick;                 // 北通手柄
    JoystickEvent _jsEvent{};             // 手柄按钮按下事件
    bool isSafeButtonPressed = false;   // 安全按钮LB是否按下
    bool isEnableButtonPressed = false; // 使能按钮START是否按下

    LinearMotor _linearMotor;           //油门开度控制 直线电机

protected:
    size_t getHash(const std::string &str) {
        // 获取string对象得字符串值并传递给HAHS_STRING_PIECE计算，获取得返回值为该字符串HASH值
        return HASH_STRING_PIECE(str.c_str());
    }

    /// @brief 处理手柄指令
    void joystickHandler() {

        while (isRunning) {
            if (!_joystick.sample(&_jsEvent)) {
                continue;
            }

            // 处理Button信息
            if (_jsEvent.isButton()) {
                switch (_jsEvent.number) {
                    case BETOP_BUTTON_LB:
                        if (_jsEvent.value == 0) {
                            isSafeButtonPressed = false; //安全按钮松开
                        } else {
                            isSafeButtonPressed = true;  //安全按钮抬起
                            std::cout << _f % Color::GREEN << "[INFO]" << _timer.format(4, _fmt)
                                      << "Joystick Safe Button Pressed " << _def << std::endl;
                        }
                        break;
                    case BETOP_BUTTON_A:
                        if (_jsEvent.value != 0 && isEnableButtonPressed && isSafeButtonPressed) {
                            //TODO: 位置减
//                            std::cout << _f % Color::GREEN << "[INFO]" << _timer.format(4, _fmt)
//                                      << "Joystick Position MORE " << _def << std::endl;
                            _linearMotor.addOffset(-200);
                        }
                        break;
                    case BETOP_BUTTON_Y:
                        if (_jsEvent.value != 0 && isEnableButtonPressed && isSafeButtonPressed) {
                            //TODO: 位置增
//                            std::cout << _f % Color::GREEN << "[INFO]" << _timer.format(4, _fmt)
//                                      << "Joystick Position LESS " << _def << std::endl;
                            _linearMotor.addOffset(200);
                        }
                        break;
                    case BETOP_BUTTON_START:
                        if (_jsEvent.value != 0) {
                            //TODO: 使能按钮，必须按下之后才可以操作
                            isEnableButtonPressed = true;
                        }
                        break;
                    case BETOP_BUTTON_BACK:
                        if (_jsEvent.value != 0) {
                            //TODO: 执行归位，迅速释放
                            isEnableButtonPressed = false; //按动back，说明遇到紧急情况，使能断掉
                            _linearMotor.setOffset(0);
                            _linearMotor.setAccelerator(0);
                        }
                        break;
                    default:
                        break;
                }
            }
                // 处理Axis信息
            else if (_jsEvent.isAxis()) {
                switch (_jsEvent.number) {
                    case BETOP_AXIS_RT:
                        if (_jsEvent.value != 0 && isEnableButtonPressed && isSafeButtonPressed) {
                            //TODO: 0~32767
                        }
                        break;
                    default:
                        break;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1)); //等待1ms

        }


//        while (isRunning) {
//            if (_joystick.sample(&_jsEvent)) {
//                if (_jsEvent.isButton()) {
//                    if ((_jsEvent.number == BETOP_BUTTON_A) || (_jsEvent.number == BETOP_BUTTON_Y))
//                        printf("Button %u is %s\n",
//                               _jsEvent.number,
//                               _jsEvent.value == 0 ? "up" : "down");
//                } else if (_jsEvent.isAxis()) {
//                    if ((_jsEvent.number == BETOP_AXIS_LT) || (_jsEvent.number == BETOP_AXIS_RT))
//                        printf("Axis %u is at position %d\n", _jsEvent.number, _jsEvent.value);
//                }
//            }
//
//            usleep(1000);
//        }

    }

    /// @brief 处理切骨逻辑
    void boneCuttingHandler() {

        //六维力传感器开始循环接收数据
        auto rtMode = _ftPtr->getRealTimeDataMode();
        auto rtDataValid = _ftPtr->getRealTimeDataValid();
        _ftPtr->startRealTimeDataRepeatedly<float>(boost::bind(&BoneCuttingRobot::ftDataHandler, this, _1), rtMode,
                                                   rtDataValid);

        //从示教点中读取所需的点和速度限制等
        speed speedLimit;
        robpose p0;
        robjoint j0;

        if (!getSpeedLimit(SPEED_LIMIT, speedLimit)) {
            std::cout << _f % Color::RED << "[ERROR]" << _timer.format(4, _fmt) << "Can not get speed limit named >"
                      << SPEED_LIMIT << "<" << _def << std::endl;
        } else {
            std::cout << _f % Color::GREEN << "[INFO]" << _timer.format(4, _fmt)
                      << "Successfully get speed limit named >"
                      << SPEED_LIMIT << "<" << _def << std::endl;
        }

        if (!getJointSpacePoint(J0, j0)) {
            std::cout << _f % Color::RED << "[ERROR]" << _timer.format(4, _fmt)
                      << "Can not get joint space point named >" << J0 << "<" << _def << std::endl;
        } else {
            std::cout << _f % Color::GREEN << "[INFO]" << _timer.format(4, _fmt)
                      << "Successfully get joint space point named >" << J0 << "<" << _def << std::endl;
        }

        if (!getCartesianSpacePoint(P0, p0)) {
            std::cout << _f % Color::RED << "[ERROR]" << _timer.format(4, _fmt)
                      << "Can not get cartesian space point named >" << P0 << "<" << _def << std::endl;
        } else {
            std::cout << _f % Color::GREEN << "[INFO]" << _timer.format(4, _fmt)
                      << "Successfully get cartesian space point named >" << P0 << "<" << _def << std::endl;
        }

        //先使能再下电，英立说要不这样做，会有bug
//        robotPowerOn();
//        robotPowerOff();

        while (isRunning) {
            torqueTimerE(0); //阻塞等待总线数据到来

            //每个循环先获取一下当前机器人位姿
            auto pos = getRobotPosition();
            //TODO: DEBUG信息输出
            std::cout << "\r" << _f % Color::MAGENTA << "[DEBUG]" << _timer.format(4, _fmt) << pos[0] << "\t" << pos[1]
                      << "\t" << pos[2] << _def << std::flush;

            switch (_cutState) {
                case CS_WAIT:
                    break;
                case CS_START:
                    break;
                case CS_FINISH:
                    break;
                default:
                    break;
            }
        }
    }

    /// @brief 处理六维力数据
    void ftDataHandler(std::vector<SRI::RTData<float>> &rtData) {
        //TODO: DEBUG信息输出
//        static int count = 0;
//        std::cout << "\r" << _f % Color::CYAN << "[DEBUG]" << _timer.format(4, _fmt) << "[" << count << "] RT Data is ->  ";
//        for(int i = 0; i < rtData.size(); i++) {
//            for(int j = 0; j < 6; j++) {
//                std::cout << "Ch " << j << ": " << rtData[i][j] << "\t";
//            }
//            std::cout << _def << std::flush;
//        }
//        count++;

        _ftMutex.lock();

        for (auto &ft : rtData) {

            for (int i = 0; i < 3; i++) {
                _ftValue[i] = ft[i];
//                _ftFilterValue[i] = _lp2[i].update(_ftValue[i]);
            }


            switch (_ftState) {
                case FS_IDLE:
                    _ftGravComp.resize(6, 0.0);
                    _ftCaliSum.resize(6, 0.0);
                    _ftCaliCount = 0;

                    break;

                case FS_CALIBRATION: // 进入校准状态
                    _ftCaliCount++;
                    for (int i = 0; i < 3; i++) {
                        _ftCaliSum[i] += ft[i];
                    }

                    if (_ftCaliCount >= FT_CALI_NUM) {
                        for (int i = 0; i < 3; i++) {
                            _ftGravComp[i] = _ftCaliSum[i] / _ftCaliCount;
                        }
                        _ftCaliSum.resize(6, 0.0);
                        _ftCaliCount = 0;

                        _ftState = FS_FILTER; //进入滤波模式
                    }

                    break;

                case FS_FILTER: // 进入滤波模式
                    for (int i = 0; i < 3; i++) {
                        _ftFilterValue[i] = _lp2[i].update(_ftValue[i]);
                    }

                    break;

                case FS_NORMAL: // 进入正常模式
                    std::cout << "\r" << _f % Color::CYAN << "[DEBUG]" << _timer.format(4, _fmt) << "FT Data is -> "
                              << "x: " << _ftValue[0] << "\t" << "y: " << _ftValue[1] << "\t" << "z: " << _ftValue[2]
                              << _def << std::flush;

                    break;

                default:
                    std::cout << _f % Color::RED << "[ERROR]" << _timer.format(4, _fmt)
                              << "Unrecognized FT Sensor State" << _def << std::endl;
                    break;
            }

            newFtDataComing = true; // 新数据到来标志
        }

        _ftMutex.unlock();

    }

};


#endif //BONECUTTINGROBOT_BONECUTTINGROBOT_HPP
