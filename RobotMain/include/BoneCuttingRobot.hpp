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

#define BCR_VER 0.1

#define ENABLE_ETHERCAT
//#define ENABLE_FTSENSOR
#define ENABLE_JOYSTICK
//#define ENABLE_KEYBOARD

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
#include <boost/asio.hpp>
#include <boost/function.hpp>

#include "RobotInterface.h"
//#include "kinematicInterface.h"

//sri FT sensor
#include "sri/ftsensor.hpp"
#include "sri/commethernet.hpp"

#include "DigitalFilters.hpp"

//Betop Joystick
#include "Joystick.hpp"
#include "LinearMotor.hpp"

///////////////////LYL///////////////////////////
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "key.h"
#include "RobotLib.h"
#include "kinematicInterface.h"
#include "RobotInterface.h"
#include "tcp.h"
#include "SignalFilter.h"

#include "readdata.h"

extern KEY_DATA msg_key;

double dt = 0;

int RobotOrAdditionaxis = 0;
int RobotOrAdditionaxis_index = 0;

#define OFFSETPOSE_INIFILE "/hanbing/data/offsetpose.POINT"
#define LEN_NAME_POINT 20
#define CONTROL_PEROID_T 10
#define debug_print_teachpoint_yl

#define FILTER_FREQ 2
#define FILTER_DAMP 0.7
#define FILTER_PEROID 0.01

#define POSITION_DELTA_THRESHOLD 2.0
#define INC_POSITION_PER 0.06

#define POSITION_NOISE_NOFORCE 0.1

float offset_X[9] = {10, 10, 20, 30, 40, -10, -20, -30, -40};
signed char _mode = 8;

/////////////////////////////////////////////////


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
                         _lp2(6, new LowPassFilter2(DELTA_T, 1 / (2 * M_PI * CUTOFF_FREQ))),
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
        std::cout << _b % Color::BLUE << ">>>>>>>>>>> BONE CUTTING WORKING THREAD IS RUNNING v" << BCR_VER
                  << " >>>>>>>>>>>" << _def
                  << std::endl;
//        boost::thread(&BoneCuttingRobot::boneCuttingHandler, this).detach();
//        boost::thread(&BoneCuttingRobot::boneCuttingYL, this).detach();
    }

    /// @brief 停止工作线程
    void stopWorkingThread() {
        isRunning = false;
    }

    /// @brief 初始化机器人
    void init() {
#ifdef ENABLE_ETHERCAT
        _robotName = get_name_robotEC_deviceHandle_c(getEC_deviceName(0, NULL), 0);
        if (_robotName == nullptr) {
            std::cout << _f % Color::RED << "Robot is not initialized" << _def << std::endl;
            return;
        }

        robot_setmode_c(_robotName, &_modes[0]); //设置运行模式

        _interval = get_BusyTs_s_c(getEC_deviceName(0, NULL)); //获取总线读取间隔 单位是秒？
        std::cout << _f % Color::GREEN << "[INFO]" << _timer.format(4, _fmt) << "EtherCAT bus interval is: "
                  << _interval << _def << std::endl;
#endif

        /***** 读取各种所需数据 *****/
        readJointSpacePoints("/hanbing/data/robjoint.POINT"); //读取关节示教点

        readCartesianSpacePoints("/hanbing/data/robpose.POINT"); // 读取笛卡尔空间示教点

        readSpeedLimits("/hanbing/data/speed.POINT"); // 读取速度限制

        readCuttingOffsets("/hanbing/data/offsetpose.POINT"); //读取切骨偏移量,

        /***** 初始化六维力传感器 *****/
#ifdef ENABLE_FTSENSOR
        std::cout << _f % Color::GREEN << "[INFO]" << _timer.format(4, _fmt) << "FT Sensor is initializing ...." << _def
                  << std::flush;

        _cePtr = std::make_shared<SRI::CommEthernet>(FTSENSOR_IP, 4008);
        _ftPtr = std::make_shared<SRI::FTSensor>(_cePtr.get());

        std::cout << " finished" << std::endl;

        std::cout << _f % Color::GREEN << "[INFO]" << _timer.format(4, _fmt) << "FT Sensor IP Address: "
                  << _ftPtr->getIpAddress() << _def << std::endl;
#endif

        /***** 手柄遥控 *****/
#ifdef ENABLE_JOYSTICK
        if (!_joystick.isFound()) {
            std::cout << _f % Color::YELLOW << "[WARNING]" << _timer.format(4, _fmt) << "Joystick is not found."
                      << _def << std::endl;
        } else {
            boost::thread(&BoneCuttingRobot::joystickHandler, this).detach();
        }

        boost::thread(&LinearMotor::startProcessCyclicing, &_linearMotor).detach();
#endif

        /***** 按键检测 *****/
#ifdef ENABLE_KEYBOARD
        boost::thread(&BoneCuttingRobot::keyHandler, this).detach();
#endif

//        bcrInstance = this;
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
    /// @param axisId 轴ID （从1开始）
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

    bool isTargetEndEffectorPositionReached(const robpose &rpose) {

        return false;
    }

    bool isTargetJointPositionReached(const robjoint &rjoint) {

        return false;
    }

    bool isTargetOffsetReached(const offsetpose &roffsetpose) {

        return false;
    }

    /// @brief 获取基于基准位姿偏移后的末端位姿
    /// \return
    robpose getEndEffectPoseAfterOffset(const robpose &rpose, const offsetpose &opose) {
        return Offs(&rpose, opose.x, opose.y, 0, 0, 0, opose.a); // 这个不知道对不对？
    }

    void moveLine(const robpose &rpose, const speed &rspeed) {
        moveL(const_cast<robpose *>(&rpose), const_cast<speed *>(&rspeed), NULL, NULL, NULL);
    }

    void moveCircle(const robpose &rpose, const robpose &rpose_mid, const speed &rspeed) {
        moveC(const_cast<robpose *>(&rpose), const_cast<robpose *>(&rpose_mid), const_cast<speed *>(&rspeed), NULL,
              NULL, NULL);
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

    std::vector<LowPassFilter2 *> _lp2; // 保存2阶低通滤波器指针

    bool newFtDataComing = false;
    boost::mutex _ftMutex;          // 用于力传感器读取的互斥量

    bool isRunning = true;

    Joystick _joystick;                 // 北通手柄
    JoystickEvent _jsEvent{};             // 手柄按钮按下事件
    bool isSafeButtonPressed = false;   // 安全按钮LB是否按下
    bool isEnableButtonPressed = false; // 使能按钮START是否按下

    LinearMotor _linearMotor;           //油门开度控制 直线电机

    bool _powerFlag = false;
    bool _startFlag = false;

    //从示教点中读取所需的点和速度限制等
    speed _speedLimit;
    robpose _p0;
    robjoint _j0;

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
//                            std::cout << _f % Color::GREEN << "[INFO]" << _timer.format(4, _fmt)
//                                      << "Joystick Safe Button Pressed " << _def << std::endl;
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

    }

    /// @brief yl处理切骨逻辑
    void boneCuttingYL() {

        // Key detection
            static pthread_t PthKey;
            static pthread_attr_t attr;
            int err=0;

            pthread_attr_init( &attr );
            pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);

            err = pthread_create(&PthKey, &attr, thread_key, NULL);



        //六维力传感器开始循环接收数据
#ifdef ENABLE_FTSENSOR
        auto rtMode = _ftPtr->getRealTimeDataMode();
        auto rtDataValid = _ftPtr->getRealTimeDataValid();
        _ftPtr->startRealTimeDataRepeatedly<float>(boost::bind(&BoneCuttingRobot::ftDataHandler, this, _1), rtMode,
                                                   rtDataValid);
#endif

        char *robot_name = NULL;
        robot_name = get_name_robotEC_deviceHandle_c(getEC_deviceName(0, NULL), 0);
        size_t nbytes;
        int numJoints, numPoses, numSpeeds, numOffsetPoses;
        int dataId;
        int inquery_speed_id = 65535;
        int inquery_speed_v3012 = 65535;
        int inquery_joint_id = 65535;
        int inquery_pose_id = 65535;
        int inquery_pose_p1005 = 65535;


        float get_X_sensor1; //力传感器X向力
        float get_Y_sensor1; //力传感器Y向力
        float get_Z_sensor1; //力传感器Z向力

        float get_X_sensor1_afterFilter; //力传感器X向滤波后的数据（X向下为正）
        float get_X_sensor1_afterFilter_last; //
        float get_X_sensor1_dot; //力变化率

        double Torque_filter_in[3][3];
        double Torque_filter_out[3][3];

        unsigned char first_into_loop = 1; // 是否是第一次进入循环标志

        float Force_d = 0.0;
        float Force_error = 0.0;
        float Position_delta = 0.0;
        float Position_delta_last = 0.0;
        float Position_now = 0.0;
        double Position_ref_pf = 0.0;
        float Position_d = 0.0;
        float Position_d_first = 0.0;
        float Position_error = 0.0;
        float Admit_K = 2.0;
        char Finished_Force_depth = 1;
        robpose rpose_After_Offs;

        int index_movel = 0;
        int step_movel_num = 0;


        unsigned char flagInterpolation = 0;
        int index_timer = 0;
        double position_realtime[3];


        nbytes = sizeof(robjoint);
        numJoints = getDataNum(_robjoint); //去读取/hanbing/data/robjoint.POINT中存储的点
        robjoint *rjoint;
        char **name_rjoint;
        rjoint = (robjoint *) malloc(numJoints * nbytes);
        name_rjoint = (char **) malloc(numJoints * sizeof(char *));

        for (dataId = 0; dataId < numJoints; dataId++) {
            name_rjoint[dataId] = (char *) malloc(LEN_NAME_POINT * sizeof(char));
            getDataName(_robjoint, dataId, name_rjoint[dataId]);
            getrobjoint(name_rjoint[dataId], &rjoint[dataId]); //获取关节位置
        }


        nbytes = sizeof(robpose);
        numPoses = getDataNum(_robpose); //获取姿态数据个数用于分配内存（笛卡尔空间示教点，/hanbing/data/robpose.POINT中存储的点）
        robpose *rpose;
        char **name_rpose;
        rpose = (robpose *) malloc(numPoses * nbytes);
        name_rpose = (char **) malloc(numPoses * sizeof(char *)); //分配字符串数组指针空间

        for (dataId = 0; dataId < numPoses; dataId++) {
            name_rpose[dataId] = (char *) malloc(LEN_NAME_POINT * sizeof(char)); //分配数组每个字符串空间
            getDataName(_robpose, dataId, name_rpose[dataId]);
            getrobpose(name_rpose[dataId], &rpose[dataId]); //获取机器人姿态
        }


        nbytes = sizeof(speed);
        numSpeeds = getDataNum(_speed); //获取速度数据个数用于分配内存
        speed *rspeed;
        char **name_rspeed;
        rspeed = (speed *) malloc(numSpeeds * nbytes);
        name_rspeed = (char **) malloc(numSpeeds * sizeof(char *));

        for (dataId = 0; dataId < numSpeeds; dataId++) {
            name_rspeed[dataId] = (char *) malloc(LEN_NAME_POINT * sizeof(char));
            getDataName(_speed, dataId, name_rspeed[dataId]);
            getspeed(name_rspeed[dataId], &rspeed[dataId]);
        }


        numOffsetPoses = getDataNum_points(OFFSETPOSE_INIFILE);
        double **roffsetpose;
        int *roffsetpose_forceflag;
        int offsetPosesNum = 0;

        roffsetpose_forceflag = (int *) malloc(numOffsetPoses * sizeof(int)); // 每一个robpose对应的forceflag
        roffsetpose = (double **) malloc(numOffsetPoses * sizeof(double *));   // 每一个robpose对应位置
        for (dataId = 0; dataId < numOffsetPoses; dataId++) {
            roffsetpose[dataId] = (double *) malloc(2 * sizeof(double));
        }
        if (ReadOffset_XY_Fromfile(roffsetpose, roffsetpose_forceflag, &offsetPosesNum, OFFSETPOSE_INIFILE) !=
            0) //把offsetpose.POINT中的点读取回来
        {
            return;
        }

        if (offsetPosesNum != numOffsetPoses) {
            printf("read offsetPose content failed!\r\n");
            printf("read offsetPose Error Code: [%d,%d]!\r\n", numOffsetPoses, offsetPosesNum);
            return;
        }

        printf(" ****************************************\r\n");
        printf("\033[1;32m --->【roboffset type has %d points】 \r\n \033[0m", offsetPosesNum);

        for (dataId = 0; dataId < numOffsetPoses; dataId++) {
            printf("\033[1;32m --->【roboffset type %d point name】: [%f, %f, %d] \r\n \033[0m",
                   dataId + 1, roffsetpose[dataId][0], roffsetpose[dataId][1], roffsetpose_forceflag[dataId]);
        }

        for (dataId = 0; dataId < numJoints; dataId++) {
            if (strcmp(name_rjoint[dataId], "j0") == 0) //寻找名为j0的关节空间位置，找到了就把id赋值给inquery_joint_id，之后退出循环
            {
                inquery_joint_id = dataId;
                break;
            }
        }

        for (dataId = 0; dataId < numPoses; dataId++) {
            if (strcmp(name_rpose[dataId], "p0") == 0) //robpose.POINT中寻找名为p0的笛卡尔空间位置
            {
                inquery_pose_id = dataId;
            } else if (strcmp(name_rpose[dataId], "p201005") == 0) //寻找名为p201005的笛卡尔空间位置
            {
                inquery_pose_p1005 = dataId;
            }
            if (inquery_pose_id != 65535 && inquery_pose_p1005 != 65535) {
                break;
            }

        }
        if (inquery_pose_id == 65535 || inquery_pose_p1005 == 65535) {
            printf("inquery reference pose content failed!\r\n");
            printf("read reference pose Error Code: [%d,%d]!\r\n", inquery_pose_id, inquery_pose_p1005);
            return;
        }

        for (dataId = 0; dataId < numSpeeds; dataId++) {
            if (strcmp(name_rspeed[dataId], "v0") == 0) // robspeed中寻找v0的速度约束
            {
                inquery_speed_id = dataId;
            } else if (strcmp(name_rspeed[dataId], "v3012") == 0) // robspeed中寻找v3012的速度约束
            {
                inquery_speed_v3012 = dataId;
            }

            if (inquery_speed_id != 65535 && inquery_speed_v3012 != 65535) {
                break;
            }
        }


        printf("\033[1;32m --->【***********************Query Results***********************】\r\n \033[0m");
        if (inquery_joint_id != 65535 && inquery_pose_id != 65535 && inquery_speed_id != 65535 &&
            inquery_speed_v3012 != 65535)
            printf("\033[1;32m ---speed:%s, joint:%s, pose:%s. speed-V3012:%s ----\r\n \033[0m",
                   name_rspeed[inquery_speed_id], name_rjoint[inquery_joint_id], name_rpose[inquery_pose_id],
                   name_rspeed[inquery_speed_v3012]);


        signed char act_mode[10] = {_mode, _mode, _mode, _mode, _mode, _mode, _mode, _mode, _mode, _mode};
        robot_setmode_c(robot_name, act_mode);
        dt = get_BusyTs_s_c(getEC_deviceName(0, NULL)); //获取总线读取间隔？？


        //先使能再下电，英立说要不这样做，会有bug
        robot_power_c(robot_name); //使能
        robot_poweroff_c(robot_name); //下电


        index_movel = 1; //没使用过
        while (1) //进入主循环
        {
            torqueTimerE(0); //阻塞等待总线数据到来

            /***** 力控步进循环处理以及无力控退刀处理 *****/
            if ((1 == msg_key.power) && (1 == msg_key.start)) //必须同时输入power和start才开始执行
            {

                if (Finished_Force_depth && step_movel_num < numOffsetPoses) //条件：单次力控结束但后续还有点需要走
                {

                    while (step_movel_num < numOffsetPoses &&
                           roffsetpose_forceflag[step_movel_num] == 0) //如果还有点要走并且无力控，则一直走到指定位置
                    {
                        rpose_After_Offs = Offs(&rpose[inquery_pose_p1005], roffsetpose[step_movel_num][0],
                                                roffsetpose[step_movel_num][1], 0, 0, 0, 0);
                        step_movel_num++;
                        moveL(&rpose_After_Offs, &rspeed[inquery_speed_v3012], NULL, NULL, NULL); // moveL直线运动
                    }

                    if (step_movel_num >= numOffsetPoses) //如果已经完所有点，power和start置0
                    {
                        msg_key.power = 0;
                        msg_key.start = 0;
                    }

                    //力控步进更新。每一个offset力控一次，点没走完且为力控模式
                    msg_key.firststart = 1;
                    first_into_loop = 1;
                    flagInterpolation = 0;
                    Finished_Force_depth = 0;
                }

            }

            /***** 每次循环获取机器人关节位置 *****/
            robot_getposition_angle(robot_name, position_realtime); //获取机器人当前位置（3个关节）
            Position_now = position_realtime[0]; //Position_now存储的是第一个关节的位置

            /***** 插补代码，如果每次循环位移不能超过POSITION_DELTA_THRESHOLD *****/
            if ((1 == msg_key.power) && (1 == msg_key.start)) //必须同时输入power和start才开始执行
            {
                if (flagInterpolation == 1) { //插补标志位 363L 和 372L
                    if ((Position_ref_pf - Position_now) > POSITION_DELTA_THRESHOLD) {
                        Position_ref_pf = Position_now + POSITION_DELTA_THRESHOLD;
                    } else if ((Position_now - Position_ref_pf) > POSITION_DELTA_THRESHOLD) {
                        Position_ref_pf = Position_now - POSITION_DELTA_THRESHOLD;
                    }

                    axis_setposition_angle(robot_name, Position_ref_pf, 1);
                }
            }

            /***** UNUSED *****/
            if (index_timer < CONTROL_PEROID_T) //
            {
                index_timer++;
                continue;
            } else {
                index_timer = 0;
            }

            /***** 力控部分代码，在力传感器初始化完毕之后开始处理 *****/
            if (init_finish_flag) //这个标志位在shareData.h中声明，应该是力传感器初始化是否成功的标志位
            {
                pthread_mutex_lock(&mutex_torquesensor1);
                get_X_sensor1 = force_X_sensor1;
                get_Y_sensor1 = force_Y_sensor1;
                get_Z_sensor1 = force_Z_sensor1;
                pthread_mutex_unlock(&mutex_torquesensor1);

                /**** 力控：力传感器数据的处理 ****/
                if (first_into_loop) //如果是第一次进循环，就把传感器的值赋给滤波器作为初值
                {
                    Torque_filter_in[0][0] = get_X_sensor1;
                    Torque_filter_in[0][1] = get_X_sensor1;
                    Torque_filter_in[0][2] = get_X_sensor1;

                    Torque_filter_out[0][0] = get_X_sensor1;
                    Torque_filter_out[0][1] = get_X_sensor1;
                    Torque_filter_out[0][2] = get_X_sensor1;
                    get_X_sensor1_afterFilter_last = get_X_sensor1;

                    first_into_loop = 0;
                } else {
                    Torque_filter_in[0][0] = get_X_sensor1;
                }

                //2阶低通滤波
                LowPass_order2(FILTER_FREQ, FILTER_DAMP, FILTER_PEROID, Torque_filter_out[0], Torque_filter_in[0]);
                get_X_sensor1_afterFilter = Torque_filter_out[0][0]; //获取滤波之后的力

                get_X_sensor1_dot = (get_X_sensor1_afterFilter - get_X_sensor1_afterFilter_last) / dt; // 力变化率，但未使用过
                get_X_sensor1_afterFilter_last = get_X_sensor1_afterFilter; // 保存上一次的力


                /**** 力控：获取滤波后力传感器数据值 ****/
                if ((1 == msg_key.power) && (1 == msg_key.start)) //必须同时输入power和start才开始执行
                {
                    if (msg_key.firststart == 1) // 是首次输入start（前面会重新赋值）
                    {
                        Position_d = position_realtime[0]; //初次启动，期望位置赋值为当前位置
                        Position_d_first = Position_d;
                        msg_key.firststart = 0;
                    }


                    Position_error = Position_d - Position_now; //期望位置与实际位置误差

                    /***** 力控：导纳控制计算 *****/
                    Force_d = -3.0; //期望力，X向下为正，因此期望的切骨力为负. -3N
                    Force_error = Force_d - get_X_sensor1_afterFilter;

                    Position_delta = Force_error / Admit_K; //导纳控制，计算位置变化


                    if (step_movel_num < 1) {
                        printf("Error in Force Control in offset pose index");
                        return;
                    }


                    if (Position_now - Position_d_first <
                        roffsetpose[step_movel_num][1] - roffsetpose[step_movel_num - 1][1]) { //每一次力控的距离是否已经超过offset量

                        //当期望位置和实际位置误差<2.0,则给position_d一个0.06增量，为何？
                        if (Position_error < 2.0)
                            Position_d += INC_POSITION_PER; //期望位置增加0.06的增量（单位mm?）

                    } else {
                        Finished_Force_depth = 1; //单次力控已经走完

                        if (step_movel_num < numOffsetPoses)
                            step_movel_num++;
                    }

                    if (fabs(Position_delta - Position_delta_last) < POSITION_NOISE_NOFORCE) {
                        Position_delta = Position_delta_last;
                    }
                    Position_delta_last = Position_delta;

                    Position_ref_pf = Position_d - Position_delta;

                    flagInterpolation = 1;

                    printf("\033[1;32m --->force_now:%f; force_delta:%f; pos_now: %f; pos_delta:%f; posref:%f \r\n \033[0m",
                           get_X_sensor1_afterFilter, Force_error, Position_now, Position_delta, Position_ref_pf);

                    if (axis_getstatus_c(robot_name, 1) != 4663) {
                        msg_key.power = 0;
                    }
                } else { //
                    flagInterpolation = 0;
                }


            }

            //////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////

        }

        for (dataId = 0; dataId < numJoints; dataId++) {
            free(name_rjoint[dataId]);
        }
        free(name_rjoint);
        free(rjoint);

        for (dataId = 0; dataId < numPoses; dataId++) {
            free(name_rpose[dataId]);
        }
        free(name_rpose);
        free(rpose);

        for (dataId = 0; dataId < numSpeeds; dataId++) {
            free(name_rspeed[dataId]);
        }
        free(name_rspeed);
        free(rspeed);

        for (dataId = 0; dataId < numOffsetPoses; dataId++) {
            free(roffsetpose[dataId]);
        }
        free(roffsetpose);
        free(roffsetpose_forceflag);

        printf("All is OK!");

    }

    /// @brief 处理切骨逻辑
    void boneCuttingHandler() {

        //六维力传感器开始循环接收数据
#ifdef ENABLE_FTSENSOR
        auto rtMode = _ftPtr->getRealTimeDataMode();
        auto rtDataValid = _ftPtr->getRealTimeDataValid();
        _ftPtr->startRealTimeDataRepeatedly<float>(boost::bind(&BoneCuttingRobot::ftDataHandler, this, _1), rtMode,
                                                   rtDataValid);
#endif

        if (!getSpeedLimit(SPEED_LIMIT, _speedLimit)) {
            std::cout << _f % Color::RED << "[ERROR]" << _timer.format(4, _fmt) << "Can not get speed limit named >"
                      << SPEED_LIMIT << "<" << _def << std::endl;
        } else {
            std::cout << _f % Color::GREEN << "[INFO]" << _timer.format(4, _fmt)
                      << "Successfully get speed limit named >"
                      << SPEED_LIMIT << "<" << _def << std::endl;
        }

        if (!getJointSpacePoint(J0, _j0)) {
            std::cout << _f % Color::RED << "[ERROR]" << _timer.format(4, _fmt)
                      << "Can not get joint space point named >" << J0 << "<" << _def << std::endl;
        } else {
            std::cout << _f % Color::GREEN << "[INFO]" << _timer.format(4, _fmt)
                      << "Successfully get joint space point named >" << J0 << "<" << _def << std::endl;
        }

        if (!getCartesianSpacePoint(P0, _p0)) {
            std::cout << _f % Color::RED << "[ERROR]" << _timer.format(4, _fmt)
                      << "Can not get cartesian space point named >" << P0 << "<" << _def << std::endl;
        } else {
            std::cout << _f % Color::GREEN << "[INFO]" << _timer.format(4, _fmt)
                      << "Successfully get cartesian space point named >" << P0 << "<" << _def << std::endl;
        }

        //先使能再下电，英立说要不这样做，会有bug
        robotPowerOn();
        robotPowerOff();

        /// @brief 循环开始， 在这里处理切骨逻辑判断
        while (isRunning) {
//            std::cout << "[DEBUG] 1" << std::endl;
            torqueTimerE(0); //阻塞等待总线数据到来
//            std::cout << "[DEBUG] 2" << std::endl;
            //每个循环先获取一下当前机器人位姿
            auto pos = getRobotPosition();
            //TODO: DEBUG信息输出
//            std::cout << "\r" << _f % Color::MAGENTA << "[DEBUG]" << _timer.format(4, _fmt) << " pos-> " << pos[0] << "\t" << pos[1]
//                      << "\t" << pos[2] << " force-> " << _ftValue[0] << "\t" << _ftValue[1] << _def << std::flush;
//            std::cout << "[DEBUG] 3" << std::endl;
            switch (_cutState) {
                case CS_WAIT: //初始化后会进入这个状态
                    //TODO：等待切骨状态处理
                    if (_powerFlag && _startFlag) {
                        std::cout << _b % Color::YELLOW << ">>>>>>>>>>>START CUTTING>>>>>>>>>>>>>" << _def
                                  << std::endl;
                        //1. 先等待力传感器校准
                        _ftState = FS_CALIBRATION;
                        _cutState = CS_START;

                        robotPowerOn(); //机器人使能
                    }
                    break;
                case CS_START:
                    //TODO:开始切骨处理
                    cuttingStartProcess();
                    break;
                case CS_FINISH: //切骨结束后处理，处理完回到CS_WAIT状态
                    //TODO: 切骨完成处理

                    _cutState = CS_WAIT;
                    break;
                default:
                    break;
            }
        }
    }

    void cuttingStartProcess() {
        if (_ftState != FS_FILTER) {
            return;
        }

        offsetpose op = _cuttingOffsets.front(); // 读取偏移点FIFO的栈顶点


        static int i = 0;
        setAxisPosition(1, 10 * sin(0.01 * i++));
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

//        _ftMutex.lock();

        for (auto &ft : rtData) {

            for (int i = 0; i < 3; i++) {
                _ftValue[i] = ft[i];
                _ftFilterValue[i] = _lp2[i]->update(_ftValue[i]);
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

                        std::cout << _f % Color::CYAN << "[DEBUG]" << _timer.format(4, _fmt) << "Calibration done. "
                                  << "x: " << _ftGravComp[0] << "\t" << "y: " << _ftGravComp[1] << "\t" << "z: "
                                  << _ftGravComp[2]
                                  << _def << std::flush;

                        _ftState = FS_FILTER; //进入滤波模式
                    }

                    break;

                case FS_FILTER: // 进入滤波模式
                    init_finish_flag = 1;
                    force_X_sensor1 = _ftValue[0];
                    force_Y_sensor1 = _ftValue[1];
                    force_Z_sensor1 = _ftValue[2];
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

//        _ftMutex.unlock();

    }

    void keyHandler() {
        boost::asio::io_service _ioService; //用于按键检测

        boost::asio::posix::stream_descriptor stream(_ioService, STDIN_FILENO);

        boost::asio::streambuf b;

        boost::function<void(boost::system::error_code, size_t)> readHandler;

        readHandler = [&](boost::system::error_code ec, size_t len) {
            if (ec) {
                std::cerr << "exit with " << ec.message() << std::endl;
            } else {
                std::istream is(&b);
                std::string line;
                std::getline(is, line);
                if (line == "power") {
                    std::cout << "\r" << _f % Color::CYAN << "[DEBUG]" << _timer.format(4, _fmt) << line
                              << ": Detected Power Order."
                              << _def << std::endl;
                    _powerFlag = true;
                } else if (line == "start") {
                    std::cout << "\r" << _f % Color::CYAN << "[DEBUG]" << _timer.format(4, _fmt) << line
                              << ": Detected Start Order."
                              << _def << std::endl;
                    _startFlag = true;
                }

                async_read_until(stream, b, "\n", readHandler);
            }
        };

        async_read_until(stream, b, "\n", readHandler);

        _ioService.run();

    }

};


#endif //BONECUTTINGROBOT_BONECUTTINGROBOT_HPP
