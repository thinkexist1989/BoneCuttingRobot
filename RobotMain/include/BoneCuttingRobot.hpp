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

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include "RobotInterface.h"
//#include "kinematicInterface.h"

struct offsetpose {
    double x; // x方向偏移
    double y; // y方向偏移
    double a; // 角度偏移
    bool f; // 是否有力控
};

enum ForegroundColor {
    FG_BLACK = 30,
    FG_RED = 31,
    FG_GREEN = 32,
    FG_YELLOW = 33,
    FG_BLUE = 34,
    FG_MAGENTA = 35,
    FG_CYAN = 36,
    FG_WHITE = 37,
    FG_DEFAULT = 39,
};

enum BackgroundColor {
    BG_BLACK = 40,
    BG_RED = 41,
    BG_GREEN = 42,
    BG_YELLOW = 43,
    BG_BLUE = 44,
    BG_MAGENTA = 45,
    BG_CYAN = 46,
    BG_WHITE = 47,
    BG_DEFAULT = 49
};

class TermColor {
    ForegroundColor _fg;
    BackgroundColor _bg;
public:
    TermColor(ForegroundColor fg, BackgroundColor bg) : _fg(fg), _bg(bg) {}

    friend std::ostream &
    operator<<(std::ostream &os, const TermColor &tc) {
        return os << "\033[1;" << tc._fg << ";" << tc._bg << "m";
    }
};

}

class BoneCuttingRobot {

public:
    BoneCuttingRobot() : _modes(10, 8) {
        init(); //初始化
    }

    ~BoneCuttingRobot() {

    }

    void init() {
        _robotName = get_name_robotEC_deviceHandle_c(getEC_deviceName(0, NULL), 0);
        if (_robotName == nullptr) {
            std::cout << "Robot is not initialized" << std::endl;
            return;
        }

        robot_setmode_c(_robotName, &_modes[0]); //设置运行模式

        _interval = get_BusyTs_s_c(getEC_deviceName(0, NULL)); //获取总线读取间隔 单位是秒？
        std::cout << "EtherCAT bus interval is: " << _interval << std::endl;


        /***** 读取各种所需数据 *****/
        getJointSpacePoints("/hanbing/data/robjoint.POINT"); //读取关节示教点

        getCartesianSpacePoints("/hanbing/data/robpose.POINT"); // 读取笛卡尔空间示教点

        getSpeedLimits("/hanbing/data/speed.POINT"); // 读取速度限制

        getCuttingOffsets("/hanbing/data/offsetpose.POINT"); //读取切骨偏移量

        /*****  *****/

    }

    void robotPowerOn() {
        if (_robotName == nullptr) {
            std::cout << "Robot is not initialized" << std::endl;
            return;
        }

        robot_power_c(_robotName);
    }

    void robotPowerOff() {
        if (_robotName == nullptr) {
            std::cout << "Robot is not initialized" << std::endl;
            return;
        }

        robot_poweroff_c(_robotName);
    }

    void getJointSpacePoints(std::string fileName) {
        _jointSpacePoints.clear();

        boost::property_tree::ptree pt;
        boost::property_tree::read_ini(fileName, pt);

        for (auto &section : pt) {
            robjoint rj;
            rj.angle[0] = section.second.get<double>("1");
            rj.angle[1] = section.second.get<double>("2");
            rj.angle[2] = section.second.get<double>("3");
            rj.angle[3] = section.second.get<double>("4");
            rj.angle[4] = section.second.get<double>("5");
            rj.angle[5] = section.second.get<double>("6");
            rj.dof = 6;

            _jointSpacePoints[section.first] = rj;
        }
    }

    void getCartesianSpacePoints(std::string fileName) {
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
    }

    void getSpeedLimits(std::string fileName) {
        _speedLimits.clear();

        boost::property_tree::ptree pt;
        boost::property_tree::read_ini(fileName, pt);

        for (auto &section : pt) {
            speed sp;
            sp.per[0] = section.second.get<double>("per1");
            sp.per[1] = section.second.get<double>("per2");
            sp.per[2] = section.second.get<double>("per3");
            sp.per[3] = section.second.get<double>("per4");
            sp.per[4] = section.second.get<double>("per5");
            sp.per[5] = section.second.get<double>("per6");
            sp.per_flag = section.second.get<int>("per_flag");

            sp.tcp = section.second.get<double>("tcp");
            sp.per_flag = section.second.get<int>("tcp_flag");

            sp.orl = section.second.get<double>("orl");
            sp.orl_flag = section.second.get<int>("orl_flag");

            sp.dof = 6;

            _speedLimits[section.first] = sp;
        }
    }

    void getCuttingOffsets(std::string fileName) {
        _cuttingOffsets = std::queue<offsetpose>(); // 清空队列

        boost::property_tree::ptree pt;
        boost::property_tree::read_ini(fileName, pt);

        for (auto &section : pt) {
            offsetpose op;
            op.x = section.second.get<double>("x");
            op.y = section.second.get<double>("y");
            op.a = section.second.get<double>("a");
            op.f = section.second.get<bool>("f");

            _cuttingOffsets.push(op);
        }
    }

private:
    char *_robotName = nullptr; //机器人名，用于上电、下电等操作
    std::vector<signed char> _modes; //运行模式 8为csp
    double _interval; //dt，总线频率，应该是1 ms = 1000 us = 1000000 ns

    std::map<std::string, robjoint> _jointSpacePoints; //关节空间示教点 /hanbing/data/robjoint.POINT
    std::map<std::string, robpose> _cartesianSpacePoints; //笛卡尔空间示教点 /hanbing/data/robpose.POINT
    std::map<std::string, speed> _speedLimits; //笛卡尔空间速度 /hanbing/data/speed.POINT

    std::queue<offsetpose> _cuttingOffsets; //切骨路径规划点，相对于当前位置（先移动到j点）的偏移

};


#endif //BONECUTTINGROBOT_BONECUTTINGROBOT_HPP
