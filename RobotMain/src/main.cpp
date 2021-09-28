/*
 * main.cpp
 *
 *  Created on: 2019-12-2
 *      Author: Liyingli
 *      modified by: Yang Luo 2021.03.28
 */
#include <stdlib.h>
#include <stdio.h>
#include <csignal>

#include <QtCore>
#include <QProcess>
#include <QString>
#include <QDebug>
#include <QTextStream>
#include <QFile>

#include <iostream>

#include "functionTest.h"
//#include"RobotControl.h"
#include "RobotLib.h"
//#include "tcp.h"

#include "sri/ftsensor.hpp"
#include "sri/commethernet.hpp"
#include "BoneCuttingRobot.hpp"
#include "Joystick.hpp"

#include <boost/thread.hpp>

bool isRuning = true;

/// \brief 处理终端的Ctrl-C信号
/// \param signo
void signalHandler(int signo) {
    if (signo == SIGINT) {
        std::cout << "\033[1;31m" << "[!!SIGNAL!!]" << "INTERRUPT by CTRL-C" << "\033[0m" << std::endl;
        isRuning = false;
        exit(0);
    }

}

/// \brief 在Linux下判断进程是否正在运行
/// \param processName
/// \return
bool isProcessExist(QString processName) {
    QProcess process;
    QStringList options;
    options << "-c" << "ps -ef | awk '{print $8}'";
    process.start("/bin/bash", options);
    process.waitForFinished();

    QByteArray result = process.readAllStandardOutput();
    QString str = result;
    if(str.contains(processName))
        return true;
    else
        return false;
}

/// \brief 启动八室的主站y2程序，逻辑是先调用initECM.sh来卸载网卡驱动，之后运行runECM.sh，其输出重定向至output.txt文件中
///        runECM.sh中内容修改为
///                            1 #!/bin/sh
///                            2 cd /opt/ECMworkspace_64
///                            3 nice -n -20 ./y2 -f eni.xml -i8254x 6 1 -b 1000 -v 3 -perf -t 0 -sp 6000 > output.txt
/// \return
bool startECMy2() {
    QProcess process;
    QStringList options;
    options << "/opt/ECMworkspace_64/initECM.sh";
    process.start("/bin/bash", options);
    process.waitForFinished();
    std::cout << process.readAllStandardOutput().data();

    QFile file("/opt/ECMworkspace_64/output.txt");
    file.open(QIODevice::ReadWrite | QIODevice::Truncate | QIODevice::Text);
    QTextStream s(&file);

    options.clear();
    options << "/opt/ECMworkspace_64/runECM.sh";
    process.setProgram("/bin/bash");
    process.setArguments(options);
    process.setWorkingDirectory("/opt/ECMworkspace_64/");
    qint64 pid;
    process.startDetached(&pid);
//    qDebug() << process.waitForStarted();



    while(1) {

        QString result = s.readLine();
        if(!result.isEmpty())
            qDebug() << result;
//        QByteArray result = process.readAllStandardError();
//        std::cout << process.readAllStandardOutput().data();

        if(result.contains("Ethernet link cable disconnected")) {
            std::cout << "\033[1;31m" << "ERROR: Ethernet link cable disconnected!" << "\033[0m" << std::endl;
            return false;
        }
        else if(result.contains("Bus configuration mismatch")) {
            std::cout << "\033[1;31m" << "ERROR: Ethernet link cable disconnected!" << "\033[0m" << std::endl;
            return false;
        }
        else if(result.contains("Device or resource busy")) {
            std::cout << "\033[1;31m" << "ERROR: Device or resource busy!" << "\033[0m" << std::endl;
            return false;
        }
        else if(result.contains("Job times during startup <INIT> to <OP>")) {
            qDebug() << "EtherCAT Started";
            return true;
        }

    }

    return false;
}

int main(int argc, char *argv[]) {

    QCoreApplication a(argc, argv);

    bool isEcatRun = isProcessExist("./y2");
    qDebug() << "EtherCAT is " << isEcatRun;
    if(isEcatRun) {
//        qDebug() << "The EtherCAT Master is already Running!";
    }
    else {
        qDebug() << "The EtherCAT Master is not Running. Run[Y/y] or Simulation[N/n]?: ";
        QTextStream s(stdin);
        QString val = s.readLine();

        if(val.toUpper() == "Y") { // 启动主站并需要确定主站启动成功
            if(!startECMy2()) {
                std::cout << "\033[1;31m" << "Starting EtherCAT Master Failed!" << "\033[0m" << std::endl;
                exit(1); // 退出
            }
        }
        else if(val.toUpper() == "N") { // 启动仿真
            qDebug() << "Simulation Start";
        }
    }

    //---------------------八室初始化----------------------------
    int err = 0;
    command_arg arg;
    err = commandLineParser(argc, argv, &arg);
    if (0 != err) {
        return -1;
    }
    err = system_initialize(&arg);

    if (0 != err) {
        return err;
    }

    if (signal(SIGINT, signalHandler) == SIG_ERR) {
        std::cout << "\033[1;31m" << "Can not catch SIGINT" << "\033[0m" << std::endl;
    }

    BoneCuttingRobot bcr;
    bcr.startWorkingThread();


    //------------------------wait----------------------------------
    return a.exec();
}
