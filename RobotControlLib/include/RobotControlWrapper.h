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

@Created on: 2021.03.28
*/

#ifndef INCLUDE_ROBOTCONTROL_H_
#define INCLUDE_ROBOTCONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

/// SIA-Controller中的机器人控制是通过调用libRobotControlLib.so中的MainControl()函数实现的
/// 由于其采用的C编译，因此利用一个RobotControlWrapper来包装接口
/// 此函数主要是封装一个C的接口
/// \return
int MainControl();

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_ROBOTCONTROL_H_ */
