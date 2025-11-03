/*********************************************************************
File name:      ryhandlib_port.h
Author:         zhanglf
Version:        v1.0
Date:           2024.11.07
Description:    ryhandlib 库对外接口（ROS 无关，C/C++ 皆可用）
*********************************************************************/

#ifndef RYHANDLIB_PORT_H
#define RYHANDLIB_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ryhandlib.h"  // 提供 s16_t/u8_t/CanMsg_t/RyCanServoBus_t 等类型定义

// 全局对象（由库/实现文件定义，这里仅声明）
extern RyCanServoBus_t stuServoCan;
extern CanMsg_t stuListenMsg[40];
extern ServoData_t sutServoDataW[15];
extern ServoData_t sutServoDataR[15];
extern volatile s16_t uwTick;

// CAN 回调及写入接口（需在你的实现中提供）
extern void MyHoockCallBck(CanMsg_t stuMsg, void * para);
extern s8_t BusWrite(CanMsg_t stuMsg);
extern void CallBck0(CanMsg_t stuMsg, void * para);

// 角度/弧度、数值映射相关工具
extern float rad_to_deg(float rad);
extern float deg_to_rad(float deg);
extern int   map_rad90_to_value(float rad);
extern int   map_rad75_to_value(float rad);
extern int   map_rad_to_value_full_range(float rad);
extern float value_to_rad90(int value);
extern float value_to_rad75(int value);
extern float value_to_rad_full_range(int value);

extern float cmd_to_radx(int cmd, float radmax);
extern int   radx_to_cmd(float rad, float radmax);

extern double evaluatePolynomial(double coefficients[], int degree, double x);

// 更新电机位置（hand_lr: 左右手标志）
extern void update_motor_positions(float *rads, int hand_lr);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // RYHANDLIB_PORT_H