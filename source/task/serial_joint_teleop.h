#ifndef SERIAL_JOINT_TELEOP_H
#define SERIAL_JOINT_TELEOP_H

#include "main.h"
#include "usart.h"
#include "uart_api.h"
#include "motor_DM.h"
#include "cmsis_os2.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "arm_math.h"
#include "tool.h"

// 常量定义
#define JOINT_NUM 7

// 全局变量定义
uint8_t Angle_rx_msg_Buffer[256];
DM_motor_t *joint_motor[JOINT_NUM];
uart_rx_t Angle_msg; // 
uart_msg_t Angle_rx_msg;
float joint_radian[6] = {0};
uint8_t firstEnableFlag = 0;

/**
 * @brief 角度接收回调
 * 
 * @param buf 接收缓冲数组
 * @param len 数组长度
 */
void Angle_Receive_Callback(uint8_t *buf, uint32_t len);

/**
 * @brief 串口角度接收初始化
 * 
 */
void angle_msg_rx_init(void); 

/**
 * @brief 电机初始化
 * 
 */
void joint_motor_init(void);

/**
 * @brief 电机数据更新
 * 
 */
void Joint_Motor_Refresh(void);


/**
 * @brief 电机使能
 * 
 */
void Joint_Motor_Enable(void);

/** 任务函数 **/

/// 串口接收角度任务
void uart_Transmit_Angle(void *argment);

/// 关节跟随角度任务
void jointFollowAngle(void *argument);

#endif
