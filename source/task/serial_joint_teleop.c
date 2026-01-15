#include "serial_joint_teleop.h"



/**
 * @brief 角度接收回调
 * 
 * @param buf 接收缓冲数组
 * @param len 数组长度
 */
void Angle_Receive_Callback(uint8_t *buf, uint32_t len)
{
    (void)len; // len暂时没有使用到
    if (firstEnableFlag == 0) {
        firstEnableFlag = 1;
    }
    else {
        for (int i = 0; i < 6; i++) {
            int tmp = 0;
            // 每个弧度占 4 个字符
            sscanf((const char*)&buf[i * 4], "%04d", &tmp);
            joint_radian[i] = tmp / 1000.0f;
            joint_radian[i] = joint_radian[i] - PI;
        }
    }
}

/**
 * @brief 串口角度接收初始化
 * 
 */
void angle_msg_rx_init(void)
{
    Angle_msg.rx_msg = &Angle_rx_msg;
    Angle_msg.rx_msg -> pBuffer = Angle_rx_msg_Buffer;
    Angle_msg.rx_msg -> huart = &huart7;
    Angle_msg.rx_msg -> Len = 256;
    uart7_rx_hook = Angle_Receive_Callback;
    uart_rx_init(&Angle_msg);
}

/**
 * @brief 电机初始化
 * 
 */
void joint_motor_init(void)
{
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        joint_motor[joint_index] = pvPortMalloc(sizeof(DM_motor_t));
        //配置 can1
        joint_motor[joint_index]->can_cfg.port = CAN2_PORT;
        joint_motor[joint_index]->tmp.PMAX = 12.5f;
        joint_motor[joint_index]->tmp.VMAX = 3.0f;
        joint_motor[joint_index]->tmp.TMAX = 1.0f;
        // 配置can id
        joint_motor[joint_index]->can_cfg.id = 0x01 + joint_index;
        joint_motor[joint_index]->motor_msg.can_msg.id = 0x11+ joint_index;
        Motor_DM_Init(joint_motor[joint_index]);
    }
}

/**
 * @brief 电机数据更新
 * 
 */
void Joint_Motor_Refresh(void)
{
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        Motor_DM_Refresh(joint_motor[joint_index]);
    }
}

/**
 * @brief 电机使能
 * 
 */
void Joint_Motor_Enable(void)
{
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        // osDelay(100);
        Motor_DM_Enable(joint_motor[joint_index]);
    }
}

// 任务函数
void uart_Transmit_Angle(void *argment)
{
    UNUSED(argment);
    osDelay(10);
    angle_msg_rx_init();
    while (1) {
        osDelay(10);
    }
}


void jointFollowAngle(void *argument)
{
    UNUSED(argument);
    joint_motor_init();
    osDelay(100);
    Joint_Motor_Enable(); // 使能所有电机
    while (1) {
        Joint_Motor_Refresh();

        PosSpeed_CtrlMotorDM(joint_motor[1],limit(joint_radian[1], 0, 1.5), 1); // pitch轴控制
        osDelay(1);
        PosSpeed_CtrlMotorDM(joint_motor[2],limit(-joint_radian[2], 0, 1.5), 1); // pitch轴控制
        osDelay(1);
        PosSpeed_CtrlMotorDM(joint_motor[3],joint_radian[3],  1); // roll轴控制

        osDelay(1);
        PosSpeed_CtrlMotorDM(joint_motor[4],limit(-joint_radian[4],-1.5, 1.5), 1); // pitch轴控制
        osDelay(1);

        PosSpeed_CtrlMotorDM(joint_motor[5],joint_radian[5], 1); // roll轴控制
        osDelay(1);
    }
}
