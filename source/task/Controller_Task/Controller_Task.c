#ifndef __CONTROLLER_TASK_H__
#define __CONTROLLER_TASK_H__

#include "Controller_Task.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "crc_api.h"
#include "referee_protocol.h"
#include "string.h"
#include "arm_math_types.h"
#include "motor_DJI.h"
#include "uart_api.h"
#include "usart.h"
#include <stdio.h>
#include <stdint.h>

#define FRAME_HEADER_LENGTH 5 // 帧头数据长度
#define CMD_ID_LENGTH 2       // 命令码ID数据长度
#define DATA_LENGTH 24        // 数据段长度
#define FRAME_TAIL_LENGTH 2   // 帧尾数据长度
#define DATA_FRAME_LENGTH (FRAME_HEADER_LENGTH + CMD_ID_LENGTH + DATA_LENGTH + FRAME_TAIL_LENGTH) // 整个数据帧的长度

#define CONTROLLER_CMD_ID 0x0302 // 自定义控制器命令码

float32_t test_angle[6];


const uint8_t CRC8_INIT = 0xff;
const uint8_t CRC8_table[256] =
{
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};
/**
  * @brief          calculate the crc8  
  * @param[in]      pch_message: data
  * @param[in]      dw_length: stream length = data + checksum
  * @param[in]      ucCRC8: init CRC8
  * @retval         calculated crc8
  */
/**
  * @brief          计算CRC8
  * @param[in]      pch_message: 数据
  * @param[in]      dw_length: 数据和校验的长度
  * @param[in]      ucCRC8:初始CRC8
  * @retval         计算完的CRC8
  */
uint8_t get_CRC8_check_sum(unsigned char *pch_message,unsigned int dw_length,unsigned char ucCRC8)
{
    unsigned char uc_index;
    while (dw_length--)
    {
        uc_index = ucCRC8^(*pch_message++);
        ucCRC8 = CRC8_table[uc_index];
    }
    return(ucCRC8);
}

/**
  * @brief          append CRC8 to the end of data
  * @param[in]      pch_message: data
  * @param[in]      dw_length:stream length = data + checksum
  * @retval         none
  */
/**
  * @brief          添加CRC8到数据的结尾
  * @param[in]      pch_message: 数据
  * @param[in]      dw_length: 数据和校验的长度
  * @retval         none
  */
void append_CRC8_check_sum(unsigned char *pch_message, unsigned int dw_length)
{
    unsigned char ucCRC = 0;
    if ((pch_message == 0) || (dw_length <= 2))
    {
        return;
    }
    ucCRC = get_CRC8_check_sum((unsigned char *)pch_message, dw_length - 1, CRC8_INIT);
    pch_message[dw_length - 1] = ucCRC;
}

    typedef struct {
     struct  {
        uint8_t sof;              // 起始字节，固定值为0xA5
        uint16_t data_length;     // 数据帧中data的长度
        uint8_t seq;              // 包序号
        uint8_t crc8;             // 帧头CRC8校验
    } frame_header;
        uint16_t cmd_id;     // 命令码
        uint8_t data[DATA_LENGTH];    // 自定义控制器的数据帧
        uint16_t frame_tail; // 帧尾CRC16校验

    } Controller_t;
uint8_t Transmit_Frame_Array[DATA_FRAME_LENGTH] = {0};
Controller_t Transmit_Frame_Data = {0};
void Data_Concatenation(const uint8_t *pData)
{
    static uint8_t seq = 0;
    uint8_t zero[2] = {0,0};
    // 帧头数据
    Transmit_Frame_Data.frame_header.sof = 0xA5;
    Transmit_Frame_Data.frame_header.data_length = DATA_LENGTH;
    Transmit_Frame_Data.frame_header.seq = seq;
    append_CRC8_check_sum((uint8_t *)(&Transmit_Frame_Data.frame_header), FRAME_HEADER_LENGTH);
    
    // 命令码ID
    Transmit_Frame_Data.cmd_id = CONTROLLER_CMD_ID;
    
    // 数据段
    memcpy(Transmit_Frame_Data.data, pData, DATA_LENGTH);
    memcpy(Transmit_Frame_Data.data+DATA_LENGTH-2, zero, 2);

    // 帧尾CRC16，整包校验
    Append_CRC16_Check_Sum((uint8_t *)(&Transmit_Frame_Data), DATA_FRAME_LENGTH);

    if (seq == 0xff) 
    {
        seq = 0;
    }
    else 
    {
        seq++;
    }
}
uart_msg_t Controller_Frame_tx_msg;
void Controller_Frame_tx_msg_init(uart_msg_t *tx_msg, uint8_t *tx_buf, uint32_t length)
{
    
    tx_msg->huart = &huart7;
    tx_msg->pBuffer = tx_buf;
    tx_msg->Len = length;
}
void Controller_Task(void *argument)
{
    /* USER CODE BEGIN Controller_Task */
    UNUSED(argument);
    float32_t Joint_Angle[6];
    float32_t Angle_Zero_Point[6];
    DJI_motor_t *Controller_Motor_DJI;
    DJI_motor_t *Controller_Motor_6020;
    uart_msg_t Controller_Uart_tx_msg;
    uint8_t Uart_Send_Buffer[] = {0};
    // Tx_Message.frame_header.sof = 0xA5;
    // Tx_Message.frame_header.data_length = DATA_LENGTH;
    // Tx_Message.frame_header.seq = seq;

    Controller_Motor_DJI = pvPortMalloc(sizeof(DJI_motor_t));
    Controller_Motor_6020 = pvPortMalloc(sizeof(DJI_motor_t));
    Motor_Init_DJI(&Controller_Motor_DJI, &Controller_Motor_6020); 
    osDelay(200);
    for(int i = 0;i < 6;i++)
    {
        Controller_Angle_Refresh(Controller_Motor_DJI,  Controller_Motor_6020, Joint_Angle);
        Angle_Zero_Point[i] = Joint_Angle[i];
    }

    Controller_Frame_tx_msg_init(&Controller_Frame_tx_msg, Transmit_Frame_Array, DATA_FRAME_LENGTH);
    Controller_Uart_tx_init(&Controller_Uart_tx_msg, Uart_Send_Buffer);

    for(;;)
    {
        Controller_Angle_Refresh(Controller_Motor_DJI,  Controller_Motor_6020, Joint_Angle);
        for(int i = 0;i < 6;i++)
        {
            Joint_Angle[i] = Joint_Angle[i] - Angle_Zero_Point[i];
            if(Joint_Angle[i] < -180)
            {
                Joint_Angle[i] += 360;
            }
            else if(Joint_Angle[i] > 180)
            {
                Joint_Angle[i] -= 360;
            }   
            Joint_Angle[i] += 180;
            Joint_Angle[i] = Joint_Angle[i] * 3.1415926 / 180;
            test_angle[i] = Joint_Angle[i];
        }
        UART_Message_Trans(Joint_Angle, Uart_Send_Buffer);

        Data_Concatenation(Uart_Send_Buffer); // 将 Uart 作为数据段
        Controller_Frame_tx_msg.pBuffer = Transmit_Frame_Array; 
        Controller_Frame_tx_msg.Len = DATA_FRAME_LENGTH;
        // Controller_Uart_tx_msg.pBuffer = Uart_Send_Buffer;
        // Controller_Uart_tx_msg.Len = CONTROLLER_UART_DATA_LEN;
        uart_tx_send_IT(&Controller_Uart_tx_msg);
        osDelay(10);
    }
}

void Motor_Init_DJI(DJI_motor_t **Controller_Motor_DJI, DJI_motor_t **Controller_Motor_6020) 
{

  if (*Controller_Motor_DJI == NULL) 
  {
    return;
  }
  memset(*Controller_Motor_DJI, 0, sizeof(DJI_motor_t)); // 清零内存
  (*Controller_Motor_DJI)->can_cfg.port = CAN1_PORT;
  (*Controller_Motor_DJI)->can_cfg.id = Controller_Motor_DJI_All_ID;
  (*Controller_Motor_DJI)->motor_msg[Controller_Motor_3508_Joint_2].can_msg.id =Controller_Motor_3508_Joint_2_ID;
  (*Controller_Motor_DJI)->motor_msg[Controller_Motor_2006_Joint_3].can_msg.id =Controller_Motor_2006_Joint_3_ID;
  (*Controller_Motor_DJI)->motor_msg[Controller_Motor_2006_Joint_4].can_msg.id =Controller_Motor_2006_Joint_4_ID;
  (*Controller_Motor_DJI)->motor_msg[Controller_Motor_2006_Joint_5].can_msg.id =Controller_Motor_2006_Joint_5_ID;
  Motor_DJI_Init(*Controller_Motor_DJI);

  if (*Controller_Motor_6020 == NULL) 
  {
    return;
  }
  memset(*Controller_Motor_6020, 0, sizeof(DJI_motor_t)); 
  (*Controller_Motor_6020)->can_cfg.port = CAN1_PORT;
  (*Controller_Motor_6020)->can_cfg.id = Controller_Motor_6020_All_ID;
  (*Controller_Motor_6020)->motor_msg[Controller_Motor_6020_Joint_0].can_msg.id =Controller_Motor_6020_Joint_0_ID;
  (*Controller_Motor_6020)->motor_msg[Controller_Motor_6020_Joint_1].can_msg.id =Controller_Motor_6020_Joint_1_ID;
  Motor_DJI_Init(*Controller_Motor_6020);
}

void Controller_Angle_Refresh(DJI_motor_t *Controller_Motor_DJI, DJI_motor_t *Controller_Motor_6020, float32_t *Joint_Angle)
{
    Motor_DJI_Refresh(Controller_Motor_DJI);
    Motor_DJI_Refresh(Controller_Motor_6020);
    for(int i = 0;i < 4;i++)
    {
        Joint_Angle[i] = Controller_Motor_DJI->motor_msg[i].motor_angle * Trans_Angle + 180;
    }
    for(int i = 0;i < 2;i++)
    {
        Joint_Angle[i + 4] = Controller_Motor_6020->motor_msg[i].motor_angle * Trans_Angle + 180;
    }
}

void UART_Message_Trans(float32_t float_array[], uint8_t out_buf[]) 
{
    for (int i = 0; i < 6; i++)
    {
        int scaled = (int)(float_array[i] * 1000.0f);
        if (scaled < 0) {
            scaled = 0;
        } else if (scaled > 9999) {
            scaled = 9999;
        }

        char four_digit[5];
        (void)snprintf(four_digit, sizeof(four_digit), "%04d", scaled);
        memcpy(&out_buf[i * 4], four_digit, 4);
    }
}

uart_status_t UART_Message_Parse(const uint8_t in_buf[], float32_t out_array[])
{
    for (int i = 0; i < 6; i++)
    {
        int value = 0;
        for (int j = 0; j < 4; j++)
        {
            uint8_t c = in_buf[i * 4 + j];
            if (c < '0' || c > '9') {
                return UART_ERROR;
            }
            value = value * 10 + (int)(c - '0');
        }
        out_array[i] = ((float32_t)value) / 1000.0f;
    }
    return UART_OK;
}


void Controller_Uart_tx_init(uart_msg_t *tx_msg, uint8_t *tx_buf)
{
    tx_msg->huart = &huart7;
    tx_msg->pBuffer = tx_buf;
    tx_msg->Len = CONTROLLER_UART_DATA_LEN;
}
#endif /* __CONTROLLER_TASK_H__ */
