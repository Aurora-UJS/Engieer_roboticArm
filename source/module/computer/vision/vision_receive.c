#include "vision_receive.h"
#include "crc8_crc16.h"
#include "uart_api.h"
#include "CRC8_CRC16.h"
#include "string.h"


ReceivedPacketVision_t ReceivedPacketVision = {0};
void vision_receive_solve(uint8_t *rx_buf)
{
    if (rx_buf[0] == 0xA5 && verify_CRC16_check_sum(rx_buf, 48))
    {
        if (verify_CRC16_check_sum(rx_buf, 48))
        {
            memcpy(&ReceivedPacketVision, rx_buf, sizeof(ReceivedPacketVision_t));
        }
    }
}