#include "main.h"
#include "gyroscope.h"

extern UART_HandleTypeDef huart8;
extern DMA_HandleTypeDef hdma_uart8_rx;

uint8_t gyroscope_rx_buf[2][GYROSCOPE_NUM];
static void gyroscope_parse(volatile const uint8_t *sbus_buf, imu_angle_t *imu_angle);
imu_angle_t imu_angle;

const imu_angle_t *gyroscope_control_point(void)
{
    return &imu_angle;
}


void gyroscope_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    

    // enable the DMA transfer for the receiver request
    // 使能DMA串口接收
    SET_BIT(huart8.Instance->CR3, USART_CR3_DMAR);

    // enalbe idle interrupt
    // 使能空闲中断
    __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);

    // disable DMA
    // 失效DMA
    __HAL_DMA_DISABLE(&hdma_uart8_rx);
    while (hdma_uart8_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_uart8_rx);
    }

    hdma_uart8_rx.Instance->PAR = (uint32_t)&(UART8->DR);
    // memory buffer 1
    // 内存缓冲区1
    hdma_uart8_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    // memory buffer 2
    // 内存缓冲区2
    hdma_uart8_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    // data length
    // 数据长度
    hdma_uart8_rx.Instance->NDTR = dma_buf_num;
    // enable double memory buffer
    // 使能双缓冲区
    SET_BIT(hdma_uart8_rx.Instance->CR, DMA_SxCR_DBM);

    // enable DMA
    // 使能DMA
    __HAL_DMA_ENABLE(&hdma_uart8_rx);
}
void gyroscope_unable(void)
{
    __HAL_UART_DISABLE(&huart8);
}
void gyroscope_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart8);
    __HAL_DMA_DISABLE(&hdma_uart8_rx);

    hdma_uart8_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_uart8_rx);
    __HAL_UART_ENABLE(&huart8);
}

// 陀螺仪初始
void gyroscope_control_init(void)
{
    gyroscope_Init(gyroscope_rx_buf[0], gyroscope_rx_buf[1], GYROSCOPE_NUM);
}

void slove_gyroscope_lost(void)
{
    gyroscope_restart(GYROSCOPE_NUM);
}
void slove_gyroscope_error(void)
{
    gyroscope_restart(GYROSCOPE_NUM);
}

// 串口中断
void UART8_IRQHandler(void)
{

    if (huart8.Instance->SR & UART_FLAG_RXNE) // 接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart8);
    }
    else if (UART8->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart8);

        if ((hdma_uart8_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_uart8_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = GYROSCOPE_NUM - hdma_uart8_rx.Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_uart8_rx.Instance->NDTR = GYROSCOPE_NUM;

            // set memory buffer 1
            // 设定缓冲区1
            hdma_uart8_rx.Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_uart8_rx);

            if (this_time_rx_len == GYROSCOPE_FRAME_LENGTH)
            {
                gyroscope_parse(gyroscope_rx_buf[0], &imu_angle);
                // 记录数据接收时间
                // detect_hook(DBUS_TOE);
                // sbus_to_usart1(sbus_rx_buf[0]);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_uart8_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = GYROSCOPE_NUM - hdma_uart8_rx.Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_uart8_rx.Instance->NDTR = GYROSCOPE_NUM;

            // set memory buffer 0
            // 设定缓冲区0
            DMA1_Stream6->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_uart8_rx);

            if (this_time_rx_len == GYROSCOPE_FRAME_LENGTH)
            {
                // 处理陀螺器数据
                gyroscope_parse(gyroscope_rx_buf[1], &imu_angle);
                // 记录数据接收时间
                // detect_hook(DBUS_TOE);
                // sbus_to_usart1(sbus_rx_buf[1]);
            }
        }
    }
}
/**
 * @brief          陀螺仪协议解析
 * @param[in]      sbus_buf: 原生数据指针
 * @param[out]     rc_ctrl: 陀螺仪数据指
 * @retval         none
 */
		
static void gyroscope_parse(volatile const uint8_t *sbus_buf, imu_angle_t *imu_angle)
{
    if (sbus_buf == NULL || imu_angle == NULL)
    {
        return;
    }
			 
    if (gyroscope_sumcrc(sbus_buf))
    {
				 imu_angle->last_yaw=imu_angle->yaw;
        imu_angle->ax = (short)(((short)sbus_buf[3] << 8) | sbus_buf[2]) / 32768.0f * 16.0f*9.5f;
        imu_angle->ay = (short)(((short)sbus_buf[5] << 8) | sbus_buf[4]) / 32768.0f * 16.0f*9.8f;
        imu_angle->az = (short)(((short)sbus_buf[7] << 8) | sbus_buf[6]) / 32768.0f * 16.0f*9.8f;

        imu_angle->gx = (short)(((short)sbus_buf[14] << 8) | sbus_buf[13]) / 32768.0f * 2000.0f;
        imu_angle->gy = (short)(((short)sbus_buf[16] << 8) | sbus_buf[15]) / 32768.0f * 2000.0f;
        imu_angle->gz = (short)(((short)sbus_buf[18] << 8) | sbus_buf[17]) / 32768.0f * 2000.0f;
							 
        imu_angle->pitch= (short)(((short)sbus_buf[25] << 8) | sbus_buf[24]) / 32768.0f * 180.0f;
        imu_angle->roll= (short)(((short)sbus_buf[27] << 8) | sbus_buf[26]) / 32768.0f * 180.0f;
        imu_angle->yaw= (short)(((short)sbus_buf[29] << 8) | sbus_buf[28]) / 32768.0f * 180.0f;
			
			imu_angle->yaw_diff = imu_angle->yaw - imu_angle->last_yaw;
				
				if(imu_angle->yaw_diff > 150.0f)
						imu_angle->yaw_counts --;
				else if(imu_angle->yaw_diff < -150.0f)
						imu_angle->yaw_counts ++;
				imu_angle->yaw_real = imu_angle->yaw + imu_angle->yaw_counts * 360.0f;
			
			
			
        imu_angle->temp = (short)(((short)sbus_buf[9] << 8) | sbus_buf[8]) / 32768.0f * 96.38f + 36.53f;
        imu_angle->v = (short)(((short)sbus_buf[20] << 8) | sbus_buf[19]) / 100.0f;
    }
}
bool_t gyroscope_sumcrc(volatile const uint8_t *sbus_buf)
{
    if (sbus_buf == NULL)
    {
        return 0;
    }

    // 定义每个校验段的起始位置和长度
    const size_t crc_lengths[] = { 10, 10, 10 };  // 每个 CRC 校验段的长度
    const size_t crc_offsets[] = { 0, 11, 22 };  // 每个 CRC 校验段的起始位置
    const size_t crc_check_positions[] = { 10, 21, 32 };  // 校验 CRC 的位置

    // 计算每个校验段的 CRC
    for (int i = 0; i < 3; ++i)
    {
        uint8_t calculated_crc = 0;

        // 计算 CRC，遍历每个字节进行加和
        for (size_t j = 0; j < crc_lengths[i]; ++j)
        {
            calculated_crc += sbus_buf[crc_offsets[i] + j];
        }

        // 校验计算出的 CRC 是否与预期值匹配
        if (calculated_crc != sbus_buf[crc_check_positions[i]])
        {
            return 0;  // 如果有任何一个校验失败，直接返回 false
        }
    }

    return 1;  // 所有 CRC 校验通过，返回 true
}


