#ifndef _GYROSCOPE_H
#define _GYROSCOPE_H
#include "struct_typedef.h"
#include "main.h"

#define GYROSCOPE_NUM 66u

#define GYROSCOPE_FRAME_LENGTH 33u
/* ----------------------- Data BMI088_DATA ------------------------------------- */
typedef struct BMI088_DATA
{
            //���ٶ�
     fp32 ax;
		fp32 ay;
		fp32 az;
	//���ٶ�
		fp32 gx;				 
		fp32 gy;
		fp32 gz;
    //�Ƕ�
		fp32 pitch;
        
	fp32 yaw;
        fp32 roll;
	
		fp32 temp;//�¶�
        fp32 v;//��ѹ


		fp32 last_yaw;
		fp32 yaw_diff;
		fp32 yaw_counts;
		fp32 yaw_real;		
		fp32 yaw_real_filtered;
		fp32		gz_filtered;
} imu_angle_t; 

fp32 map_encoder_value(fp32 encoder_value);
extern void gyroscope_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void gyroscope_unable(void);
extern void gyroscope_restart(uint16_t dma_buf_num);

extern void gyroscope_control_init(void);

extern void slove_gyroscope_lost(void);
extern void slove_gyroscope_error(void);

extern bool_t gyroscope_sumcrc(volatile const uint8_t *sbus_buf);
extern const imu_angle_t *gyroscope_control_point(void);

#endif
