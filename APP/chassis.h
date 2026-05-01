#ifndef chassis_H
#define chassis_H
#include "main.h"

#define A6020_M 1300    //¸ú궨ҥԆ̨³¬ǰʱ£¬´˿̶020±«¾ø¶Ԗµ	
#define v_revolve 5000	
#define v_max 4000

/*---------------------遥控参数---------------------*/
#define HIGH_RmeoteSpeed		660
#define NORMAL_RemoteSpeed		200
/*---------------------云台参数---------------------*/
/*---------------------底盘参数---------------------*/
#define IS_CHASSIS_PID_DEBUG	0

#define PowerLimtParam			6.0f
#define MaxGivenSpeed			7500

#define ChassisP				6.0f   //3.0  //底盘pid
#define ChassisI				0.1f   //0.08
#define ChassisD				0.0f
#define ChassisILim				4000
#define MaxOutValue				12000  //最大输出量


#define ABS(x) ((x)>0? (x):(-(x)))
#define FlagCheck(z) ((z)>0 ? 1 : 0)


typedef struct
{
	int16_t RemoteLefUpSpeed;    //左上给定速度
	int16_t RemoteLefDoSpeed;    //左下给定速度
	int16_t RemoteRigUpSpeed;    //右上给定速度
	int16_t RemoteRigDoSpeed;    //右下给定速度
}RemoteParam_Struct;

typedef struct
{
	
	int16_t LimtOutValue;//限制功率后的所有电机输出量的总和
	float 	MergeFinalOutValue;//所有电机输出量的总和
	int16_t LefUpFinalOutValue;		//左上最后输出值
	int16_t LefDoFinalOutValue;		//左下最后输出值
	int16_t RigUpFinalOutValue;		//右上最后输出值
	int16_t RigDoFinalOutValue;		//右下最后输出值
	
}ChassisParam_Struct;

typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //µ×ÅÌ»á¸úËæÔÆÌ¨Ïà¶Ô½Ç¶È

  CHASSIS_VECTOR_revolve,       			//µ×ÅÌÐ¡ÍÓÂÝ


} chassis_mode_e;

void chassis_task(void *argument) ;

void chassis_set_control(void);	//µ×ÅÌËÙ¶ÈÉè¶¨
void chassis_set_v(void); //µ×ÅÌËÙ¶È¿ØÖÆ

void chassis_set_mode(chassis_mode_e *chassis_mode);    //µ×ÅÌÄ£Ê½¿ØÖÆ



#endif

