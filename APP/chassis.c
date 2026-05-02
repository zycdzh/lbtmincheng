#include "chassis.h"
#include "main.h"
#include "pid.h"

// ȫ�ֱ������ٶȺ�ƫ����
fp32 vy;
fp32 vx; 
int16_t yaw_angle;

//extern PidTypeDef fallow_angle_pid;  // 6020 �Ƕȸ���PID
//extern PidTypeDef fallow_speed_pid;	// 6020 �ٶȸ���PID	

fp32 vy1;
fp32 vx1;

fp32 vy2;
fp32 vx2;

fp32 v1;			// PID������������ٶ�
fp32 v2;
fp32 v3;
fp32 v4;

float Klimit = 1;
float Plimit = 0;
double Scaling1, Scaling2, Scaling3, Scaling4;

// ���ʿ�����ر���
double Chassis_pidout;
float Chassis_pidout_max;

fp32 Power;
fp32 Power_Buffer;

fp32 power_v1;			// ���ֵĹ��ʿ���
fp32 power_v2;
fp32 power_v3;
fp32 power_v4;

fp32 chassis_set_v1;
fp32 chassis_set_v2;
fp32 chassis_set_v3;
fp32 chassis_set_v4;


extern pid_Motor all_motor_pid;
fp32 A6020_angle;
fp32 A6020_speed;
fp32 A6020_M1;

chassis_mode_e chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; // Ĭ��ģʽ

const RC_ctrl_t *RC; 
const motor_system_t *c;
const imu_angle_t *imu;



 ChassisParam_Struct chassisparam ;
 
 RemoteParam_Struct remoteparam;

void ChassisPidCtr(int16_t LefUpTarspd,int16_t LefDoTarspd,int16_t RigUpTarspd,int16_t RigDoTarspd);
int16_t Ramp_function(int16_t delta_Min,int16_t target,int16_t now, int16_t ramp_Coeff, uint8_t mode);
void chassis_set_control(void);
void ChassisController(void) ;

void chassis_task(void *argument)
{
    c = get_motor_control_point();   // ��ȡ������Ƶ�
    RC = get_remote_control_point();
    imu = gyroscope_control_point();
  
    while (1)
    {
        // ���õ���ģʽ�Ϳ���
        chassis_set_mode(&chassis_mode);
        chassis_set_control();
        chassis_set_v();
				ChassisController();

//        // ���ң�����Ƿ����ض���ť
			
        if (toe_is_error(DBUS_TOE))
        {
            CAN_cmd_chassis(0, 0, 0, 0);  // ������°�ť��ֹͣ���
        }
        else
        {
           CAN_cmd_chassis(chassisparam.LefUpFinalOutValue,
														chassisparam.RigUpFinalOutValue,
														chassisparam.LefDoFinalOutValue,
														chassisparam.RigDoFinalOutValue);  // �������ͼ������ٶ�
        }
        
        osDelay(2);  // ��ʱ���������
    }
}

void chassis_set_mode(chassis_mode_e *chassis_mode)
{
    // ����ң���������л�����ģʽ
    if (RC->rc.s[1] == 0x01)
    {
        *chassis_mode = CHASSIS_VECTOR_revolve;
    }
    else if (RC->rc.s[1] == 0x03)
    {
        *chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
    }
    else
    {
        if ((RC->key.v & KEY_PRESSED_OFFSET_F) && ((*chassis_mode) != CHASSIS_VECTOR_revolve))
        {
            *chassis_mode = CHASSIS_VECTOR_revolve;
        }
        if ((RC->key.v & KEY_PRESSED_OFFSET_R) && ((*chassis_mode) != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW))
        {
            *chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
        }
    }
}

void chassis_set_control(void)
{
	
		int x = 0;                // ��ʼ�� x ������ٶ�
    static int x2 = 0;        // x �����ƽ���ٶȣ�ʹ�� static ����ֵ������ÿ�ε���ʱ���ã�
    int y = 0;                // ��ʼ�� y ������ٶ�
    static int y2 = 0;        // y �����ƽ���ٶȣ�ʹ�� static ����ֵ������ÿ�ε���ʱ���ã�
    double sin = 0.0;         // ���ڴ洢����ֵ�������ϵ�����ֵ��
    double cos = 0.0;         // ���ڴ洢����ֵ�������ϵ�����ֵ��
    int pwm1 = 0;             // ���1��PWM�źţ���ǰ��
    int pwm2 = 0;             // ���2��PWM�źţ���ǰ��
    int pwm3 = 0;             // ���3��PWM�źţ����
    int pwm4 = 0;             // ���4��PWM�źţ��Һ�
    int pwm0 = 0;             // ���ڼ�������PWMֵ�ı���




    // ����ң������������ϵͳ�ٶ�
   if (chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        A6020_M1 = c->can1_motors.yaw.ecd - A6020_M;
        if (A6020_M1 >= 4046 || A6020_M1 <= -4046)
        {
            if (A6020_M1 <= -4045)
                A6020_M1 = A6020_M1 + 8191;
            if (A6020_M1 >= 4046)
                A6020_M1 = A6020_M1 - 8191;
        }
				

        // �ǶȺ��ٶȵ�PID����
        A6020_angle = PID_Calc(&all_motor_pid.pid_follow_angle, A6020_M1, 0);
        A6020_speed = PID_Calc(&all_motor_pid.pid_follow_speed, c->can1_motors.yaw.speed_rpm, A6020_angle);
				
      
			x =  RC->rc.ch[2]-FlagCheck(RC->key.v & KEY_PRESSED_OFFSET_A) * HIGH_RmeoteSpeed 
            + FlagCheck(RC->key.v & KEY_PRESSED_OFFSET_D) * HIGH_RmeoteSpeed ; 
      y = RC->rc.ch[3]+FlagCheck(RC->key.v & KEY_PRESSED_OFFSET_W) * HIGH_RmeoteSpeed 
            - FlagCheck(RC->key.v & KEY_PRESSED_OFFSET_S) * HIGH_RmeoteSpeed ;
    }
		
		 // ͨ�� Ramp_function ���ٶȽ���ƽ������������˲ʱ�仯���µ��̲��ȶ�
    // ��һ������ 66 ���ٶ�ƽ����ʱ�䳣����x �� y �ֱ��ǵ�ǰĿ���ٶȣ�x2 �� y2 ���ϴε�ƽ��ֵ
    x2 = Ramp_function(66, x, x2, 6, 0);  // ����ƽ����� x �����ٶ�
    y2 = Ramp_function(66, y, y2, 6, 1);  // ����ƽ����� y �����ٶ�
 /****************************************/
    // ������̷����ϵ�����ֵ������ֵ
    // sin �� cos �ֱ��ʾ����ı仯��������������ٶȷ���
    float denominator = sqrt(x2 * x2 + y2 * y2);
    if (denominator < 0.001f) {
        sin = 0;
        cos = 0;
    } else {
        sin = y2 / denominator;
        cos = x2 / denominator;
    }
// ѡ��ϴ���ٶ�ֵ��������� PWM��ȷ��������㹻���������
    if (ABS(y2) > ABS(x2)) {
        pwm0 = ABS(y2);  // ��� y �����ٶȸ���ѡ�� y ����ľ���ֵ��Ϊ����ٶ�
    } else {
        pwm0 = ABS(x2);  // ��� x �����ٶȸ���ѡ�� x ����ľ���ֵ��Ϊ����ٶ�
    }		
	
		
		pwm1 = pwm0 * (sin + cos);  // ��ǰ���
    pwm2 = pwm0 * (sin - cos);  // ��ǰ���
    pwm3 = pwm0 * (sin - cos);  // �����
    pwm4 = pwm0 * (sin + cos);  // �Һ���

    // ��������� PWM ֵӳ�䵽ң�������ٶ�ָ���У�����������ٶ�����
    remoteparam.RemoteLefUpSpeed = pwm1 * MaxGivenSpeed / 660;
    remoteparam.RemoteRigUpSpeed = pwm2 * MaxGivenSpeed / 660;
    remoteparam.RemoteLefDoSpeed = pwm3 * MaxGivenSpeed / 660;
    remoteparam.RemoteRigDoSpeed = pwm4 * MaxGivenSpeed / 660;
		



   }

///**
//  * @brief          һ�׵�ͨ�˲�����
//  * @author         RM
//  * @param[in]      һ�׵�ͨ�˲��ṹ��
//  * @param[in]      �����ʱ�䣬��λ s
//  * @retval         ���ؿ�
//  */
//void first_order_filter_cali(fp32 input2, fp32 input)
//{
//   input2 = input;
//   out =
//       0.1666666667f / (0.1666666667f
//	+ 0.002f) * first_order_filter_type->out
//	+0.002f/ (first_order_filter_type->num[0] 
//	+ 0.002f) * first_order_filter_type->input;
//} 
//	

 	static float last_vx = 0, last_vy = 0;
void chassis_set_v(void)
{
    // ����ģʽ����ÿ�����ӵ��ٶ�

    

    if (chassis_mode == CHASSIS_VECTOR_revolve)
   {
				if(RC->rc.s[1] != 0x02){
	
		vx = 0.8 * last_vx + 0.2 * (RC->rc.ch[2] * 5);
		vy = 0.8 * last_vy + 0.2 * (RC->rc.ch[3] * 5);
		last_vx = vx;
		last_vy = vy;

	}else{
			vy = 0;			vx = 0;
		if(RC->key.v & KEY_PRESSED_OFFSET_W){
				vy = v_max;
			
		}
		if(RC->key.v & KEY_PRESSED_OFFSET_S){
				vy = -v_max;
				
		}
		if(RC->key.v & KEY_PRESSED_OFFSET_A){
			
				vx = -v_max;
		}
		if(RC->key.v & KEY_PRESSED_OFFSET_D){
				
				vx = v_max;
		}
	}	
     
	
			
				yaw_angle = -(c->can1_motors.yaw.ecd - A6020_M) / 22.766666;
        // ����ƫ����ת���ٶ�
		
	
        vy1 = vy * (float)(cos(yaw_angle / 57.3f)) - vx * (float)(sin(yaw_angle / 57.3f));
      vx1 = vy * (float)(sin(yaw_angle / 57.3f)) + vx * (float)(cos(yaw_angle / 57.3f));


// ���������ٶ�
chassis_set_v1 = vy1 + vx1;
chassis_set_v2 = -vy1 + vx1;
chassis_set_v3 = vy1 - vx1;
chassis_set_v4 = -vy1 - vx1;

// ������ת�ٶȣ���̬������
chassis_set_v1 += v_revolve;
chassis_set_v2 += v_revolve;
chassis_set_v3 += v_revolve;
chassis_set_v4 += v_revolve;	
	
	
      


    }else{
				 chassis_set_v1 =0;
        chassis_set_v2 =0;
        chassis_set_v3 = 0;
        chassis_set_v4 =0;
		
		
		}
}






/**
  * @brief  ���̿���
  * @param  ��
  * @retval ��
 **/
void ChassisController(void)
{
	if(remoteparam.RemoteLefUpSpeed>MaxGivenSpeed)  remoteparam.RemoteLefUpSpeed =MaxGivenSpeed;
	if(remoteparam.RemoteRigUpSpeed>MaxGivenSpeed)  remoteparam.RemoteRigUpSpeed =MaxGivenSpeed;
	if(remoteparam.RemoteLefDoSpeed>MaxGivenSpeed)  remoteparam.RemoteLefDoSpeed =MaxGivenSpeed;
	if(remoteparam.RemoteRigDoSpeed>MaxGivenSpeed)  remoteparam.RemoteRigDoSpeed =MaxGivenSpeed;
	
	if(remoteparam.RemoteLefUpSpeed<-MaxGivenSpeed) remoteparam.RemoteLefUpSpeed =-MaxGivenSpeed;
	if(remoteparam.RemoteRigUpSpeed<-MaxGivenSpeed) remoteparam.RemoteRigUpSpeed =-MaxGivenSpeed;
	if(remoteparam.RemoteLefDoSpeed<-MaxGivenSpeed) remoteparam.RemoteLefDoSpeed =-MaxGivenSpeed;
	if(remoteparam.RemoteRigDoSpeed<-MaxGivenSpeed) remoteparam.RemoteRigDoSpeed =-MaxGivenSpeed;
		
	ChassisPidCtr(remoteparam.RemoteLefUpSpeed,-remoteparam.RemoteRigUpSpeed,
	remoteparam.RemoteLefDoSpeed,-remoteparam.RemoteRigDoSpeed);//�������Է���
}

void ChassisPidCtr(int16_t LefUpTarspd,int16_t LefDoTarspd,int16_t RigUpTarspd,int16_t RigDoTarspd)
{
	
			  get_chassis_power_and_buffer(&Power, &Power_Buffer);
					//Power_Buffer=50.0f;
			  if (chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
   {
	      chassisparam.LefUpFinalOutValue = PID_Calc(&all_motor_pid.pid_front_right, c->can1_motors.front_right.speed_rpm,LefUpTarspd+A6020_speed  );
        chassisparam.RigUpFinalOutValue = PID_Calc(&all_motor_pid.pid_front_left, c->can1_motors.front_left.speed_rpm, LefDoTarspd+A6020_speed );  
        chassisparam.LefDoFinalOutValue = PID_Calc(&all_motor_pid.pid_back_left,  c->can1_motors.back_left.speed_rpm,RigUpTarspd+A6020_speed  );  
        chassisparam.RigDoFinalOutValue = PID_Calc(&all_motor_pid.pid_back_right, c->can1_motors.back_right.speed_rpm,RigDoTarspd+A6020_speed);  
	 }
	  if (chassis_mode == CHASSIS_VECTOR_revolve)
   {
		 	  chassisparam.LefUpFinalOutValue = PID_Calc(&all_motor_pid.pid_front_right, c->can1_motors.front_right.speed_rpm,chassis_set_v1);
        chassisparam.RigUpFinalOutValue = PID_Calc(&all_motor_pid.pid_front_left, c->can1_motors.front_left.speed_rpm,chassis_set_v2);  
        chassisparam.LefDoFinalOutValue = PID_Calc(&all_motor_pid.pid_back_left,  c->can1_motors.back_left.speed_rpm,  chassis_set_v3);  
        chassisparam.RigDoFinalOutValue = PID_Calc(&all_motor_pid.pid_back_right, c->can1_motors.back_right.speed_rpm,chassis_set_v4);  

	 
	 
	 }
// �жϵ�Դ���幦���Ƿ�С�� 50.0f
if (Power_Buffer < 50.0f) {
    // �����ĸ��������������ܾ���ֵ���ϲ�Ϊһ��ֵ����ʾ��ǰ�Ĺ�������
    chassisparam.MergeFinalOutValue = 
        ABS(chassisparam.LefUpFinalOutValue) +
        ABS(chassisparam.RigUpFinalOutValue) +
        ABS(chassisparam.LefDoFinalOutValue) +
        ABS(chassisparam.RigDoFinalOutValue);

    // ���㹦������ֵ��`chassisPowerBuffer` ��ƽ��ֵ����Ԥ��Ĺ������Ʋ���
    chassisparam.LimtOutValue = 
        Power_Buffer *
        Power_Buffer *
        PowerLimtParam;

    // ����ÿ����������տ���ֵ�����ݹ�������ֵ��������
    if (chassisparam.MergeFinalOutValue > 0) {
    chassisparam.LefUpFinalOutValue = 
            chassisparam.LimtOutValue * chassisparam.LefUpFinalOutValue / chassisparam.MergeFinalOutValue;
    chassisparam.RigUpFinalOutValue = 
            chassisparam.LimtOutValue * chassisparam.RigUpFinalOutValue / chassisparam.MergeFinalOutValue;
    chassisparam.LefDoFinalOutValue = 
            chassisparam.LimtOutValue * chassisparam.LefDoFinalOutValue / chassisparam.MergeFinalOutValue;
    chassisparam.RigDoFinalOutValue = 
            chassisparam.LimtOutValue * chassisparam.RigDoFinalOutValue / chassisparam.MergeFinalOutValue;
    } else {
        chassisparam.LefUpFinalOutValue = 0;
        chassisparam.RigUpFinalOutValue = 0;
        chassisparam.LefDoFinalOutValue = 0;
        chassisparam.RigDoFinalOutValue = 0;
    }
	}
}

/**
  * @brief  �ٶȻ��庯��
  * @param  1����С����ֵ2��Ŀ���ٶ�3�������ٶ�4����������5������ģʽ
  * @retval ��
 **/
int16_t Ramp_function(int16_t delta_Min,int16_t target,int16_t now, int16_t ramp_Coeff, uint8_t mode)
{
	int16_t Lenght=target-now;	
	if(ABS(Lenght)<delta_Min)
		now=target;
	else
	{
		if(mode == 0)
		{
			if(target < 0)
			{
				if(ABS(Lenght) > HIGH_RmeoteSpeed)
				{
					return target / 3;
				}
			}
		}
		if(Lenght>0)
			now+=ramp_Coeff;
		else if(Lenght<0)
			now-=ramp_Coeff;
	}
	return now;
}
