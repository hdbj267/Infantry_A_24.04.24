/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia 
 * @Teammate：
 * @Version: V3.0
 * @Date:2021.4.13
 * @Description:   关于底盘的控制
 * @Note:       
 * @Others: 
**/
#include "chassis_task.h"
#include "pid.h"
#include "can1_app.h"
#include "can2_app.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "pid.h"
#include "math.h"
#include "rule.h"
#include "connect_task.h"
#include "main.h"
#include "oled.h"
#include "stdio.h"
#include "stdlib.h"
#include "time.h"

chassis_control_data_t chassis_control_data;
chassis_pid_t chassis_pid;
float last_in=0;
float T=0.001;
float out;

float forwardfeed(float in)
{
//	int k1=2000000000,k2=491200000;
// out=k1*(in-last_in)/T-k2*in;
//	if((in<70)&&(in>-70))
//		out=0;
//	else
//	out=0.7*(0.05*in-79.36);
	//last_in=in;
	if(in<17)
		out=0;
	else if(in>=17&&in<103)
		out=(double)0.0778*in+562.3;
	else if(in>=103&&in<157)
		out=(double)0.1319*in+559.8;
	else if(in>=157&&in<193)
		out=(double)0.0804*in+567.7;
	else if(in>=193&&in<232)
		out=(double)0.0553*in+572.5;
	else if(in>=232&&in<275)
		out=(double)0.0602*in+571.5;
	else if(in>=275)
		out=(double)0.0399*in+575.5;

	return out;
}

int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}
/**
  * @brief        	底盘pid初始化
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void chassis_pid_init(pid_t *pid, cali_pid_t *cali_pid)
{
	pid->kp = cali_pid->kp;
	pid->ki = cali_pid->ki;
	pid->kd = cali_pid->kd;
	
	pid->ioutMax = cali_pid->ioutput_max;
	pid->outputMax = cali_pid->output_max;
	
	pid->mode = cali_pid->mode;
	
	pid->Calc = &PID_Calc;
	pid->Reset =  &PID_Reset;
}


/**
  * @brief          底盘初始化
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void chassis_init(chassis_control_data_t *chassis, chassis_pid_t *chassis_pid)
{
	chassis->connect = get_connect_data_point();
	chassis->cm1_msg = get_cm1_msg_point();
	chassis->cm2_msg = get_cm2_msg_point();
	chassis->cm3_msg = get_cm3_msg_point();
	chassis->cm4_msg = get_cm4_msg_point();
	chassis->yaw_motor_msg = get_yaw_motor_msg_point();
	
	chassis_pid_init(&chassis_pid->cm1_pid, &cali_chassis_pid.cm_pid);
	chassis_pid_init(&chassis_pid->cm2_pid, &cali_chassis_pid.cm_pid);
	chassis_pid_init(&chassis_pid->cm3_pid, &cali_chassis_pid.cm_pid);
	chassis_pid_init(&chassis_pid->cm4_pid, &cali_chassis_pid.cm_pid);
	
	chassis_pid_init(&chassis_pid->rotate_pid, &cali_chassis_pid.rotate_pid);
}
/**
  * @brief        小陀螺下的运动解算 
  * @author         
  * @param[in]      
  * @retval			
  * @note        //前面会出现负号，是因为6020是反向安装的，也可根据实际调试得到 
  */
void rotate_motion_mode_process(chassis_control_data_t *chassis)
{
	chassis->rotate_motion.yaw_current_ecd = chassis->yaw_motor_msg->encoder.raw_value;
	if(chassis->chassis_control_mode_flag)
	{
		chassis->rotate_motion.yaw_init_ecd = GAMBAL_YAW_INIT_ENCODE_VALUE_RHOMB;
	}
	else
	{
		chassis->rotate_motion.yaw_init_ecd = GAMBAL_YAW_INIT_ENCODE_VALUE_COMMON;
	}
	//得到从中值沿逆时针方向0到360度变化的角度
	if(chassis->rotate_motion.yaw_current_ecd <= chassis->rotate_motion.yaw_init_ecd)
	{
		chassis->rotate_motion.											   \
		chassis_gimbal_angle = (float)(chassis->rotate_motion.yaw_init_ecd \
							    - chassis->rotate_motion.yaw_current_ecd)  \
								* GAMBAL_ENCODE_TO_ANGLE;
	}
	else if(chassis->rotate_motion.yaw_current_ecd > chassis->rotate_motion.yaw_init_ecd)
	{
		chassis->rotate_motion.   													   \
		chassis_gimbal_angle = 360.0f - (float)(chassis->rotate_motion.yaw_current_ecd \
										 - chassis->rotate_motion.yaw_init_ecd)        \
										 * GAMBAL_ENCODE_TO_ANGLE;
	}
#if 1
	chassis->forward_back_set = (int16_t)                                                         \
								((float)chassis->forward_back *  								  \
								 cos(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI) + \
							    (float)chassis->left_right * 									  \
								(-sin(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI)));
	
	chassis->left_right_set =   (int16_t)                                                         \
								((float)chassis->forward_back *  								  \
								 sin(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI) + \
							    (float)chassis->left_right * 									  \
								 cos(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI));
#else
	//forward_and_left test
	chassis->forward_back_set = (int16_t)                                                        \
								((float)chassis->forward_back *  								 \
								 cos(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI));
	chassis->left_right_set =   (int16_t)                                                        \
								((float)chassis->forward_back *  								 \
								 sin(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI));
								
	//left_and_right test
	chassis->forward_back_set =  (int16_t)
								 ((float)chassis->left_right * 									 \
								 (-sin(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI)));
	chassis->left_right_set = 	 (int16_t)
								 ((float)chassis->left_right * 									 \
								  cos(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI));
#endif
	
}
/**
  * @brief        获取移动控制量
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void get_forward_back_value(chassis_control_data_t *chassis)
{
	int16_t speed = 0;
	if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_HIGH_SPEED_KEY)
	{
		
		speed = CHASSIS_MOUSE_CTRL_HIGH_SPPED;

	}
	else 
	{
		speed = CHASSIS_MOUSE_CTRL_NORMAL_SPPED;

	}

	if(chassis->connect->can2_rc_ctrl.control_mode == REMOTE_MODE)      
	{
		if ( RC_abs(chassis->connect->can2_rc_ctrl.rc.ch3) < 500 || RC_abs(chassis->connect->can2_rc_ctrl.rc.ch2) < 500)
		{
			if(chassis->connect->can2_rc_ctrl.rc.ch3 < 0 && chassis->connect->can2_rc_ctrl.rc.ch2 < 0)
			{	
				chassis->forward_back = -((chassis->connect->can2_rc_ctrl.rc.ch3 * chassis->connect->can2_rc_ctrl.rc.ch3)/500) *  \
										CHASSIS_RC_CTRL_SPPED_MAX_FACT;
				chassis->left_right = -((chassis->connect->can2_rc_ctrl.rc.ch2 *   chassis->connect->can2_rc_ctrl.rc.ch2)/500) *   \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
			}
			else if (chassis->connect->can2_rc_ctrl.rc.ch3 < 0 )
			{
				chassis->forward_back = -((chassis->connect->can2_rc_ctrl.rc.ch3 * chassis->connect->can2_rc_ctrl.rc.ch3)/500) *  \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
				chassis->left_right = (chassis->connect->can2_rc_ctrl.rc.ch2 *   chassis->connect->can2_rc_ctrl.rc.ch2)/500 *   \
								CHASSIS_RC_CTRL_SPPED_MAX_FACT;
			}
			else if ( chassis->connect->can2_rc_ctrl.rc.ch2 < 0 )
			{
				chassis->forward_back = (chassis->connect->can2_rc_ctrl.rc.ch3 * chassis->connect->can2_rc_ctrl.rc.ch3)/500 *  \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
				chassis->left_right = -((chassis->connect->can2_rc_ctrl.rc.ch2 *   chassis->connect->can2_rc_ctrl.rc.ch2)/500) *   \
								    CHASSIS_RC_CTRL_SPPED_MAX_FACT;
			}
			else 
			{
				chassis->forward_back = (chassis->connect->can2_rc_ctrl.rc.ch3 * chassis->connect->can2_rc_ctrl.rc.ch3)/500 *  \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
				chassis->left_right = (chassis->connect->can2_rc_ctrl.rc.ch2 *   chassis->connect->can2_rc_ctrl.rc.ch2)/500 *   \
								    CHASSIS_RC_CTRL_SPPED_MAX_FACT;
			}
		}
		else 
		{
			chassis->forward_back = chassis->connect->can2_rc_ctrl.rc.ch3 *    \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
			chassis->left_right = chassis->connect->can2_rc_ctrl.rc.ch2 *      \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
		}
	}
	else if(chassis->connect->can2_rc_ctrl.control_mode ==  KEY_MOUSE_MODE)   //鼠标键模式  *hyj
	{
		if(chassis->connect->can2_rc_ctrl.mouse.key & ROBOT_COMMON_MODE_KEY)       //普通底盘运动 V
		{
			chassis->chassis_control_mode_flag = 0;

		}
		else if(chassis->connect->can2_rc_ctrl.mouse.key & ROBOT_RHOMB_MODE_KEY)   //斜方形底盘运动 C
		{
			chassis->chassis_control_mode_flag = 1;
		}

		if(chassis->chassis_control_mode_flag)	//斜方形底盘运动
		{
			//forward and back
			if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_FORWARD_KEY)//W
			{
				chassis->forward_back = speed;
				chassis->left_right = -speed;
			}
			else
			 if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_BACK_KEY)//S
			{
				chassis->forward_back = -speed;
				chassis->left_right = speed;
			}
			//left and right
			else if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_LEFT_KEY)//L
			{
				chassis->forward_back = -speed;
				chassis->left_right = -speed;

			}
			else if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_RIGHT_KEY)//R
			{
				chassis->forward_back = speed;
				chassis->left_right = speed;
			}
			else
			{
				chassis->forward_back = 0;
				chassis->left_right = 0;
			}	
		}
		else
		{
			//forward and back
			if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_FORWARD_KEY)//W
			{
				chassis->forward_back = speed*3/4;

			}
			else if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_BACK_KEY)//S
			{
				chassis->forward_back = -speed*3/4;

			}
			else
			{
				chassis->forward_back = 0;
			}
			//left and right
			if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_LEFT_KEY)//L
			{
				chassis->left_right = -speed*3/4;
			}
			else if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_RIGHT_KEY)//R
			{
				chassis->left_right = speed*3/4;
			}
			else
			{
				chassis->left_right = 0;
			}	
		}
	}	
}
/**
  * @brief         获取底盘旋转值  云台旋转逆时针编码值变大  左5000 右3000
  * @author         
  * @param[in]      
  * @retval			
  * @note          底盘跟随pid目前不稳定 可能某些量的转换过程中方向有错误
  */
float rotate_abs(float val)
{
	if(val < 0)
	{
		val = -val;
	}
	return val;
}
void get_rotate_value(chassis_control_data_t *chassis, chassis_pid_t *chassis_pid)
{
	if(chassis->connect->can2_rc_ctrl.work_mode == ROBOT_COMMON_MODE)	//	底盘跟随pid
	{
		if(chassis->chassis_control_mode_flag)
		{
			chassis_pid->rotate_pid.set =  GAMBAL_YAW_INIT_ENCODE_VALUE_RHOMB;
		}
		else
		{	
			chassis_pid->rotate_pid.set = GAMBAL_YAW_INIT_ENCODE_VALUE_COMMON;
		}
		chassis_pid->rotate_pid.fdb = (float)(chassis->yaw_motor_msg->encoder.raw_value \
									+ ((chassis->connect->can2_rc_ctrl.gyro.yaw_set \
								    -chassis->connect->can2_rc_ctrl.gyro.yaw_fdb) * GAMBAL_YAW_angle_VALUE+0.5f));//
		chassis_pid->rotate_pid.Calc(&chassis_pid->rotate_pid);
		
		chassis->rotate = chassis_pid->rotate_pid.output;//由负修改为正   6.25
		chassis->rotate_buff_flag = 0;

	}
	else if(chassis->connect->can2_rc_ctrl.work_mode == ROBOT_ROTATE_MOTION_MODE)   //变速小陀螺    *hyj
	{
		if(freertos_run_time % 1000 == 0)      
        {
			srand(xTaskGetTickCount());
			//czh
			if(chassis->connect->can2_rc_ctrl.rc.ch3 == 0||chassis->connect->can2_rc_ctrl.rc.ch2 == 0)
			{
				chassis->rotate = CHASSIS_ROTATE_STOP_SPEED;//500u;
				chassis->rotate_buff_flag = 0;
			}
			else
			//czh
			chassis->rotate = rand() % CHASSIS_ROTATE_BUFF_SPEED + CHASSIS_ROTATE_BASE_SPEED;// 200u;
			chassis->rotate_buff_flag = 1;
		}
	 if(chassis->rotate_buff_flag != 1)     //空档期默认为基础速度       
	 {
	 	chassis->rotate = 500u;	//CHASSIS_ROTATE_STOP_SPEED（1000） CHASSIS_ROTATE_BASE_SPEED（600）
	 }
	}
	else if(chassis->connect->can2_rc_ctrl.work_mode == ROBOT_ROTATE_STOP_MODE)	//静止小陀螺
	{
		chassis->rotate = CHASSIS_ROTATE_STOP_SPEED;//1500u;
		chassis->rotate_buff_flag = 0;
	}
	else 
	{
		chassis->rotate = 0;
	}
}
/**
  * @brief        更新底盘电机设定值和反馈值
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void chassis_set_and_fdb_update(chassis_control_data_t *chassis, \
								chassis_pid_t *chassis_pid)
{
	switch(chassis->connect->can2_rc_ctrl.work_mode)
	{
		case ROBOT_CALI_MODE:
		case ROBOT_INIT_MODE:
		case ROBOT_INIT_END_MODE:
		{
			chassis->forward_back = 0; //模式转换时清零
			chassis->left_right = 0;
			chassis->rotate = 0;

			chassis->forward_back_set = 0;
			chassis->left_right_set = 0;
			chassis->rotate_set = 0;
		}break;
		case ROBOT_COMMON_MODE: //普通底盘跟随模式
		{
			get_forward_back_value(chassis);
			get_rotate_value(chassis, chassis_pid);
			
			chassis->forward_back_set = chassis->forward_back;
			chassis->left_right_set = chassis->left_right;
			chassis->rotate_set = chassis->rotate;
		}break;
		case ROBOT_ROTATE_MOTION_MODE: //运动小陀螺模式
		{
			get_forward_back_value(chassis);//获取控制值，再使用下面函数做转换
			rotate_motion_mode_process(chassis);//运动小陀螺解算
			get_rotate_value(chassis, chassis_pid);
			
			chassis->rotate_set = chassis->rotate;
		}break;
		case ROBOT_ROTATE_STOP_MODE: //静止高速小陀螺模式
		{
			get_rotate_value(chassis, chassis_pid);
			
			chassis->forward_back_set = 0;
			chassis->left_right_set = 0;
			chassis->rotate_set = chassis->rotate;
			// chassis->connect->can2_rc_ctrl.gyro.yaw_set;
			// chassis->connect->can2_rc_ctrl.gyro.yaw_fdb;
			// LED_P6x8Str(16,5,(uint8_t *)"yaw_set:");
			// LED_PrintValueI(75,5,chassis_control_data.connect->can2_rc_ctrl.gyro.yaw_fdb);
		}break;
		default:
		{
			chassis->forward_back = 0;
			chassis->left_right = 0;
			chassis->rotate = 0;
			
			chassis->forward_back_set = 0;
			chassis->left_right_set = 0;
			chassis->rotate_set = 0;
		}break;
	}
	
#if 0
	chassis->rotate_set = 0; //单独调试使用 不需要旋转量
#endif	
	/*新版限底盘功率应该是在这里改，思路：给四个轮子的转速设定值同时乘上一个比例值，bnrssbstever
	然后根据四个电机电流总和来计算底盘功率，然后一旦这个功率超过裁判系统允许的最大功率（即将超过），
	就将这个比例系数减小，否则增加*/
	
	//cm1 为右上电机 依次逆时针
	chassis->cm1_set = - chassis->forward_back_set + chassis->left_right_set + chassis->rotate_set;
	chassis->cm2_set = chassis->forward_back_set + chassis->left_right_set + chassis->rotate_set;
	chassis->cm3_set = chassis->forward_back_set - chassis->left_right_set + chassis->rotate_set;
	chassis->cm4_set = - chassis->forward_back_set - chassis->left_right_set + chassis->rotate_set;
	
	chassis->cm1_fdb = chassis->cm1_msg->encoder.filter_rate;
	chassis->cm2_fdb = chassis->cm2_msg->encoder.filter_rate;
	chassis->cm3_fdb = chassis->cm3_msg->encoder.filter_rate;
	chassis->cm4_fdb = chassis->cm4_msg->encoder.filter_rate;
	//绿灯
	if(chassis->connect->can2_rc_ctrl.work_mode == ROBOT_INIT_MODE)
	{
		HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
	}
}

/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void chassis_pid_calculate(chassis_control_data_t *chassis,  \
						   chassis_pid_t *chassis_pid)
{
	chassis_pid->cm1_pid.set = chassis->cm1_set;
	chassis_pid->cm2_pid.set = chassis->cm2_set;
	chassis_pid->cm3_pid.set = chassis->cm3_set;
	chassis_pid->cm4_pid.set = chassis->cm4_set;
	
	chassis_pid->cm1_pid.fdb = chassis->cm1_fdb;
	chassis_pid->cm2_pid.fdb = chassis->cm2_fdb;
	chassis_pid->cm3_pid.fdb = chassis->cm3_fdb;
	chassis_pid->cm4_pid.fdb = chassis->cm4_fdb;
	
	chassis_pid->cm1_pid.Calc(&chassis_pid->cm1_pid);
	chassis_pid->cm2_pid.Calc(&chassis_pid->cm2_pid);
	chassis_pid->cm3_pid.Calc(&chassis_pid->cm3_pid);
	chassis_pid->cm4_pid.Calc(&chassis_pid->cm4_pid);
}


void chassis_forwardfeed(chassis_control_data_t *chassis)
{
 chassis->cm1_ff=forwardfeed(RC_abs(chassis->cm1_msg->encoder.filter_rate));
 chassis->cm2_ff=forwardfeed(RC_abs(chassis->cm2_msg->encoder.filter_rate));
	chassis->cm3_ff=forwardfeed(RC_abs(chassis->cm3_msg->encoder.filter_rate));
	chassis->cm4_ff=forwardfeed(RC_abs(chassis->cm4_msg->encoder.filter_rate));
	if(chassis->cm1_msg->encoder.filter_rate>0)
	{
	chassis->cm2_ff=-chassis->cm2_ff;
	chassis->cm3_ff=-chassis->cm3_ff;		
	}
	else
	{
	chassis->cm1_ff=-chassis->cm1_ff;
	chassis->cm4_ff=-chassis->cm4_ff;
	}
		
}

/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
 
void chassis_control_loop(chassis_control_data_t *chassis, \
						  chassis_pid_t *chassis_pid)
{
	chassis->given_current.cm1 = chassis_pid->cm1_pid.output + chassis->cm1_ff;
	chassis->given_current.cm2 = chassis_pid->cm2_pid.output + chassis->cm2_ff;
	chassis->given_current.cm3 = chassis_pid->cm3_pid.output + chassis->cm3_ff;
	chassis->given_current.cm4 = chassis_pid->cm4_pid.output + chassis->cm4_ff;
	
	if(chassis->connect->can2_rc_ctrl.control_mode == GUI_CALI_MODE)
	{
		set_chassis_stop();
	}
	else 
	{
		set_chassis_behaviour(chassis->given_current.cm1,
							  chassis->given_current.cm2,
							  chassis->given_current.cm3,
							  chassis->given_current.cm4);
	}
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
extern TaskHandle_t GUI_task_Handler;
void set_GUI_task_state(chassis_control_data_t *chassis)
{
	if(chassis->connect->can2_rc_ctrl.work_mode == ROBOT_CALI_MODE)
	{	// 需要恢复GUI任务
		LED_Init(); // OLED屏幕初始化，防止屏幕花掉
		osThreadResume(GUI_task_Handler);//任务恢复
	}
	else
	{	// 需要挂起GUI任务
		osThreadSuspend(GUI_task_Handler);//任务挂起
	}
	
	if(chassis->connect->can2_rc_ctrl.control_mode == ROBOT_CALI_MODE)
	{
		HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
	}
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           void chassis_task(void * pvParameters)
  */
 
void chassis_task(void *argument)
{
	TickType_t current_time = 0;

	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	chassis_init(&chassis_control_data, &chassis_pid);
	
	while(1)
	{
		current_time = xTaskGetTickCount();                         //当前系统时间       *hyj
		chassis_set_and_fdb_update(&chassis_control_data, &chassis_pid);
		chassis_power_limit();
		chassis_pid_calculate(&chassis_control_data, &chassis_pid);
		chassis_forwardfeed(&chassis_control_data);
		chassis_control_loop(&chassis_control_data, &chassis_pid);
		// if(chassis_control_data.connect->can2_rc_ctrl.work_mode == ROBOT_CALI_MODE)
		// {
		// 	set_GUI_task_state(&chassis_control_data);
		// }
		vTaskDelayUntil(&current_time, 1);       //1ms一次         *hyj
	}
}

