/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia 
 * @Teammate：
 * @Version: V3.0
 * @Date:2021.4.13
 * @Description:   人机交互OLED显示
 * @Note:       
 * @Others: 
**/
#include "GUI_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "oled.h"
#include "remote_app.h"
#include "pid.h"
#include "judge.h"
#include "adc.h"
#include "rule.h"

rc_analog_key_t rc_analog_key;
extern chassis_control_data_t chassis_control_data;
extern chassis_pid_t chassis_pid;
extern motor_msg_t yaw_motor_msg;
extern connect_t connect_data;
extern robot_status_t robot_status;
extern judge_manegement_t judge_manegement;
extern shoot_data_t shoot_data;
extern ground_robot_position_t robot_position;
extern power_heat_data_t power_heat;
extern projectile_allowance_t bullets_remaining;
extern TaskHandle_t chassis_task_Handler;
extern robot_t Infantry;
extern hurt_data_t robot_hurt;

void get_rc_analoy_key_value(rc_analog_key_t *analoy_key, RC_ctrl_t *rc_ctrl_data)
{
	if(analoy_key == NULL || rc_ctrl_data == NULL)
	{
		return;
	}
	if(rc_ctrl_data->rc.ch3>ANALOY_KEY_THRE_VALUE)  analoy_key->key_up = 1;//上
	else analoy_key->key_up = 0;
	
	if(rc_ctrl_data->rc.ch3<(-ANALOY_KEY_THRE_VALUE))  analoy_key->key_down = 1;//下
	else analoy_key->key_down = 0;
	
	if(rc_ctrl_data->rc.ch2>ANALOY_KEY_THRE_VALUE)  analoy_key->key_back = 1;//右
	else analoy_key->key_back = 0;
	
	if(rc_ctrl_data->rc.ch2<(-ANALOY_KEY_THRE_VALUE))  analoy_key->key_ok = 1;//左
	else analoy_key->key_ok = 0;
}
void rc_analoy_key_init(void)
{
	rc_analog_key.key_back = 0;
	rc_analog_key.key_down = 0;
	rc_analog_key.key_ok = 0;
	rc_analog_key.key_up = 0;
	rc_analog_key.rc_ctrl_data_point = get_rc_data_point();
}
/****************************OLED调试所用变量******************************/
uint8_t key_num = 0;
#define     oled_key_release    0
#define     oled_key_up         1
#define     oled_key_down       2
#define     oled_key_left       3
#define     oled_key_right      4
#define     oled_key_center     5

void Motor_message()
{
	LED_Fill(0x00);
	while(1)
	{
		LED_P6x8Str(28,0,(uint8_t *)"Motor_message");
		LED_P6x8Str(3,1,(uint8_t *)"CM_1:");
		LED_PrintValueI(40,1,cm1_msg.encoder.filter_rate);
		LED_P6x8Str(3,2,(uint8_t *)"CM_2:");
		LED_PrintValueI(40,2,cm2_msg.encoder.filter_rate);
		LED_P6x8Str(3,3,(uint8_t *)"CM_3:");
		LED_PrintValueI(40,3,cm3_msg.encoder.filter_rate);
		LED_P6x8Str(3,4,(uint8_t *)"CM_4:");
		LED_PrintValueI(40,4,cm4_msg.encoder.filter_rate);

		LED_P6x8Str(3,5,(uint8_t *)"Yaw:");
		LED_PrintValueI(40,5,yaw_motor_msg.encoder.raw_value);
		
		if(key_scanf()==oled_key_left)
		{
			while(key_scanf());	//等待按键松开
			break;
		}
		osDelay(100);
	}
}
void Robot_message(void)
{
	LED_Fill(0x00);
	while(1)
	{
		LED_P6x8Str(28,0,(uint8_t *)"Robot_message");
		LED_P6x8Str(3,1,(uint8_t *)"ID:");
		LED_PrintValueC(54,1,robot_status.robot_id );
		LED_P6x8Str(3,2,(uint8_t *)"HP:");
		LED_PrintValueI(54,2,robot_status.current_HP );
		LED_P6x8Str(3,3,(uint8_t *)"Level:");
		LED_PrintValueC(54,3,robot_status.robot_level );
		LED_P6x8Str(3,4,(uint8_t *)"Pre_HP:");
		LED_PrintValueI(54,4, Infantry.Pre_HP );
		LED_P6x8Str(3,5,(uint8_t *)"Hurt_tp:");
		LED_PrintValueC(54,5, robot_hurt.HP_deduction_reason );
		LED_P6x8Str(3,6,(uint8_t *)"Hurt_ar:");
		LED_PrintValueC(54,6, robot_hurt.armor_id );
		if(key_scanf()==oled_key_left)
		{
			while(key_scanf());	//等待按键松开
			break;
		}
		osDelay(100);
	}
}
void shoot_message(void)
{
	LED_Fill(0x00);
	while(1)
	{
		LED_P6x8Str(28,0,(uint8_t *)"shoot_message");
		LED_P6x8Str(3,1,(uint8_t *)"type:");
		LED_PrintValueI(40,1,shoot_data.bullet_type);
		LED_P6x8Str(3,2,(uint8_t *)"freq:");
		LED_PrintValueI(50,2,shoot_data.launching_frequency);
		LED_P6x8Str(3,3,(uint8_t *)"speed:");
		LED_PrintValueF(50,3,shoot_data.initial_speed,2);
		LED_P6x8Str(3,4,(uint8_t *)"heat17:");	// 17mm 枪口热量
//		LED_PrintValueI(50,4,power_heat.shooter_id1_17mm_cooling_heat);
//		LED_P6x8Str(3,5,(uint8_t *)"V_lim:");
//		LED_PrintValueI(50,5,robot_status.shooter_id1_17mm_speed_limit);
//		LED_P6x8Str(3,6,(uint8_t *)"H_lim:");
//		LED_PrintValueI(50,6,robot_status.shooter_id1_17mm_cooling_limit);
//		LED_P6x8Str(3,7,(uint8_t *)"count:");	
//		LED_PrintValueI(50,7,bullets_remaining.shootnum);	//统计发弹量,每触发一次则认为发射了一颗
		if(key_scanf()==oled_key_left)
		{
			while(key_scanf());	//等待按键松开
			break;
		}
		osDelay(100);
	}
}
void power_message(void)
{
	LED_Fill(0x00);
	while(1)
	{
		LED_P6x8Str(28,0,(uint8_t *)"power_message:");
		LED_P6x8Str(3,1,(uint8_t *)"volt:");	// 底盘输出电压 单位 毫伏
		LED_PrintValueI(40,1,power_heat.chassis_voltage);
		LED_P6x8Str(3,2,(uint8_t *)"curre:");	// 底盘输出电流 单位 毫安
		LED_PrintValueI(50,2,power_heat.chassis_current);
		LED_P6x8Str(3,3,(uint8_t *)"power:");	// 底盘输出功率 单位 W 瓦 
		LED_PrintValueF(50,3,power_heat.chassis_power,2);
		LED_P6x8Str(3,4,(uint8_t *)"buff:");	// 底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
		LED_PrintValueI(40,4,power_heat.buffer_energy);
		/*
		// 底盘输出电压 单位 毫伏
		// 底盘输出电流 单位 毫安
		// 底盘输出功率 单位 W 瓦 
		// 底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
		// 17mm 枪口热量
		// 42mm 枪口热量
		*/
		LED_P6x8Str(3,5,(uint8_t *)"C_Lim:");	// 底盘功率限制
		LED_PrintValueI(40,5,robot_status.chassis_power_limit);
		if(key_scanf()==oled_key_left)
		{
			while(key_scanf());	//等待按键松开
			break;
		}
		osDelay(100);  
	}
}
void GYRO_ch23_message()
{
	LED_Fill(0x00);
	while(1)
	{
		
		LED_P6x8Str(10,1,(uint8_t *)"GYRO_set:");
		LED_PrintValueF(65,1,connect_data.can2_rc_ctrl.gyro.yaw_set,2);

		LED_P6x8Str(10,2,(uint8_t *)"GYRO_fdb");
		LED_PrintValueF(65,2,connect_data.can2_rc_ctrl.gyro.yaw_fdb,2);

		LED_P6x8Str(10,3,(uint8_t *)"ch2");
		LED_PrintValueI(65,3,connect_data.can2_rc_ctrl.rc.ch2);

		LED_P6x8Str(10,4,(uint8_t *)"ch3");
		LED_PrintValueI(65,4,connect_data.can2_rc_ctrl.rc.ch3);
		if(key_scanf()==oled_key_left)
		{
			while(key_scanf());	//等待按键松开
			break;
		}
		osDelay(100);
	}
}

void Motor_test(void)
{
	int16_t test_motor1_duty=0, test_motor2_duty=0, test_motor3_duty=0, test_motor4_duty=0;
	int choose_num=1;
	LED_Fill(0x00);
	osThreadSuspend(chassis_task_Handler);//任务挂起
	while(1)
	{
		LED_P6x8Char(3, choose_num, '*');
		LED_P6x8Str(28,0,(uint8_t *)"power_message:");
		LED_P6x8Str(10,1,(uint8_t *)"Motor1");
		LED_P6x8Str(10,2,(uint8_t *)"Motor2:");
		LED_P6x8Str(10,3,(uint8_t *)"Motor3:");
		LED_P6x8Str(10,4,(uint8_t *)"Motor4:");
		LED_P6x8Str(10,5,(uint8_t *)"Null:");
		LED_P6x8Str(10,6,(uint8_t *)"Null:");
		LED_P6x8Str(10,7,(uint8_t *)"Return:");
		while(!key_scanf());
		delay_ms(20);
		key_num = key_scanf();
		while(key_scanf());
		delay_ms(100);
		if(key_num == oled_key_up)
		{
			choose_num++;
			if(choose_num==8)
				choose_num=1;
		}
		else if (key_num == oled_key_up)
		{
			choose_num--;
			if(choose_num==0)
				choose_num=7;
		}
		else if(key_num == oled_key_left)
		{
			switch (choose_num)
			{
			case 1:
				test_motor1_duty-=100;break;
			case 2:
				test_motor2_duty-=100;break;
			case 3:
				test_motor3_duty-=100;break;
			case 4:
				test_motor4_duty-=100;break;
			default:
				break;
			}
		}
		else if(key_num == oled_key_right)
		{
			switch (choose_num)
			{
			case 1:
				test_motor1_duty+=100;break;
			case 2:
				test_motor2_duty+=100;break;
			case 3:
				test_motor3_duty+=100;break;
			case 4:
				test_motor4_duty+=100;break;
			case 5:
				break;
			case 6:
				break;
			default:
				break;
			}
		}
		else if(key_scanf()==oled_key_center && choose_num==7)
		{
			while(key_scanf());	//等待按键松开
			osThreadResume(chassis_task_Handler);//任务恢复
			LED_Fill(0x00);
			break;
		}
		set_chassis_behaviour(test_motor1_duty,\
							  test_motor2_duty,\
							  test_motor3_duty,\
							  test_motor4_duty);
		osDelay(10);
	}
}
char task_ch[400] = {0};
/**
  * @brief         主菜单选择指 示，控制星号即指示标的 跳动  人机交互总函数   
  * @author         
  * @param[in]      
  * @retval		   返回空
  * @note
  */

void GUI_interaction(void)                 
{
    int choose_num = 1;
	while(1)
	{
		LED_Fill(0x00);
		LED_P6x8Str(52,0,(uint8_t *)"MENU");
    	LED_P6x8Str(10,1,(uint8_t *)"Robot_message");
		LED_P6x8Str(10,2,(uint8_t *)"Motor_message");
    	LED_P6x8Str(10,3,(uint8_t *)"shoot_message");
		LED_P6x8Str(10,4,(uint8_t *)"power_message");
		LED_P6x8Str(10,5,(uint8_t *)"Motor_test_Null");
		LED_P6x8Str(10,6,(uint8_t *)"GYRO_ch23_message");
		
	  	LED_P6x8Char(3, choose_num, '*');
		// memset(task_ch,0,400);
		// vTaskGetRunTimeStats(task_ch);

		while(!key_scanf());
		delay_ms(20);
		key_num = key_scanf();
		while(key_scanf());
		delay_ms(20);
		if(key_num == oled_key_up)
		{
			choose_num--;
			if(choose_num==0)
				choose_num=6;
		}
		else if(key_num == oled_key_down)
		{
			choose_num++;
			if(choose_num==7)
				choose_num=1;
		}
		else if(key_num == oled_key_center)
		{
			switch(choose_num)
			{
				case 1:Robot_message();break;
				case 2:Motor_message();break;
				case 3:shoot_message();break;
				case 4:power_message();break;
				//case 5:Motor_test();break;
				case 6:GYRO_ch23_message();break;
				// case 6:Robot_message();break;
				// case 7:Robot_message();break;
				default:break;
			}
		}
		vTaskDelay(100);
	}
}

void GUI_task(void *argument)
{
	vTaskDelay(GUI_TASK_INIT_TIME);
	LED_Init(); // OLED屏幕初始化
	rc_analoy_key_init();
	while(1)
	{
		GUI_interaction();
	}
}

