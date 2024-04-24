/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia 
 * @Teammate��
 * @Version: V3.0
 * @Date:2021.4.13
 * @Description:  CAN2Ӧ�ã���������ϵͳ��Ϣ�Ĵ��䡢AC�����Ϣ�Ľ�����
 * @Note:       
 * @Others: 
**/
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "connect_task.h"
#include "can.h"
#include "can1_app.h"
#include "can2_app.h"
#include "judge.h"
#include "stm32f4xx_hal_can.h"
#include "chassis_task.h"

extern CAN_RxHeaderTypeDef can2_rx_header;
extern uint8_t can2_rx_data[CAN_RX_BUF_SIZE];
extern CAN_TxHeaderTypeDef can2_tx_header;
extern uint8_t can2_tx_data[CAN_TX_BUF_SIZE];
extern CAN_HandleTypeDef hcan2;

extern chassis_control_data_t chassis_control_data;
motor_msg_t yaw_motor_msg = {0};
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note    Ĭ�ϵ����can����Ƶ��Ϊ1KHZ       
  */ 

static void yaw_motor_msg_process(motor_msg_t *m, uint8_t aData[])
{
	int16_t i;
	m->encoder.filter_rate_sum = 0;//��������
	m->encoder.last_raw_value = m->encoder.raw_value; 
	if(m->encoder.start_flag==0)//�ϵ�ɼ�ԭʼ�Ƕ�
	{
		m->encoder.ecd_bias = (aData[0]<<8)|aData[1];//��ʼλ��
		m->encoder.last_raw_value = (aData[0]<<8)|aData[1];
		m->encoder.raw_value = m->encoder.last_raw_value;
		m->encoder.start_flag = 1;
	}
	else
	{
		m->encoder.raw_value = (aData[0]<<8)|aData[1];
	}
	
	m->encoder.diff = m->encoder.raw_value - m->encoder.last_raw_value;
	if(m->encoder.diff < -6000)//���α������ķ���ֵ���̫�󣬱�ʾȦ�������˸ı�                         
	{                          //7500���ݿ������ڿɵ�������֤һ������������ ת�ӻ�е�Ƕȱ仯С��8191-7500=691���� 
		m->encoder.round_cnt ++;
		m->encoder.ecd_raw_rate = m->encoder.diff + 8192;
	}
	else if(m->encoder.diff > 6000)
	{
		m->encoder.round_cnt --;
		m->encoder.ecd_raw_rate = m->encoder.diff - 8192;
	}
	else
	{
		m->encoder.ecd_raw_rate = m->encoder.diff;
	}
	//����õ��Ƕ�ֵ����Χ���������
	m->encoder.ecd_angle = (float)(m->encoder.raw_value - m->encoder.ecd_bias)*360/8192  \
								   + m->encoder.round_cnt * 360;
	
	m->encoder.rate_buf[m->encoder.buf_count++] = m->encoder.ecd_raw_rate;
	if(m->encoder.buf_count == RATE_BUF_SIZE)
	{
		m->encoder.buf_count = 0;
	}
	//�����ٶ�ƽ��ֵ
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		m->encoder.filter_rate_sum += m->encoder.rate_buf[i];
	}
	m->encoder.filter_rate = (int32_t)(m->encoder.filter_rate_sum/RATE_BUF_SIZE);	
	/*---------------------�Ǳ���������------------------------*/
	m->speed_rpm = (uint16_t)(aData[2] << 8 | aData[3]);     
	m->given_current = (uint16_t)(aData[4] << 8 | aData[5]); 
	m->temperate = aData[6];          
}

extern chassis_pid_t chassis_pid;
void can2_message_progress(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{
	if(pHeader == NULL || aData == NULL )
	{
		return;
	}
	switch(pHeader->StdId)
	{
		//get rc control 
		case CAN2_CONNECT_RC_CTRL_STD_ID:
		{
			connect_rc_ctrl_process(&connect_data ,aData);
		}break;

		//get gimbal control 
		case CAN2_YAW_MOTOR_STD_ID:
		{
			yaw_motor_msg_process(&yaw_motor_msg ,aData); 
		}break;
		case CAN2_CONNECT_CM_GYRO_STD_ID:
		{
			connect_gyro_data_process(&connect_data ,aData);
		}
		break;
		case CAN2_CONNECT_UIFLAG_STD_ID :
		{
			get_uiflag_to_chassis(&connect_data ,aData);
		}break;
		default: break;
	}
}


extern robot_status_t robot_status;
extern power_heat_data_t power_heat;
extern shoot_data_t shoot_data; 
extern hurt_data_t robot_hurt;

void send_shoot_17mm_data(robot_status_t *robot_status)  
{
	can2_tx_header.StdId = CAN2_SHOOT_17mm_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x08;

    can2_tx_data[0] = (uint8_t)(robot_status->shooter_barrel_cooling_value>>8);
    can2_tx_data[1] = (uint8_t)(robot_status->shooter_barrel_cooling_value);
    can2_tx_data[2] = (uint8_t)(robot_status->shooter_barrel_heat_limit>>8);
    can2_tx_data[3] = (uint8_t)(robot_status->shooter_barrel_heat_limit);
		can2_tx_data[4] = (uint8_t)(30>>8);//
    can2_tx_data[5] = (uint8_t)(30);
    can2_tx_data[6] = (uint8_t)(power_heat.shooter_17mm_1_barrel_heat>>8);
    can2_tx_data[7] = (uint8_t)(power_heat.shooter_17mm_1_barrel_heat);//��Ϊǹ������
	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0);
}
void send_shoot_judge_data(void)  
{
	can2_tx_header.StdId = CAN2_SHOOT_JUDGE_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x08;

    can2_tx_data[0] = (uint8_t)(((uint16_t)(shoot_data.initial_speed))>>8);     //������λС��
    can2_tx_data[1] = (uint8_t)((uint16_t)(shoot_data.initial_speed));
    can2_tx_data[2] = (uint8_t)(robot_hurt.HP_deduction_reason);
    can2_tx_data[3] = (uint8_t)(robot_status.power_management_shooter_output);
	can2_tx_data[4] = (uint8_t)(robot_status.robot_id);
    can2_tx_data[5] = (uint8_t)(chassis_control_data.forward_back_set>>8) ;
    can2_tx_data[6] = (uint8_t)(chassis_control_data.forward_back_set) ;
    can2_tx_data[7] = 0;
	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0);
}

motor_msg_t *get_yaw_motor_msg_point(void)
{
	return &yaw_motor_msg;
}

