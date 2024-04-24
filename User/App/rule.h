#ifndef _RULE_H
#define _RULE_H

#include "judge.h"
#include "struct_typedef.h"

typedef struct
{
    uint16_t Pre_HP;                  //��һѪ��
    uint16_t hurt_HP;                 //��Ѫ��
    uint16_t allow_power;             //���ư����ֵ����
	uint8_t Armor_Huar_flag;          //װ���˺���־
    uint8_t Off_Line_flag;            //���߱�־
	uint8_t Excess_Speed_shoot_flag;  //�����ٱ�־
    uint8_t Excess_Heat_flag;         //��������־
    uint8_t Over_Power_flag;          //�����ʱ�־
	uint8_t Armor_BUMP_flag;          //װ��ײ����־

    float over_power_P;               //�����ٷֱ�
    uint16_t over_power_part;         //��������

}robot_t;


extern robot_status_t robot_status;
extern power_heat_data_t power_heat;
extern hurt_data_t robot_hurt; 


void robot_hurt_analysis(void);
void chassis_power_limit(void);
void hurt_init(void);
#endif
