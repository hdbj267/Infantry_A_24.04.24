#ifndef GUI_TASK_H
#define GUI_TASK_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "remote_app.h"
#include "chassis_task.h"

#define GUI_TASK_INIT_TIME       (10)
#define ANALOY_KEY_THRE_VALUE    (50) 

typedef struct
{
	int16_t key_up;
	int16_t key_down;
	int16_t key_ok;
	int16_t key_back;
	void *rc_ctrl_data_point;
}rc_analog_key_t;

extern rc_analog_key_t rc_analog_key;
void rc_analoy_key_init(void);
void get_rc_analoy_key_value(rc_analog_key_t *rc_analoy_key, RC_ctrl_t *rc_ctrl_data);
void GUI_task(void *argument);

#endif
