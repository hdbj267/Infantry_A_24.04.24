
#include "test_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "judge.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "usart.h"
#include "stdarg.h"
#include "judge.h"
#include "connect_task.h"

#define operator_ID Robot_ID_Red_Infantry4


uint8_t UI_Seq=0;	//包序号
extern  robot_status_t robot_status;


/**
  * @brief  通过自己的ID，返回对应操作手客户端ID
  * @param  void
  * @retval 
  * @attention  
  */
//uint16_t	operator_ID;
//int Operator_ID(void)
//{
//	if(robot_status.robot_id < 10)        //红方
//	{
//		operator_ID = robot_status.robot_id + 0x0100 ;
//	}
//	else if (robot_status.robot_id > 10)  //蓝方
//	{
//		operator_ID = robot_status.robot_id + 0x0064 ;
//	}
//	return operator_ID;
//}

void UI_graphic(graphic_data_struct_t* graphic_data_struct, uint8_t name[3],uint32_t operate,uint32_t graphic ,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y )
{	
	for(int i=0;i<3;i++)
		graphic_data_struct->graphic_name[i]=name[i];
	
	graphic_data_struct->operate_tpye=operate;
	graphic_data_struct->graphic_tpye=graphic;
	graphic_data_struct->layer=layer;
	graphic_data_struct->color=color;

	graphic_data_struct->width=width;
	graphic_data_struct->start_x=start_x;
	graphic_data_struct->start_y=start_y;
	graphic_data_struct->end_x=end_x;
	graphic_data_struct->end_y=end_y;
}

void UI_Line(interaction_figure_t* graphic_data_struct, uint8_t name[3],       \
	uint32_t operate,uint32_t layer,uint32_t color,           \
	uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y )
{	
	for(int i=0;i<3;i++)
	graphic_data_struct->figure_name[i]=name[i];
	
	graphic_data_struct->operate_tpye=operate;
	graphic_data_struct->figure_tpye=UI_Graph_Line;
	graphic_data_struct->layer=layer;
	graphic_data_struct->color=color;

	graphic_data_struct->details_a = 0;
	graphic_data_struct->details_b = 0;
	
	graphic_data_struct->width=width;
	graphic_data_struct->start_x=start_x;
	graphic_data_struct->start_y=start_y;
	
	graphic_data_struct->details_c=0;
	graphic_data_struct->details_d=end_x;
	graphic_data_struct->details_e=end_y;
}

void UI_Rectangle(interaction_figure_t* graphic_data_struct, uint8_t name[3],            \
	uint32_t operate,uint32_t graphic ,uint32_t layer,uint32_t color,                       \
	uint32_t width,uint32_t start_x,uint32_t start_y,       \
	uint32_t end_x,uint32_t end_y)
{	

	for(int i=0;i<3;i++)
	graphic_data_struct->figure_name[i]=name[i];
	
	graphic_data_struct->operate_tpye=operate;
	graphic_data_struct->figure_tpye=UI_Graph_Rectangle;
	graphic_data_struct->layer=layer;
	graphic_data_struct->color=color;

	graphic_data_struct->details_a = 0;
	graphic_data_struct->details_b = 0;
	


	graphic_data_struct->width=width;
	graphic_data_struct->start_x=start_x;
	graphic_data_struct->start_y=start_y;
	
	
	graphic_data_struct->details_c=0;
	graphic_data_struct->details_d=end_x;
	graphic_data_struct->details_e=end_y;
}


void UI_Circle(interaction_figure_t* graphic_data_struct, uint8_t name[3],	\
	uint32_t operate,uint32_t graphic ,uint32_t layer,uint32_t color,	\
	uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t radius )
{	
	for(int i=0;i<3;i++)
	graphic_data_struct->figure_name[i]=name[i];
	
	graphic_data_struct->operate_tpye=operate;
	graphic_data_struct->figure_tpye=UI_Graph_Circle;
	graphic_data_struct->layer=layer;
	graphic_data_struct->color=color;

	graphic_data_struct->details_a = 0;
	graphic_data_struct->details_b = 0;
	
	graphic_data_struct->width=width;
	graphic_data_struct->start_x=start_x;
	graphic_data_struct->start_y=start_y;
	
	graphic_data_struct->details_c=radius;
	graphic_data_struct->details_d=0;
	graphic_data_struct->details_e=0;
}
void UI_Ellipse(interaction_figure_t* graphic_data_struct, uint8_t name[3],            \
	uint32_t operate,uint32_t graphic ,uint32_t layer,uint32_t color,                       \
	uint32_t width,uint32_t start_x,uint32_t start_y,       \
	uint32_t XHalfAxis,uint32_t YHalfAxis)
{	
	for(int i=0;i<3;i++)
	graphic_data_struct->figure_name[i]=name[i];
	
	graphic_data_struct->operate_tpye=operate;
	graphic_data_struct->figure_tpye=UI_Graph_Ellipse;
	graphic_data_struct->layer=layer;
	graphic_data_struct->color=color;

	graphic_data_struct->details_a = 0;
	graphic_data_struct->details_b = 0;
	

	graphic_data_struct->width=width;
	graphic_data_struct->start_x=start_x;
	graphic_data_struct->start_y=start_y;
	
	
	graphic_data_struct->details_c=0;
	graphic_data_struct->details_d=XHalfAxis;
	graphic_data_struct->details_e=YHalfAxis;
}

void UI_Arc(interaction_figure_t* graphic_data_struct, uint8_t name[3],	\
	uint32_t operate,uint32_t layer,uint32_t color,	\
	uint32_t start_angle,uint32_t end_angle,uint32_t width,	\
	uint32_t start_x,uint32_t start_y,uint32_t XHalfAxis,uint32_t YHalfAxis )
{	
	for(int i=0;i<3;i++)
	graphic_data_struct->figure_name[i]=name[i];
	
	graphic_data_struct->operate_tpye=operate;
	graphic_data_struct->figure_tpye=UI_Graph_Arc;
	graphic_data_struct->layer=layer;
	graphic_data_struct->color=color;

	graphic_data_struct->details_a = start_angle;
	graphic_data_struct->details_b = end_angle;
	
	graphic_data_struct->width=width;
	graphic_data_struct->start_x=start_x;
	graphic_data_struct->start_y=start_y;
	
	graphic_data_struct->details_c=0;
	graphic_data_struct->details_d=XHalfAxis;
	graphic_data_struct->details_e=YHalfAxis;
}




void UI_Float(interaction_figure_t* graphic_data_struct, uint8_t name[3],	\
	uint32_t operate, uint32_t layer,uint32_t color,	\
	uint32_t size,uint32_t width,uint32_t start_x,uint32_t start_y,float Data )
{	
	uint32_t data=0;
	for(int i=0;i<3;i++)
	graphic_data_struct->figure_name[i]=name[i];
	
	graphic_data_struct->operate_tpye=operate;
	graphic_data_struct->figure_tpye=UI_Graph_Float;
	graphic_data_struct->layer=layer;
	graphic_data_struct->color=color;

	graphic_data_struct->details_a = size;
	graphic_data_struct->details_b = 0;
	
	graphic_data_struct->width=width;
	graphic_data_struct->start_x=start_x;
	graphic_data_struct->start_y=start_y;
	
	data=Data*1000;
	graphic_data_struct->details_c= data			&0x0000003f;
	graphic_data_struct->details_d= (data>>10)&0x0000007f;
	graphic_data_struct->details_e= (data>>21)&0x0000007f;
}


void UI_Int(interaction_figure_t* graphic_data_struct, uint8_t name[3],	\
	uint32_t operate, uint32_t layer,uint32_t color,	\
	uint32_t size,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t Data  )
{	
	for(int i=0;i<3;i++)
	graphic_data_struct->figure_name[i]=name[i];
	
	graphic_data_struct->operate_tpye=operate;
	graphic_data_struct->figure_tpye=UI_Graph_Int;
	graphic_data_struct->layer=layer;
	graphic_data_struct->color=color;
	
	graphic_data_struct->details_a=size;
	graphic_data_struct->details_b = 0;
	
	graphic_data_struct->width=width;
	graphic_data_struct->start_x=start_x;
	graphic_data_struct->start_y=start_y;
	
	graphic_data_struct->details_c= Data			&0x0000003f;
	graphic_data_struct->details_d= (Data>>10)&0x0000007f;
	graphic_data_struct->details_e= (Data>>21)&0x0000007f ;
}

void UI_Char(ext_client_custom_character_t* string_data_struct, uint8_t name[3],          \
	uint32_t operate, uint32_t layer,uint32_t color,     \
	uint32_t size,uint32_t length,uint32_t width,uint32_t start_x,uint32_t start_y,uint8_t* Data )
{	
	for(int i=0;i<3;i++)
	string_data_struct->grapic_data_struct.figure_name[i]=name[i];
	
	string_data_struct->grapic_data_struct.operate_tpye=operate;
	string_data_struct->grapic_data_struct.figure_tpye=UI_Graph_Char;
	string_data_struct->grapic_data_struct.layer=layer;
	string_data_struct->grapic_data_struct.color=color;
	
	string_data_struct->grapic_data_struct.details_a = size;
	string_data_struct->grapic_data_struct.details_b = length;
	
	string_data_struct->grapic_data_struct.width=width;
	string_data_struct->grapic_data_struct.start_x=start_x;
	string_data_struct->grapic_data_struct.start_y=start_y;
	
	string_data_struct->grapic_data_struct.details_c=0;
	string_data_struct->grapic_data_struct.details_d=0;
	string_data_struct->grapic_data_struct.details_e=0;
	
	for(int i=0;i<length;i++)
	string_data_struct->data[i]=Data[i];
}

/*
客户端绘制图形

输入参数
				cnt	输入几个图像，1，2，5，7
				...	graphic_data_struct_t类型的图片类型
*/
	uint8_t* client_Data;
void UI_Graphic_ReFresh(int cnt,...)
{
	ext_Send_User_Data_t	ShowData;			//客户端信息
	graphic_data_struct_t graphic_data;

	uint8_t CliendTxBuffer[SEND_MAX_LEN];
	
	va_list UI;
	va_start(UI,cnt);
	
//ShowData.txFrameHeader.SOF = UI_SOF;
//	ShowData.txFrameHeader.data_length =6+cnt*15;
//	ShowData.txFrameHeader.seq = UI_Seq;
	
	CliendTxBuffer[0]=UI_SOF;
	CliendTxBuffer[1]=(6+cnt*15)&0x00ff;
	CliendTxBuffer[2]=((6+cnt*15)>>8)&0x00ff;
	CliendTxBuffer[3]=UI_Seq;
	
//	memcpy(CliendTxBuffer, &ShowData,LEN_HEADER-1);//写入帧头数据							//该函数把uint_8当uingt_16写入
	Append_CRC8_Check_Sum(CliendTxBuffer, LEN_HEADER);//写入帧头CRC8校验码
	ShowData.CmdID = ROBOT_COMMUNICATION_ID;
	
	
	switch(cnt+0x0100)//数据段内容的ID
	{
		case UI_Data_ID_Draw1:ShowData.dataFrameHeader.data_cmd_id=UI_Data_ID_Draw1;break;
		case UI_Data_ID_Draw2:ShowData.dataFrameHeader.data_cmd_id=UI_Data_ID_Draw2;break;
		case UI_Data_ID_Draw5:ShowData.dataFrameHeader.data_cmd_id=UI_Data_ID_Draw5;break;
		case UI_Data_ID_Draw7:ShowData.dataFrameHeader.data_cmd_id=UI_Data_ID_Draw7;break;
	
	}
	
	
	ShowData.dataFrameHeader.send_ID 	 = robot_status.robot_id ;//发送者的ID
	ShowData.dataFrameHeader.receiver_ID = operator_ID;//客户端的ID，只能为发送者机器人对应的客户端
	
	memcpy(CliendTxBuffer+LEN_HEADER, &ShowData.CmdID, LEN_CMDID+LEN_DATAFH);//写入数据
	
	for(int i=0;i<cnt;i++)
	{
		graphic_data=va_arg (UI,graphic_data_struct_t);
		client_Data=(uint8_t*)&graphic_data;
		for(int t=0;t<15;t++)
		{
			CliendTxBuffer[LEN_HEADER+LEN_CMDID+LEN_DATAFH+i*15+t]=client_Data[t];
		}
	}
	
	
	Append_CRC16_Check_Sum(CliendTxBuffer,LEN_HEADER+LEN_CMDID+LEN_DATAFH+cnt*15+LEN_TAIL);//写入数据段CRC16帧尾校验码	

	while(HAL_UART_Transmit(&huart3, CliendTxBuffer, LEN_HEADER+LEN_CMDID+LEN_DATAFH+cnt*15+LEN_TAIL, 1000)!=HAL_OK);      //发数据  5+2+6+数据+2
	
	UI_Seq++;
	
	va_end(UI);
	
	

}

/*
客户端绘制字符图形

输入参数
				string_data_struct_t_	字符串数据结构体
*/
	ext_Send_User_Data_t	ShowData;
	uint8_t CliendTxBuffer[SEND_MAX_LEN];
	uint8_t*	string_data;

void UI_Char_ReFresh(ext_client_custom_character_t* string_data_struct)
{

//	ShowData.txFrameHeader.SOF = UI_SOF;
//	ShowData.txFrameHeader.data_length =6+45;
//	ShowData.txFrameHeader.seq = UI_Seq;
	
	CliendTxBuffer[0]=UI_SOF;
	CliendTxBuffer[1]=(6+45)&0x00ff;
	CliendTxBuffer[2]=((6+45)>>8)&0x00ff;
	CliendTxBuffer[3]=UI_Seq;

	Append_CRC8_Check_Sum(CliendTxBuffer, LEN_HEADER);//写入帧头CRC8校验码
	ShowData.CmdID = ROBOT_COMMUNICATION_ID;
	ShowData.dataFrameHeader.data_cmd_id=UI_Data_ID_DrawChar;
	ShowData.dataFrameHeader.send_ID 	 = robot_status.robot_id ;//发送者的ID
	ShowData.dataFrameHeader.receiver_ID = operator_ID;//客户端的ID，只能为发送者机器人对应的客户端
	
	memcpy(CliendTxBuffer+LEN_HEADER, &ShowData.CmdID, LEN_CMDID+LEN_DATAFH);//写入数据
	
	string_data=(uint8_t*)string_data_struct;
	for(int i=0;i<45;i++)
	{
		CliendTxBuffer[LEN_HEADER+LEN_CMDID+LEN_DATAFH+i]=string_data[i];
	}
	
	Append_CRC16_Check_Sum(CliendTxBuffer,LEN_HEADER+LEN_CMDID+LEN_DATAFH+45+LEN_TAIL);//写入数据段CRC16帧尾校验码	

	while(HAL_UART_Transmit(&huart3, CliendTxBuffer, LEN_HEADER+LEN_CMDID+LEN_DATAFH+45+LEN_TAIL, 1000)!=HAL_OK);      //发数据  5+2+6+数据+2
	
	UI_Seq++;
}


/*
删除图片

输入参数
				
				operate								0：空操作，1：删除图层，2：删除全部
				layer									图层
*/

void UI_Del(uint8_t operate,uint8_t layer)
{
	ext_Send_User_Data_t	ShowData;
	uint8_t CliendTxBuffer[SEND_MAX_LEN];
//	ShowData.txFrameHeader.SOF = UI_SOF;
//	ShowData.txFrameHeader.data_length =8;
//	ShowData.txFrameHeader.seq = UI_Seq;
	CliendTxBuffer[0]=UI_SOF;CliendTxBuffer[1]=(8)&0x00ff;CliendTxBuffer[2]=((8)>>8)&0x00ff;CliendTxBuffer[3]=UI_Seq;
	
	Append_CRC8_Check_Sum(CliendTxBuffer, LEN_HEADER);//写入帧头CRC8校验码
	ShowData.CmdID = ROBOT_COMMUNICATION_ID;
	
	
	ShowData.dataFrameHeader.data_cmd_id=UI_Data_ID_Del;
	ShowData.dataFrameHeader.send_ID 	 = robot_status.robot_id;//发送者的ID
	ShowData.dataFrameHeader.receiver_ID = operator_ID;//客户端的ID，只能为发送者机器人对应的客户端
	
	memcpy(CliendTxBuffer+LEN_HEADER, &ShowData.CmdID, LEN_CMDID+LEN_DATAFH);//写入数据
	
	CliendTxBuffer[LEN_HEADER+LEN_CMDID+LEN_DATAFH]=operate;
	CliendTxBuffer[LEN_HEADER+LEN_CMDID+LEN_DATAFH+1]=layer;
	
	
	
	Append_CRC16_Check_Sum(CliendTxBuffer,LEN_HEADER+LEN_CMDID+LEN_DATAFH+2+LEN_TAIL);//写入数据段CRC16帧尾校验码
	
	while(HAL_UART_Transmit(&huart3, CliendTxBuffer,LEN_HEADER+LEN_CMDID+LEN_DATAFH+2+LEN_TAIL, 1000)!=HAL_OK);      //发数据  5+2+6+数据+2
	
	UI_Seq++;
}
interaction_figure_t	G1;
interaction_figure_t	G2;
interaction_figure_t G3;
ext_client_custom_character_t	G4;
interaction_figure_t G5;
UI_Graph7_t UI_Graph7;


/* ??????????? */
uint16_t y01 = 455;
uint16_t y02 = 420;
uint16_t y03 = 280;
uint16_t y04 = 230;
uint16_t UI_PushUp_Counter = 261;
uint8_t UI_Capacitance=30;
float Capacitance_X;
float X = 1850.0f;
int a;

interaction_figure_t	G10;
interaction_figure_t	G11;
interaction_figure_t	G12;
interaction_figure_t	G13;
interaction_figure_t	G14;
interaction_figure_t	G15;
interaction_figure_t	G16;

interaction_figure_t	G20;
interaction_figure_t	G21;
interaction_figure_t	G22;
interaction_figure_t	G23;
interaction_figure_t	G24;

ext_client_custom_character_t C1;
ext_client_custom_character_t C2;
extern connect_t connect_data;
void test_task(void *argument)
{
	vTaskDelay(300);

    while(1)
    {
	//		a = Operator_ID();
		UI_PushUp_Counter++;
		//视觉标志位
		if(UI_PushUp_Counter % 101 == 0)
		{
			UI_Char(&C1,"201", UI_Graph_Add, 0, UI_Color_Green,  22, 10, 3,  100, 800, "vision OFF");
			UI_Char_ReFresh(&C1);
			continue;
		}
		if(UI_PushUp_Counter % 151 == 0)
		{
			UI_Char(&C2,"202", UI_Graph_Add, 0, UI_Color_Green,  22, 10, 3,  100, 700, "data   OFF");
			UI_Char_ReFresh(&C2);
			continue;
		}
			//中央标尺
		if(UI_PushUp_Counter % 201 == 0) //静态UI预绘制 中央标尺1
		{
			UI_Line(&G10, "001", UI_Graph_Add, 0, UI_Color_Green, 1,  840,   y01,  920,   y01); //第一行左横线
			UI_Line(&G11, "002", UI_Graph_Add, 0, UI_Color_Green, 1,  950,   y01,  970,   y01); //第一行十字横
			UI_Graphic_ReFresh(2,G10,G11);
			continue;
		}
		if(UI_PushUp_Counter % 251 == 0) //静态UI预绘制 中央标尺1
		{
			UI_Line(&G12, "003", UI_Graph_Add, 0, UI_Color_Green, 1, 1000,   y01, 1080,   y01); //第一行右横线
			UI_Line(&G13, "004", UI_Graph_Add, 0, UI_Color_Green, 1,  960,y01-10,  960,y01+10); //第一行十字竖
			UI_Graphic_ReFresh(2,G12,G13);
			continue;
		}
				if(UI_PushUp_Counter % 301 == 0) //静态UI预绘制 中央标尺1
		{
			UI_Line(&G14, "005", UI_Graph_Add, 0, UI_Color_Green, 1,  870,   y02,  930,   y02); //第二行左横线
			UI_Line(&G15, "006", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y02,  960,   y02); //第二行中心点
			UI_Graphic_ReFresh(2,G14,G15);
			continue;
		}
		if(UI_PushUp_Counter % 351 == 0) //静态UI预绘制 中央标尺1
		{
			UI_Line(&G16, "007", UI_Graph_Add, 0, UI_Color_Green, 1,  990,   y02, 1050,   y02); //第二行右横线
			UI_Graphic_ReFresh(1,G16);
			continue;
		}
		if(UI_PushUp_Counter % 401 == 0) //静态UI预绘制 小陀螺预警线
		{
			UI_Line(&G20, "101", UI_Graph_Add, 1, UI_Color_Yellow, 2,  630,   30,  780,  100);
			UI_Line(&G21, "102", UI_Graph_Add, 1, UI_Color_Yellow, 2,  780,  100,  930,  100);
			UI_Graphic_ReFresh(2,G20,G21);
			continue;
		}
		if(UI_PushUp_Counter % 451 == 0) //静态UI预绘制 小陀螺预警线
		{
			UI_Line(&G22, "103", UI_Graph_Add, 1, UI_Color_Yellow, 2,  990,  100, 1140,  100);
			UI_Line(&G23, "104", UI_Graph_Add, 1, UI_Color_Yellow, 2, 1140,  100, 1290,   30);
			UI_Graphic_ReFresh(2,G22,G23);
			continue;
		}
		if(UI_PushUp_Counter % 501 == 0) //静态UI预绘制 小陀螺预警线
		{
			UI_Line(&G24, "105", UI_Graph_Add, 1, UI_Color_Yellow, 5,  959,  100,  960,  100);
			UI_Graphic_ReFresh(1,G24);
			continue;
		}
		if(UI_PushUp_Counter % 11 == 0)
		{
			if(connect_data.vision_flag == 1)
				UI_Char(&C1,"201", UI_Graph_Change, 0, UI_Color_Green,  22, 10, 3,  100, 800, "vision ON");
			else 
				UI_Char(&C1,"201", UI_Graph_Change, 0, UI_Color_Green,  22, 10, 3,  100, 800, "vision OFF");
			UI_Char_ReFresh(&C1);			
			continue;
		}
		if(UI_PushUp_Counter % 21 == 0)
		{
			if(connect_data.recog_flag == 1) 
				UI_Char(&C2,"202", UI_Graph_Change, 0, UI_Color_Green,  22, 10, 3,  100, 700, "data   ON");
			else 
				UI_Char(&C2,"202", UI_Graph_Change, 0, UI_Color_Green,  22, 10, 3,  100, 700, "data   OFF");
			UI_Char_ReFresh(&C2);			
			continue;
		}
		
			vTaskDelay(TRANSMIT_SHOW_DATA_TIME);       //35ms???
    }
}



