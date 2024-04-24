#ifndef TEST_TASK_H
#define TEST_TASK_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "judge.h"

#define TRANSMIT_SHOW_DATA_TIME       (35)     //上限传速30HZ


#define    LEN_HEADER    5          //帧头长
#define    LEN_CMDID     2          //命令码长度
#define		 LEN_DATAFH		 6				//数据帧头长度
#define    LEN_TAIL      2	      //帧尾CRC16

#define SEND_MAX_LEN     200


/****************************开始标志*********************/
#define UI_SOF 0xA5
/****************************CMD_ID数据********************/
#define UI_CMD_Robo_Exchange 0x0301    
/****************************内容ID数据********************/
#define UI_Data_ID_Del 0x100 
#define UI_Data_ID_Draw1 0x101
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104
#define UI_Data_ID_DrawChar 0x110
/****************************红方机器人ID********************/
#define UI_Data_RobotID_RHero 1         
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************蓝方机器人ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************红方操作手ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************蓝方操作手ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A
/***************************删除操作***************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
/***************************图形配置参数__图形操作********************/
#define UI_Graph_ADD 1
#define UI_Graph_Change 2
#define UI_Graph_Del 3
/***************************图形配置参数__图形类型********************/
#define UI_Graph_Line 0         //直线
#define UI_Graph_Rectangle 1    //矩形
#define UI_Graph_Circle 2       //整圆
#define UI_Graph_Ellipse 3      //椭圆
#define UI_Graph_Arc 4          //圆弧
#define UI_Graph_Float 5        //浮点型
#define UI_Graph_Int 6          //整形
#define UI_Graph_Char 7         //字符型
/***************************图形配置参数__图形颜色********************/
#define UI_Color_Main 0         //红蓝主色
#define UI_Color_Yellow 1
#define UI_Color_Green 2
#define UI_Color_Orange 3
#define UI_Color_Purplish_red 4 //紫红色
#define UI_Color_Pink 5
#define UI_Color_Cyan 6         //青色
#define UI_Color_Black 7
#define UI_Color_White 8

/* 
	交互数据，包括一个统一的数据段头结构，
	包含了内容 ID，发送者以及接受者的 ID 和内容数据段，
	整个交互数据的包总共长最大为 128 个字节，
	减去 frame_header,cmd_id,frame_tail 以及数据段头结构的 6 个字节，
	故而发送的内容数据段最大为 113。
	整个交互数据 0x0301 的包上行频率为 10Hz。
	
机器人 ID：
	1，英雄(红)；
	2，工程(红)；
	3/4/5，步兵(红)；
	6，空中(红)；
	7，哨兵(红)；
	
	101，英雄(蓝)；
	102，工程(蓝)；
	103/104/105，步兵(蓝)；
	106，空中(蓝)；
	107，哨兵(蓝)。 
客户端 ID： 
	0x0101 英雄操作手客户端(红) ；
	0x0102 工程操作手客户端(红 )；
	0x0103/0x0104/0x0105，步兵操作手客户端(红)；
	0x0106，空中操作手客户端(红)； 

	0x0165，英雄操作手客户端(蓝)；
	0x0166，工程操作手客户端(蓝)；
	0x0167/0x0168/0x0169，操作手客户端步兵(蓝)；
	0x016A，空中操作手客户端(蓝)。 
*/
/* 交互数据接收信息：0x0301  */
typedef  struct 
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 

/* 
	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180
	发送频率：上限 10Hz

	字节偏移量 	大小 	说明 				备注 
	0 			2 		数据的内容 ID 		0xD180 
	2 			2 		发送者的 ID 		需要校验发送者机器人的 ID 正确性 
	4 			2 		客户端的 ID 		只能为发送者机器人对应的客户端 
	6 			4 		自定义浮点数据 1 	 
	10 			4 		自定义浮点数据 2 	 
	14 			4 		自定义浮点数据 3 	 
	18 			1 		自定义 8 位数据 4 	 

*/

//typedef  struct
//{
//	uint8_t graphic_name[3];     //图形名（在删除，修改等操作中，作为客户端的索引）    
//	//图像配置  （协议上有可参考）
//	uint32_t operate_tpye:3;     //图形操作
//	uint32_t graphic_tpye:3;     //图形类型
//	uint32_t layer:4;            //图层数
//	uint32_t color:4;            //颜色
//    
//	uint32_t start_angle:9;      //起始角度
//	uint32_t end_angle:9;        //终止角度

//	uint32_t width:10;           //线宽
//	uint32_t start_x:11;         //起点x坐标
//	uint32_t start_y:11;         //起点y坐标

//	uint32_t radius:10;          //字体大小或者半径
//	uint32_t end_x:11;           //终点x坐标
//	uint32_t end_y:11;           //终点y坐标

//}graphic_data_struct_t; //图形数据


//typedef struct
//{ 
//   uint8_t graphic_name[3]; 		//图形名（在删除，修改等操作中，作为客户端的索引）  
//   uint32_t operate_tpye:3; 	
//   uint32_t graphic_tpye:3; 
//   uint32_t layer:4; 
//   uint32_t color:4; 
//   uint32_t start_angle:9;
//   uint32_t end_angle:9;
//   uint32_t width:10; 
//   uint32_t start_x:11; 
//   uint32_t start_y:11;
//   float graph_Float;              //浮点数据
//} float_data_struct_t;

//typedef struct
//{ 
//   uint8_t graphic_name[3]; 		//图形名（在删除，修改等操作中，作为客户端的索引）  
//   uint32_t operate_tpye:3; 	
//   uint32_t graphic_tpye:3; 
//   uint32_t layer:4; 
//   uint32_t color:4; 
//   uint32_t start_angle:9;
//   uint32_t width:10; 
//   uint32_t start_x:11; 
//   uint32_t start_y:11;
//   int graph_int;              //整形数据
//} int_data_struct_t;

typedef struct
{
   graphic_data_struct_t graphic_data_struct;
   uint8_t show_Data[30];
} string_data_struct_t_;                  //字符串数据



typedef struct
{
   graphic_data_struct_t    grapic_data_struct[7];    //可变成数组，传多个数据

}ext_client_custom_graphic_single_t;

typedef struct
{
	uint8_t SOF; 	//数据帧起始字节，固定值为 0xA5
	uint16_t data_length;
	uint8_t seq;	//包序号
	uint8_t CRC8;	//帧头 CRC8 校验
} frame_header_t;
//帧头  命令码   数据段头结构  数据段   帧尾
//上传客户端
typedef  struct
{
	frame_header_t   						txFrameHeader;  //帧头
	uint16_t		 						CmdID;          //命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	ext_client_custom_graphic_single_t      clientData;     //客户端数据段
	uint16_t		 						FrameTail;      //帧尾
}ext_Send_User_Data_t;




#endif



