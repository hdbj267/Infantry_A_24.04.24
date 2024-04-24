#ifndef __OLED_H__
#define __OLED_H__

#include "main.h"

#include "stm32f4xx_hal.h"
/*****************************������������*****************************/


typedef char *pchar;
typedef unsigned char *puchar;
typedef int  *pint;
typedef unsigned int *puint;

typedef char int8;
typedef int int16;
typedef long int int32;

typedef unsigned char uint8;
typedef unsigned int uint16;
typedef unsigned long int uint32;
/*****************************************
���� �� ��:��ʼ�����Ŷ���
��˵ �� ��:ֱ���������IO�Ϳ�����*/
/*----RST-----B10-----*/
/*----DC-----B9-----*/
/*----SCLK-----B3-----*/
/*----MOSI-----A7-----*/
#define LED_SCLH       PBout(3)=1
#define LED_SCLL       PBout(3)=0

#define LED_SDAH       PAout(7)=1
#define LED_SDAL       PAout(7)=0

#define LED_RSTH       PBout(10)=1
#define LED_RSTL       PBout(10)=0

#define LED_DCH        PBout(9)=1
#define LED_DCL        PBout(9)=0
//����f429
//#define LED_SCLH       PBout(10)=1
//#define LED_SCLL       PBout(10)=0

//#define LED_SDAH       PBout(9)=1
//#define LED_SDAL       PBout(9)=0

//#define LED_RSTH       PBout(3)=1
//#define LED_RSTL       PBout(3)=0

//#define LED_DCH        PAout(7)=1
//#define LED_DCL        PAout(7)=0

//#define OLED_SCL_PIN	E26
//#define OLED_SDA_PIN	E27
//#define OLED_RST_PIN	E28
//#define OLED_DC_PIN	A4
//#define CS_PIN PTC14

//#define INIT 0
//#define LED_SCL_Init  gpio_init(SCL_PIN,GPO,0) // ʱ�ӳ�ʼ������
//#define LED_SDA_Init  gpio_init(SDA_PIN,GPO,0)//���ݿ�D1
//#define LED_RST_Init  gpio_init(RST_PIN,GPO,0)//��λ���ܵ�ƽ
//#define LED_DC_Init   gpio_init(DC_PIN,GPO,0)//ƫ�ó���
//#define LED_CS_Init   gpio_init(CS_PIN,GPO,INIT)//ƫ�ó���


//#define LED_SCLH       gpio_set (SCL_PIN	, 1)
//#define LED_SCLL       gpio_set (SCL_PIN	, 0)

//#define LED_SDAH       gpio_set (SDA_PIN	, 1)
//#define LED_SDAL       gpio_set (SDA_PIN	, 0)

//#define LED_RSTH       gpio_set (RST_PIN	, 1)
//#define LED_RSTL       gpio_set (RST_PIN	, 0)

//#define LED_DCH        gpio_set (DC_PIN	, 1)
//#define LED_DCL        gpio_set (DC_PIN	, 0)


/************************************************/

void  LEDPIN_Init(void);   //LED�������ų�ʼ��
void LED_Init(void);
void LED_CLS(void);
void LED_Set_Pos(uint8 x, uint8 y);//�������꺯��
void LED_WrDat(uint8 data);   //д���ݺ���
void LED_P6x8Char(uint8 x,uint8 y,uint8 ch);
void LED_P6x8Str(uint8 x,uint8 y,uint8 ch[]);
void LED_P8x16Str(uint8 x,uint8 y,uint8 ch[]);
void LED_P14x16Str(uint8 x,uint8 y,uint8 ch[]);
void LED_PXx16MixStr(uint8 x, uint8 y, uint8 ch[]);
void LED_PrintBMP(uint8 x0,uint8 y0,uint8 x1,uint8 y1,uint8 bmp[]); 
void LED_Fill(uint8 dat);
void LED_PrintValueC(uint8 x, uint8 y,int8 data);
void LED_PrintValueI(uint8 x, uint8 y, int16 data);
void LED_PrintValueI1(uint8 x, uint8 y, int16 data);
void LED_PrintValueI2(uint8 x, uint8 y, int16 data);
void LED_PrintValueF(uint8 x, uint8 y, float data, uint8 num);
void LED_PrintValueFP(uint8 x, uint8 y, uint16 data, uint8 num);
void LED_PrintEdge(void);
void LED_Cursor(uint8 cursor_column, uint8 cursor_row);
void LED_PrintLine(void);
 
 
void OLED_Set_Pos(unsigned char x, unsigned char y) ;//�ⲿ��ͼ����ʾ��
 
void img_show(void);
void team_name(void);
void look_look(void);

void oled_init(void);

#endif



