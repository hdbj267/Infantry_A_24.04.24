#include "printf.h"
#include "delay.h"

#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOSʹ��	  
#endif
  
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART6->SR&0X40)==0);//ѭ������,ֱ���������   
	USART6->DR = (u8) ch;      
	return ch;
}
#endif 
