/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   2.4g无线模块/nrf24l01+/slave 测试
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 iSO STM32 开发板 
  * 论坛    :http://www.chuxue123.com
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "stm32f10x.h"
#include "bsp_usart1.h"
#include "bsp_spi_nrf.h"
#include "STM32_I2C.h"
#include "DMP.h"
#include "delay.h"
#include "math.h"
#include "string.h"

u8 status;		 //用于判断接收/发送状态
u8 rxbuf[100];	 //接收缓冲
u8 i; 


#define q30  1073741824.0f
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
float Yaw,Roll,Pitch;

void StringToChar(u8 *String,u8 Chr[3][6]);
void FloatToInt(u8 *tmp,float flo);
 /**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{    
	u8 tmp1[15],tmp2[10],tmp3[5];
	
  /* 串口1初始化 */
  USART1_Config(); 
	i2cInit();
  /*SPI接口初始化*/   
  SPI_NRF_Init(); 
	DMP_Init();
  printf("\r\n 这是一个 NRF24L01 无线传输实验 \r\n");
  printf("\r\n 这是无线传输 从机端 的反馈信息\r\n");
  printf("\r\n   正在检测NRF与MCU是否正常连接。。。\r\n");

  /*检测NRF模块与MCU的连接*/
  status = NRF_Check();   		
  if(status == SUCCESS)	   
    printf("\r\n      NRF与MCU连接成功\r\n");  
  else	  
    printf("\r\n  NRF与MCU连接失败，请重新检查接线。\r\n");

  while(1)
  {  		 	
     unsigned long sensor_timestamp;
     short gyro[3], accel[3], sensors;
     unsigned char more;
     long quat[4];
			
	 dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,
                &more);	

     if (sensors & INV_WXYZ_QUAT )
	 {
	 	 q0=quat[0] / q30;
		 q1=quat[1] / q30;
		 q2=quat[2] / q30;
		 q3=quat[3] / q30;
		 
		 Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3 - 0.83; // pitch
  	 Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3 + 0.84; // roll
		 Yaw = 	atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3 - 10.42;		
		}
		
		NRF_TX_Mode();
		FloatToInt(tmp1,Pitch);
		FloatToInt(tmp2,Roll);
		FloatToInt(tmp3,Yaw);		
		strcat(tmp2,tmp3);
		strcat(tmp1,tmp2);
		printf("%s\r\n",tmp1); 
			
    /*开始发送数据*/	
	  NRF_Tx_Dat(tmp1);		
		delay_ms(100); 		
	}
	return 0;
}
/*********************************************END OF FILE**********************/
void FloatToInt(u8 *tmp,float flo)
{
	if(flo >= 0)
	{
		tmp[0] = '+';
		sprintf(tmp+1,"%2d",(int)flo);
	}
	else
	{
		sprintf(tmp,"%2d",(int)flo);
	}
}
