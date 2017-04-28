/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   2.4g无线模块/nrf24l01+/master 测试
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

/*
 * PA2  -  PG8   ce使能
 * PA1  -  PG15  cs片选
 * PA3  -  PC4   irq中断
 */
u8 status;	//用于判断接收/发送状态
u8 rxbuf[256];			 //接收缓冲
int i=0;

#define q30  1073741824.0f
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
float Yaw,Roll,Pitch;

void StringToChar(u8 *String,u8 Chr[3][6]);
void FloatToInt(u8 *tmp,float flo);
void DMP_Value_Init(void);
void Nvic_Init(void);

void qiehuan(u16 pindao)   //切换频道
{
	if(pindao == 20)
		{
		NRF_CE_LOW();	
	  SPI_NRF_WriteReg(NRF_WRITE_REG+RF_CH,CHANAL);      //设置RF通信频率 
	  NRF_CE_HIGH();
		delay_ms(100);
		}
	else if(pindao == 40)
		{
		NRF_CE_LOW();	
	  SPI_NRF_WriteReg(NRF_WRITE_REG+RF_CH,CHANAL1);      //设置RF通信频率 
	  NRF_CE_HIGH();
		delay_ms(100);
		}
		
	else if(pindao == 60)
		{
		NRF_CE_LOW();	
	  SPI_NRF_WriteReg(NRF_WRITE_REG+RF_CH,CHANAL2);      //设置RF通信频率 
	  NRF_CE_HIGH();
		delay_ms(100);
		}		
}
 /**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)                  
{  
	u8 Chr[3][6];
	i2cInit();
	Nvic_Init();
  SPI_NRF_Init();

  /* 串口1初始化 */
  USART1_Config();

  /*检测NRF模块与MCU的连接*/
  status = NRF_Check(); 

  /*判断连接状态*/  
  if(status == SUCCESS)	   
    printf("\r\n      NRF与MCU连接成功！\r\n");  
  else	  
    printf("\r\n  NRF与MCU连接失败，请重新检查接线。\r\n");

  while(1)
  {		
			u8 tmpx[60]="{x:",tmpy[20]="y:",tmpz[20]="z:";		
			NRF_RX_Mode();
			/*等待接收数据*/
			qiehuan(CHANAL);  //20频道下发送
			NRF_Rx_Dat(rxbuf);
			StringToChar(rxbuf,Chr);	
			strcat(tmpx,Chr[0]);
			strcat(tmpy,Chr[1]);
			strcat(tmpz,Chr[2]);
			delay_ms(10);
		
			qiehuan(CHANAL1);  //40频道下发送
			NRF_Rx_Dat(rxbuf);
			StringToChar(rxbuf,Chr);	
			strcat(tmpx,Chr[0]);
			strcat(tmpy,Chr[1]);
			strcat(tmpz,Chr[2]);
			delay_ms(10);	

			qiehuan(CHANAL2);  //60频道下发送
			NRF_Rx_Dat(rxbuf);
			StringToChar(rxbuf,Chr);	
			strcat(tmpx,Chr[0]);
			strcat(tmpy,Chr[1]);
			strcat(tmpz,Chr[2]);
			strcat(tmpx,",");
			strcat(tmpy,",");
			strcat(tmpz,"}");
			delay_ms(10);

			strcat(tmpx,tmpy);
			strcat(tmpx,tmpz);
			Uart1_Put_String(tmpx);			
			memset(tmpx,0,60);     //清空数组
			memset(tmpy,0,20);
			memset(tmpz,0,20);
  }
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

void StringToChar(u8 *String,u8 Chr[3][6])
{
	u8 i=0,j=0,z=0;
	for(i=0;i<3;i++)
	{
		if(z > 15)
			z=0;
		if(String[z] == '+')
		{
			Chr[i][0] = '.';
			for(j=1;j<6;j++)
			{
				Chr[i][j] = String[z++];
				if((String[z] == '+') || (String[z] == '-') || (String[z] == '\0'))		
				{
					Chr[i][++j] = '\0';
					break;
				}
			}
		}	
		
		else if(String[z] == '-')
		{

			Chr[i][0] = '.';
			for(j=1;j<6;j++)
			{
				Chr[i][j] = String[z++];
				if((String[z] == '+') || (String[z] == '-') || (String[z] == '\0'))		
				{
					Chr[i][++j] = '\0';
					break;
				}
			}
		}		
	}
}


void DMP_Value_Init(void)
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
}

void Nvic_Init(void)
{
 	NVIC_InitTypeDef NVIC_InitStructure; 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
}
