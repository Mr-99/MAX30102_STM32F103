/*************************************************************************************
HXDZ-30102:
	VCC<->3.3V
	GND<->GND
	SCL<->PB7
	SDA<->PB8
	IM<->PB9
0.96inch OLED :
	VCC<->3.3V
	GND<->GND
	SCL<->PA5
	SDA<->PA6
	RST<->PA3
	DC<->PA4
	CS<->PA2
USB-TTL:
	5V<->5V
	GND<->GND
	RXD<->PA9
	TXD<->PA10
**************************************************************************************/
#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "max30102.h" 
#include "myiic.h"
#include "oled.h"
#include "algorithm_by_RF.h"
//自己定义
#define BUFFER_SIZE (FS*ST)
uint8_t max_id;
	//Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every ST seconds
  float n_spo2,ratio,correl;  //SPO2 value
  int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
  int32_t n_heart_rate; //heart rate value
  int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is valid
  int32_t i;
  char hr_str[10];
	uint8_t temp[6];
int main()
{
	uint32_t elapsedTime,timeStart;
	uint32_t aun_ir_buffer[BUFFER_SIZE]; //infrared LED sensor data
	uint32_t aun_red_buffer[BUFFER_SIZE];  //red LED sensor data
	float old_n_spo2;  // Previous SPO2 value
	uint8_t uch_dummy,k;
	//END
	NVIC_Configuration();
	delay_init();	    	 //延时函数初始化	  
	uart_init(115200);	 	//串口初始化为115200
	//max30102初始化操作
	max30102_reset(); //resets the MAX30102
  delay_ms(1000);
  maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);  //Reads/clears the interrupt status register
  max30102_init();  //initialize the MAX30102
	//读ID
	max_id= max30102_Bus_Read(REG_PART_ID);
  old_n_spo2=0.0;
  //初始化结束
while(1)
{
  //buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
  //read BUFFER_SIZE samples, and determine the signal range
  for(i=0;i<BUFFER_SIZE;i++)
  {
     while(MAX30102_INT==1);   //wait until the interrupt pin asserts;
		max30102_FIFO_ReadBytes(REG_FIFO_DATA,temp);//从MAX30102读数据
		//红色三字节数字，红外三字节数据
		aun_red_buffer[i] =  (long)((long)((long)temp[0]&0x03)<<16) | (long)temp[1]<<8 | (long)temp[2];    // Combine values to get the actual number
		aun_ir_buffer[i] = (long)((long)((long)temp[3] & 0x03)<<16) |(long)temp[4]<<8 | (long)temp[5];   // Combine values to get the actual number
     
	}	
	  //calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
    rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl); 
	 			if((ch_hr_valid == 1)&&(ch_spo2_valid == 1))
				{
	      printf("HR=%i, ", n_heart_rate); 
				printf("HRvalid=%i, ", ch_hr_valid);
				printf("SpO2=%i, ", (uint16_t)n_spo2);
				printf("SPO2Valid=%i\r\n", ch_spo2_valid);
				}
}
return 1;
}
     
