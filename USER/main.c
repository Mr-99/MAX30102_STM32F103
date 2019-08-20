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
static void SystemInit (void);
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
	uint32_t aun_ir_buffer[BUFFER_SIZE]; //infrared LED sensor data
	uint32_t aun_red_buffer[BUFFER_SIZE];  //red LED sensor data
	uint8_t uch_dummy;
	SystemInit();
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
				else
				{
			  printf("HR=%i, ", n_heart_rate); 
				printf("HRvalid=%i, ", ch_hr_valid);
				printf("SpO2=%i, ", (uint16_t)n_spo2);
				printf("SPO2Valid=%i\r\n", ch_spo2_valid);
				}
}
return 1;
} 
static void SystemInit (void)
{
	
                /* Set HSION bit */
                RCC->CR |= (uint32_t)0x00000001;
                
                // select HSI as PLL source
                RCC->CFGR |= (uint32_t)RCC_CFGR_PLLSRC_HSI_Div2;        
                
                //PLLCLK=8/2*12=48M
                RCC->CFGR |= (uint32_t)RCC_CFGR_PLLMULL9;
                
                 /* HCLK = SYSCLK/1      */
            RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
                 
                /* Enable PLL */
            RCC->CR |= RCC_CR_PLLON;
                
            /* Wait till PLL is ready */
            while((RCC->CR & RCC_CR_PLLRDY) == 0)
            {
            }
            /* Select PLL as system clock source */
            RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
            RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    
                        
                /* Wait till PLL is used as system clock source */
            while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
            {
            }

//  /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
//  /* Set HSION bit */
//  RCC->CR |= (uint32_t)0x00000001;

//  /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
//#ifndef STM32F10X_CL
//  RCC->CFGR &= (uint32_t)0xF8FF0000;
//#else
//  RCC->CFGR &= (uint32_t)0xF0FF0000;
//#endif /* STM32F10X_CL */   
//  
//  /* Reset HSEON, CSSON and PLLON bits */
//  RCC->CR &= (uint32_t)0xFEF6FFFF;

//  /* Reset HSEBYP bit */
//  RCC->CR &= (uint32_t)0xFFFBFFFF;

//  /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
//  RCC->CFGR &= (uint32_t)0xFF80FFFF;

//#ifdef STM32F10X_CL
//  /* Reset PLL2ON and PLL3ON bits */
//  RCC->CR &= (uint32_t)0xEBFFFFFF;

//  /* Disable all interrupts and clear pending bits  */
//  RCC->CIR = 0x00FF0000;

//  /* Reset CFGR2 register */
//  RCC->CFGR2 = 0x00000000;
//#elif defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || (defined STM32F10X_HD_VL)
//  /* Disable all interrupts and clear pending bits  */
//  RCC->CIR = 0x009F0000;

//  /* Reset CFGR2 register */
//  RCC->CFGR2 = 0x00000000;      
//#else
//  /* Disable all interrupts and clear pending bits  */
//  RCC->CIR = 0x009F0000;
//#endif /* STM32F10X_CL */
//    
//#if defined (STM32F10X_HD) || (defined STM32F10X_XL) || (defined STM32F10X_HD_VL)
//  #ifdef DATA_IN_ExtSRAM
//    SystemInit_ExtMemCtl(); 
//  #endif /* DATA_IN_ExtSRAM */
//#endif 

//  /* Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers */
//  /* Configure the Flash Latency cycles and enable prefetch buffer */
//  SetSysClock();

//#ifdef VECT_TAB_SRAM
//  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM. */
//#else
//  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH. */
//#endif 
}
