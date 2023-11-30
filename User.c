#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <RTL.h>
#include <Net_Config.h>
#include <LPC17xx.h>                    /* LPC17xx definitions               */

#include <core_cm3.h>
//#define  FOSC    12000000
//#define  FCCLK   (FOSC*8)
//#define  FPCLK   (FCCLK/4)
#include "User.h"
#include "HTTP.h"
#include "Client.h"
#include "Server.h"
#include "Announce.h"
#include "I2C_EEPROM.h"
#include "USER_EEPROM.h"
#include "HTTP_CGI.h"
#include "Iiot.h"
#include "RTC.h"
#include "Modbus_Server.h"
#include "UART.h"
#include "ADC.h"
#include "GPIO.h"
#include "SPI.h"
#include "TIMER.h" 
#include "SSP.h"
#include "TDSXGL.h"	
#include "TDSXGA.h"

#define TRUE 1
uint8_t GC_RecieveBuff[GK_RECEIVE_LENGTH]={0},GC_RecieveBuff1[GK_RECEIVE_LENGTH]={0},GC_RX_Flag=0,GC_RX_Flag1=0;
//static uint8_t  GC_ArrayPutPtr=0;//GC_ArrayPutPtr1=0;
extern unsigned char GC_IECEnableFlag_GA,GC_IECEnableFlag_GL,GC_Ticker30Sec_Flag;
extern uint64_t GL_Ticker30Sec;
uint8_t G_TxBuff[40]={0}; 
uint8_t G_TxBuff1[40]={0}; 
uint16_t AXE410_ModRtuCRC(uint8_t *,uint16_t);
static uint32_t GLI_Pclk,GLI_Fdiv;
 uint8_t Byte=0;
char a[]="\n\rApplied Embedded";
uint16_t sec,min,hou,day,mon,year;
unsigned char str3[150];
char on1=1,off1=1, on2=1,off2=1;
void AXE410_Timer0Init(void)
{
	LPC_SC->PCONP |= (1<<1); // System Control register status, set Timer0 power on(PCTIM0)
	LPC_SC->PCLKSEL0 |= ((0<<3)|(1<<2)); //Peripheral clock selection for Timer0
	LPC_TIM0->TCR = (1<<1); //Reset the counter
	LPC_TIM0->PR = 0; //Reset PR
	LPC_TIM0->PC = 0; //Reset PC, when PC= PR {TC +=1, PC=0}, therefore TC incremented every cycle
  LPC_TIM0->MCR = 0x03; //Match control register, TC reset if MR0 matches it,an interrupt is generated when MR0,MR1 matches the value in the TC
  LPC_TIM0->MR0 = GK_DELAY_1MS; //1ms delay
	LPC_TIM0->TC =0x00000000;
	LPC_TIM0->TCR = (1<<0); //Enables the counting
	LPC_TIM0->IR =0x00000000;// clear interrupt flag 
	NVIC_EnableIRQ(TIMER0_IRQn);	// Enable an interrupt
	NVIC_SetPriority(TIMER0_IRQn,5);	// Interrupt priority is 3(High priority starts from 0)
}
void AXE410_Timer2Init(void)
{
	/*LPC_SC->PCONP |= (1<<1); // System Control register status, set Timer0 power on(PCTIM0)
	LPC_SC->PCLKSEL0 |= ((0<<3)|(1<<2)); //Peripheral clock selection for Timer0
	LPC_TIM0->TCR = (1<<1); //Reset the counter
	LPC_TIM0->PR = 0; //Reset PR
	LPC_TIM0->PC = 0; //Reset PC, when PC= PR {TC +=1, PC=0}, therefore TC incremented every cycle
  LPC_TIM0->MCR = 0x03; //Match control register, TC reset if MR0 matches it,an interrupt is generated when MR0,MR1 matches the value in the TC
  LPC_TIM0->MR0 = GK_DELAY_1MS; //1ms delay
	LPC_TIM0->TC =0x00000000;
	LPC_TIM0->TCR = (1<<0); //Enables the counting
	LPC_TIM0->IR =0x00000000;// clear interrupt flag 
	NVIC_EnableIRQ(TIMER0_IRQn);	// Enable an interrupt
	NVIC_SetPriority(TIMER0_IRQn,3);	// Interrupt priority is 3(High priority starts from 0)*/
	LPC_SC->PCONP |= (1<<22); // System Control register status, set Timer2 power on(PCTIM2)
	LPC_SC->PCLKSEL1 |= ((0<<13)|(1<<12)); //Peripheral clock selection for Timer0
	LPC_TIM2->TCR = (1<<1); //Reset the counter
	LPC_TIM2->PR = 0; //Reset PR
	LPC_TIM2->PC = 0; //Reset PC, when PC= PR {TC +=1, PC=0}, therefore TC incremented every cycle
  LPC_TIM2->MCR = 0x03; //Match control register, TC reset if MR0 matches it,an interrupt is generated when MR0,MR1 matches the value in the TC
  LPC_TIM2->MR0 = GK_DELAY_1SEC; //1ms delay
	LPC_TIM2->TC =0x00000000;
	LPC_TIM2->TCR = (1<<0); //Enables the counting
	LPC_TIM2->IR =0x00000000;// clear interrupt flag 
	NVIC_EnableIRQ(TIMER2_IRQn);
	NVIC_SetPriority(TIMER2_IRQn,6);
}

// Timer1 used to generate 1micro second ticker in polling 
void AXE410_Timer1Init(void)
{
	LPC_SC->PCONP |= (1<<2); // System Control register status, set Timer1 power on(PCTIM1)
	LPC_SC->PCLKSEL0 |= ((0<<5)|(1<<4)); //Peripheral clock selection for Timer1
	LPC_TIM1->PR = GK_DELAY_1US; //Reset PR
	LPC_TIM1->PC = 0; //Reset PC, when PC= PR {TC +=1, PC=0}, therefore TC incremented every cycle
  LPC_TIM1->MCR = 0x02; //Match control register, TC reset if MR1 matches it,an interrupt is generated when MR1 matches the value in the TC
	LPC_TIM1->TC = 0x00000000;
}

// Timer1 micro second ticker handling function
uint8_t AXE410_DelayUs(uint32_t Time)
{
	LPC_TIM1->MR0 = Time;
	LPC_TIM1->TCR = 0x02;	// Reset timer
	LPC_TIM1->TCR = 0x01;	// Start timer
	while(LPC_TIM1->TC != LPC_TIM1->MR0);
	LPC_TIM1->PR = 0; //Reset PR
	//LPC_TIM1->TCR = 0x01;	// set timer
	return TRUE;
}


// Timer0 milli second interrupt handler function
void TIMER2_IRQHandler (void) 
{
	if ( LPC_TIM2->IR & (0x1<<0) )
  {
    LPC_TIM2->IR = 0x1<<0;//clear interrupt flag 
		GL_Ticker1Sec++;
//		TDSXGXXXA_BackupProcess();
		if((GL_Ticker1Sec>=5))//5ms
		{
			LPC_GPIO2->FIOPIN ^= GK_USER_LED3_PIN;
			GL_Ticker1Sec=0;
		}
  }
}
// Timer0 milli second interrupt handler function
void TIMER0_IRQHandler (void) 
{
	if ( LPC_TIM0->IR & (0x1<<0) )
  {
    LPC_TIM0->IR = 0x1<<0;//clear interrupt flag 
		GL_Ticker1Ms++;
//		TDSXGXXXA_BackupProcess();
		if((GL_Ticker1Ms>=5000))//5ms
		{
			LPC_GPIO2->FIOPIN ^= GK_USER_LED1_PIN;
			GL_Ticker1Ms=0;
		}
  }
}	
int main(void)
{
AXE410_UserIoInit();
AXE410_Timer0Init();
AXE410_Timer2Init();
AXE410_Timer1Init();
 while (1) 
{
		 //AXE410_DelayUs(100000);
	  // LPC_GPIO2->FIOSET = GK_USER_LED4_PIN;
	   //AXE410_DelayUs(100000);
	   //LPC_GPIO2->FIOCLR = GK_USER_LED4_PIN;
	
	uint32_t LLI_i;		
	LPC_GPIO2->FIOSET = GK_USER_LED4_PIN;
	for(LLI_i=0;LLI_i<5000000;LLI_i++);
	LPC_GPIO2->FIOCLR = GK_USER_LED4_PIN;
	for(LLI_i=0;LLI_i<5000000;LLI_i++);
}
}
