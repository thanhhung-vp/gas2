/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CLCD_I2C.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define FLASH_BASE_ADDR 0x08007C00 //page 31
#define PAGE_SIZE 1024 // 1 KB/ page
#define RECORD_SIZE 40             // size ban ghi
#define MAX_RECORDS 10
uint32_t recordCount = 0;

char buffer_1[64]; // buffer truyen TX
char buffer_2[64];
char rxBuffer[64] = {0};  //buffer nhan RX
char buffer_ppm[16];
uint8_t rxIndex = 0;

volatile uint8_t ledRedState = 0;
volatile uint32_t ledPeriod = 499;
volatile uint8_t sendingLogs = 0;
float ppm=0;
#define RECORD_COUNT_ADDR 0x0800A400 // Page 41 luu recordCount
 uint8_t systemState =0; // 0: Dung    1:Hoat Dong


#define DEBOUNCE_DELAY 100 // 50ms debounce
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define INPUT_ANALOG							( (uint8_t) 0x00)
#define INPUT_FLOATING						( (uint8_t) 0x01)
#define INPUT_PUPD								( (uint8_t) 0x02)

#define OUTPUT_PP									( (uint8_t) 0x00)
#define OUTPUT_OD									( (uint8_t) 0x01)
#define OUTPUT_AF_PP							( (uint8_t) 0x02)
#define OUTPUT_AF_OD							( (uint8_t) 0x03)

#define MODE_INPUT								( (uint8_t) 0x00)
#define MODE_OUTPUT_10MHZ					( (uint8_t) 0x01)
#define MODE_OUTPUT_2MHZ					( (uint8_t) 0x02)
#define MODE_OUTPUT_50MHZ					( (uint8_t) 0x03)

#define NOPULL 		( (uint8_t) 0x00)
#define PU 				( (uint8_t) 0x01)
#define PD				( (uint8_t) 0x02)

typedef enum
{
  RISING,
  FALLING,
	CHANGE
} EXTI_trigger;

#define portA 0
#define portB 1
#define portC 2
#define portD 3
#define portE 4
#define portF 5
#define portG 6

#define ADC_Channel_0                               ((uint8_t)0x00)
#define ADC_Channel_1                               ((uint8_t)0x01)
#define ADC_Channel_2                               ((uint8_t)0x02)
#define ADC_Channel_3                               ((uint8_t)0x03)
#define ADC_Channel_4                               ((uint8_t)0x04)
#define ADC_Channel_5                               ((uint8_t)0x05)
#define ADC_Channel_6                               ((uint8_t)0x06)
#define ADC_Channel_7                               ((uint8_t)0x07)
#define ADC_Channel_8                               ((uint8_t)0x08)
#define ADC_Channel_9                               ((uint8_t)0x09)
#define ADC_Channel_10                              ((uint8_t)0x0A)
#define ADC_Channel_11                              ((uint8_t)0x0B)
#define ADC_Channel_12                              ((uint8_t)0x0C)
#define ADC_Channel_13                              ((uint8_t)0x0D)
#define ADC_Channel_14                              ((uint8_t)0x0E)
#define ADC_Channel_15                              ((uint8_t)0x0F)
#define ADC_Channel_16                              ((uint8_t)0x10)
#define ADC_Channel_17                              ((uint8_t)0x11)

//PWM
typedef enum
{
  PWM_Pin1,
  PWM_Pin2,
	PWM_Pin3,
} PWM_Pin;

typedef enum
{
  PWM_CH1,
  PWM_CH2,
	PWM_CH3,
	PWM_CH4
} PWM_CH;

typedef enum
{
  PA9PA10,//USART1
  PB6PB7,//USART1
	PA2PA3,//USART2
	PB10PB11//USART3
} USART_Pin;

typedef struct
{
  char * buffer;
	uint16_t size;
	uint16_t in;
	uint16_t out;
	uint16_t num;
} USART_ST;

#define USART1_BUFFER_SIZE 100
#define USART2_BUFFER_SIZE 100
#define USART3_BUFFER_SIZE 100

char USART1_BUFFER[USART1_BUFFER_SIZE];
char USART2_BUFFER[USART1_BUFFER_SIZE];
char USART3_BUFFER[USART1_BUFFER_SIZE];

USART_ST USART1_ST={USART1_BUFFER, USART1_BUFFER_SIZE, 0, 0, 0 };
USART_ST USART2_ST={USART2_BUFFER, USART2_BUFFER_SIZE, 0, 0, 0 };
USART_ST USART3_ST={USART3_BUFFER, USART3_BUFFER_SIZE, 0, 0, 0 };
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
CLCD_I2C_Name LCD1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
//void NVICx_Init(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
//void GPIOx_Init(GPIO_TypeDef  *GPIOx, uint8_t Pin, uint8_t Mode, uint8_t Pull, uint8_t Speed);
//void GPIO_WritePin(GPIO_TypeDef  *GPIOx, uint8_t Pin, uint8_t Value);
//uint8_t GPIO_ReadPin(GPIO_TypeDef  *GPIOx, uint8_t Pin);
//void GPIO_TogglePin(GPIO_TypeDef  *GPIOx, uint8_t Pin);
//void EXTIx_Init(GPIO_TypeDef  *GPIOx, uint8_t Pin, EXTI_trigger Trigger);
//void ADCx_Init(ADC_TypeDef *ADCx,uint8_t Channel);
//uint16_t ADCx_Read(ADC_TypeDef *ADCx,uint32_t Channel);
//void TIMx_Init(TIM_TypeDef *TIMx,uint16_t ARR, uint16_t PSC);
//void USARTx_Init(USART_TypeDef *USARTx, USART_Pin Pins, uint32_t Baud);
//char USARTx_GetC(USART_TypeDef* USARTx);
//uint16_t USARTx_isEMPTY(USART_TypeDef* USARTx);
//uint16_t USARTx_GetS(USART_TypeDef* USARTx, char *Str, uint16_t len);
//void USARTx_PutC(USART_TypeDef *USARTx,char c);
//void USARTx_PutS(USART_TypeDef *USARTx,char *Str);

uint8_t flag_exti0;
uint8_t reset=0;
uint16_t adc_ch0=0;
char c=0;
char BUFFER[100];


// LIB GPIO
void GPIOx_Init(GPIO_TypeDef  *GPIOx, uint8_t Pin, uint8_t Mode, uint8_t Pull, uint8_t Speed)
{

		if(GPIOx==GPIOA)RCC->APB2ENR |= 1<<2; //GPIOA bit so 2
		else if(GPIOx==GPIOB)RCC->APB2ENR |= 1<<3; //GPIOB bit so 3
		else if(GPIOx==GPIOC)RCC->APB2ENR |= 1<<4; //GPIOC bit so 4
		else if(GPIOx==GPIOD)RCC->APB2ENR |= 1<<5; //GPIOD bit so 5

		if(Pin<8){
			GPIOx->CRL &=~(0xF<<Pin*4);
			GPIOx->CRL |=((Mode<<2)+Speed)<<(Pin*4);
		}else{
			GPIOx->CRH &=~(0xF<<((Pin-8)*4));
			GPIOx->CRH |=((Mode<<2)+Speed)<<((Pin-8)*4);
		}
		if(Pull==PU)GPIOx->ODR |= 1<<(Pin);
		else GPIOx->ODR &= ~(1<<Pin);
}


//------------ WRITE -------READ------------TOGGLEpin
void GPIO_WritePin(GPIO_TypeDef  *GPIOx, uint8_t Pin, uint8_t Value){
	if(Value==1)GPIOx->BSRR |= 1<<Pin;  //SET  = 1
	else GPIOx->BSRR |= 1<<(Pin+16);  //RESET PB2 = 0
}

uint8_t GPIO_ReadPin(GPIO_TypeDef  *GPIOx, uint8_t Pin){
	return((GPIOx->IDR &(1<<Pin))==0)? 0:1;
}

void GPIO_TogglePin(GPIO_TypeDef  *GPIOx, uint8_t Pin){
	GPIOx->ODR^=1<<Pin;
}



//------------EXTI-----------------


void NVICx_Init(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority){
  uint32_t prioritygroup = 0x00U;
  prioritygroup = NVIC_GetPriorityGrouping();
  NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, PreemptPriority, SubPriority));
	NVIC_EnableIRQ(IRQn);
}
void EXTIx_Init(GPIO_TypeDef  *GPIOx, uint8_t Pin, EXTI_trigger Trigger){

	IRQn_Type IRQn;
	uint8_t port=0;

	RCC->APB2ENR |= (1<<0); //Bit 0 AFIOEN: Alternate function I/O clock enable

	if(GPIOx==GPIOA){port=portA;	}
	if(GPIOx==GPIOB){port=portB;	}
	if(GPIOx==GPIOC){port=portC;	}
	if(GPIOx==GPIOD){port=portD;	}
	if(GPIOx==GPIOE){port=portE;	}



	if(Pin==0) 	{IRQn=EXTI0_IRQn;}
	if(Pin==1) 	{IRQn=EXTI1_IRQn;}
	if(Pin==2) 	{IRQn=EXTI2_IRQn;}
	if(Pin==3) 	{IRQn=EXTI3_IRQn;}
	if(Pin==4) 	{IRQn=EXTI4_IRQn;}
	if(Pin==5) 	{IRQn=EXTI9_5_IRQn;}
	if(Pin==6) 	{IRQn=EXTI9_5_IRQn;}
	if(Pin==7) 	{IRQn=EXTI9_5_IRQn;}
	if(Pin==8) 	{IRQn=EXTI9_5_IRQn;}
	if(Pin==9) 	{IRQn=EXTI9_5_IRQn;}
	if(Pin==10) {IRQn=EXTI15_10_IRQn;}
	if(Pin==11) {IRQn=EXTI15_10_IRQn;}
	if(Pin==12) {IRQn=EXTI15_10_IRQn;}
	if(Pin==13) {IRQn=EXTI15_10_IRQn;}
	if(Pin==14) {IRQn=EXTI15_10_IRQn;}
	if(Pin==15) {IRQn=EXTI15_10_IRQn;}

	if (Trigger == RISING) 			 {GPIOx_Init(GPIOx,Pin, INPUT_PUPD, PD, 0); EXTI->RTSR |= 1<<Pin;}
	else if (Trigger == FALLING) {GPIOx_Init(GPIOx,Pin, INPUT_PUPD, PU, 0);	EXTI->FTSR |=	1<<Pin;}
	else if	(Trigger == CHANGE)	 {GPIOx_Init(GPIOx,Pin, INPUT_FLOATING, NOPULL, 0);EXTI->RTSR |= 1<<Pin;EXTI->FTSR |=	1<<Pin;}


	AFIO->EXTICR[Pin/4] &= ~(0xf<<((Pin%4)*4));
	AFIO->EXTICR[Pin/4] |= (port<<((Pin%4)*4));

	EXTI->IMR |= 1<<Pin;
	NVICx_Init(IRQn, 0x0a, Pin); //TIM IRQn

}
volatile uint32_t dem_exti0 = 0;
volatile uint32_t last_press_time0 = 0;
#define DEBOUNCE_TIME 250

void EXTI0_IRQHandler(void)
{
  //systemState=0;
	if(EXTI->PR  & (1<<0)){
		if(GPIO_ReadPin(GPIOB,0)==0){

			uint32_t current_time = HAL_GetTick();

			if ((current_time - last_press_time0) >= DEBOUNCE_TIME ) {

						systemState = !systemState;
						if(systemState==0){flag_exti0=1;}
						dem_exti0++;
						last_press_time0 = current_time;
			}
	}

}
	EXTI->PR |=(1<<0);
}
volatile uint32_t dem_exti1 = 0;
volatile uint32_t last_press_time1 = 0;
volatile uint8_t stable_state1 = 0;
void EXTI1_IRQHandler(void)
{

	if(EXTI->PR  & (1<<1)){
		if(GPIO_ReadPin(GPIOB,1)==0){
		uint32_t current_time = HAL_GetTick();

			if ((current_time - last_press_time1) >= DEBOUNCE_TIME) {

						dem_exti1++;
						reset=1;
						last_press_time1 = current_time;
			}
	}
}

	EXTI->PR |=(1<<1);
}


// -------------------LIB ADC------------------

void ADCx_Init(ADC_TypeDef *ADCx,uint8_t Channel){
	if(ADCx==ADC1) RCC->APB2ENR |= 1<<9;//adc1
	if(ADCx==ADC2) RCC->APB2ENR |= 1<<10;//adc1
	if(Channel==ADC_Channel_0) GPIOx_Init(GPIOA, 0,INPUT_ANALOG,NOPULL,0);
	if(Channel==ADC_Channel_1) GPIOx_Init(GPIOA, 1,INPUT_ANALOG,NOPULL,0);
	if(Channel==ADC_Channel_2) GPIOx_Init(GPIOA, 2,INPUT_ANALOG,NOPULL,0);
	if(Channel==ADC_Channel_3) GPIOx_Init(GPIOA, 3,INPUT_ANALOG,NOPULL,0);
	if(Channel==ADC_Channel_4) GPIOx_Init(GPIOA, 4,INPUT_ANALOG,NOPULL,0);
	if(Channel==ADC_Channel_5) GPIOx_Init(GPIOA, 5,INPUT_ANALOG,NOPULL,0);
	if(Channel==ADC_Channel_6) GPIOx_Init(GPIOA, 6,INPUT_ANALOG,NOPULL,0);
	if(Channel==ADC_Channel_7) GPIOx_Init(GPIOA, 7,INPUT_ANALOG,NOPULL,0);
	if(Channel==ADC_Channel_8) GPIOx_Init(GPIOB, 0,INPUT_ANALOG,NOPULL,0);
	if(Channel==ADC_Channel_9) GPIOx_Init(GPIOB, 1,INPUT_ANALOG,NOPULL,0);
	ADCx->CR1 |=0<<8; //0: Scan mode disabled
	//ADCx->CR2|= (1<<1)|(1<<0);//1: Continuous conversion mode; 1: Enable ADC and to start conversion
	//ADCx->CR1 |=(0<<11); //0:  Discontinuous mode on regular channels disabled
	ADCx->CR2|= (1<<0);//1: Continuous conversion mode; 1: Enable ADC and to start conversion
	ADCx->CR2|=0<<11;//0: Right Alignment
	ADCx->CR2|=1<<20;//1: Conversion on external event enabled

	ADCx->CR2 &=~(7<<17);// ghi 3 bit 1 sau do dao nguoc bit 17 18 19 = 0 0 0
	ADCx->CR2 |=7<<17; //ghi 3 bit 1 vao 3 bit 17 18 19 1 1 1

	ADCx->SQR1&=~(15<<20);
	ADCx->SQR1|=(0<<20);//0000: 1 conversion

	if(Channel<10){
	ADCx->SMPR2&=~(7<<Channel*3);
	ADCx->SMPR2|=(7<<Channel*3);
	}
	else{
		ADCx->SMPR1&=~(7<<((Channel-10)*3));
		ADCx->SMPR1|=(7<<((Channel-10)*3));
	}
	ADCx->SQR3&=~(31<<0);
	ADCx->SQR3|= Channel;

	ADCx->CR2|=1<<3;//1: Initialize calibration register.
	while(ADCx->CR2&(1<<3));

	ADCx->CR2|=1<<2;//1: Enable calibration.
	while(ADCx->CR2&(1<<2));

	ADCx->CR2|=(1<<22)| (1<<0); // 1: Starts conversion of regular channels  // 1: Enable ADC and to start conversion
}
uint16_t ADCx_Read(ADC_TypeDef *ADCx,uint32_t Channel){

	if(Channel<10){
	ADCx->SMPR2&=~(7<<Channel*3);
	ADCx->SMPR2|=(7<<Channel*3);
	}
	else{
		ADCx->SMPR1&=~(7<<((Channel-10)*3));
		ADCx->SMPR1|=(7<<((Channel-10)*3));
	}
	//ADCx->SQR3&=~(31<<0);
	ADCx->SQR3= Channel;
	ADCx->CR2|=(1<<22)| (1<<0); // 1: Starts conversion of regular channels  // 1: Enable ADC and to start conversion
	while(!(ADCx->SR&(1<<1)));// Conversation complete
		return ADCx->DR;
}



//----------------------LIB TIMER---------------------
//TIMER


void TIMx_Init(TIM_TypeDef *TIMx,uint16_t ARR, uint16_t PSC){
	uint8_t t=0;  // SubPriority
	IRQn_Type IRQn;
	if(TIMx==TIM1) {RCC->APB2ENR|= 1<<11;IRQn= TIM1_UP_IRQn, t=1;}  // RCC TIM1

	if(TIMx==TIM2) {RCC->APB1ENR|= 1<<0; IRQn= TIM2_IRQn; t=2;}  // RCC TIM2
	if(TIMx==TIM3) {RCC->APB1ENR|= 1<<1; IRQn= TIM3_IRQn; t=3;}  // RCC TIM3
	if(TIMx==TIM4) {RCC->APB1ENR|= 1<<2; IRQn= TIM4_IRQn; t=4;}	 // RCC TIM4


	TIMx->PSC= PSC-1;  // ARR
	TIMx->ARR= ARR-1;  // PSC

	//TIMx->DIER|= 1<<0; //	 UIE 1: Update interrupt enabled. START IT
	NVICx_Init(IRQn, 1, t); //TIM IRQn

	//TIMx->CR1|= 1<<0;  //  CEN 1: Counter enabled

}

void TIM2_IRQHandler(void){
  if(TIM2->SR  & (1<<0)){
		ledRedState = !ledRedState;
		GPIO_WritePin(GPIOA, 3, ledRedState);
		TIM2->ARR= ledPeriod;  // ARR
	}

	TIM2->SR  &=~(1<<0);
}


//---------------USART----------------
void USARTx_Init(USART_TypeDef *USARTx, USART_Pin Pins, uint32_t Baud){
	uint8_t u=0;
	IRQn_Type IRQn;
	uint32_t PCLKx=0;

	float USARTDIV=0.0;
	uint16_t Mantissa=0 , Fraction=0;

	RCC->APB2ENR |= (1<<0); //Bit 0 AFIOEN: Alternate function I/O clock enable

 if			(USARTx==USART1) {PCLKx=(HAL_RCC_GetHCLKFreq() >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos]);RCC->APB2ENR |= (1<<14); IRQn=USART1_IRQn; u=1;}
 else if(USARTx==USART2) {PCLKx=(HAL_RCC_GetHCLKFreq() >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]);RCC->APB1ENR |= (1<<17); IRQn=USART2_IRQn; u=2;}
 else if(USARTx==USART3) {PCLKx=(HAL_RCC_GetHCLKFreq() >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]);RCC->APB1ENR |= (1<<18); IRQn=USART3_IRQn; u=3;}


	if(Pins == PA9PA10){ //USART1
		GPIOx_Init(GPIOA, 9, OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);// TX
		GPIOx_Init(GPIOA, 10, INPUT_FLOATING,NOPULL,0);// RX
	}
	if(Pins == PB6PB7){ //USART1
		AFIO->MAPR |= 1<<2; //remap PB6 PB7 ennable
		GPIOx_Init(GPIOB, 6, OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);// TX
		GPIOx_Init(GPIOB, 7, INPUT_FLOATING,NOPULL,0);// RX
	}
	if(Pins == PA2PA3){ //USART2
		GPIOx_Init(GPIOA, 2, OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);// TX
		GPIOx_Init(GPIOA, 3, INPUT_FLOATING,NOPULL,0);// RX
	}
	if(Pins == PB10PB11){ //USART3
		GPIOx_Init(GPIOB, 10, OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);// TX
		GPIOx_Init(GPIOB, 11, INPUT_FLOATING,NOPULL,0);// RX
	}

	USARTDIV= (float)PCLKx/(16.0 * Baud );
	Mantissa=(uint16_t)USARTDIV;
	Fraction=(uint16_t)(USARTDIV- Mantissa)*16;

	USARTx->BRR = (Mantissa<<4) + Fraction;  //Baud


	USARTx->CR1 |= (1<<2) | (1<<3); //Bit 2 RE: Receiver enable Bit 3 TE: Transmitter enable
	USARTx->CR1 |= 1<<5; //Bit 5 RXNEIE: RXNE interrupt enable
	NVICx_Init(IRQn, 1, u);

	USARTx->CR1 |= 1<<13;  //Bit 13 UE: USART enable

}

void USARTtoBUFF(USART_ST *u, char c){
	if(u->in<u->size){
		u->buffer[u->in] = c;
		u->in++;
		u->num++;
	if(u->in == u->size) u->in=0;
	}
}



char USARTx_GetC(USART_TypeDef* USARTx){
		USART_ST *u;
		char c=0;
		if			(USARTx==USART1)  	{u=&USART1_ST;}
		else if(USARTx==USART2) 		{u=&USART2_ST;}
		else if(USARTx==USART3) 		{u=&USART3_ST;}

	if(u->num>0){

		c=u->buffer[u->out];
		u->out++;
		u->num--;
		if(u->out==u->size) u->out=0;
	}

		return c;
}


uint16_t USARTx_isEMPTY(USART_TypeDef* USARTx){
		USART_ST *u;
		if			(USARTx==USART1)  	{u=&USART1_ST;}
		else if(USARTx==USART2) 		{u=&USART2_ST;}
		else if(USARTx==USART3) 		{u=&USART3_ST;}

			return u->num;
}



uint16_t USARTx_GetS(USART_TypeDef* USARTx, char *Str, uint16_t len){
	uint16_t i=0;
	char c=0;

	if(USARTx_isEMPTY(USARTx)==0) return 0;

	while(i<len){
	c = USARTx_GetC(USARTx);
	if(c){
		Str[i]=c;
		if(Str[i]=='\n') {i++; break;}
		else i++;

	}
}
	Str[i]='\0';
	return i;
}

void USART1_IRQHandler(void){
  if ((USART1->SR & USART_SR_RXNE) != RESET){

			//USARTx_PutC(USART1,USART1->DR);
				USARTtoBUFF(&USART1_ST, USART1->DR);
	}
}
void USARTx_PutC(USART_TypeDef *USARTx,char c){
		while((USARTx->SR & (1<<7))==0); //Bit 7 TXE: Transmit data register empty
		USARTx->DR= c;


}

void USARTx_PutS(USART_TypeDef *USARTx,char *Str){

	while(*Str) USARTx_PutC(USARTx, * Str++);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float Ro = 300559.0; // Calibrated in clean air
// tinh toan gia tri PPM
float calculatePPM(uint32_t adcValue) {
  float Vout = (adcValue * 3.3) / 4095.0; // Vref = 3.3V
  float Rs = ((5.0 * 10000.0) / Vout) - 10000.0; // Vcc = 5V, Rl = 10kO
  float ratio = Rs / Ro;
  float ppm= 94.0 * pow(ratio, -1.5); // Formula for LPG
	ppm = (float)((int)(ppm * 100 + 0.5)) / 100;
	if (ppm > 10000.0) ppm = 10000.0; // Cap at maximum detectable range
  return ppm;
}
//Gui ppm sang ESP32 qua UART de hien thi len E_RA APP
void sendPPM(float ppm) {

		sprintf(buffer_1,"PPM:%.2f\r\n", ppm);
		USARTx_PutS(USART1,buffer_1);//
}
//SET BUZZER, RELAY khi ppm>500
void control_OUTPUT(float ppm) {
  if (ppm > 500) {
    GPIO_WritePin(GPIOA, 4, 1);
		GPIO_WritePin(GPIOA, 5, 1);
  } else {
    GPIO_WritePin(GPIOA, 4, 0);
		GPIO_WritePin(GPIOA, 5, 0);
  }
}
//DIEU KHIEN LED THEO PPM
void controlLED(float ppm) {

  if (ppm < 200) {// khong co khi gas
    GPIO_WritePin(GPIOA, 1, 1);
		GPIO_WritePin(GPIOA, 2, 0);
		GPIO_WritePin(GPIOA, 3, 0);

    TIM2->CR1 &= ~(1 << 0);
		TIM2->DIER &= ~(1 << 0);

					CLCD_I2C_SetCursor(&LCD1, 0, 0);
					CLCD_I2C_WriteString(&LCD1,"PPM:");

					CLCD_I2C_SetCursor(&LCD1, 4, 0);
					sprintf(buffer_ppm,"%.2f",ppm);
					CLCD_I2C_WriteString(&LCD1,buffer_ppm);

					CLCD_I2C_SetCursor(&LCD1, 0, 1);
					CLCD_I2C_WriteString(&LCD1,"State: 1");

					CLCD_I2C_SetCursor(&LCD1, 9, 1);
					CLCD_I2C_WriteString(&LCD1,"Alarm:0");

  } else if (ppm >= 200 && ppm <300) {// khi gas thap
		GPIO_WritePin(GPIOA, 1, 0);
    GPIO_WritePin(GPIOA, 2, 1);
		GPIO_WritePin(GPIOA, 3, 0);


    TIM2->CR1 &= ~(1 << 0);
		TIM2->DIER &= ~(1 << 0);


			CLCD_I2C_SetCursor(&LCD1, 0, 0);
			CLCD_I2C_WriteString(&LCD1,"PPM:");

			CLCD_I2C_SetCursor(&LCD1, 4, 0);
			sprintf(buffer_ppm,"%.2f",ppm);
			CLCD_I2C_WriteString(&LCD1,buffer_ppm);

			CLCD_I2C_SetCursor(&LCD1, 0, 1);
			CLCD_I2C_WriteString(&LCD1,"State: 1");

			CLCD_I2C_SetCursor(&LCD1, 9, 1);
			CLCD_I2C_WriteString(&LCD1,"Alarm:1");
  } else if (ppm >= 300 && ppm < 500) {//nong do khi gas cao
		ledPeriod = 5000;//500ms

		TIM2->DIER|= 1<<0; //	 UIE 1: Update interrupt enabled. START IT
		TIM2->CR1|= 1<<0;  //  CEN 1: Counter enabled


			CLCD_I2C_SetCursor(&LCD1, 0, 0);
			CLCD_I2C_WriteString(&LCD1,"PPM:");

			CLCD_I2C_SetCursor(&LCD1, 4, 0);
			sprintf(buffer_ppm,"%.2f",ppm);
			CLCD_I2C_WriteString(&LCD1,buffer_ppm);

			CLCD_I2C_SetCursor(&LCD1, 0, 1);
			CLCD_I2C_WriteString(&LCD1,"State: 1");

			CLCD_I2C_SetCursor(&LCD1, 9, 1);
			CLCD_I2C_WriteString(&LCD1,"Alarm:2");

		GPIO_WritePin(GPIOA, 1, 0);
		GPIO_WritePin(GPIOA, 2, 0);
  } else {
		GPIO_WritePin(GPIOA, 1, 0);
		GPIO_WritePin(GPIOA, 2, 0);



			CLCD_I2C_SetCursor(&LCD1, 0, 0);
			CLCD_I2C_WriteString(&LCD1,"PPM:");

			CLCD_I2C_SetCursor(&LCD1, 4, 0);
			sprintf(buffer_ppm,"%.2f",ppm);
			CLCD_I2C_WriteString(&LCD1,buffer_ppm);

			CLCD_I2C_SetCursor(&LCD1, 0, 1);
			CLCD_I2C_WriteString(&LCD1,"State: 1");

			CLCD_I2C_SetCursor(&LCD1, 9, 1);
			CLCD_I2C_WriteString(&LCD1,"Alarm:3");

    float freq = 2.0 + ((ppm - 500.0) / 9500.0) * 8.0; // 2hz-10hz
    if (freq > 10.0) freq = 10.0;
    ledPeriod = (uint32_t)(10000.0 / (2.0 * freq)) - 1;

		TIM2->DIER|= 1<<0; //	 UIE 1: Update interrupt enabled. START IT
		TIM2->CR1|= 1<<0;  //  CEN 1: Counter enabled
  }
}


//xoá page flash
void FlashEraseSector(uint32_t sector) {
  FLASH_EraseInitTypeDef eraseInit;
  uint32_t PageError;
  HAL_FLASH_Unlock();
  eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  eraseInit.PageAddress = sector;
  eraseInit.NbPages = 1;
  HAL_FLASHEx_Erase(&eraseInit, &PageError);
  HAL_FLASH_Lock();
}
//ghi data vao flash
void FlashWriteData(uint32_t address, uint8_t *data, uint32_t size) {
  HAL_FLASH_Unlock();
  for (uint32_t i = 0; i < size; i += 4) {
    uint32_t word = *(uint32_t*)(data + i);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i, word);
  }
  HAL_FLASH_Lock();
}
// doc data tu flash
void FlashReadData(uint32_t address, uint8_t *data, uint32_t size) {
  for (uint32_t i = 0; i < size; i++) {
    data[i] = *(uint8_t*)(address + i);
  }
}
////luu gia tri RecordCount vào flash
void saveRecordCount() {
  uint8_t data[4] = {0};
  memcpy(data, &recordCount, sizeof(recordCount));
  FlashEraseSector(RECORD_COUNT_ADDR);
  FlashWriteData(RECORD_COUNT_ADDR, data, sizeof(recordCount));
}
//doc gia tri RecordCount khi stm32 khoi dong
void loadRecordCount() {
  FlashReadData(RECORD_COUNT_ADDR, (uint8_t*)&recordCount, sizeof(recordCount));
}
void resetSystem() { //khi nhan nut SW2 RESET HE THONG


					CLCD_I2C_SetCursor(&LCD1, 0, 0);
					CLCD_I2C_WriteString(&LCD1,"                ");

					CLCD_I2C_SetCursor(&LCD1, 0, 1);
					CLCD_I2C_WriteString(&LCD1,"State: 0");

					CLCD_I2C_SetCursor(&LCD1, 9, 1);
					CLCD_I2C_WriteString(&LCD1,"       ");
  recordCount = 0;
  //saveRecordCount();
//  for (uint32_t i = 0; i < MAX_RECORDS; i++) {
//    FlashEraseSector(FLASH_BASE_ADDR + i * PAGE_SIZE);
//  }
  systemState = 0;  // cho ve trang thai dung STOP
  GPIO_WritePin(GPIOA, 1 ,0);
	GPIO_WritePin(GPIOA, 2, 0);
	GPIO_WritePin(GPIOA, 3, 0);
	GPIO_WritePin(GPIOA, 4, 0);
	GPIO_WritePin(GPIOA, 5, 0);
	GPIO_WritePin(GPIOA,6,1);  // khi he thong o trang thai dung thi LED_GREEN ON

  TIM1->CR1 &= ~(1 << 0);
	TIM1->DIER &= ~(1 << 0);
}
//Luu buffer log time + ppm vao page flash begin 0x08007C00, 0X08008000, 0x08008400...vvv
void saveLogToFlash(const char* logStr) {
  uint32_t pageIndex = recordCount % MAX_RECORDS;
  uint32_t address = FLASH_BASE_ADDR + pageIndex * PAGE_SIZE; // Page 31 + pageIndex
  uint8_t data[RECORD_SIZE] = {0};
  strncpy((char*)data, logStr, strlen(logStr));
  FlashEraseSector(address);
  FlashWriteData(address, data, RECORD_SIZE);
  recordCount++;
	saveRecordCount();
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	GPIOx_Init(GPIOA,1, OUTPUT_PP, NOPULL, MODE_OUTPUT_50MHZ);  //LED BLUE
	GPIOx_Init(GPIOA,2, OUTPUT_PP, NOPULL, MODE_OUTPUT_50MHZ);	//LED YELLOW
	GPIOx_Init(GPIOA,3, OUTPUT_PP, NOPULL, MODE_OUTPUT_50MHZ);	//LED RED
	GPIOx_Init(GPIOA,4, OUTPUT_PP, NOPULL, MODE_OUTPUT_50MHZ);	//VAN GAS
	GPIOx_Init(GPIOA,5, OUTPUT_PP, NOPULL, MODE_OUTPUT_50MHZ);	//BUZZER
	GPIOx_Init(GPIOA,6, OUTPUT_PP, NOPULL, MODE_OUTPUT_50MHZ);	//LED GREEN
	GPIOx_Init(GPIOB,2, OUTPUT_PP, NOPULL, MODE_OUTPUT_50MHZ);	//LED PB2 ONBOARD
	EXTIx_Init(GPIOB,0, FALLING);
	EXTIx_Init(GPIOB,1, FALLING);
	ADCx_Init(ADC1,ADC_Channel_0);															//ADC
	TIMx_Init(TIM2,5000, 7200);																	//TIMER1 500ms

	//USARx1_Init(USART3, PB10PB11, 115200);											//USART3 PB10 TX PB11 RX 1152000 BAUDRATE


	CLCD_I2C_Init(&LCD1,&hi2c1,0x4e,16,2);

	CLCD_I2C_SetCursor(&LCD1, 0, 0);
	CLCD_I2C_WriteString(&LCD1,"");
	CLCD_I2C_SetCursor(&LCD1, 0, 1);
	CLCD_I2C_WriteString(&LCD1,"State: 0");



	GPIO_WritePin(GPIOA, 1 ,0);
	GPIO_WritePin(GPIOA, 2, 0);
	GPIO_WritePin(GPIOA, 3, 0);
	GPIO_WritePin(GPIOA, 4, 0);
	GPIO_WritePin(GPIOA, 5, 0);
	GPIO_WritePin(GPIOA,6,1);  // khi he thong o trang thai dung thi LED_GREEN ON
	//loadRecordCount();

	uint32_t lastPPMTime = 0;
  uint32_t Interval = 500;

	USARTx_Init(USART1, PA9PA10, 115200);												//USART1 PA9 TX PA10 RX 1152000 BAUDRATE
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


				uint32_t currentTime = HAL_GetTick();
    if (systemState == 1 &&currentTime - lastPPMTime >= Interval) // Trang thai Hoat Dong
			{
				//GPIO_TogglePin(GPIOB,2);
				GPIO_WritePin(GPIOA,6,0);
				lastPPMTime = currentTime;
				adc_ch0=ADCx_Read(ADC1,ADC_Channel_0);
				ppm = calculatePPM(adc_ch0);

				CLCD_I2C_SetCursor(&LCD1, 0, 0);
				CLCD_I2C_WriteString(&LCD1,"PPM:");

				CLCD_I2C_SetCursor(&LCD1, 4, 0);
				sprintf(buffer_ppm,"%.2f",ppm);
				CLCD_I2C_WriteString(&LCD1,buffer_ppm);

				CLCD_I2C_SetCursor(&LCD1, 0, 1);
				CLCD_I2C_WriteString(&LCD1,"State: 1");

				control_OUTPUT(ppm);

				controlLED(ppm);
				sendPPM(ppm);
		}

			if (flag_exti0==1 && systemState == 0) {


				CLCD_I2C_SetCursor(&LCD1, 0, 0);
				CLCD_I2C_WriteString(&LCD1,"PPM:");

				CLCD_I2C_SetCursor(&LCD1, 4, 0);
				sprintf(buffer_ppm,"%.2f",ppm);
				CLCD_I2C_WriteString(&LCD1,buffer_ppm);

				CLCD_I2C_SetCursor(&LCD1, 0, 1);
				CLCD_I2C_WriteString(&LCD1,"State: 0");

				GPIO_WritePin(GPIOA, 1 ,0);
				GPIO_WritePin(GPIOA, 2, 0);
				GPIO_WritePin(GPIOA, 3, 0);
				GPIO_WritePin(GPIOA, 4, 0);
				GPIO_WritePin(GPIOA, 5, 0);
				GPIO_WritePin(GPIOA,6,1);  // khi he thong o trang thai dung thi LED_GREEN ON
				TIM2->CR1 &= ~(1 << 0);
				TIM2->DIER &= ~(1 << 0);
				flag_exti0=0;
			}
			if(reset==1){
				resetSystem();
				reset=0;
			}

			// nhan data tu ESP32 qua UART

			if (USARTx_GetS(USART1, BUFFER, 100))
				{
					GPIO_WritePin(GPIOB,2,1);
						if (strncmp(BUFFER, "TIME:", 5) == 0) {
								if (strlen(BUFFER) <= 40) {
										//saveLogToFlash(BUFFER);
								}
						}
						GPIO_WritePin(GPIOB,2,0);
				}


  /* USER CODE END 3 */
}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
