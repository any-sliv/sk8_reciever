/*
 * app.c
 *
 *  Created on: 21 Feb 2021
 *      Author: mjsli
 */

#include "stm32f1xx_hal.h"
#include "MY_NRF24.h"
#include "app.h"

#define RX_SIZE 8
#define TX_SIZE 8
#define FILTER_VAL 	10
#define DC_HOLD 	100

extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim4;

/* RADIO VARIABLES 	*/
const uint64_t TxpipeAddress = 0xBEEFDEAD;
const uint64_t RxpipeAddress = 0x11223344AA;
char rxData[RX_SIZE];
char txData[TX_SIZE];
/* 					*/

uint16_t throttleTim = 0;
uint8_t batStat = 0;
uint8_t tim3Cnt = 0;
uint16_t dcCnt = 0;
uint8_t tim4Cnt = 0;
const uint16_t throttleIdle = 2636;
uint64_t tim3Sum;
bool dcFlag;

char uartBuffer[16];
uint32_t adcData[3];
sm_stamp timer;

static void SetThrottle(uint8_t *data);
static uint16_t throttleConversion(uint8_t *adc);


void AppInit(void)
{
	HAL_ADC_Start_DMA(&hadc1, adcData, 3);
	HAL_Delay(1);

	NRF24_begin(GPIOB, GPIO_PIN_2, GPIO_PIN_1, hspi1);

	NRF24_openReadingPipe(16, RxpipeAddress);
	NRF24_setAutoAck(true);

	NRF24_setPayloadSize(32);
	NRF24_setChannel(56);
	NRF24_setDataRate(RF24_250KBPS);
	NRF24_startListening();
}


void AppLoop(void)
{
	if(NRF24_available())
	{
		NRF24_read(rxData, 1);
	}

	sprintf(uartBuffer, "%u", htim4.Instance->CCR1);
	HAL_UART_Transmit(&huart2, (uint8_t *) uartBuffer, 32, 1);
	HAL_UART_Transmit(&huart2, (uint8_t *) uartBuffer, 32, 1);
}


static void SetThrottle(uint8_t *data)
{
	//throttleTim = throttleConversion(data);
	tim3Sum += throttleConversion(data);
	tim3Cnt++;

	if(tim3Cnt >= FILTER_VAL)
	{
		throttleTim = tim3Sum / FILTER_VAL;
		tim3Cnt = 0;
		tim3Sum = 0;

		if(!(dcFlag))
		{
			htim4.Instance -> CCR1 = (throttleTim * 1.25);
			dcCnt = 0;
		}
	}

	if(dcFlag)
	{
		dcCnt++;
		if(dcCnt >= DC_HOLD) htim4.Instance -> CCR1 = throttleIdle;
		else htim4.Instance -> CCR1 = (throttleTim * 1.25);

		if(dcCnt > 65500) dcCnt = 65500;
	}

	if((htim4.Instance -> CCR1) > 3500) htim4.Instance -> CCR1 = 3500;
	if((htim4.Instance -> CCR1) < 2000) htim4.Instance -> CCR1 = 2000;
}


static uint16_t throttleConversion(uint8_t *adc)
{
	uint16_t thr = 0;
	thr += ((uint8_t)adc[0] - '0') * 1000;
	thr += ((uint8_t)adc[1] - '0') * 100;
	thr += ((uint8_t)adc[2] - '0') * 10;
	thr += ((uint8_t)adc[3] - '0') * 1;
	return thr;
}
