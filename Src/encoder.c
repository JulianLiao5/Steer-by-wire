/*
 * encoder.c
 *
 *  Created on: Apr 30, 2020
 *      Author: LiaoMeng
 */

#include "encoder.h"

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim3;

volatile int32_t gEncoderPeriodInt = 0;
uint16_t gEncoderUpdateTimes = 0;
int32_t gEncoderCounterDiff = 0;

volatile uint8_t gEncoderSendBusy = 0;
uint8_t gEncoderSendBuffer[9];
volatile uint8_t gEncoderSendCounter = 0;

volatile uint8_t gEncoderPeriodSendFlag = 0;
uint8_t gEncoderPeriodSendBuffer[] = "encdr1234";

#define Encoder_Times_MAX 1000

void Encoder_GetPeriod(void) {
	static int32_t _period_raw = 0;

	_period_raw = (gEncoderUpdateTimes << 16) + gEncoderCounterDiff;
	if (htim3.Instance->CR1 & TIM_CR1_DIR) {
		gEncoderPeriodInt = _period_raw;
	} else {
		gEncoderPeriodInt = -_period_raw;
	}
}

void Encoder_UpdateCounter(void) {
	static int32_t _counter_last = 0;
	static int32_t _counter = 0;
	_counter = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
	gEncoderCounterDiff = _counter - _counter_last;
	_counter_last = _counter;

	Encoder_GetPeriod();
	gEncoderUpdateTimes = 0;
}

void Encoder_UpdateTimer(void) {
	if (gEncoderUpdateTimes < Encoder_Times_MAX) {
		gEncoderUpdateTimes++;
	}
	if (gEncoderUpdateTimes == Encoder_Times_MAX) {
		Encoder_GetPeriod();
		gEncoderUpdateTimes++;
	}

}

void Encoder_Send_Init(uint8_t _buf[], int32_t _val) {
	static uint8_t __i = 0;
	for (__i = 0; __i < 5; __i++) {
		gEncoderSendBuffer[__i] = _buf[__i];
	}
	for (__i = 0; __i < 4; __i++) {
		gEncoderSendBuffer[5 + __i] = _val >> (8 * (3 - __i));
	}
	gEncoderSendCounter = 0;
	gEncoderSendBusy = 1;
	huart2.Instance->DR = gEncoderSendBuffer[0] & 0xff;
}

void Encoder_Send_Handle(void) {
	gEncoderSendCounter++;
	if (gEncoderSendCounter < 9) {
		gEncoderSendBusy = 1;
		huart2.Instance->DR = gEncoderSendBuffer[gEncoderSendCounter] & 0xff;
	} else {
		gEncoderSendBusy = 0;
	}
}
