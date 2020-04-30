/*
 * encoder.h
 *
 *  Created on: Apr 30, 2020
 *      Author: LiaoMeng
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "stm32f4xx_hal.h"


extern volatile uint8_t gEncoderPeriodSendFlag;
extern volatile uint8_t gEncoderSendBusy;

extern volatile uint8_t gEncoderPeriodSendBuffer[];

void Encoder_UpdateCounter(void);
void Encoder_UpdateTimer(void);
void Encoder_Send_Init(uint8_t _buf[], int32_t _val);
void Encoder_Send_Handle(void);

#endif /* INC_ENCODER_H_ */
