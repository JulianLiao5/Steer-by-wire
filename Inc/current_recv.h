/*
 * current_recv.h
 *
 *  Created on: Apr 30, 2020
 *      Author: 01
 */

#ifndef INC_CURRENT_RECV_H_
#define INC_CURRENT_RECV_H_

#include "stm32f4xx_hal.h"

extern volatile uint8_t gCurrentReceiveCounter;
extern volatile uint8_t gCurrentReceiveFlag;

void Current_Receive_Handle(void);

#endif /* INC_CURRENT_RECV_H_ */
