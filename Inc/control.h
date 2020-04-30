/*
 * control.h
 *
 *  Created on: Apr 30, 2020
 *      Author: 01
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "stm32f4xx_hal.h"

enum{_Current_Pos,_Current_Neg};

extern uint16_t gCurrentInt;
extern uint16_t gCurrentCompare;
extern uint8_t gCurrentPolarity;

#endif /* INC_CONTROL_H_ */
