/*
 * control.c
 *
 *  Created on: Apr 30, 2020
 *      Author: 01
 */

#include "control.h"

uint8_t gCurrentPolarity = _Current_Pos;

volatile uint8_t gCurrentUpdate = 0;
volatile uint8_t gVoltageUpdate = 0;
