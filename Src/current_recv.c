/*
 * current_recv.c
 *
 *  Created on: Apr 30, 2020
 *      Author: LiaoMeng
 */

#include "current_recv.h"

extern UART_HandleTypeDef huart2;

void Current_Receive_Handle(void)
{
  static uint16_t _data = 0;
  _data = (huart2.Instance->DR) & 0xFF;

}
