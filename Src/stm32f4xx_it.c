/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#include "encoder.h"
#include "current_recv.h"

void Encoder_UpdateCounter(void);
void Encoder_UpdateTimer(void);

int sampl,pulse1;
extern float rpm;
extern float pid;
extern float sp;
extern float SP;
float error,Kp=1,Ki=0,Kd=2.5,P,I,integral,derivativ,pre_error,dt=0.01,D;
int j;
extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart2;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	sampl++;
	// 10字节的buffer，这个数据会被传输给UART
	char msg[10];
	sprintf(msg, "%hu\r\n", sampl);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	if (sampl == 1000)  // every 1000ms or 1sec count freq then rpm
	{
		rpm = (float) pulse1 / 4 * 60;
		sampl = 0;
		pulse1 = 0;
	}
	if (sampl >= 500) {
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	}

	j++;
	if (j >= 10) {
		error = SP - (float) pulse1 / 4 * 60;
		integral += (error * dt);
		derivativ = (error - pre_error) / dt;
		P = Kp * error;
		I = Ki * integral;
		D = Kd * derivativ;
		pid = pid + P + I + D;

		pre_error = error;

		if (pid > sp) {
			pid = sp;
		} else if (pid < 0) {
			pid = 0;
		}

		TIM2->CCR3 = pid * 999 / 1380;
		j = 0;
	}
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */
/**
* @brief This function handles TIM214 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */
	if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE))
	{
		__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
		Encoder_UpdateTimer();
	}
	if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC1))
	{
		__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC1);
		Encoder_UpdateCounter();
	}
	return;
  /* USER CODE END TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM14_IRQn 1 */

  /* USER CODE END TIM14_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	//error detection
	//needs to reset receive state machine when error occurs
	if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE))
	{
		__HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF);
		gCurrentReceiveCounter = 0;
	}
	if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_FE))
	{
		__HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_FEF);
		gCurrentReceiveCounter = 0;
	}
	if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_PE))
	{
		__HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_PEF);
		gCurrentReceiveCounter = 0;
	}
	if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC))
	{
		__HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_TC);
		Encoder_Send_Handle();
	}
	if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE))
	{
		Current_Receive_Handle();
	}

	return;
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
