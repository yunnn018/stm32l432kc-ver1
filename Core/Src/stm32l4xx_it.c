/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <control1.h>
#include <math.h>
#include <tim.h>

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
extern volatile uint32_t msecCnt;
extern uint16_t Theta_e;
extern uint16_t wm_e;
extern uint16_t wm_cmd;
extern Currents adc_avg;
extern Currents Ioffset;

//extern volatile uint8_t hall_u_old, hall_u,hall_v_old, hall_v,hall_w_old, hall_w;
//extern volatile uint16_t speed_count, speed, theta_add;
//extern volatile int32_t theta_ini, theta_hall, speed_old;
//extern volatile int32_t speed_buf[12];
//extern volatile int32_t speed_avg;
//extern volatile uint16_t wm_rpm;
//extern volatile uint8_t buffer_count, a;

extern uint8_t hall_u, hall_v, hall_w, hall_u_old, hall_v_old, hall_w_old;
extern uint16_t  speed, speed_old, speed_count;
extern uint16_t theta_ini, theta_add;
extern uint16_t speed_buf[12], buffer_count;
extern int16_t wm_hall;
extern int16_t theta_hall, speed_avg;
extern uint8_t hall_index;
static uint16_t speed_sum;
extern uint16_t spc;
extern const uint16_t HALL_ANGLE_TABLE[8];

extern uint16_t I0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim7;
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
   while (1)
  {
  }
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
  * @brief This function handles Prefetch fault, memory access fault.
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
	if(msecCnt < 20000) //20秒
		msecCnt++;
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles ADC1 global interrupt.
  */
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */

  /* USER CODE END ADC1_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_IRQn 1 */

  /* USER CODE END ADC1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM15 global interrupt.
  */
void TIM1_BRK_TIM15_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM15_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM15_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  adc_avg = ADC_AverageValue();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
  if(msecCnt >= 3000 && msecCnt <= 4000)
  {
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  }

  if(msecCnt > 4000 && msecCnt < 4500)
  {
	  Vdq.vd = 180;
	  Vdq.vq = 0;
	  VAlphaBeta = Rev_Park(Vdq,0);
	  CALC_SVPWM(&VAlphaBeta);

  }

  if (msecCnt > 5000)
  {
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	  Iab = Get_Phase_Currents(&adc_avg, &Ioffset);
	  IAlphaBeta = Clarke(&Iab);
	  Idq = Park(&IAlphaBeta,Theta_e);
	  Vdq.vd = PID_Regulator(0,Idq.id,&Flux);
	  Vdq.vq = PID_Regulator(Iq_ref,Idq.iq,&Torque);
//	  Vdq.vd = 500;
//	  Vdq.vq = 0;
	  VAlphaBeta = Rev_Park(Vdq,Theta_e);
	  if(msecCnt > 6500) Qi = Qinner(Vdq, &Idq, wm_e*24/10);	//
	  CALC_SVPWM(&VAlphaBeta);
  }

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts.
  */
void TIM1_TRG_COM_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_TRG_COM_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
//  /* 讀取 GPIO 狀態 */
//  hall_u = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
//  hall_v = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
//  hall_w = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
//
//  if ((hall_u_old != hall_u) || (hall_v_old != hall_v) || (hall_w_old != hall_w))
//  {
//      /* 霍爾狀態改變，更新舊值 */
//      hall_u_old = hall_u;
//      hall_v_old = hall_v;
//      hall_w_old = hall_w;
//
//      /* 根據三相霍爾輸入判斷初始角度 */
//      if (hall_u == 0 && hall_v == 0 && hall_w == 1)
//      {
//          theta_ini = 1666;  //  60°
//      }
//      else if (hall_u == 0 && hall_v == 1 && hall_w == 1)
//      {
//          theta_ini = 3333;  // 120°
//      }
//      else if (hall_u == 0 && hall_v == 1 && hall_w == 0)
//      {
//          theta_ini = 5000;  // 180°
//      }
//      else if (hall_u == 1 && hall_v == 1 && hall_w == 0)
//      {
//          theta_ini = 6666;  // 240°
//      }
//      else if (hall_u == 1 && hall_v == 0 && hall_w == 0)
//      {
//          theta_ini = 8333;  // 300°
//      }
//      else if (hall_u == 1 && hall_v == 0 && hall_w == 1)
//      {
//          theta_ini = 0;     //   0°
//      }
//
//      /* 計算轉速 (RPM) */
//      if (speed_count == 0)
//      {
//          speed = 0;
//      }
//      else
//      {
//          speed = 40000 / speed_count;	//wm
//      }
//
//      theta_add   = 0;
//      speed_count = 0;
//
//      if (speed > 1000)
//      {
//          speed = speed_old;
//      }
//      else
//      {
//          speed_old = speed;
//      }
//      if(buffer_count==12)
//      {
//    	  buffer_count=0;
//      }
//      else
//      {
//    	  speed_buf[buffer_count]=speed;	//在timer7做平均
//    	  buffer_count++;
//      }
//  }
//  else
//  {
//      speed_count++;
//  }


    hall_u = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
    hall_v = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
    hall_w = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
    hall_index = hall_u + (hall_v << 1) + (hall_w << 2);

    if((hall_u_old != hall_u) || (hall_v_old != hall_v) || (hall_w_old != hall_w))
    {
    	hall_u_old = hall_u;
    	hall_v_old = hall_v;
    	hall_w_old = hall_w;
    	theta_ini = HALL_ANGLE_TABLE[hall_index];
    	theta_hall = theta_ini;

    	//讀取TIM2當前計數作為speed_count
    	speed_count = __HAL_TIM_GET_COUNTER(&htim6);

    	if(speed_count == 0)
    	{
    		speed = 0;
    	}
    	else
    	{
    		//假設以最快速度運轉，可以算出最大速度可以到2500000rpm (小馬達)，實際會根據speed count變化
    		speed = (1000000 * 10 / 4) / speed_count; // 2500000 = 60(degree)/0.000001(1cnt)/360(degree/s to RPS) * 60(RPS to RPM) / 4(pole pair)
    		spc = speed_count;
    	}

    	//限制速度範圍
    	if(speed > 1000) speed = speed_old;	//觀測速度上限
    	else speed_old = speed;

    	// 更新環形緩衝區並計算平均速度
    	speed_sum -= speed_buf[buffer_count]; // 減去舊值
    	speed_buf[buffer_count] = speed;     // 更新新值
    	speed_sum += speed;                  // 加上新值
    	buffer_count = (buffer_count + 1) % 12; // 環形緩衝區指針更新
    	speed_avg = speed_sum / 12;

    	//清零TIM2計數器
    	__HAL_TIM_SET_COUNTER(&htim6, 0);
    }
    /*** hallsensor檢測轉速 ***/
    if(speed_count == 0) speed_count = 1;
    theta_hall += (1666 * 125 / speed_count); //每格階梯1666，每隔時間(1000000/speed_count)/8000=125/speed_count
    theta_hall = (theta_hall % 10000 + 10000) % 10000;
    wm_hall = speed_avg;

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  if(msecCnt > 5000 )
  {
	  wm_e = (msecCnt-5000)*wm_cmd/1000;		//有個斜率到命令轉速 2秒到wm_cmd
	  if(wm_e >= wm_cmd) wm_e = wm_cmd; //wm_cmd命令轉速、ms時間
	  Theta_e += (wm_e*4*10/60); // 速度(rpm)/60 *極對數*10，1圈1萬/1000(1秒1000次)、角度累加到10000
	  if(Theta_e >= 10000)  Theta_e -= 10000; //角度累加，一圈10000  1ms加多少，一直累加到10000
	  else if(Theta_e < 0) Theta_e += 10000;
  }

  if(msecCnt >= 3500)
  {
	  adc_copy = adc_avg;
	  Ioffset = Update_Current_Offset(&adc_copy,&Idq,msecCnt);
  }

  if(msecCnt > 8000){
  			Iq_ref = I0 + PID_Regulator(0, Qi, &Q);
  		}

  __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
