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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <control1.h>
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

Currents   IAlphaBeta;
Currents   Ioffset;
Currents   adc_avg;
Currents   adc_copy;
Currents   Iab;
Currents   Idq;
Voltages   VAlphaBeta;
Voltages   Vdq;

uint32_t msecCnt = 0;
PID_Struct Torque;
PID_Struct Flux;
PID_Struct Speed;
PID_Struct PLL_w;
PID_Struct Q;
uint16_t I0;
int16_t Iq_ref;
uint16_t theta=0, wm_e=0, Theta_e = 0;
uint16_t wm_cmd = 300;

int16_t Qi;

//volatile uint8_t hall_u_old, hall_u,hall_v_old, hall_v,hall_w_old, hall_w;
//volatile uint16_t speed_count = 0, speed = 0, theta_add = 0;
//volatile int32_t theta_ini = 0, theta_hall = 0, speed_old = 0;
//volatile int32_t speed_buf[12] = {0};
//volatile int32_t speed_avg = 0;
//volatile uint16_t wm_rpm = 0;
//volatile uint8_t buffer_count = 0, a = 0;

uint8_t hall_u, hall_v, hall_w, hall_u_old, hall_v_old, hall_w_old;
uint16_t  speed=0, speed_old=0, speed_count=0;
uint16_t theta_ini=0, theta_add=0;
uint16_t speed_buf[12]={0}, buffer_count=0;
int16_t wm_hall=0;
int16_t theta_hall=0, speed_avg=0;
uint8_t hall_index=0;

uint16_t spc=0;
const uint16_t HALL_ANGLE_TABLE[8]={0, 8333, 5000, 6666, 1666, 9999, 3333, 0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

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
  HAL_Delay(2999);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

//  HAL_ADC_Start_DMA(&hadc1,DataBuffer,BATCH_DATA_LEN);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)DataBuffer, BATCH_DATA_LEN);

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim2); // 開啟中斷
//  HAL_TIMEx_HallSensor_Start_IT(&htim2);


  PID_Init(&Torque,&Flux,&Speed,&PLL_w,&Q);
//  uint16_t a, b, c;
  I0 = 300;
  Iq_ref = I0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  a=__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
//	  b=__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
//	  c=__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
//	  printf(",%d, %d, %d\r\n", a,b,c);
//	  printf(",%d ,%d\r\n", adc_avg.IADC1,adc_avg.IADC2);
//	  printf("%d ,%d ,%d\r\n", Ioffset.IADC1,Ioffset.IADC2 ,msecCnt);

//	  printf(",%d,%d,%lu,%d\r\n",Idq.id,Idq.iq,msecCnt,Theta_e);

//	  printf(",%d,%d,%d,%d\r\n",Iab.ia,Iab.ib,msecCnt,Theta_e);
//	  printf(",%d,%d,%d\r\n",Vdq.vd,Vdq.vq,msecCnt);
//	  printf(",%d,%d,%d\r\n",IAlphaBeta.ialpha,IAlphaBeta.ibeta,msecCnt);
	  printf(",%d,%d,%d,%d,%d\r\n",IAlphaBeta.ialpha,IAlphaBeta.ibeta,Idq.id,Idq.iq,msecCnt);
//	  printf(",%d,%d,%d\r\n",wm_rpm,theta_hall,msecCnt);
//	  printf(",%d,%d,%d,%d\r\n",wm_hall,theta_hall,msecCnt,speed_count);
//	  printf("%d,%d,%d,%d,%d\r\n",msecCnt, Idq.id, Idq.iq, Qi, Iq_ref);
//
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 36;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, 100);
  return len;
}


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
