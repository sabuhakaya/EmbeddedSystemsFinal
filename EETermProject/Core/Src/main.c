/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LCD.h"
#include<stdio.h>
#include<stdbool.h>
#include <string.h>
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
 ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */




#define IN1Port in1_GPIO_Port
#define IN1Pin in1_Pin

#define IN2Port in2_GPIO_Port
#define IN2Pin  in2_Pin

#define IN3Port in3_GPIO_Port
#define IN3Pin  in3_Pin

#define IN4Port in4_GPIO_Port
#define IN4Pin  in4_Pin

#define stepsperrev 2048


/* To produce us Delay */
void delay (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

/*set the RPM for the Stepper */
void stepper_set_rpm (int rpm)  // Set rpm--> max 13, min 1,,,  went to 14 rev/min
{
	delay(60000000/stepsperrev/rpm);
}


/*Set a particular pin */
void SetPin (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

/*Reset a particular pin */
void ResetPin (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

/* Wave Drive for Stepper
 * It Energizes 1 Electromagnet at a time
 * 1 revolution is 2048 steps = 512 Sequences
 * Here Sequence stands for one complete loop inside this function
 * As there are 4 steps in one loop/sequence, 2048 steps = 512 sequences
 */
void stepper_wave_drive (int step)
{
	switch (step){
		case 0:
			  SetPin(IN1Port, IN1Pin);   // IN1 SET
			  ResetPin(IN2Port, IN2Pin);   // IN2 RESET
			  ResetPin(IN3Port, IN3Pin);   // IN3 RESET
			  ResetPin(IN4Port, IN4Pin);   // IN4 RESET
			  break;

		case 1:
			  ResetPin(IN1Port, IN1Pin);   // IN1 RESET
			  SetPin(IN2Port, IN2Pin);   // IN2 SET
			  ResetPin(IN3Port, IN3Pin);   // IN3 RESET
			  ResetPin(IN4Port, IN4Pin);   // IN4 RESET
			  break;

		case 2:
			  ResetPin(IN1Port, IN1Pin);   // IN1 RESET
			  ResetPin(IN2Port, IN2Pin);   // IN2 RESET
			  SetPin(IN3Port, IN3Pin);   // IN3 SET
			  ResetPin(IN4Port, IN4Pin);   // IN4 RESET
			  break;

		case 3:
			  ResetPin(IN1Port, IN1Pin);   // IN1 RESET
			  ResetPin(IN2Port, IN2Pin);   // IN2 RESET
			  ResetPin(IN3Port, IN3Pin);   // IN3 RESET
			  SetPin(IN4Port, IN4Pin);   // IN4 SET

		}
}

/* Step the motor by some particular angle
 * DIRECTION: 0 -> CK, 1 -> CCK
 */

void stepper_step_angle (float angle, int direction, int rpm)
{
	float anglepersequence = 0.703125;  // 360 degrees= 512 sequences
	int numberofsequences = (int) (angle/anglepersequence);

	for (int seq=0; seq<numberofsequences; seq++)
	{
		if (direction == 0)  // for clockwise
		{
			for (int step=7; step>=0; step--)
			{
				stepper_wave_drive(step);
				stepper_set_rpm(rpm);
			}

		}

		else if (direction == 1)  // for Counter-clockwise
		{
			for (int step=0; step<8; step++)
			{
				stepper_wave_drive(step);
				stepper_set_rpm(rpm);
			}
		}
	}
}


uint16_t ADC_VAL;
float voltage;
float angle;
float currentAngle = 0;

/* Rotate the motor */
void Stepper_rotate (int angle, int rpm)
{
	int changeinangle = 0;
	changeinangle = angle-currentAngle;  // calculate the angle by which the motor needed to be rotated
	if (changeinangle > 2)  // clockwise
	{
		stepper_step_angle (changeinangle,0,rpm);
		currentAngle = angle;  // save the angle as current angle
	}
	else if (changeinangle <-2) // CCK
	{
		changeinangle = -(changeinangle);
		stepper_step_angle (changeinangle,1,rpm);
		currentAngle = angle;
	}
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t adcVal0, adcVal1;
ADC_ChannelConfTypeDef sConfig;
bool lamps[] = {0,0};
int timeTick=0;
char msg[32];
char tem1[32];
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);
	
  lcd_init(_LCD_4BIT, _LCD_FONT_5x8, _LCD_2LINE);

  lcd_print(1,1,"Displaying Values");
  HAL_Delay(100);
  lcd_clear();
  /* Initializes SHT2x temperature/humidity sensor and sets the resolution. */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(timeTick<1000){
			sConfig.Channel=ADC_CHANNEL_0;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			HAL_ADC_Start(&hadc1);
			
			if(HAL_ADC_PollForConversion(&hadc1, 5)==HAL_OK)
			{
				adcVal0=HAL_ADC_GetValue(&hadc1);
				adcVal0=(adcVal0*500)/4096;
				//adcVal0=(adcVal0/4095+0.095)/0.009;
				HAL_Delay(50);
				lcd_clear();
				char tem[32]="";
				sprintf(tem,"%lu", adcVal0);
				lcd_print(1,1,"Temperature");
				lcd_print(1,13,tem);
				HAL_Delay(50);
			}
			else{
				lcd_print(1,1,"Temp.Sensor Corrupt ");
				HAL_Delay(50);
			}
			HAL_ADC_Stop(&hadc1);
			HAL_Delay(50);
			
			char lmp[32]="";
			sprintf(lmp,"Lamps: %s %s",lamps[0] ? "ON":"OFF", lamps[1]? "ON":"OFF");
			lcd_print(2,1,lmp);
		 
			//char tim[32]="";
			//sprintf(tim,"        Time Tick: %d", timeTick);
			//lcd_print(4,1,tim);
			//HAL_Delay(50);
			
			char msg[32]="";
			sprintf(msg,"Temperature=%lu Celcius\r\n", adcVal0);
			HAL_UART_Transmit(&huart1,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
			memset(msg, 0, 32); // reset msg
			sprintf(msg,"Lamp0=%s \r\n", lamps[0] ? "ON":"OFF");
			HAL_UART_Transmit(&huart1,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
			memset(msg, 0, 32); // reset msg
			sprintf(msg,"Lamp1=%s \r\n", lamps[1] ? "ON":"OFF");
			HAL_UART_Transmit(&huart1,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
			memset(msg, 0, 32); // reset msg
			sprintf(msg,"TimeTick=%d s\r\n", timeTick);
			HAL_UART_Transmit(&huart1,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
		 
			HAL_Delay(10);
		}
		else{  
			  lcd_clear();
				lcd_print(1,1,"System on Sleep");
				HAL_Delay(50);
			  lamps[0]=false;
				lamps[1]=false;
        HAL_GPIO_WritePin(lamp0_GPIO_Port, lamp0_Pin, GPIO_PIN_RESET ); // Toggle The Output (LED) Pin
        HAL_GPIO_WritePin(lamp1_GPIO_Port, lamp1_Pin, GPIO_PIN_RESET ); // Toggle The Output (LED) Pin
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 90;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_EN_Pin|LCD_RS_Pin|LCD_D4_Pin|LCD_D5_Pin
                          |LCD_D6_Pin|LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, in1_Pin|in2_Pin|in3_Pin|in4_Pin
                          |lamp0_Pin|lamp1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_EN_Pin LCD_RS_Pin LCD_D4_Pin LCD_D5_Pin
                           LCD_D6_Pin LCD_D7_Pin */
  GPIO_InitStruct.Pin = LCD_EN_Pin|LCD_RS_Pin|LCD_D4_Pin|LCD_D5_Pin
                          |LCD_D6_Pin|LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : input0_Pin input1_Pin */
  GPIO_InitStruct.Pin = input0_Pin|input1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : pir_Pin UP_Pin DOWN_Pin */
  GPIO_InitStruct.Pin = pir_Pin|UP_Pin|DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : in1_Pin in2_Pin in3_Pin in4_Pin
                           lamp0_Pin lamp1_Pin */
  GPIO_InitStruct.Pin = in1_Pin|in2_Pin|in3_Pin|in4_Pin
                          |lamp0_Pin|lamp1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == input0_Pin) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
      HAL_GPIO_TogglePin(lamp0_GPIO_Port, lamp0_Pin); // Toggle The Output (LED) Pin
			lamps[0]=!lamps[0];
			timeTick=0;
    }
		else if(GPIO_Pin == input1_Pin) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
      HAL_GPIO_TogglePin(lamp1_GPIO_Port, lamp1_Pin); // Toggle The Output (LED) Pin
			lamps[1]=!lamps[1];
			timeTick=0;
    }
		else if(GPIO_Pin == pir_Pin) // If The INT Source Is EXTI Line9 (A9 Pin)
		{
			/// Motor down
			timeTick=0;
		}
		/// HIGH PRESCELAR INTERRUPTS
		else if(GPIO_Pin == UP_Pin) // If The INT Source Is EXTI Line9 (A9 Pin)
		{
			Stepper_rotate(10,10);
			timeTick=0;
			/// Motor up
		}
		else if(GPIO_Pin == DOWN_Pin) // If The INT Source Is EXTI Line9 (A9 Pin)
		{
			Stepper_rotate(-10,10);
			timeTick=0;

			/// Motor down
		}	

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    timeTick++;
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
