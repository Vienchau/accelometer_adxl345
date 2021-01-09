/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "bno055_stm32.h"
#include <stdio.h>
#include <math.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t timestamp; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
struct __FILE{																															// structure for print data
	int handle;
};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Clear the busy fault when connecting BNO to STM32F4	
	
void I2C1_ClearBusyFlagErratum(I2C_HandleTypeDef *instance)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	int timeout =100;
	int timeout_cnt=0;

	// 1. Clear PE bit.
	instance->Instance->CR1 &= ~(0x0001);

	//  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
	GPIO_InitStruct.Mode         = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Alternate    = GPIO_AF4_I2C1;
	GPIO_InitStruct.Pull         = GPIO_PULLUP;
	GPIO_InitStruct.Speed        = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStruct.Pin          = GPIO_PIN_6;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	GPIO_InitStruct.Pin          = GPIO_PIN_7;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);


	// 3. Check SCL and SDA High level in GPIOx_IDR.
	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
	{
			timeout_cnt++;
			if(timeout_cnt>timeout)
					return;
	}

	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
	{
			//Move clock to release I2C
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_Delay(2);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

			timeout_cnt++;
			if(timeout_cnt>timeout)
					return;
	}

	// 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

	//  5. Check SDA Low level in GPIOx_IDR.
	while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
	{
			timeout_cnt++;
			if(timeout_cnt>timeout)
					return;
	}

	// 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	//  7. Check SCL Low level in GPIOx_IDR.
	while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
	{
			timeout_cnt++;
			if(timeout_cnt>timeout)
					return;
	}

	// 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	// 9. Check SCL High level in GPIOx_IDR.
	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
	{
			timeout_cnt++;
			if(timeout_cnt>timeout)
					return;
	}

	// 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

	// 11. Check SDA High level in GPIOx_IDR.
	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
	{
			timeout_cnt++;
			if(timeout_cnt>timeout)
					return;
	}

	// 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;

	GPIO_InitStruct.Pin = GPIO_PIN_6;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_7;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

	// 13. Set SWRST bit in I2Cx_CR1 register.
	instance->Instance->CR1 |= 0x8000;

	__nop();

	// 14. Clear SWRST bit in I2Cx_CR1 register.
	instance->Instance->CR1 &= ~0x8000;

	__nop();

	// 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
	instance->Instance->CR1 |= 0x0001;

	// Call initialization function.
	HAL_I2C_Init(instance);
}
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	for(volatile uint8_t i=0; i<255; i++) {													// Find BNO address and clear busy fault if any
		if (HAL_I2C_IsDeviceReady(&hi2c1,i,1,10) == HAL_OK) {
			//break;
		}
		else 
			I2C1_ClearBusyFlagErratum(&hi2c1);
	}
	
	bno055_assignI2C(&hi2c1);
	//bno055_setup_alternate();
	bno055_setup();
	bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);

	bno055_vector_t euler, lin_accel, gyro, mag;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	char txData[40];
	
	while (1)
	{
		timestamp = HAL_GetTick();
		euler= bno055_getVectorEuler();
		lin_accel = bno055_getVectorLinearAccel();
		gyro = bno055_getVectorGyroscope();
		//mag = bno055_getVectorMagnetometer();
		/*
		if (roll != old_roll || pitch != old_pitch) {																			// Check available data (Updated data)
			printf("\tTime: %d \tEuler(R-P-Y): %.3f %.3f\n",HAL_GetTick(),roll,pitch);			// Debugging and output data
			sprintf(txData, "%d,%.4f,%.4f,%.4f",HAL_GetTick() roll, pitch, yaw);
			//HAL_Delay(2000);
			old_roll = roll; old_pitch = pitch;																						// Update filter
		}*/
		
		//printf("\tTime: %d \tEuler(R-P-Y): %.3f %.3f\n",HAL_GetTick(),roll,pitch);			// Debugging and output data
		sprintf(txData, "%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", timestamp, euler.y, euler.z, euler.x, lin_accel.x, lin_accel.y, lin_accel.z, gyro.x, gyro.y, gyro.z);
		
		//sprintf(txData, "%d,%.4f,%.4f,%.4f\n", timestamp, roll, pitch, yaw);
		//printf("Heading: %.2f Roll: %.2f Pitch: %.2f\r\n", yaw, roll, pitch);
		
		//HAL_Delay(500);
		//v = bno055_getVectorQuaternion();
		//printf("W: %.2f X: %.2f Y: %.2f Z: %.2f\r\n", v.w, v.x, v.y, v.z);
		//sprintf(txData, "%d,%.4f,%.4f,%.4f,%.4f\n", timestamp, v.w, v.x, v.y, v.z);
		
		HAL_UART_Transmit(&huart2, (uint8_t *)txData, strlen(txData),10);
		
		//HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
	/*send text over SWV*/
int fputc(int ch, FILE *f) {															// Method for print by ITM block in STM32F4
			ITM_SendChar(ch);																			//send method for SWV
			return(ch);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
