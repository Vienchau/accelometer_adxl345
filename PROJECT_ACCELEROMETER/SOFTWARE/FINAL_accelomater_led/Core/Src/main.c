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
#include "i2c.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
int16_t x,y,z;
float xg,yg,zg;
uint8_t data_org[6];
uint8_t devid =0;
uint16_t data3[] = {0x0000,0x0001, 0x0003, 0x0007, 0x000F, 0x001F, 0x003F,0x007F, 0x00FF, 0x01FF, 0x03FF, 0x07FF, 0x0FFF,  0x1FFF, 0x3FFF, 0x7FFF, 0xFFFF};
uint32_t data1[] = {0x000001, 0x000003, 0x000007, 0x00000F, 0x00001F, 0x00003F,0x00007F, 0x0000FF, 0x0001FF, 0x0003FF, 0x0007FF, 0x000FFF, 0x001FFF, 0x003FFF, 0x007FFF, 0x00FFFF, 0X01FFFF, 0x03FFFF, 0x07FFFF, 0x0FFFFF, 0x1FFFFF, 0x3FFFFF,0x7FFFFF, 0XFFFFFF};
	  uint32_t down_out = 0x003800;
	  uint32_t up_out = 0xC00004;
	  uint32_t right_out = 0x0F0000;
	  uint32_t left_out = 0x0001E0;
	  uint16_t down_in = 0x01C0;
	  uint16_t up_in = 0x2003;
	  uint16_t right_in  = 0x0018;
	  uint16_t left_in = 0x0C00;
uint8_t i1,i2, i3, i4, i5, i6, i7, i8, i9, i10, i11, i12;
uint8_t count =0;
	  
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#define adxl_address (0x53<<1)
	void led_effect_1(void);
	void led_effect_2(void);
	void down_to_left (void);
	void down_to_right (void);
    void down_to_up (void);
void up_to_left (void);
void up_to_right (void);
void up_to_down(void);
void right_to_up (void);
void right_to_down (void);
void right_to_left (void);
void left_to_up (void);
void left_to_down (void);
void left_to_right (void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void adxl_reg_write(uint8_t reg, uint8_t value)
{
	uint8_t data[2];
	data[0]= reg;
	data[1]=value;
	HAL_I2C_Master_Transmit(&hi2c1, (adxl_address|0), data, sizeof(data), 100);
}
void adxl_read_value(uint8_t reg)
{
	
	//hi2c1.Instance->CR1 &= ~(0x0001);	
//	hi2c1.Instance->CR1 &= ~(0x0500);	
	HAL_I2C_Mem_Read(&hi2c1, (adxl_address|1), reg, 1, (uint8_t *)data_org, 6, 100);
}


void adxl_set_up(void)
{

	HAL_I2C_Mem_Read(&hi2c1, (adxl_address|1), 0x00, 1, &devid, 6, 100);
	adxl_reg_write(0x31, 0x01);//+-4g
	adxl_reg_write(0x2D, 0x00);//reset the power control
	adxl_reg_write(0x2D, 0x08);//wake up as 8hz and measure mode
	
}
void shift_register( uint8_t *data, uint16_t size)
{
	//set the latch pin low util all bit was shifted

	HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);
	// shift the bits
	while(HAL_SPI_Transmit(&hspi1, data, size , 300) != HAL_OK);
	//set the latch pin high to transmit
	HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_SET);
	//done
}
void shift_register2( uint8_t *data, uint16_t size)
{
	//set the latch pin low util all bit was shifted

	HAL_GPIO_WritePin(LATCH2_GPIO_Port, LATCH2_Pin, GPIO_PIN_RESET);
	// shift the bits
	while(HAL_SPI_Transmit(&hspi2, data, size , 300) != HAL_OK);
	//set the latch pin high to transmit
	HAL_GPIO_WritePin(LATCH2_GPIO_Port, LATCH2_Pin, GPIO_PIN_SET);
	//done
}
void I2C1_ClearBusyFlagErratum(I2C_HandleTypeDef *instance)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	int timeout =100;
	int timeout_cnt=0;	

	// 1. Clear PE bit.
	instance->Instance->CR1 &= ~(0x0001);

	//  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
	GPIO_InitStruct.Mode         = GPIO_MODE_OUTPUT_OD;
	//GPIO_InitStruct.Alternate    = GPIO_AF1_I2C1;
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
	//GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;

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
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
//for(volatile uint8_t i=0; i<255; i++) 
//		{
//	  // Find BNO address and clear busy fault if any
//		if (HAL_I2C_IsDeviceReady(&hi2c1,i,1,10) == HAL_OK) {
//			//break;
//		}
//		else 
//			I2C1_ClearBusyFlagErratum(&hi2c1);
//	}
	
	adxl_set_up();
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, 0);
	HAL_Delay(100);
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(RST2_GPIO_Port, RST2_Pin, 0);
	HAL_Delay(100);
	HAL_GPIO_WritePin(RST2_GPIO_Port, RST2_Pin, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 if(count ==0)
		 {
	  int16_t x1 ,y1 ;
	   adxl_read_value(0x32);
	  x = ((data_org[1]<<8)|data_org[0]);
	  y = ((data_org[3]<<8)|data_org[2]);
	  z = ((data_org[5]<<8)|data_org[4]);	  
	
	  if(y > 45 )
	  {
		  HAL_Delay(1000);
		  adxl_read_value(0x32);
			x1 = ((data_org[1]<<8)|data_org[0]);
			y1 = ((data_org[3]<<8)|data_org[2]);
			
		  if ((x1 > 65)  && (i1 != 6) )
		  {
			  down_to_left();
			  i2= i3 = i4= i5 = i6 = i7= i8 = i10 =i9 = i11 = i12 =0;
			  //break;
		  }
		  if ((x1 < -45)   && ( i2 != 6))
		  {
			  down_to_right();
			  i1= i3 = i4= i5 = i6 = i7= i8 = i10 =i9 = i11 = i12 =0;
			 // break;
		  }
		   if ((y1 > 45) && (z <30))
		  {
			shift_register2((uint8_t *) &down_out, 3);
			shift_register((uint8_t *) &down_in, 2);
			  //break;
		  }
		  if ((y1 < -45) && (i9 != 1))
		  {
			  down_to_up();
			  i1=i2 = i3 = i4= i5 = i6 = i7= i8 = i10 = i11 = i12 = 0;
			  //break;
		  }
			  

	  }
	  
	  if(x  > 45 )
	  {
		  HAL_Delay(1000);
		  adxl_read_value(0x32);
			x1 = ((data_org[1]<<8)|data_org[0]);
			y1 = ((data_org[3]<<8)|data_org[2]);	
		  
		  if ((x1 > 45)  )
		  {
			  shift_register2((uint8_t *) &left_out, 3);
			shift_register((uint8_t *) &left_in, 2);
			  //break;
		  }
		  if ((x1 < -65) && ( i12 != 1))
		  {
			  left_to_right();
			  i1= i2= i3 = i4= i5 = i6 = i7= i8 = i9 = i10 = i11 =0;
			//  break;
		  }
		   if ((y1 > 55) && (i8 != 5))
		  {
			left_to_down();
			  i1=i2= i3 = i4= i5 = i6 = i7= i12 = i9 = i10 = i11 =0;
			 // break;
		  }
		  if ((y1 < -45) && (i7 != 5))
		  {
			  left_to_up();
			  i1= i2 = i3 = i4= i5 = i6 = i9= i8 = i10 = i11 = i12 = 0;
			 // break;
		  }
			  

	  }
	  
	   if(y < -45 )
	  {
		  HAL_Delay(1000);
		  adxl_read_value(0x32);
			x1 = ((data_org[1]<<8)|data_org[0]);
			y1 = ((data_org[3]<<8)|data_org[2]);
			
		  if ((x1 > 45) && (i3 != 6) )
		  {
			  up_to_left();
			  i2= i1 = i4= i5 = i6 = i7= i8 = i10 =i9 = i11 = i12 =0;
			 // break;
		  }
		  if ((x1 < -45) && ( i4 != 6))
		  {
			  up_to_right();
			  i1= i3 = i2= i5 = i6 = i7= i8 = i10 =i9 = i11 = i12 =0;
			  //break;
		  }
		   if ((y1 > 45) && (i10 != 1))
		  {
			up_to_down();
			  i1= i3 = i4= i5 = i6 = i7= i8 = i2 =i9 = i11 = i12 =0;
			 // break;
		  }
		  if ((y1 < -45) )
		  {
			 shift_register2((uint8_t *) &up_out, 3);
			shift_register((uint8_t *) &up_in, 2);
			  //break;
		  }
		
	  }
	  
	  if(x  < -45 )
	  {
		  HAL_Delay(1000);
		  adxl_read_value(0x32);
			x1 = ((data_org[1]<<8)|data_org[0]);
			y1 = ((data_org[3]<<8)|data_org[2]);
			
		  if ((x1 > 65) && (i11!=1)  )
		  {
			right_to_left();
			  i1= i2= i3 = i4= i5 = i6 = i7= i12 = i9 = i10 = i8 =0;
			//  break;
		  }
		  if ((x1 < -65) && ( z > 40))
		  {
			 shift_register2((uint8_t *) &right_out, 3);
			shift_register((uint8_t *) &right_in, 2);
			 // break;
		  }
		   if ((y1 > 75) && (z >40)  && (i6 != 5))
		  {
			right_to_down();
			  i1=i2= i3 = i4= i5 = i8 = i7= i12 = i9 = i10 = i11 =0;
			  //break;
		  }
		  if ((y1 < -65) && (i5 != 5))
		  {
			  right_to_up();
			  i1=i2 = i3 = i4= i7 = i6 = i9= i8 = i10 = i11 = i12 = 0;
			  //break;
		  }
			  
		
	  } 
	  }
		 
	  if (count == 1)
		  {
					led_effect_1();
		  }
		 
		if(count == 2 )
			{
					led_effect_2();
					led_effect_2();
					led_effect_2();
					led_effect_2();
			}
		if(count ==3)
			{
				count = 0;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */
void led_effect_1(void)
{
	
		   int i;
	 for(i =0; i<=15; i++)
	{
		shift_register( (uint8_t *) &data3[i], 2);
		HAL_Delay(50);	
	}	

	 for(i =15; i>=0; i--)
	{
		shift_register((uint8_t *) &data3[i], 2);
		HAL_Delay(50);	
	}		
		 for(i =0; i<=23; i++)
	{
		shift_register2( (uint8_t *) &data1[i], 3);
		HAL_Delay(50);	
	}	

	 for(i =23; i>=0; i--)
	{
		shift_register2((uint8_t *) &data1[i], 3);
		HAL_Delay(50);	
	}		
}

void led_effect_2(void)
{
	shift_register2((uint8_t *) &data1[23], 3);
	shift_register((uint8_t *) &data3[15], 2);
	HAL_Delay(200);
	shift_register2((uint8_t *) &data1[0], 3);
	shift_register((uint8_t *) &data3[0], 2);
	HAL_Delay(200);
}

void down_to_left (void)
{
	uint32_t down_out_2;
	uint16_t down_in_2;
	
	  for(i1 =0; i1 <= 6; i1++)
	  {
		
		  down_out_2 = down_out;
		  down_in_2 = down_in;
		  down_out_2 = (down_out_2)>>i1;
		  down_in_2 = (down_in_2) << (i1-2);
	  shift_register( (uint8_t *) &(down_in_2), 2);
	  shift_register2( (uint8_t *) &(down_out_2), 3);
	  HAL_Delay(70);  
	  }	 
	  shift_register( (uint8_t *) &(left_in), 2);
	  shift_register2( (uint8_t *) &(left_out), 3);
	  HAL_Delay(50);
}
void down_to_right (void)
{
	uint32_t down_out_2;
	uint16_t down_in_2;
	
	  for(i2 =0; i2 <= 6; i2++)
	  {		
		  down_out_2 = down_out;
		  down_in_2 = down_in;
		  down_out_2 = (down_out_2)<<i2;
		  down_in_2 = (down_in_2) >> (i2-3);
	  shift_register( (uint8_t *) &(down_in_2), 2);
	  shift_register2( (uint8_t *) &(down_out_2), 3);
	  HAL_Delay(50);  
	  }	 
	   shift_register( (uint8_t *) &(right_in), 2);
	  shift_register2( (uint8_t *) &(right_out), 3);
	  HAL_Delay(50);
}

void down_to_up (void)
{ 
	shift_register( (uint8_t *) &(up_in), 2);
	  shift_register2( (uint8_t *) &(up_out), 3);
	HAL_Delay(50);
	i9 =1;
}

void up_to_left (void)
{
	uint32_t up_out_2;
	uint16_t up_in_2;
	
	  for(i3 =0; i3 <= 6; i3++)
	  {
		
		  up_out_2 = up_out;
		  up_in_2 = up_in;
		  up_out_2 = (up_out_2) << i3;
		  up_in_2 = (up_in_2) >> (i3 -3) ;
	  shift_register( (uint8_t *) &(up_in_2), 2);
	  shift_register2( (uint8_t *) &(up_out_2), 3);
	  HAL_Delay(50);  
	  }	 
	  shift_register( (uint8_t *) &(left_in), 2);
	  shift_register2( (uint8_t *) &(left_out), 3);
	  HAL_Delay(50);
}

void up_to_right (void)
{
	uint32_t up_out_2;
	uint16_t up_in_2;
	
	  for(i4 =0; i4 <= 6; i4++)
	  {
		
		  up_out_2 = up_out;
		  up_in_2 = up_in;
		  up_out_2 = (up_out_2) >> i4;
		  up_in_2 = (up_in_2) << (i4 -3) ;
	  shift_register( (uint8_t *) &(up_in_2), 2);
	  shift_register2( (uint8_t *) &(up_out_2), 3);
	  HAL_Delay(50);  
	  }	 
	  shift_register( (uint8_t *) &(right_in), 2);
	  shift_register2( (uint8_t *) &(right_out), 3);
	  HAL_Delay(50);
}

void up_to_down(void)
{
	  shift_register( (uint8_t *) &(down_in), 2);
	  shift_register2( (uint8_t *) &(down_out), 3);
	HAL_Delay(50);
	i10 =1;
}

void right_to_up (void)
{
	uint32_t right_out_2;
	uint16_t right_in_2;
	
	  for(i5 =0; i5 <= 5; i5++)
	  {
		
		  right_out_2 = right_out;
		  right_in_2 = right_in;
		  right_out_2 = (right_out_2)<<i5;
		  right_in_2 = (right_in_2) >> (i5-1);
	  shift_register( (uint8_t *) &(right_in_2), 2);
	  shift_register2( (uint8_t *) &(right_out_2), 3);
	  HAL_Delay(50);  
	  }	 
	  shift_register( (uint8_t *) &(up_in), 2);
	  shift_register2( (uint8_t *) &(up_out), 3);
	  HAL_Delay(50);
}

void right_to_down (void)
{
	uint32_t right_out_2;
	uint16_t right_in_2;
	
	  for(i6 =0; i6 <= 5; i6++)
	  {
		
		  right_out_2 = right_out;
		  right_in_2 = right_in;
		  right_out_2 = (right_out_2) >> i6;
		  right_in_2 = (right_in_2) << (i6-1);
	  shift_register( (uint8_t *) &(right_in_2), 2);
	  shift_register2( (uint8_t *) &(right_out_2), 3);
	  HAL_Delay(50);  
	  }	 
	  shift_register( (uint8_t *) &(down_in), 2);
	  shift_register2( (uint8_t *) &(down_out), 3);
	  HAL_Delay(50);
}

void right_to_left (void)
{
	shift_register( (uint8_t *) &(left_in), 2);
	shift_register2( (uint8_t *) &(left_out), 3);
	HAL_Delay(50);
	i11 =1;
}

void left_to_up (void)
{
	uint32_t left_out_2;
	uint16_t left_in_2;
	
	  for(i7 =0; i7 <= 5; i7++)
	  {
		
		  left_out_2 = left_out;
		  left_in_2 = left_in;
		  left_out_2 = (left_out_2)>>i7;
		  left_in_2 = (left_in_2)<<(i7-1);
	  shift_register( (uint8_t *) &(left_in_2), 2);
	  shift_register2( (uint8_t *) &(left_out_2), 3);
	  HAL_Delay(50);  
	  }	 
	  shift_register( (uint8_t *) &(up_in), 2);
	  shift_register2( (uint8_t *) &(up_out), 3);
	  HAL_Delay(50);
}

void left_to_down (void)
{
	uint32_t left_out_2;
	uint16_t left_in_2;
	
	  for(i8 =0; i8 <= 5; i8++)
	  {
		
		  left_out_2 = left_out;
		  left_in_2 = left_in;
		  left_out_2 = (left_out_2)<<i8;
		  left_in_2 = (left_in_2)>>(i8-1);
	  shift_register( (uint8_t *) &(left_in_2), 2);
	  shift_register2( (uint8_t *) &(left_out_2), 3);
	  HAL_Delay(50);  
	  }	 
	  shift_register( (uint8_t *) &(down_in), 2);
	  shift_register2( (uint8_t *) &(down_out), 3);
	  HAL_Delay(50);
}

void left_to_right (void)
{
	shift_register( (uint8_t *) &(right_in), 2);
	  shift_register2( (uint8_t *) &(right_out), 3);
	  HAL_Delay(50);
	i12 =1;
}

void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin)
{
	if(GPIO_Pin == BUTTON_Pin)
		{
			count ++;
		}
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
