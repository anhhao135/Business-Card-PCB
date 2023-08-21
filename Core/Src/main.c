/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdbool.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define IMU_I2C_ADDR (0b1101000 << 1)
#define IMU_REG_DEVICE_CONFIG 0x01
#define IMU_REG_DRIVE_CONFIG2 0x04
#define IMU_REG_WHOAMI 0x75
#define IMU_WHOAMI_VALUE 0x67
#define IMU_REG_TEMP_DATA1 0x09
#define IMU_REG_TEMP_DATA0 0x0A
#define IMU_REG_MCLK_RDY 0x00

#define IMU_REG_ACCEL_DATA_X1 0x0B
#define IMU_REG_ACCEL_DATA_X0 0x0C

#define IMU_REG_PWR_MGMT0 0x1F

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

int16_t IMUAccelX = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


int RandomInt(int min, int max){
   return min + rand() / (RAND_MAX / (max - min + 1) + 1);
}


void PowerIMUOn(){
	uint8_t data = 0b00001111;
	HAL_I2C_Mem_Write(&hi2c1, IMU_I2C_ADDR, IMU_REG_PWR_MGMT0, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

int16_t GetIMUAccelX(){

	uint8_t data1;
	HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, IMU_REG_ACCEL_DATA_X1, I2C_MEMADD_SIZE_8BIT, &data1, 1, HAL_MAX_DELAY);
	uint8_t data0;
	HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, IMU_REG_ACCEL_DATA_X0, I2C_MEMADD_SIZE_8BIT, &data0, 1, HAL_MAX_DELAY);

	int16_t data = (((int16_t)data1) << 8) + data0;

	return data;
}


int DetectCardShake(){
	if (abs(GetIMUAccelX()) > 6000){
		return 1;
	}
	else {
		return 0;
	}
}


int VerifyIMU(){
	uint8_t data;
	HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, IMU_REG_WHOAMI, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	if (data == IMU_WHOAMI_VALUE){
		return 1;
	}
	else {
		return 0;
	}
}

void OneHotEncodeLED(int lightIndex){

	bool ledSwitch = false;

	HAL_GPIO_WritePin(GPIOF, RED1_Pin, ledSwitch);
	HAL_GPIO_WritePin(RED2_GPIO_Port, RED2_Pin, ledSwitch);
	HAL_GPIO_WritePin(GPIOA, RED3_Pin, ledSwitch);
	HAL_GPIO_WritePin(GPIOA, YEL1_Pin, ledSwitch);
	HAL_GPIO_WritePin(GPIOA, YEL2_Pin, ledSwitch);
	HAL_GPIO_WritePin(GPIOF, YEL3_Pin, ledSwitch);
	HAL_GPIO_WritePin(GPIOA, GRN1_Pin, ledSwitch);
	HAL_GPIO_WritePin(GPIOA, GRN2_Pin, ledSwitch);
	HAL_GPIO_WritePin(GPIOA, GRN3_Pin, ledSwitch);


	ledSwitch = true;


	switch (lightIndex){

		case 0:
			HAL_GPIO_WritePin(GPIOF, RED1_Pin, ledSwitch);
			break;
		case 1:
			HAL_GPIO_WritePin(RED2_GPIO_Port, RED2_Pin, ledSwitch);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOA, RED3_Pin, ledSwitch);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOA, YEL1_Pin, ledSwitch);
			break;
		case 4:
			HAL_GPIO_WritePin(GPIOA, YEL2_Pin, ledSwitch);
			break;
		case 5:
			HAL_GPIO_WritePin(GPIOF, YEL3_Pin, ledSwitch);
			break;
		case 6:
			HAL_GPIO_WritePin(GPIOA, GRN1_Pin, ledSwitch);
			break;
		case 7:
			HAL_GPIO_WritePin(GPIOA, GRN2_Pin, ledSwitch);
			break;
		case 8:
			HAL_GPIO_WritePin(GPIOA, GRN3_Pin, ledSwitch);
			break;
		case 9:
			break;

	}


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
  /* USER CODE BEGIN 2 */

  while(!VerifyIMU()){}

  PowerIMUOn();

  for (int i = 0; i < 10; i ++){
	  OneHotEncodeLED(i);
	  HAL_Delay(100);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (DetectCardShake()){
		  OneHotEncodeLED(RandomInt(0,8));
	  }



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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, RED1_Pin|YEL3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, YEL2_Pin|YEL1_Pin|GRN3_Pin|GRN2_Pin
                          |GRN1_Pin|RED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RED2_GPIO_Port, RED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RED1_Pin YEL3_Pin */
  GPIO_InitStruct.Pin = RED1_Pin|YEL3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : YEL2_Pin YEL1_Pin GRN3_Pin GRN2_Pin
                           GRN1_Pin RED3_Pin */
  GPIO_InitStruct.Pin = YEL2_Pin|YEL1_Pin|GRN3_Pin|GRN2_Pin
                          |GRN1_Pin|RED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU_INT1_Pin IMU_INT2_Pin */
  GPIO_InitStruct.Pin = IMU_INT1_Pin|IMU_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RED2_Pin */
  GPIO_InitStruct.Pin = RED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
