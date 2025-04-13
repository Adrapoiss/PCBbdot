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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lis3mdl.h" //draiverid
#include "stdio.h" //standardteek
#include "custom_bus.h" //spi init
#include "stm32l4xx_hal_spi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LIS3MDL_CS_GPIO_Port GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int32_t magX = 0; //Live expression muutujad silumiseks
int32_t magY = 0;
int32_t magZ = 0;


uint16_t heartbeat = 0; //peamiselt kasutust saav, eri väärtused annavad siludes teada kus programmi töö katkes
uint16_t read_status = 0;

LIS3MDL_Object_t lis3mdl; //magnetomeetri objekt
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void LIS3MDL_CS_Select(void); //CS signaalikontroll
void LIS3MDL_CS_Deselect(void);
int __io_putchar(int ch); //oli UARTI jaoks, pragu iganenud

void Read_Magnetometer(void);
int32_t LIS3MDL_DummyInit(void); //IO bus registreerimise workaround
int32_t LIS3MDL_DummyDeInit(void);//vahepeal ei sobinud init ja deinit liikmete NULL deklaratsioon
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int32_t LIS3MDL_DummyInit(void) {
    return LIS3MDL_OK;
}

int32_t LIS3MDL_DummyDeInit(void) {
    return LIS3MDL_OK;
}

void LIS3MDL_CS_Select(void) {
    HAL_GPIO_WritePin(LIS3MDL_CS_GPIO_Port, LIS3MDLTR_CS_Pin, GPIO_PIN_RESET);
}

void LIS3MDL_CS_Deselect(void) {
    HAL_GPIO_WritePin(LIS3MDL_CS_GPIO_Port, LIS3MDLTR_CS_Pin, GPIO_PIN_SET);
}

int32_t Write_LIS3MDL(void *handle, uint8_t reg, uint8_t *data, uint16_t len) {
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)handle; //kuna handle tekitatakse teises failis peab castima nii
    reg |= 0x40; // mitmebaidine kirjutamine
    LIS3MDL_CS_Select();
    HAL_SPI_Transmit(hspi, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(hspi, data, len, HAL_MAX_DELAY);
    LIS3MDL_CS_Deselect();
    return LIS3MDL_OK;
}

int32_t Read_LIS3MDL(void *handle, uint8_t reg, uint8_t *data, uint16_t len) {
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)handle;
    reg |= 0x80 | 0x40; //mitmebaidine lugemine
    LIS3MDL_CS_Select();
    HAL_SPI_Transmit(hspi, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, data, len, HAL_MAX_DELAY);
    LIS3MDL_CS_Deselect();
    return LIS3MDL_OK;
}

void Read_Magnetometer(void) { //ei ole praegu kasutusel
    LIS3MDL_Axes_t axes;
    if (LIS3MDL_MAG_GetAxes(&lis3mdl, &axes) == LIS3MDL_OK) {
        printf("X: %ld, Y: %ld, Z: %ld\r\n", axes.x, axes.y, axes.z);
    } else {
        read_status = 1;
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
  /* USER CODE BEGIN 2 */
  BSP_SPI1_Init();
  //Debug muutujad
  uint8_t id;              // WHO_AM_I väärtuse salvestamiseks live expressionina lugeda ei õnnestunud aga kuni sensor töötab ei ole vajalik
  uint8_t ctrl_reg1_value;
  printf("----------Starting SPI Magnetometer on SPI1------------\r\n");

  LIS3MDL_IO_t io_local; //I/O init
  io_local.BusType = LIS3MDL_SPI_4WIRES_BUS;
  io_local.Address = 0;
  io_local.WriteReg = (LIS3MDL_WriteReg_Func)Write_LIS3MDL;
  io_local.ReadReg = (LIS3MDL_ReadReg_Func)Read_LIS3MDL;
  io_local.GetTick = (LIS3MDL_GetTick_Func)HAL_GetTick;
  io_local.Delay = HAL_Delay;
  io_local.Init = LIS3MDL_DummyInit; //Suvalised "peibutis"funktsioonid, ei tohiks muuta sensori toimimist
  io_local.DeInit = LIS3MDL_DummyDeInit;// kuid vahepeal oli probleeme IO busi registreerimisega

  lis3mdl.IO = io_local; //io structi külge kirjutamine

  if (LIS3MDL_RegisterBusIO(&lis3mdl, &io_local) != LIS3MDL_OK) {
	  heartbeat = 1; // Bus Io regamine ei õnnestunud
	  Error_Handler();
  }

  lis3mdl.Ctx.handle = &hspi1;
  lis3mdl.Ctx.write_reg = Write_LIS3MDL;
  lis3mdl.Ctx.read_reg = Read_LIS3MDL;
  lis3mdl.Ctx.mdelay = HAL_Delay;

  //registri lugemise teist
  if (LIS3MDL_ReadID(&lis3mdl, &id) == LIS3MDL_OK) {
	  printf("WHO_AM_I: 0x%02X\r\n", id);//ei kuvata praegu kuskil
	  if (id != LIS3MDL_ID) {
		  heartbeat = 3;
		  Error_Handler();
	  }
  } else {
	  heartbeat = 4;
	  Error_Handler();
  }

	if (LIS3MDL_Init(&lis3mdl) != LIS3MDL_OK) {
		heartbeat=5;
		Error_Handler();
	}
	if (lis3mdl_operating_mode_set(&lis3mdl.Ctx, LIS3MDL_CONTINUOUS_MODE) != LIS3MDL_OK) {
		heartbeat=6;
		Error_Handler();
	}
	if (lis3mdl_data_rate_set(&lis3mdl.Ctx, LIS3MDL_UHP_155Hz) != LIS3MDL_OK) {
		heartbeat=7;
		Error_Handler();
	}
	if (LIS3MDL_MAG_SetFullScale(&lis3mdl, 4) != LIS3MDL_OK) {
		heartbeat=8;
		Error_Handler();
	}
	if (LIS3MDL_Read_Reg(&lis3mdl, LIS3MDL_CTRL_REG1, &ctrl_reg1_value) != LIS3MDL_OK) {
		heartbeat=9;
		Error_Handler();
	}

	ctrl_reg1_value |= 0x80; //temp_en biti seadistus, praegu pole vaja sest ei kasuta temp sensorit
	if (LIS3MDL_Write_Reg(&lis3mdl, LIS3MDL_CTRL_REG1, ctrl_reg1_value) != LIS3MDL_OK) {
		heartbeat=10;
		Error_Handler();
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	LIS3MDL_Axes_t mag;         // X,Y,Z struct
	lis3mdl_axis3bit16_t mag_raw; // Toorandmed
	float sensitivity;         // tundlikuse näit
	float odr;                 // Output data rate
	float eelmine_b_x;         // Eelmise mõõtmise X telje väärtus
	float measurement = 0;		//mõõtmise loendur

	LIS3MDL_MAG_GetOutputDataRate(&lis3mdl, &odr);

	while (1)
	{
		HAL_GPIO_WritePin(GPIOB, LED_ERR_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, LED_DB2_Pin, GPIO_PIN_RESET); //kõik viigud madalaks enne uut mõõtmist
		HAL_GPIO_WritePin(GPIOB, LED_DB1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HB_FIN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, HB_RIN_Pin, GPIO_PIN_RESET);
		if (lis3mdl_magnetic_raw_get(&lis3mdl.Ctx, mag_raw.i16bit) == LIS3MDL_OK) {
			mag.x = (int32_t)((int16_t)mag_raw.i16bit[0]);
			mag.y = (int32_t)((int16_t)mag_raw.i16bit[1]);
			mag.z = (int32_t)((int16_t)mag_raw.i16bit[2]);

			LIS3MDL_MAG_GetSensitivity(&lis3mdl, &sensitivity);
			mag.x = (int32_t)((float)mag.x * sensitivity);
			mag.y = (int32_t)((float)mag.y * sensitivity);
			mag.z = (int32_t)((float)mag.z * sensitivity);
			magX = mag.x;
			magY = mag.y; //live expressionis vaatamiseks muutujad
			magZ = mag.z;


			if(measurement != 0){ //esimese mõõtmise korral ei tehta midagi

				float b_dot_x = mag.x - eelmine_b_x;
				eelmine_b_x = mag.x;
				if(b_dot_x > 5 || b_dot_x < -5){ //kui muut on väga väike ei jää algoritm pendeldama, peab katsetama kuhu maani on mõistlik
					//HB_FIN ja HB_RIN on H-silla vastavad viigud, mis võivad praegu valet pidi olla.
					if (b_dot_x > 0) {
						HAL_GPIO_WritePin(GPIOB, LED_DB2_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOB, HB_FIN_Pin, GPIO_PIN_SET);
					} else if (b_dot_x < 0) {
						HAL_GPIO_WritePin(GPIOB, LED_DB1_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOA, HB_RIN_Pin, GPIO_PIN_SET);
					}
				}
			}
			measurement++;

		  }
		  else {
			  HAL_GPIO_WritePin(GPIOB, LED_ERR_Pin, GPIO_PIN_SET); //lugemist ei toimunud läheb punane led põlema.
		  }

	  HAL_Delay((uint32_t)(1000.0f / odr)); //viivitus ODR põhjal, saaks muuta sensori lugema teatud Hz peal kui vaja, aga
	  	  	  	  	  	  	  	  	  	  	  //näen et mõistlikum on teha ajaarvestamist koodisiseselt
	  heartbeat++; //debug muutuja
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LIS3MDLTR_CS_Pin|HB_RIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, HB_FIN_Pin|LED_DB2_Pin|LED_DB1_Pin|LED_ERR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LIS3MDLTR_CS_Pin HB_RIN_Pin */
  GPIO_InitStruct.Pin = LIS3MDLTR_CS_Pin|HB_RIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : HB_FIN_Pin LED_DB2_Pin LED_DB1_Pin LED_ERR_Pin */
  GPIO_InitStruct.Pin = HB_FIN_Pin|LED_DB2_Pin|LED_DB1_Pin|LED_ERR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
	  HAL_GPIO_TogglePin(GPIOB, LED_ERR_Pin);
	  HAL_Delay(2000);
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
