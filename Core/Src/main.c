/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "usbd_hid.h"
#include <stdbool.h>
#include "button.h"
#include "usbd_customhid.h"
#include "usbd_custom_hid_if.h"
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

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct
{
	uint8_t leftPlay   : 1;
	uint8_t leftPause  : 1;
	uint8_t leftCue    : 1;
	uint8_t leftStop   : 1;
	uint8_t rightPlay  : 1;
	uint8_t rightPause : 1;
	uint8_t rightCue   : 1;
	uint8_t rightStop  : 1;
} Buttons_t;

typedef struct
{
	Buttons_t  buttons;
	uint8_t    leftPitch;
	uint8_t    rightPitch;
	uint8_t    leftVolume;
	uint8_t    rightVolume;
	uint8_t    crossFader;
} UsbHidVdjController_t;

GPIO_PinState buttonB1State;
GPIO_PinState buttonOut1State;
GPIO_PinState buttonOut2State;
GPIO_PinState buttonOut3State;
GPIO_PinState buttonOut4State;

UsbHidVdjController_t VdjCtrlReport;
extern USBD_HandleTypeDef hUsbDeviceFS;

ButtonHandler_t BtnBlue =
{
	.ActiveState = GPIO_PIN_SET,
	.DebounceCtr = 0u,
	.DebounceOff = 20u,
	.DebounceOn  = 20u,
	.IsPressed   = false,
	.WasPressed  = false,
	.Padding     = 0u
};

ButtonHandler_t BtnOut1 =
{
	.ActiveState = GPIO_PIN_RESET,
	.DebounceCtr = 0u,
	.DebounceOff = 200u,
	.DebounceOn  = 200u,
	.IsPressed   = false,
	.WasPressed  = false,
	.Padding     = 0u
};

ButtonHandler_t BtnOut2 =
{
	.ActiveState = GPIO_PIN_RESET,
	.DebounceCtr = 0u,
	.DebounceOff = 200u,
	.DebounceOn  = 200u,
	.IsPressed   = false,
	.WasPressed  = false,
	.Padding     = 0u
};

ButtonHandler_t BtnOut3 =
{
	.ActiveState = GPIO_PIN_RESET,
	.DebounceCtr = 0u,
	.DebounceOff = 200u,
	.DebounceOn  = 200u,
	.IsPressed   = false,
	.WasPressed  = false,
	.Padding     = 0u
};

ButtonHandler_t BtnOut4 =
{
	.ActiveState = GPIO_PIN_RESET,
	.DebounceCtr = 0u,
	.DebounceOff = 200u,
	.DebounceOn  = 200u,
	.IsPressed   = false,
	.WasPressed  = false,
	.Padding     = 0u
};
/* USER CODE END 1 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  //uint8_t buttonB1DebounceCtr = 0u;
  //GPIO_PinState buttonB1State;
  //bool buttonPressed = false;
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
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	buttonB1State = HAL_GPIO_ReadPin(BUTTON0_GPIO_Port, BUTTON0_Pin);
	buttonOut1State = HAL_GPIO_ReadPin(PUSHBUTTON1_GPIO_Port, PUSHBUTTON1_Pin);
	buttonOut2State = HAL_GPIO_ReadPin(PUSHBUTTON2_GPIO_Port, PUSHBUTTON2_Pin);
	buttonOut3State = HAL_GPIO_ReadPin(PUSHBUTTON3_GPIO_Port, PUSHBUTTON3_Pin);
	buttonOut4State = HAL_GPIO_ReadPin(PUSHBUTTON4_GPIO_Port, PUSHBUTTON4_Pin);

	ButtonPressed( &BtnBlue, buttonB1State );
	ButtonPressed( &BtnOut1, buttonOut1State );
	ButtonPressed( &BtnOut2, buttonOut2State );
	ButtonPressed( &BtnOut3, buttonOut3State );
	ButtonPressed( &BtnOut4, buttonOut4State );

	// Blue button on discovery board
	if(   ( BtnBlue.WasPressed == false )
	   && ( BtnBlue.IsPressed == true   ) )
	{
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		BtnBlue.WasPressed = true;
	}
	// External button1 on BTN_DPSW board
	if(   ( BtnOut1.WasPressed == false )
	   && ( BtnOut1.IsPressed == true   ) )
	{
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		BtnOut1.WasPressed = true;
		if( VdjCtrlReport.buttons.leftPlay == 0 )
		{
			VdjCtrlReport.buttons.leftPlay = 1;
			VdjCtrlReport.buttons.leftPause = 0u;
		}
		else
		{
			VdjCtrlReport.buttons.leftPlay = 0;
			VdjCtrlReport.buttons.leftPause = 1u;
		}
	}
	// External button2 on BTN_DPSW board
	if(   ( BtnOut2.WasPressed == false )
	   && ( BtnOut2.IsPressed == true   ) )
	{
		HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
		BtnOut2.WasPressed = true;
	}
	// External button3 on BTN_DPSW board
	if(   ( BtnOut3.WasPressed == false )
	   && ( BtnOut3.IsPressed == true   ) )
	{
		HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
		BtnOut3.WasPressed = true;
	}
	// External button4 on BTN_DPSW board
	if(   ( BtnOut4.WasPressed == false )
	   && ( BtnOut4.IsPressed == true   ) )
	{
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		BtnOut4.WasPressed = true;
	}

	ButtonReleased( &BtnBlue, buttonB1State );
	ButtonReleased( &BtnOut1, buttonOut1State );
	ButtonReleased( &BtnOut2, buttonOut2State );
	ButtonReleased( &BtnOut3, buttonOut3State );
	ButtonReleased( &BtnOut4, buttonOut4State );

	// Blue button on discovery board
	if(   ( BtnBlue.WasPressed == true )
	   && ( BtnBlue.IsPressed == false   ) )
	{
		BtnBlue.WasPressed = false;
	}
	// External button1 on BTN_DPSW board
	if(   ( BtnOut1.WasPressed == true )
	   && ( BtnOut1.IsPressed == false   ) )
	{
		BtnOut1.WasPressed = false;
		USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&VdjCtrlReport, sizeof(VdjCtrlReport));
	}
	// External button2 on BTN_DPSW board
	if(   ( BtnOut2.WasPressed == true )
	   && ( BtnOut2.IsPressed == false   ) )
	{
		BtnOut2.WasPressed = false;
	}
	// External button3 on BTN_DPSW board
	if(   ( BtnOut3.WasPressed == true )
	   && ( BtnOut3.IsPressed == false   ) )
	{
		BtnOut3.WasPressed = false;
	}
	// External button4 on BTN_DPSW board
	if(   ( BtnOut4.WasPressed == true )
	   && ( BtnOut4.IsPressed == false   ) )
	{
		BtnOut4.WasPressed = false;
	}

	//USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&VdjCtrlReport, sizeof(VdjCtrlReport));
	//HAL_Delay(30);

#if 0
	// button not yet pressed
	if( buttonPressed == false )
	{
		if( buttonB1State == GPIO_PIN_SET )
		{
			buttonB1DebounceCtr++;
		}
		else
		if( buttonB1DebounceCtr > 0u )
		{
			buttonB1DebounceCtr--;
		}

		if( buttonB1DebounceCtr > 20u )
		{
			HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
			buttonB1DebounceCtr = 0u;
			buttonPressed = true;
		}
	}

	// button pressed
	if ( buttonPressed == true )
	{
		if( buttonB1State == GPIO_PIN_RESET )
		{
			buttonB1DebounceCtr++;
		}
		else
		if( buttonB1DebounceCtr > 0u )
		{
			buttonB1DebounceCtr--;
		}

		if( buttonB1DebounceCtr > 20u )
		{
			buttonB1DebounceCtr = 0u;
			buttonPressed = false;
		}
	}
#endif
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON0_Pin */
  GPIO_InitStruct.Pin = BUTTON0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIPSWITCH1_Pin DIPSWITCH2_Pin DIPSWITCH3_Pin DIPSWITCH4_Pin
                           PUSHBUTTON1_Pin PUSHBUTTON2_Pin PUSHBUTTON3_Pin PUSHBUTTON4_Pin */
  GPIO_InitStruct.Pin = DIPSWITCH1_Pin|DIPSWITCH2_Pin|DIPSWITCH3_Pin|DIPSWITCH4_Pin
                          |PUSHBUTTON1_Pin|PUSHBUTTON2_Pin|PUSHBUTTON3_Pin|PUSHBUTTON4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

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

