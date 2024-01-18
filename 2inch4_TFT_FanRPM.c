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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "GUI_Paint.h"
#include "fonts.h"
#include "image_data.h"

#include "image.h"
#include "LCD_Test.h"
#include "LCD_2inch4.h"
#include "DEV_Config.h"
#include "stm32l4xx_hal.h"
#include "string.h"

volatile uint32_t captureValues[4] = {0};
volatile uint8_t captureIndex = 0;
volatile uint32_t fanRPM = 0;
volatile uint8_t captureDone = 0;


char displayString[50]; // 字符串变量，用于存储要显示的文本

volatile uint32_t CCR1,CCR2;

uint32_t risingEdgeTimestamp = 0;
uint32_t fallingEdgeTimestamp = 0;
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int lastButtonState = GPIO_PIN_RESET;
int checkButtonPressed() {
    HAL_Delay(10);

    int currentButtonState = HAL_GPIO_ReadPin(USER_GPIO_Port, USER_Pin);
    int buttonPressed = 0;

    if (lastButtonState == GPIO_PIN_SET && currentButtonState == GPIO_PIN_RESET)
    {
        buttonPressed = 1;
    }

    lastButtonState = currentButtonState;

    return buttonPressed;
}
void printRPM()
{
	while(1)
	{
		if(checkButtonPressed())
		{
			return;
		}
	}

}


//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
//
//    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
//        // 读取捕获到的脉冲值并存储到数组中
//        captureValues[captureIndex] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
//        captureIndex++;
//
//        if (captureIndex >= 4) {
//            uint32_t totalTime = 0;
//
//            for (int i = 1; i < 4; i++) {
//                totalTime += captureValues[i] - captureValues[i-1];
//            }
////          float timeInSeconds = (float)totalTime / (float)HAL_RCC_GetPCLK1Freq(); // 定时器时钟频率与PCLK1相同
////          fanRPM = (uint32_t)(60 / timeInSeconds);
//            // 计算脉冲信号的时间间隔并将其转换为RPM
//            uint32_t pclk1Freq = HAL_RCC_GetPCLK1Freq(); // PCLK1频率
//            fanRPM = (60 * pclk1Freq) / totalTime;
//
//            captureDone = 1;
//            captureIndex = 0;
//        }
//    }
//}

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//    CCR1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
//    if (CCR1 != 0)
//    {
//       CCR2 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
//       uint32_t pclk1Freq = HAL_RCC_GetPCLK1Freq();
//       float frequency=pclk1Freq/(CCR1+1);
//       // 计算输入信号的占空比
//       float duty_cycle = (float)(CCR2 + 1) * 100 / (CCR1 + 1);
//       fanRPM = duty_cycle;
//       captureDone = 1;
//    }
//}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      // TIM1通道1的输入捕获中断处理（上升沿）
      risingEdgeTimestamp = HAL_GetTick();
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
      // TIM1通道2的输入捕获中断处理（下降沿）
      fallingEdgeTimestamp = HAL_GetTick();

      // 计算上升沿到下降沿的时间差
      uint32_t pulseDuration = fallingEdgeTimestamp - risingEdgeTimestamp;

      fanRPM = pulseDuration;
      captureDone = 1;
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  uint8_t ledState = 0;
  DEV_Module_Init();
  LCD_2IN4_Init();
  LCD_2IN4_Clear(WHITE);
  Paint_NewImage(LCD_2IN4_WIDTH, LCD_2IN4_HEIGHT, ROTATE_0, WHITE);
  Paint_SetClearFuntion(LCD_2IN4_Clear);
  Paint_SetDisplayFuntion(LCD_2IN4_DrawPaint);


//  htim1.Instance->CCR1 = 500;
//  HAL_Delay(15);

 // char ld4Status[10];

  Paint_DrawString_EN(5, 100, "Press Button", &Font24, WHITE, BLUE);
//      sprintf(rpmString, "RPM: %lu", fanRPM); // Convert the RPM value to a string

 // Set_Backlight_Brightness(1000);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

	 switch (ledState)
	 {
	 	  case 0:{
	 		 printRPM();
	 	     ledState++;
	 	     break;
	 	  }
	 	  case 1:{
	 		 printRPM();
	 	     ledState = 0;
	 	     break;
	 	  }
	 }

		  if (captureDone)
		   {
			 char rpmString[11];

		     captureDone = 0;
		     HAL_Delay(100);

		     snprintf(rpmString, sizeof(rpmString), "RPM: %lu", fanRPM);

		     Paint_ClearWindows(5, 100, 5 + Font24.Width * 12, 100 + Font24.Height, WHITE);
		     Paint_DrawString_EN(5, 100, rpmString, &Font24, WHITE, BLACK);
		   }
		  else{
			  Paint_ClearWindows(5, 100, 5 + Font24.Width * 12, 100 + Font24.Height, WHITE);
			 Paint_DrawString_EN(5, 100, "Lost", &Font24, WHITE, RED);
		  }


//	  uint32_t pulseValue = captureValues[captureIndex];
//	  uint32_t timerClockFreq = HAL_RCC_GetPCLK1Freq();
//	  float timeIntervalMs = 0; // 时间间隔（毫秒）
//
//	  if (captureIndex > 0) {
//	      uint32_t timeIntervalCounts = captureValues[captureIndex] - captureValues[captureIndex - 1];
//	      timeIntervalMs = ((float)timeIntervalCounts / timerClockFreq) * 1000.0f;
//	  }
//
//	  char displayString[60]; // 增加字符串长度以适应更多字符
//	  sprintf(displayString, "Pulse: %lu   Interval:%.2f ms", pulseValue, timeIntervalMs);
//
//	  // 清除LCD上的旧显示，并显示新字符
//	  Paint_ClearWindows(5, 130, LCD_2IN4_WIDTH, 130 + Font24.Height * 3, WHITE);
//	  Paint_DrawString_EN(5, 130, displayString, &Font24, BLACK, WHITE);

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
  RCC_OscInitStruct.PLL.PLLN = 40;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
