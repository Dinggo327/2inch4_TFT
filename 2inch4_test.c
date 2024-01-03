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

char ld4Status[10];
char displayString[50]; // 字符串变量，用于存储要显示的文本
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
void brighten()
{
	while (1)
	{
		if(checkButtonPressed())
			    {
			    	return;
			    }
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);

	}
}
void darken()
{
	while (1)
		{
		if(checkButtonPressed())
				    {
				    	return;
				    }
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

		}
}
void updateLd4StatusDisplay() {
    Paint_ClearWindows(5, 40, 5 + Font24.Width * 7, 40 + Font24.Height, WHITE);
    GPIO_PinState ld4_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
    snprintf(ld4Status, sizeof(ld4Status), "LD4:%s", (ld4_state == GPIO_PIN_RESET) ? "ON" : "OFF");
    Paint_DrawString_EN(5, 40, ld4Status, &Font24, WHITE, BLUE);
}


//void Set_Backlight_Brightness(uint16_t pulseWidth)
//{
//    TIM_OC_InitTypeDef sConfigOC = {0};
//
//    sConfigOC.OCMode = TIM_OCMODE_PWM1;
//    sConfigOC.Pulse = pulseWidth;
//    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//}


//void LCD_2in4_test()
//{
//    uint32_t lastUpdateTime = HAL_GetTick(); // 初始化最后更新时�????????????????
//
//    // 初始化LCD
//    LCD_2IN4_Init();
//    LCD_2IN4_Clear(WHITE);
//    Paint_NewImage(LCD_2IN4_WIDTH, LCD_2IN4_HEIGHT, ROTATE_0, WHITE);
//    Paint_SetClearFuntion(LCD_2IN4_Clear);
//    Paint_SetDisplayFuntion(LCD_2IN4_DrawPaint);
//
//    uint16_t number = 1; // 初始化数
//    char numStr[4];      // 字符串存储数
//
//    char ld4Status[4];
//
//    while (1) //
//    {
//
//        uint32_t currentTime = HAL_GetTick();
//        if (currentTime - lastUpdateTime >= 200)
//        {
//            lastUpdateTime = currentTime; // 更新�????????????????后一次更新时�????????????????
//
//            // 清除特定区域（数字显示区域）
//            //Paint_ClearWindows(5, 10, 5 + Font24.Width*3, 10 + Font24.Height, WHITE);
//            // 读取LD4的状�????????????????
//                      GPIO_PinState ld4_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
//                      // 根据状�?�设置字符串
//                      strcpy(ld4Status, (ld4_state == GPIO_PIN_SET) ?  "OFF":"ON" );
//            // 格式化数字为字符�????????????????
//            sprintf(numStr, "%d", number);
//
//            // 绘制数字
//            Paint_DrawString_EN(5, 10, numStr, &Font24, BLACK, WHITE);
//
//            Paint_DrawString_EN(5, 40, ld4Status, &Font24, BLACK, WHITE);
//            // 更新LCD显示
//            // 这里�????????????????要添加更新LCD显示的相关代�????????????????
//
//            // 更新数字
//            if (++number > 100) {
//            	 Paint_ClearWindows(5, 10, 5 + Font24.Width*3, 10 + Font24.Height, WHITE);
//                number = 1;
//            }
//
//        }
//
//        // 这里可以添加其他非屏幕更新相关的代码
//    }
//
//}

//void LCD_2in4_test()
//{
//
//    // 初始化LCD
//    LCD_2IN4_Init();
//    LCD_2IN4_Clear(WHITE);
//    Paint_NewImage(LCD_2IN4_WIDTH, LCD_2IN4_HEIGHT, ROTATE_0, WHITE);
//    Paint_SetClearFuntion(LCD_2IN4_Clear);
//    Paint_SetDisplayFuntion(LCD_2IN4_DrawPaint);
//
//    char ld4Status[4];
//
//
//           GPIO_PinState ld4_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
//
//           strcpy(ld4Status, (ld4_state == GPIO_PIN_RESET) ? "ON": "OFF" );
//
//           Paint_DrawString_EN(5, 40, ld4Status, &Font24, BLACK, WHITE);
//
//           Paint_ClearWindows(5, 10, 5 + Font24.Width, 10 + Font24.Height, WHITE);
//}

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
//    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
//        // 第一个脉冲捕�??????????????
//        if (captureValue1 == 0) {
//            captureValue1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//        }
//        // 第二个脉冲捕�??????????????
//        else if (captureValue2 == 0) {
//            captureValue2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//            if (captureValue2 > captureValue1) {
//                // 计算两个脉冲之间的周�??????????????
//
//                uint32_t difference = captureValue2 - captureValue1; // 两个连续脉冲之间的计数器差�??
//                float interval = difference / 1000000.0f; // 脉冲间隔时间（假设定时器频率�??????????????1MHz�??????????????
//                float frequency = 1.0f / interval; // 脉冲频率（Hz�??????????????
//                uint32_t PPR = 2; // 每转脉冲�??????????????
//                fanRPM = (frequency / PPR) * 60.0f; // 转换为每分钟转数（RPM�??????????????
//                captureDone = 1;
//            }
//            captureValue1 = 0; // 重置捕获值，为下�??????????????次测量准�??????????????
//            captureValue2 = 0;
//        }
//    }
//}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	  // 检查触发中断的通道是否为TIM的通道1
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        // 读取捕获到的脉冲值并存储到数组中
        captureValues[captureIndex] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        captureIndex++;
        // 如果已经捕获了足够的脉冲数（假设为4）
        if (captureIndex >= 4) {
            uint32_t totalTime = 0; // 计算总的时间间隔
            // 计算每两个脉冲之间的时间间隔，并累加到总时间中
            for (int i = 1; i < 4; i++) {
                totalTime += captureValues[i] - captureValues[i-1];
            }

//          float timeInSeconds = (float)totalTime / (float)HAL_RCC_GetPCLK1Freq(); // 定时器时钟频率与PCLK1相同
//          fanRPM = (uint32_t)(60 / timeInSeconds);
            // 计算脉冲信号的时间间隔并将其转换为RPM
            uint32_t pclk1Freq = HAL_RCC_GetPCLK1Freq(); // PCLK1频率
            fanRPM = (60 * pclk1Freq) / totalTime;

            captureDone = 1;

            captureIndex = 0;
        }
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  uint8_t ledState = 0;
  DEV_Module_Init();
  LCD_2IN4_Init();
  LCD_2IN4_Clear(WHITE);
  Paint_NewImage(LCD_2IN4_WIDTH, LCD_2IN4_HEIGHT, ROTATE_0, WHITE);
  Paint_SetClearFuntion(LCD_2IN4_Clear);
  Paint_SetDisplayFuntion(LCD_2IN4_DrawPaint);

 // char ld4Status[10];

  char rpmString[11]; // Enough to hold the uint32_t value and '\0' character
      sprintf(rpmString, "RPM: %lu", fanRPM); // Convert the RPM value to a string

 // Set_Backlight_Brightness(1000);

 // LCD_2in4_test();

     GPIO_PinState ld4_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
  //   strcpy(ld4Status, (ld4_state == GPIO_PIN_RESET) ? "ON": "OFF" );
     snprintf(ld4Status, sizeof(ld4Status), "LD4:%s", (ld4_state == GPIO_PIN_RESET) ? "ON" : "OFF");
     Paint_DrawString_EN(5, 40, ld4Status, &Font24, WHITE, BLUE);

  //   Paint_DrawString_EN(5, 100, rpmString, &Font24, WHITE, BLACK);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch (ledState)
	 	  {
	 	  case 0:{
	 		  darken();
	 		  updateLd4StatusDisplay();
	 	      ledState++;
	 	      break;
	 	  }
	 	  case 1:{
	 		  brighten();
	 		  updateLd4StatusDisplay();
	 	      ledState = 0;
	 	      break;
	 	  }
	 	  }
	  // 启动PWM输出
	       HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	       // 启动输入捕获
	       HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	       HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	  if (captureDone)
	   {
	     // Reset the flag
	     captureDone = 0;
	     HAL_Delay(100);

	     snprintf(rpmString, sizeof(rpmString), "RPM: %lu", fanRPM);


	     Paint_ClearWindows(5, 100, 5 + Font24.Width * 10, 100 + Font24.Height, WHITE);
	     Paint_DrawString_EN(5, 100, rpmString, &Font24, WHITE, BLACK);
	   }
	  else{
		  Paint_ClearWindows(5, 100, 5 + Font24.Width * 10, 100 + Font24.Height, WHITE);
		 	     Paint_DrawString_EN(5, 100, "missed", &Font24, WHITE, RED);
	  }



// 从捕获值数组中获取当前索引处的脉冲值
uint32_t pulseValue = captureValues[captureIndex];

// 获取定时器时钟频率（PCLK1）
uint32_t timerClockFreq = HAL_RCC_GetPCLK1Freq();

// 初始化时间间隔（毫秒）
float timeIntervalMs = 0;

// 如果已经捕获到至少一个脉冲
if (captureIndex > 0) {
    // 计算两个脉冲之间的计数数目
    uint32_t timeIntervalCounts = captureValues[captureIndex] - captureValues[captureIndex - 1];

    // 将计数数目转换为时间间隔（毫秒）
    timeIntervalMs = ((float)timeIntervalCounts / timerClockFreq) * 1000.0f;
}

// 创建一个字符串以显示脉冲值和时间间隔
char displayString[60]; // 增加字符串长度以适应更多字符
sprintf(displayString, "Pulse: %lu   Interval:%.2f ms", pulseValue, timeIntervalMs);

// 清除LCD上旧的显示，并显示新的字符
Paint_ClearWindows(5, 130, LCD_2IN4_WIDTH, 130 + Font24.Height * 3, WHITE);
Paint_DrawString_EN(5, 130, displayString, &Font24, BLACK, WHITE);

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
