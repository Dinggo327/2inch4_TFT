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

char ld4Status[10];
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
//    sConfigOC.Pulse = pulseWidth; // 设置PWM的脉冲宽度
//    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // 开始PWM信号输出
//}

//void LCD_2in4_test()
//{
//	printf("LCD_2IN4_test Demo\r\n");
//	DEV_Module_Init();
//
//   printf("LCD_2IN4_ Init and Clear...\r\n");
//	LCD_2IN4_Init();
//	LCD_2IN4_Clear(WHITE);
//
//  printf("Paint_NewImage\r\n");
//	Paint_NewImage(LCD_2IN4_WIDTH,LCD_2IN4_HEIGHT, ROTATE_90, WHITE);
//
//  printf("Set Clear and Display Funtion\r\n");
//	Paint_SetClearFuntion(LCD_2IN4_Clear);
//	Paint_SetDisplayFuntion(LCD_2IN4_DrawPaint);
//
//  printf("Paint_Clear\r\n");
//	Paint_Clear(WHITE);
//  DEV_Delay_ms(1000);
//
//  printf("Painting...\r\n");
//	Paint_SetRotate(ROTATE_0);        //240*320
//	Paint_DrawString_EN (5, 10, " ",   &Font24,    WHITE,  BLACK);
//
////	Paint_DrawString_EN (5, 34, "Hello World",  &Font24,    BLUE,    CYAN);
////    Paint_DrawFloatNum  (5, 150 ,987.6543, 8,  &Font20,    WHITE,   LIGHTBLUE);
////    Paint_DrawString_EN (5,170, "WaveShare",    &Font24,    WHITE,   BLUE);
////    Paint_DrawString_CN (5,190,"1 2 3",     &Font24CN,  WHITE,   RED);
////
////	Paint_DrawRectangle (125, 240, 225, 300,    RED     ,DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
////	Paint_DrawLine      (125, 240, 225, 300,    MAGENTA ,DOT_PIXEL_2X2,LINE_STYLE_SOLID);
////	Paint_DrawLine      (225, 240, 125, 300,    MAGENTA ,DOT_PIXEL_2X2,LINE_STYLE_SOLID);
////
////	Paint_DrawCircle(150,100,  25,        BLUE    ,DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
////	Paint_DrawCircle(180,100,  25,        BLACK   ,DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
////	Paint_DrawCircle(210,100,  25,        RED     ,DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
////	Paint_DrawCircle(165,125,  25,        YELLOW  ,DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
////	Paint_DrawCircle(195,125,  25,        GREEN   ,DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
//
// //   Paint_DrawImage(gImage_1,5,70,60,60);
//
//
//	DEV_Delay_ms(3000);
//
//	printf("quit...\r\n");
//	//DEV_Module_Exit();
//
//}

//void LCD_2in4_test()
//{
//    uint32_t lastUpdateTime = HAL_GetTick(); // 初始化最后更新时�?
//
//    // 初始化LCD
//    LCD_2IN4_Init();
//    LCD_2IN4_Clear(WHITE);
//    Paint_NewImage(LCD_2IN4_WIDTH, LCD_2IN4_HEIGHT, ROTATE_0, WHITE);
//    Paint_SetClearFuntion(LCD_2IN4_Clear);
//    Paint_SetDisplayFuntion(LCD_2IN4_DrawPaint);
//
//    // ... 其他绘制操作 ...
//
//    while (1) // 主循�?
//    {
//        uint32_t currentTime = HAL_GetTick(); // 获取当前时间
//        if (currentTime - lastUpdateTime >= 1000) // �?查是否已过一�?
//        {
//            lastUpdateTime = currentTime; // 更新�?后一次更新时�?
//
//            // 只更新特定的屏幕部分
//            Paint_ClearWindows(5, 10, 5 + Font24.Width * strlen("Dinggo"), 10 + Font24.Height, WHITE); // 清除特定区域
//            Paint_DrawString_EN(5, 10, "Dinggo", &Font24, YELLOW, RED); // 重绘字符�?
//
//            // 更新LCD显示
//            // 这里你需要添加更新LCD显示的相关代码，例如调用�?个函数将画布内容渲染到屏幕上
//        }
//
//        // 这里可以添加其他非屏幕更新相关的代码
//    }
//}

//void LCD_2in4_test()
//{
//    uint32_t lastUpdateTime = HAL_GetTick(); // 初始化最后更新时�?
//
//    // 初始化LCD
//    LCD_2IN4_Init();
//    LCD_2IN4_Clear(WHITE);
//    Paint_NewImage(LCD_2IN4_WIDTH, LCD_2IN4_HEIGHT, ROTATE_0, WHITE);
//    Paint_SetClearFuntion(LCD_2IN4_Clear);
//    Paint_SetDisplayFuntion(LCD_2IN4_DrawPaint);
//
//    uint16_t number = 1; // 初始化数�?
//    char numStr[4];      // 字符串存储数�?
//
//    while (1) // 主循�?
//    {
//        uint32_t currentTime = HAL_GetTick(); // 获取当前时间
//        if (currentTime - lastUpdateTime >= 1000) // �?查是否已过一�?
//        {
//            lastUpdateTime = currentTime; // 更新�?后一次更新时�?
//
//            // 清除特定区域（数字显示区域）
//       //     Paint_ClearWindows(5, 10, 5 + Font24.Width, 10 + Font24.Height, WHITE);
//
//            // 格式化数字为字符�?
//            sprintf(numStr, "%d", number);
//
//            // 绘制数字
//            Paint_DrawString_EN(5, 10, numStr, &Font24, BLACK, WHITE);
//
//            // 更新LCD显示
//            // 这里�?要添加更新LCD显示的相关代�?
//
//            // 更新数字
//            if (++number > 100) {
//              Paint_DrawString_EN(5, 10, numStr, &Font24, BLACK, WHITE);
//                number = 1;
//            }
//        }
//
//        // 这里可以添加其他非屏幕更新相关的代码
//    }
//}

//void LCD_2in4_test()
//{
//	DEV_Module_Init();
//    LCD_2IN4_Init();
//    LCD_2IN4_Clear(WHITE);
//    Paint_NewImage(LCD_2IN4_WIDTH, LCD_2IN4_HEIGHT, ROTATE_90, WHITE);
//    Paint_SetClearFuntion(LCD_2IN4_Clear);
//    Paint_SetDisplayFuntion(LCD_2IN4_DrawPaint);
//
//    Paint_DrawImage(image_data,0, 0, 199, 112);
//
//}


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
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  uint8_t ledState = 0;
  DEV_Module_Init();
  LCD_2IN4_Init();
  LCD_2IN4_Clear(WHITE);
  Paint_NewImage(LCD_2IN4_WIDTH, LCD_2IN4_HEIGHT, ROTATE_0, WHITE);
  Paint_SetClearFuntion(LCD_2IN4_Clear);
  Paint_SetDisplayFuntion(LCD_2IN4_DrawPaint);

//  Set_Backlight_Brightness(500);
//  HAL_Delay(100);
//  htim1.Instance->CCR1 = 00;
//  HAL_Delay(25);

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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
