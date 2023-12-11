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
#include "image.h"
#include "LCD_Test.h"

//#include "image.h"
#include "LCD_Test.h"
#include "LCD_2inch4.h"
#include "DEV_Config.h"
#include "stm32l4xx_hal.h"
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
void Set_Backlight_Brightness(uint16_t pulseWidth)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pulseWidth;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void LCD_2in4_test()
{
	printf("LCD_2IN4_test Demo\r\n");  // 打印测试演示的开始信息
	DEV_Module_Init();  // 初始化设备模块，可能包括配置相关的硬件接口

    printf("LCD_2IN4_Init and Clear...\r\n");  // 打印LCD初始化和清屏的信息
	LCD_2IN4_Init();  // 初始化LCD显示屏
	LCD_2IN4_Clear(WHITE);  // 清除LCD显示屏，设置为白色背景

    printf("Paint_NewImage\r\n");  // 打印新图像绘制的信息
	Paint_NewImage(LCD_2IN4_WIDTH, LCD_2IN4_HEIGHT, ROTATE_0, WHITE);  // 创建一个新的图像，设置尺寸、旋转角度和背景颜色

    printf("Set Clear and Display Function\r\n");  // 打印设置清除和显示功能的信息
	Paint_SetClearFuntion(LCD_2IN4_Clear);  // 设置图像清除的函数
	Paint_SetDisplayFuntion(LCD_2IN4_DrawPaint);  // 设置图像显示的函数

    printf("Paint_Clear\r\n");  // 打印清除画布的信息
	Paint_Clear(WHITE);  // 清除画布，设置为白色
    DEV_Delay_ms(1000);  // 延迟1000毫秒

    printf("Painting...\r\n");  // 打印开始绘制的信息
	Paint_SetRotate(ROTATE_0);  // 设置绘制的旋转角度为0
	Paint_DrawString_EN(5, 10, "Dinggo", &Font24, YELLOW, RED);  // 在指定位置绘制英文字符串
	Paint_DrawString_EN(5, 34, "Hello World", &Font24, BLUE, CYAN);  // 绘制另一个字符串
    Paint_DrawFloatNum(5, 150, 987.6543, 8, &Font20, WHITE, LIGHTBLUE);  // 绘制一个浮点数
    Paint_DrawString_EN(5, 170, "WaveShare", &Font24, WHITE, BLUE);  // 继续绘制字符串
    Paint_DrawString_CN(5, 190, "1 2 3", &Font24CN, WHITE, RED);  // 绘制中文字符串

	Paint_DrawRectangle(125, 240, 225, 300, RED, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);  // 绘制一个矩形框
	Paint_DrawLine(125, 240, 225, 300, MAGENTA, DOT_PIXEL_2X2, LINE_STYLE_SOLID);  // 绘制一条线
	Paint_DrawLine(225, 240, 125, 300, MAGENTA, DOT_PIXEL_2X2, LINE_STYLE_SOLID);  // 绘制另一条线

	Paint_DrawCircle(150, 100, 25, BLUE, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);  // 绘制一个圆
	Paint_DrawCircle(180, 100, 25, BLACK, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);  // 绘制另一个圆
	Paint_DrawCircle(210, 100, 25, RED, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);  // 绘制第三个圆
	Paint_DrawCircle(165, 125, 25, YELLOW, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);  // 绘制第四个圆
	Paint_DrawCircle(195, 125, 25, GREEN, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);  // 绘制第五个圆

    // Paint_DrawImage(gImage_1, 5, 70, 60, 60);  //  绘制一个图像

	DEV_Delay_ms(3000);  // 延迟3000毫秒

	printf("quit...\r\n");  // 打印退出信息
	//DEV_Module_Exit();  // (被注释掉的代码) 退出设备模块
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
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
 // Set_Backlight_Brightness(1000);

  LCD_2in4_test();


  // 设置字符串的起始位置
// 	      UWORD Xstart = 30;
// 	      UWORD Ystart = 30;
//
// 	      sFONT* Font = &Font24;
//
// 	      UWORD Color_Background = BLACK;
// 	      UWORD Color_Foreground = WHITE;

 // Paint_DrawString_EN(100, 150, "hello", &Font16, BLACK, WHITE);

 // Paint_DrawString_EN(5, 34, "Hello World", &Font24, BLUE, WHITE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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
