/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "ltdc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM7

#include "arm_math.h"
#include "image_buffer.h"
#include "stm32h7xx.h"
#include <stdint.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define SCREEN_HEIGHT 480
#define SCREEN_WIDTH 640
#define ADC_BUFFER_SIZE 4096

int16_t ADCbuf[ADC_BUFFER_SIZE];
int16_t FFTbuf[ADC_BUFFER_SIZE * 2];
int16_t Magbuf[ADC_BUFFER_SIZE];

int16_t MaxValue = 0;
uint32_t MaxValueIndex = 0;

void drawBar(int x, int y, int16_t max_val) {
  if (y > SCREEN_HEIGHT / 2) {
    y = SCREEN_HEIGHT / 2;
  }
  float f = ((float)y / max_val) * SCREEN_HEIGHT / 2;
  uint16_t conv = SCREEN_HEIGHT - (uint16_t)f;

  for (int i = SCREEN_HEIGHT / 2; i < SCREEN_HEIGHT; i++) {
    if (i >= conv) {
      image_buffer[i * SCREEN_WIDTH + x] = 255;
    } else {
      image_buffer[i * SCREEN_WIDTH + x] = 0;
    }
  }
}

void drawSignal(int16_t *buf, int buflen) {
  for (int i = 0; i < SCREEN_WIDTH; i++) {
    float y = (float)buf[i*6] + 32768;
    float f = (y / 65535) * SCREEN_HEIGHT / 2;
    uint16_t conv = SCREEN_HEIGHT / 2 - (uint16_t)f;

    for (int j = 0; j < SCREEN_HEIGHT / 2; j++) {
      if (j == conv) {
        image_buffer[j * SCREEN_WIDTH + i] = 255;
      } else {
        image_buffer[j * SCREEN_WIDTH + i] = 0;
      }
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
  fill_image_buffer();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // Need to call DMA init BEFORE ADC init!!
  MX_DMA_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LTDC_Init();
  MX_ADC1_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_LTDC_SetAddress(&hltdc, (uint32_t)&image_buffer, 0);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCbuf, ADC_BUFFER_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_TIM_Base_Start_IT(&htim3);

  arm_rfft_instance_q15 fft;

  while (1) {
    // Create a copy of signal samples buffer
    int16_t signal[ADC_BUFFER_SIZE];
    memcpy(signal, ADCbuf, ADC_BUFFER_SIZE * sizeof(int16_t));

    drawSignal(signal, ADC_BUFFER_SIZE);

    // perform FFT
    arm_rfft_init_q15(&fft, ADC_BUFFER_SIZE, 0, 0);
    arm_rfft_q15(&fft, signal, FFTbuf);

    // Calculate magnitude
    arm_cmplx_mag_q15(FFTbuf, Magbuf, ADC_BUFFER_SIZE);
    //Calculate maximum value, do not include magnitude 0 (constant V)
    arm_max_q15(&Magbuf[1], ADC_BUFFER_SIZE - 1, &MaxValue, &MaxValueIndex);

    for (int i = 0, j = 0; i < SCREEN_WIDTH; i++, j += 3) {
      drawBar(i, Magbuf[j], MaxValue);
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // HAL_UART_Transmit(&huart3, "FFS\r\n", 5, 100);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
  HAL_UART_Transmit(&huart3, "ERRR\r\n", 6, 100);
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
  while (1) {
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
  /* User can add his own implementation to report the file name and line
     number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

