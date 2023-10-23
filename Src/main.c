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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
//#include "stdlib.h"
//#include "string.h"
#include "bmp3.h"
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
uint8_t startMsg[] = "Starting I2C Scanning: \r\n";
uint8_t endMsg[] = "Done! \r\n\r\n";
uint8_t buffer[128] = {0};
uint8_t space[] = ".";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                        void *intf_ptr);
static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *intf_ptr);
static void delay_us(uint32_t period, void *intf_ptr);;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int fd, char *ch, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ch, len, HAL_MAX_DELAY);
    return len;
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  if(HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
      Error_Handler();
  }
//  HAL_UART_Transmit(&huart2, startMsg, sizeof(startMsg), 10000);
//
//  for(uint8_t i = 1; i < 128; i++)
//  {
//      uint8_t ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 3, 5);
//      if( ret != HAL_OK )
//      {
//          HAL_UART_Transmit(&huart2, space, sizeof(space), 10000);
//      }
//      else
//      {
//          sprintf((char *)buffer, "0x%X", i);
//          HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), 10000);
//      }
//  }
//  HAL_UART_Transmit(&huart2, endMsg, sizeof(endMsg), 10000);

  struct bmp3_dev the_sensor;
  struct bmp3_data data = { 0 };
  struct bmp3_settings settings = { 0 };
  struct bmp3_status status = { 0 };
  uint16_t settings_sel;
  uint16_t loop = 0;

  the_sensor.chip_id = BMP3_ADDR_I2C_SEC;
  the_sensor.intf = BMP3_I2C_INTF;
  the_sensor.read = &i2c_read;
  the_sensor.write = &i2c_write;
  the_sensor.delay_us = &delay_us;
  the_sensor.intf_ptr = &hi2c1;
  the_sensor.dummy_byte = 0;

  int8_t rslt = BMP3_OK;

  rslt = bmp3_soft_reset(&the_sensor);
  if(rslt != BMP3_OK)
  {
      Error_Handler();
  }

  rslt = bmp3_init(&the_sensor);
  if(rslt != BMP3_OK)
  {
      Error_Handler();
  }

  settings.int_settings.drdy_en = BMP3_ENABLE;
  settings.press_en = BMP3_ENABLE;
  settings.temp_en = BMP3_ENABLE;

  settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
  settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
  settings.odr_filter.odr = BMP3_ODR_100_HZ;

  settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR |
                 BMP3_SEL_DRDY_EN;

  rslt = bmp3_set_sensor_settings(settings_sel, &settings, &the_sensor);
  if(rslt != BMP3_OK)
  {
      Error_Handler();
  }

  settings.op_mode = BMP3_MODE_NORMAL;
  rslt = bmp3_set_op_mode(&settings, &the_sensor);
  if(rslt != BMP3_OK)
  {
      Error_Handler();
  }

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    rslt = bmp3_get_status(&status, &the_sensor);
    if(rslt != BMP3_OK)
    {
        Error_Handler();
    }

    if((rslt == BMP3_OK) && (status.intr.drdy == BMP3_ENABLE))
    {
        rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &the_sensor);
        if(rslt != BMP3_OK)
        {
            Error_Handler();
        }

        rslt = bmp3_get_status(&status, &the_sensor);
        if(rslt != BMP3_OK)
        {
            Error_Handler();
        }

        printf("Data[%d]  T: %ld deg C, P: %lu Pa\r\n", loop, (long int)(int32_t)(data.temperature), (long unsigned int)(uint32_t)(data.pressure / 100));
        //sprintf((char *)buffer,"Data[%d]  T: %.2f deg C, P: %.2f Pa\r\n", loop, data.temperature, data.pressure / 100.0f);
        //HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), 10000);
    }

    loop++;
    HAL_Delay(2000);
  }
  //Error_Handler();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
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
}

/* USER CODE BEGIN 4 */
/**************************************************************************/
/*!
    @brief  Reads 8 bit values over I2C
*/
/**************************************************************************/
int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    I2C_HandleTypeDef *i2c = (I2C_HandleTypeDef *)intf_ptr;
    if(HAL_I2C_Mem_Read(i2c, BMP3_ADDR_I2C_SEC << 1, reg_addr, 1, reg_data, len, 100))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**************************************************************************/
/*!
    @brief  Writes 8 bit values over I2C
*/
/**************************************************************************/
int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    I2C_HandleTypeDef *i2c = (I2C_HandleTypeDef *)intf_ptr;
    if(HAL_I2C_Mem_Write(i2c, BMP3_ADDR_I2C_SEC << 1, reg_addr, 1, (uint8_t *)reg_data, len, 100))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void delay_us(uint32_t period, void *intf_ptr)
{
    uint32_t end = TIM2->CNT + 16*period; // 16 - based on 16MHz HCLK
    while(TIM2->CNT < end)
    {
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
