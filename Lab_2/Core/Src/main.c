/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************

  Developers:
  Serebrennikov A.
  Dobrotin A.
  Taralo A.
  
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "BME280.h"
#include "vl6180.h"
#include "Accel.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STACK_SIZE 128

#define P_THS 120000
#define T_THS 27
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile int8_t switch_state = -1;
volatile int8_t new_switch_state;
volatile uint32_t Tim2Cnt=0;
volatile uint32_t TickCnt;
volatile uint16_t SwitchCnt=0;
extern State_ptr VL6180_State;
extern VL6180x_AlsData_t Als;

QueueHandle_t queue_uart;
SemaphoreHandle_t xSemaphore;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

TaskHandle_t BlueTaskHandle;

void GreenTask(void *argument);
void BlueTask(void *argument);
void BMETask(void *argument);
void VLXTask(void *argument);
void ACCTask(void *argument);
void UART_print(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  xTaskCreate(GreenTask, "GreenTask", 32, NULL, tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate(BlueTask, "BlueTask", 32, NULL, tskIDLE_PRIORITY + 2, &BlueTaskHandle);
  xTaskCreate(BMETask, "BMETask", 256, NULL, tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate(VLXTask, "VLXTask", 128, NULL, tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate(ACCTask, "ACCTask", 150, NULL, tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate(UART_print, "UART_print", 80, NULL, tskIDLE_PRIORITY + 2, NULL);

  queue_uart = xQueueCreate( 256, 32 );
  xSemaphore = xSemaphoreCreateMutex();

  vTaskSuspend(BlueTaskHandle);
  vTaskStartScheduler();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void GreenTask(void *argument)
{
  while(1)
  {
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	  vTaskDelay(1000/ portTICK_PERIOD_MS);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    vTaskDelay(1000/ portTICK_PERIOD_MS);
  }
  	vTaskDelete(NULL);
}

void BlueTask(void *argument)
{
  while(1)
  {
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	  vTaskDelay(100/ portTICK_PERIOD_MS);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    vTaskDelay(100/ portTICK_PERIOD_MS);
  }
	vTaskDelete(NULL);
}

void BMETask(void *argument)
{
  volatile uint32_t pressure = 0;
  volatile float temperature = 0;
  uint8_t str_tx[32];

  BME280_Init();

  while(1)
  {
    pressure = BME280_ReadPressure();
    temperature = BME280_ReadTemperature();

    sprintf(str_tx, "Temp: %2.2fC  Press: %dPa\n\n", temperature, pressure);

    xQueueSend(queue_uart, str_tx, (( TickType_t ) 10 ) != pdPASS);
    //HAL_UART_Transmit(&huart1, str, strlen(str), 1000);

    if((pressure > P_THS) || (temperature > T_THS))
    {
      vTaskResume(BlueTaskHandle);
      sprintf(str_tx, "Warning! Check data!  \n\n");
      xQueueSend(queue_uart, str_tx, (( TickType_t ) 10 ) != pdPASS);
      ///HAL_UART_Transmit(&huart1, str, strlen(str), 1000);
    }
    else
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
      vTaskSuspend(BlueTaskHandle);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
	vTaskDelete(NULL);
}

void VLXTask(void *argument)
{
  uint8_t str_tx[32];
  xSemaphoreTake( xSemaphore, portMAX_DELAY );
  if (stmpe1600_ini())
  {
		if(vl6180_ini())
    {
      xSemaphoreGive( xSemaphore);
      vTaskDelete(NULL);
    }
  }
  xSemaphoreGive( xSemaphore);

  while(1)
  {
    xSemaphoreTake( xSemaphore, portMAX_DELAY );
    vl6180_ReadData();
    xSemaphoreGive( xSemaphore);

    sprintf(str_tx, "als: %lu \n\n", Als.lux); 
    xQueueSend(queue_uart, str_tx, (( TickType_t ) 10 ) != pdPASS);

    vTaskDelay(1000/ portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);

}

void ACCTask(void *argument)
{
  int16_t x, y, z;
  uint8_t str_tx[32];
  LIS331_t lis331;
  lis331.mode = USE_I2C; 
  lis331.address = 0x18;

  xSemaphoreTake( xSemaphore, portMAX_DELAY );
  LIS331_Init(&lis331, USE_I2C);
  xSemaphoreGive( xSemaphore);
  
  while(1)
  {
    xSemaphoreTake( xSemaphore, portMAX_DELAY );
    LIS331_ReadAxes(&lis331, &x, &y, &z);
    xSemaphoreGive( xSemaphore);
    
    sprintf(str_tx, "x: %d  y: %d  z: %d\n\n", x, y, z); 

    xQueueSend(queue_uart, str_tx, portMAX_DELAY);

    vTaskDelay(1000/ portTICK_PERIOD_MS);
  }

  vTaskDelete(NULL);
}

void UART_print(void *argument)
{
  const uint8_t rx_str[32];

  while(1)
  {
    xQueueReceive(queue_uart, (void* const)rx_str, portMAX_DELAY);
    HAL_UART_Transmit(&huart1, rx_str, strlen((const char *)rx_str), 1000);
  }

  vTaskDelete(NULL);

}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
