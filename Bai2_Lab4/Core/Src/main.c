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
#include "cmsis_os.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_gyroscope.h"
#include "stm32f429i_discovery_io.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
uint8_t Buffer[200];
float gyrRaw[3];
float rate_gyr_x;
float rate_gyr_y;
float rate_gyr_z;
int startInt;
#define DT 0.02         // [s/loop] loop period. 20ms
#define G_GAIN 0.07     // [deg/s/LSB]
/* Private variables ---------------------------------------------------------*/
/* Definitions for Task01 */
osThreadId_t Task01Handle;
const osThreadAttr_t Task01_attributes = {
  .name = "Task01",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task02 */
osThreadId_t Task02Handle;
const osThreadAttr_t Task02_attributes = {
  .name = "Task02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* USER CODE BEGIN PV */
typedef struct
{
  float gyroXangle;
  float gyroYangle;
  float gyroZangle;
} GYRO_DATA_T;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartTask01(void *argument);
void StartTask02(void *argument);

/* USER CODE BEGIN PFP */

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
  /*LCD init*/
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(1, SDRAM_DEVICE_ADDR);
  BSP_LCD_SelectLayer(1);
  BSP_LCD_DisplayOn();
  BSP_LCD_Clear(LCD_COLOR_BLUE);
  BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (16, sizeof(uint32_t), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task01 */
  Task01Handle = osThreadNew(StartTask01, NULL, &Task01_attributes);

  /* creation of Task02 */
  Task02Handle = osThreadNew(StartTask02, NULL, &Task02_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void GYRO_Task(void)
{
  /* Initialize the L3GD20 Gyroscope */
  if (BSP_GYRO_Init() != HAL_OK)
  {
    Error_Handler();
  }
  /* Infinite loop */
  GYRO_DATA_T gyro_data;
  osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever); //Wait for signal. Check cmsis_os.c for API name.
  for (;;)
  {
    //BSP_ACCELERO_GetXYZ((float *)&gyro_data);
	BSP_GYRO_GetXYZ(gyrRaw);
	rate_gyr_x = (float) gyrRaw[0]  * G_GAIN;
	rate_gyr_y = (float) gyrRaw[1]  * G_GAIN;
	rate_gyr_z = (float) gyrRaw[2]  * G_GAIN;
	gyro_data.gyroXangle+=rate_gyr_x*DT;
	gyro_data.gyroYangle+=rate_gyr_y*DT;
	gyro_data.gyroZangle+=rate_gyr_z*DT;
    /*Send gyroscope data to USB*/
    //GYRO_DATA_T *gyro_tx;
    /*do
    {
      gyro_tx = (GYRO_DATA_T *)osMailAlloc(myMailQID, osWaitForever); //Allocate memory slot
    } while (NULL == gyro_tx);                                       //Need to check for NULL because timeout value is not used in FreeRTOS
    gyro_tx->x_ang_rate = gyro_data.x_ang_rate;
    gyro_tx->y_ang_rate = gyro_data.y_ang_rate;
    gyro_tx->z_ang_rate = gyro_data.z_ang_rate;*/
    osMessageQueuePut(myQueue01Handle, &gyro_data, 0U, 0U);//Put data into Message queue. Check cmsis_os.c for API name
    BSP_LCD_Clear(LCD_COLOR_BLUE);

    if (gyro_data.gyroXangle<0&&gyro_data.gyroYangle<0)
    {
    	BSP_LCD_FillTriangle(90,120,150,240,270,240); //Top
    	BSP_LCD_FillTriangle(60, 30, 60, 120, 150, 180); //Left
    	osDelay(1000);
    }
    if (gyro_data.gyroXangle<0&&gyro_data.gyroYangle>0)
    {
    	BSP_LCD_FillTriangle(90,120,150,240,270,240); //Top
    	BSP_LCD_FillTriangle(210,240,210,120,150,180); //Right
    	osDelay(1000);
    }
    if (gyro_data.gyroXangle>0&&gyro_data.gyroYangle<0)
    {
    	BSP_LCD_FillTriangle(90, 120, 150, 60, 30, 60); //Bot
    	BSP_LCD_FillTriangle(60, 30, 60, 120, 150, 180); //Left
    	osDelay(1000);
    }
    if (gyro_data.gyroXangle>0&&gyro_data.gyroYangle>0)
    {
    	BSP_LCD_FillTriangle(90, 120, 150, 60, 30, 60); //Bot
    	BSP_LCD_FillTriangle(210,240,210,120,150,180); //Right
    	osDelay(1000);
    }
    osDelay(1000);
  }
}

void USB_Task(void)
{

  osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever); //Wait for USB host to configure port
  osThreadFlagsSet(Task02Handle, 0x1); //Set signal in Task 02
  GYRO_DATA_T gyro_data;
  /* Infinite loop */
  for (;;)
  {
	osMessageQueueGet(myQueue01Handle, &gyro_data, NULL, 0U); //Get queue
    sprintf(Buffer,"x:%f, y:%f, z:%f \n",gyro_data.gyroXangle,gyro_data.gyroYangle,gyro_data.gyroZangle);
    CDC_Transmit_HS(Buffer, strlen(Buffer));
    osDelay(1000);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask01 */
/**
  * @brief  Function implementing the Task01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask01 */
void StartTask01(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  USB_Task();
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  GYRO_Task();
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
