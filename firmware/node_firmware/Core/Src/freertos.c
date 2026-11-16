/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "spi.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* One accelerometer sample sent from SensorTask to UartTask */
typedef struct {
  int16_t  ax;     /* raw value, X axis */
  int16_t  ay;     /* raw value, Y axis */
  int16_t  az;     /* raw value, Z axis */
  uint32_t tick;   /* time of the measurement, in ms */
} AccelSample_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osMessageQueueId_t accelQueueHandle;

/* USER CODE END Variables */
/* Definitions for LedTask */
osThreadId_t LedTaskHandle;
const osThreadAttr_t LedTask_attributes = {
  .name = "LedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UartTask */
osThreadId_t UartTaskHandle;
const osThreadAttr_t UartTask_attributes = {
  .name = "UartTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* Read one MPU9250 register over SPI1 */
static uint8_t mpu_read(uint8_t reg)
{
  uint8_t tx[2] = { reg | 0x80, 0x00 };   /* bit 7 = 1 means "read" */
  uint8_t rx[2] = { 0 };
  HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET); /* select */
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);
  HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);   /* release */
  return rx[1];                            /* 2nd byte = register value */
}

/* Write one MPU9250 register over SPI1 */
static void mpu_write(uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = { reg & 0x7F, value };  /* bit 7 = 0 means "write" */
  HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, tx, 2, 100);
  HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);
}
/* Read several MPU registers in a row (burst read) */
static void mpu_read_burst(uint8_t reg, uint8_t *buf, uint16_t len)
{
  uint8_t addr = reg | 0x80;                                    /* read */
  HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &addr, 1, 100);
  HAL_SPI_Receive(&hspi1, buf, len, 100);
  HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);
}

/* USER CODE END FunctionPrototypes */

void StartLedTask(void *argument);
void StartUartTask(void *argument);
void StartSensorTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	  /* Queue of 32 samples between SensorTask and UartTask */
	  accelQueueHandle = osMessageQueueNew(32, sizeof(AccelSample_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LedTask */
  LedTaskHandle = osThreadNew(StartLedTask, NULL, &LedTask_attributes);

  /* creation of UartTask */
  UartTaskHandle = osThreadNew(StartUartTask, NULL, &UartTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartLedTask */
/**
  * @brief  Function implementing the LedTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLedTask */
void StartLedTask(void *argument)
{
  /* USER CODE BEGIN StartLedTask */
  for(;;)
  {
    HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);   /* green LED = system alive */
    osDelay(500);
  }
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartUartTask */
/* Receives samples from the queue and prints 1 out of 10 (10 lines/s) */
/* USER CODE END Header_StartUartTask */
void StartUartTask(void *argument)
{
  /* USER CODE BEGIN StartUartTask */
  setvbuf(stdout, NULL, _IONBF, 0);
  printf("\r\n=== node_firmware started ===\r\n");

  AccelSample_t s;
  uint32_t n = 0;
  for(;;)
  {
    /* wait until SensorTask puts a sample in the queue */
    if (osMessageQueueGet(accelQueueHandle, &s, NULL, osWaitForever) == osOK)
    {
      if (++n % 10 == 0)
      {
        /* +-2 g range: 16384 raw units = 1 g = 1000 mg */
        printf("t=%lu ms  ax=%d mg  ay=%d mg  az=%d mg\r\n", s.tick,
               s.ax * 1000 / 16384, s.ay * 1000 / 16384, s.az * 1000 / 16384);
      }
    }
  }
  /* USER CODE END StartUartTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/* Reads the MPU accelerometer every 10 ms and sends the sample to the queue */
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{  /* Keep the on-board LIS3DSH deselected: it shares SPI1 */
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

	  /* --- MPU init (no reset: like the working bare-metal version) --- */
	  osDelay(100);              /* let the sensor power up */
	  mpu_write(0x6A, 0x10);     /* USER_CTRL: SPI only */
	  mpu_write(0x6B, 0x01);     /* PWR_MGMT_1: awake, best clock source */
	  mpu_write(0x1C, 0x00);     /* ACCEL_CONFIG: +-2 g */
	  printf("MPU WHO_AM_I = 0x%02X\r\n", mpu_read(0x75));

  uint32_t next = osKernelGetTickCount();
  for(;;)
  {
    uint8_t raw[6];
    mpu_read_burst(0x3B, raw, 6);   /* ACCEL_XOUT_H ... ACCEL_ZOUT_L */

    AccelSample_t s;
    s.ax   = (int16_t)((raw[0] << 8) | raw[1]);
    s.ay   = (int16_t)((raw[2] << 8) | raw[3]);
    s.az   = (int16_t)((raw[4] << 8) | raw[5]);
    s.tick = osKernelGetTickCount();

    osMessageQueuePut(accelQueueHandle, &s, 0, 0);  /* don't wait if full */

    next += 10;
    /* Every 1 s: check that the MPU is still awake */

    osDelayUntil(next);             /* exactly every 10 ms */
  }
  /* USER CODE END StartSensorTask */
}
/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

