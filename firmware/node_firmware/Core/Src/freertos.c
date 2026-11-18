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
#include <math.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* One accelerometer sample sent from SensorTask to ProcessTask */
typedef struct {
  int16_t  ax;     /* raw value, X axis */
  int16_t  ay;     /* raw value, Y axis */
  int16_t  az;     /* raw value, Z axis */
  uint32_t tick;   /* time of the measurement, in ms */
} AccelSample_t;
/* Vibration features computed by ProcessTask over 1 s */
typedef struct {
  uint16_t rms_mg;    /* RMS of the vibration (gravity removed) */
  uint16_t peak_mg;   /* highest vibration value in the window */
  uint32_t tick;      /* end of the window, in ms */
} VibFeature_t;

/* Latest temperature, shared between TempTask and UartTask */
typedef struct {
  float   temp_c;
  uint8_t status;     /* 0 = OK, 1 = sensor not found, 2 = read error (PROTOCOL.md) */
} TempData_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VIB_WINDOW   100      /* 100 samples x 10 ms = 1 s window */
#define RAW_PER_G    16384    /* +-2 g range: 16384 raw units = 1 g */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osMessageQueueId_t accelQueueHandle;
osMessageQueueId_t featureQueueHandle;
osMutexId_t        tempMutexHandle;
TempData_t         g_temp = { 0.0f, 1 };   /* "not found" until the first reading */

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
/* Definitions for ProcessTask */
osThreadId_t ProcessTaskHandle;
const osThreadAttr_t ProcessTask_attributes = {
  .name = "ProcessTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TempTask */
osThreadId_t TempTaskHandle;
const osThreadAttr_t TempTask_attributes = {
  .name = "TempTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
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
/* ---------- microsecond delay with the DWT cycle counter ---------- */
static void dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - start) < ticks) { }
}

/* ---------- 1-Wire on PE7 (open drain + 4.7 k pull-up) ---------- */
#define OW_LOW()   HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET)
#define OW_REL()   HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET)
#define OW_READ()  HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin)

/* Reset pulse; returns 1 if a sensor answers (presence pulse) */
static uint8_t ow_reset(void)
{
  uint8_t present;
  taskENTER_CRITICAL();             /* 1-Wire timing must not be interrupted */
  OW_LOW();  delay_us(480);
  OW_REL();  delay_us(70);
  present = (OW_READ() == GPIO_PIN_RESET);
  taskEXIT_CRITICAL();
  delay_us(410);
  return present;
}

static void ow_write_bit(uint8_t b)
{
  taskENTER_CRITICAL();
  OW_LOW();
  if (b) { delay_us(6);  OW_REL(); delay_us(64); }
  else   { delay_us(60); OW_REL(); delay_us(10); }
  taskEXIT_CRITICAL();
}

static uint8_t ow_read_bit(void)
{
  uint8_t b;
  taskENTER_CRITICAL();
  OW_LOW();  delay_us(6);
  OW_REL();  delay_us(9);
  b = (OW_READ() == GPIO_PIN_SET);
  taskEXIT_CRITICAL();
  delay_us(55);
  return b;
}

static void ow_write_byte(uint8_t v)
{
  for (int i = 0; i < 8; i++) { ow_write_bit(v & 0x01); v >>= 1; }   /* LSB first */
}

static uint8_t ow_read_byte(void)
{
  uint8_t v = 0;
  for (int i = 0; i < 8; i++) { if (ow_read_bit()) v |= (1 << i); }
  return v;
}

/* Dallas/Maxim CRC-8, to check the data is not corrupted */
static uint8_t ow_crc8(const uint8_t *d, int len)
{
  uint8_t crc = 0;
  for (int i = 0; i < len; i++) {
    uint8_t in = d[i];
    for (int j = 0; j < 8; j++) {
      uint8_t mix = (crc ^ in) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      in >>= 1;
    }
  }
  return crc;
}

/* Reads the DS18B20. Returns 0 = OK, 1 = not found, 2 = read error (CRC) */
static uint8_t ds18b20_read(float *temp_c)
{
  uint8_t sp[9];
  if (!ow_reset()) return 1;
  ow_write_byte(0xCC);          /* SKIP ROM: only one sensor on the wire */
  ow_write_byte(0x44);          /* CONVERT T: start a measurement */
  osDelay(750);                 /* 12-bit conversion takes up to 750 ms */
  if (!ow_reset()) return 1;
  ow_write_byte(0xCC);
  ow_write_byte(0xBE);          /* READ SCRATCHPAD: 9 bytes */
  for (int i = 0; i < 9; i++) sp[i] = ow_read_byte();
  if (ow_crc8(sp, 8) != sp[8]) return 2;
  int16_t raw = (int16_t)((sp[1] << 8) | sp[0]);
  *temp_c = raw / 16.0f;        /* 1 LSB = 0.0625 degC */
  return 0;
}

/* USER CODE END FunctionPrototypes */

void StartLedTask(void *argument);
void StartUartTask(void *argument);
void StartSensorTask(void *argument);
void StartProcessTask(void *argument);
void StartTempTask(void *argument);

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
  tempMutexHandle = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* Queue of 32 raw samples: SensorTask -> ProcessTask */
  accelQueueHandle = osMessageQueueNew(32, sizeof(AccelSample_t), NULL);
  /* Queue of 8 vibration results: ProcessTask -> UartTask */
  featureQueueHandle = osMessageQueueNew(8, sizeof(VibFeature_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LedTask */
  LedTaskHandle = osThreadNew(StartLedTask, NULL, &LedTask_attributes);

  /* creation of UartTask */
  UartTaskHandle = osThreadNew(StartUartTask, NULL, &UartTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* creation of ProcessTask */
  ProcessTaskHandle = osThreadNew(StartProcessTask, NULL, &ProcessTask_attributes);

  /* creation of TempTask */
  TempTaskHandle = osThreadNew(StartTempTask, NULL, &TempTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartLedTask */
/**
  * @brief  Function implementing the LedTask thread: green LED = system alive.
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
/* Prints one line per second: vibration features + latest temperature */
/* USER CODE END Header_StartUartTask */
void StartUartTask(void *argument)
{
  /* USER CODE BEGIN StartUartTask */
  setvbuf(stdout, NULL, _IONBF, 0);
  printf("\r\n=== node_firmware started ===\r\n");

  VibFeature_t f;
  for(;;)
  {
    /* wait for the next 1-second vibration result */
    if (osMessageQueueGet(featureQueueHandle, &f, NULL, osWaitForever) == osOK)
    {
      TempData_t t;
      osMutexAcquire(tempMutexHandle, osWaitForever);
      t = g_temp;                                  /* copy under the mutex */
      osMutexRelease(tempMutexHandle);

      printf("t=%lu ms  vib RMS=%u mg  peak=%u mg  ", f.tick, f.rms_mg, f.peak_mg);
      if (t.status == 0)
      {
        int t100 = (int)(t.temp_c * 100.0f);       /* printf has no float support here */
        printf("temp=%d.%02d C\r\n", t100 / 100, abs(t100 % 100));
      }
      else
      {
        printf("temp=n/a (%s)\r\n", t.status == 1 ? "sensor not found" : "read error");
      }
    }
  }
  /* USER CODE END StartUartTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/* Reads the MPU accelerometer every 10 ms and sends the sample to accelQueue */
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
  /* Keep the on-board LIS3DSH deselected: it shares SPI1 */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  /* --- MPU init (no software reset: it leaves the data registers at 0 over SPI) --- */
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
    osDelayUntil(next);             /* exactly every 10 ms (100 Hz) */
  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartProcessTask */
/* Every 100 samples (1 s): removes gravity, computes vibration RMS and peak */
/* USER CODE END Header_StartProcessTask */
void StartProcessTask(void *argument)
{
  /* USER CODE BEGIN StartProcessTask */
  static int16_t bx[VIB_WINDOW], by[VIB_WINDOW], bz[VIB_WINDOW];
  int n = 0;
  AccelSample_t s;

  for(;;)
  {
    if (osMessageQueueGet(accelQueueHandle, &s, NULL, osWaitForever) != osOK) continue;
    bx[n] = s.ax;  by[n] = s.ay;  bz[n] = s.az;
    n++;
    if (n < VIB_WINDOW) continue;

    /* 1) mean of each axis = gravity + sensor offset (the "DC" part) */
    int32_t sx = 0, sy = 0, sz = 0;
    for (int i = 0; i < VIB_WINDOW; i++) { sx += bx[i]; sy += by[i]; sz += bz[i]; }
    float mx = (float)sx / VIB_WINDOW, my = (float)sy / VIB_WINDOW, mz = (float)sz / VIB_WINDOW;

    /* 2) remove it: what is left is the vibration only */
    float sum2 = 0.0f, peak = 0.0f;
    for (int i = 0; i < VIB_WINDOW; i++)
    {
      float dx = bx[i] - mx, dy = by[i] - my, dz = bz[i] - mz;
      float m2 = dx * dx + dy * dy + dz * dz;      /* squared amplitude of this sample */
      sum2 += m2;
      float m = sqrtf(m2);
      if (m > peak) peak = m;
    }

    /* 3) RMS = sqrt(mean of squares); convert raw units to mg */
    VibFeature_t f;
    f.rms_mg  = (uint16_t)(sqrtf(sum2 / VIB_WINDOW) * 1000.0f / RAW_PER_G);
    f.peak_mg = (uint16_t)(peak * 1000.0f / RAW_PER_G);
    f.tick    = s.tick;
    osMessageQueuePut(featureQueueHandle, &f, 0, 0);

    n = 0;                                          /* start the next window */
  }
  /* USER CODE END StartProcessTask */
}

/* USER CODE BEGIN Header_StartTempTask */
/* Reads the DS18B20 about once per second and shares the result under a mutex */
/* USER CODE END Header_StartTempTask */
void StartTempTask(void *argument)
{
  /* USER CODE BEGIN StartTempTask */
  dwt_init();                                       /* for the microsecond delays */
  for(;;)
  {
    float t = 0.0f;
    uint8_t st = ds18b20_read(&t);                  /* takes about 750 ms when present */

    osMutexAcquire(tempMutexHandle, osWaitForever);
    g_temp.status = st;
    if (st == 0) g_temp.temp_c = t;
    osMutexRelease(tempMutexHandle);

    osDelay(st == 0 ? 250 : 1000);                  /* about one reading per second */
  }
  /* USER CODE END StartTempTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
