/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
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
#include"tim.h"
#include"adc.h"
#include<math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define Servo_Min_angle 60.0f
#define Servo_Max_angle 120.0f
#define Servo_Step 5.0f
TaskHandle_t Servo_Handle;
#define Servo_Task_priority 5
#define Servo_Task_Stack_Size 128
TaskHandle_t Battery_Handle;
#define Battery_Task_priority 4
#define Battery_Task_Stack_Size 128
TaskHandle_t LED_Handle;
#define LED_Task_priority 3
#define LED_Task_Stack_Size 128
TaskHandle_t Control_Handle;
#define Control_Task_priority 6
#define Control_Task_Stack_Size 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern volatile uint16_t ppm_recv[];
extern volatile uint8_t ppm_flag;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Servo_Task(void *para);
void Battery_Task(void *para);
void LED_Task(void *para);
void Control_Task(void *para);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  xTaskCreate(Servo_Task, "Servo_Task", Servo_Task_Stack_Size, NULL, Servo_Task_priority, &Servo_Handle);
  xTaskCreate(Battery_Task, "Battery_Task", Battery_Task_Stack_Size, NULL, Battery_Task_priority, &Battery_Handle);
  xTaskCreate(LED_Task, "LED_Task", LED_Task_Stack_Size, NULL, LED_Task_priority, &LED_Handle);
  xTaskCreate(Control_Task, "Control_Task", Control_Task_Stack_Size, NULL, Control_Task_priority, &Control_Handle);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
volatile float ctrl_bias = 0.0f;  /* 转向偏差：遥控器CH1，±15° */

static uint32_t AngleToPulse(float angle)
{
  return (uint32_t)((angle / 180.0f) * 2000.0f + 500.0f);
}
void Servo_Task(void *para)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  float center = (Servo_Max_angle + Servo_Min_angle) / 2.0f;
  float amplitude = (Servo_Max_angle - Servo_Min_angle) / 2.0f;
  float phase = 0.0f;
  float phase_step = 2.0f * 3.14159265f / ((Servo_Max_angle - Servo_Min_angle) / Servo_Step * 2.0f);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  while(1)
  {
    float angle = center + amplitude * sinf(phase);
    float bias = ctrl_bias;
    uint32_t pulse1=AngleToPulse(angle + bias);
    uint32_t pulse2=AngleToPulse(180.0f - angle - bias);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pulse1);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,pulse2);
    phase += phase_step;
    if(phase >= 2.0f * 3.14159265f)
      phase -= 2.0f * 3.14159265f;
    vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(10));
  }
}
volatile float voltage;
void Battery_Task(void*para)
{
  static volatile uint16_t adc_value[1];
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc_value,1);
  while(1)
  {
    voltage=adc_value[0]*3.3f/4095;
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
void LED_Task(void*para)
{
  while(1)
  {
    if(voltage<3.0f)
    {
      HAL_GPIO_WritePin(Charge_LED_GPIO_Port,Charge_LED_Pin,GPIO_PIN_SET);
      vTaskDelay(pdMS_TO_TICKS(500));
      HAL_GPIO_WritePin(Charge_LED_GPIO_Port,Charge_LED_Pin,GPIO_PIN_RESET);
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    else
    {
      HAL_GPIO_WritePin(Charge_LED_GPIO_Port,Charge_LED_Pin,GPIO_PIN_SET);
      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }
}
void Control_Task(void*para)
{
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_4);
  while(1)
  {
    if(ppm_flag)
    {
      ppm_flag=0;
      /* CH1左右摇杆 -> 转向：1500us居中，±500us -> ±15° */
      ctrl_bias = (ppm_recv[0] - 1500) / 500.0f * 15.0f;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
/* USER CODE END Application */

