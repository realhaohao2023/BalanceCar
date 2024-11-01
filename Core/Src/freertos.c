/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "semphr.h"
#include "IIC.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "stdio.h"
#include "string.h"
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
/* USER CODE BEGIN Variables */

int16_t sLeft, sRight;

//���������ݱ���
float pitch,roll,yaw;
short gyrox,gyroy,gyroz;
short	aacx,aacy,aacz;

// //�ջ������м����
// int Vertical_out,Velocity_out,Target_Speed;
// float Med_Angle=0;//ƽ��ʱ�Ƕ�ֵƫ��������е��ֵ��


// float P_value = 0.0;
// float I_value = 0.0;
// float D_value = 0.0;

// //���� extern ���ܸ�ֵ
// extern float Vertical_Kp,Vertical_Kd;			//ֱ���� ��������Kp��0~1000��Kd��0~10��PD������
// extern float Velocity_Kp,Velocity_Ki;		    //�ٶȻ� ��������Kp��0~1��             PI������
// extern float Turn_Kp,Turn_Kd;						//ת��

extern PID Balance_PID;  //ֱ����
extern PID Speed_PID;  //�ٶȻ�
float Balance_Pwm;  //ֱ�������PWM
float velocity_Pwm;  //�ٶȻ����PWM
extern int8_t Angle_Balance;  

int Motor_Left,Motor_Right;      //���PWM����,��������PID������ֵ
//extern  Encoder_Left,Encoder_Right;



/* USER CODE END Variables */
/* Definitions for TaskDataGet */
osThreadId_t TaskDataGetHandle;
const osThreadAttr_t TaskDataGet_attributes = {
  .name = "TaskDataGet",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for TaskSpeedPID */
osThreadId_t TaskSpeedPIDHandle;
const osThreadAttr_t TaskSpeedPID_attributes = {
  .name = "TaskSpeedPID",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* Definitions for myTaskLedFlash */
osThreadId_t myTaskLedFlashHandle;
const osThreadAttr_t myTaskLedFlash_attributes = {
  .name = "myTaskLedFlash",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* Definitions for myTaskDraw */
osThreadId_t myTaskDrawHandle;
const osThreadAttr_t myTaskDraw_attributes = {
  .name = "myTaskDraw",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTaskScanKeys */
osThreadId_t myTaskScanKeysHandle;
const osThreadAttr_t myTaskScanKeys_attributes = {
  .name = "myTaskScanKeys",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskPIDDataChan */
osThreadId_t TaskPIDDataChanHandle;
const osThreadAttr_t TaskPIDDataChan_attributes = {
  .name = "TaskPIDDataChan",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myQueueKeys */
osMessageQueueId_t myQueueKeysHandle;
const osMessageQueueAttr_t myQueueKeys_attributes = {
  .name = "myQueueKeys"
};
/* Definitions for SemSpeedPID */
osSemaphoreId_t SemSpeedPIDHandle;
const osSemaphoreAttr_t SemSpeedPID_attributes = {
  .name = "SemSpeedPID"
};
/* Definitions for PIDDataGet */
osSemaphoreId_t PIDDataGetHandle;
const osSemaphoreAttr_t PIDDataGet_attributes = {
  .name = "PIDDataGet"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void DataGet(void *argument);
void SpeedPID(void *argument);
void running_led_task(void *argument);
void OledDraw(void *argument);
void ScanKeys(void *argument);
void PIDDataChange(void *argument);

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

  /* Create the semaphores(s) */
  /* creation of SemSpeedPID */
  SemSpeedPIDHandle = osSemaphoreNew(1, 1, &SemSpeedPID_attributes);

  /* creation of PIDDataGet */
  PIDDataGetHandle = osSemaphoreNew(1, 1, &PIDDataGet_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueueKeys */
  myQueueKeysHandle = osMessageQueueNew (16, sizeof(uint8_t), &myQueueKeys_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TaskDataGet */
  TaskDataGetHandle = osThreadNew(DataGet, NULL, &TaskDataGet_attributes);

  /* creation of TaskSpeedPID */
  TaskSpeedPIDHandle = osThreadNew(SpeedPID, NULL, &TaskSpeedPID_attributes);

  /* creation of myTaskLedFlash */
  myTaskLedFlashHandle = osThreadNew(running_led_task, NULL, &myTaskLedFlash_attributes);

  /* creation of myTaskDraw */
  myTaskDrawHandle = osThreadNew(OledDraw, NULL, &myTaskDraw_attributes);

  /* creation of myTaskScanKeys */
  myTaskScanKeysHandle = osThreadNew(ScanKeys, NULL, &myTaskScanKeys_attributes);

  /* creation of TaskPIDDataChan */
  TaskPIDDataChanHandle = osThreadNew(PIDDataChange, NULL, &TaskPIDDataChan_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_DataGet */
/**
  * @brief  Function implementing the TaskDataGet thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_DataGet */
void DataGet(void *argument)
{
  /* USER CODE BEGIN DataGet */
  float kLeft = 3.53;
  float kRight = 2.15;

  int8_t pitch_buf[20];
  int8_t gyroz_buf[20];

  // ����������ڴ洢��һ�εĽǶ�ֵ��ʱ���
  float last_pitch = 0.0f;
  TickType_t last_time = 0;

  /* Infinite loop */
  for(;;)
  {
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();

    //��ȡ����������
    sLeft = __HAL_TIM_GetCounter(&htim2);
    __HAL_TIM_SetCounter(&htim2, 0);
    sRight = __HAL_TIM_GetCounter(&htim3);
    __HAL_TIM_SetCounter(&htim3, 0);

    // �ٶȼ���,�����������ٶ�ϵ���������ٶ�ֵ��Ϊ100
    sLeft = sLeft / kLeft;
    sRight = -sRight / kRight;

    //printf("%d,%d\n", sLeft, sRight);

    //��ȡ����������
    mpu_dmp_get_data(&pitch,&roll,&yaw);
	  //MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
	  //MPU_Get_Accelerometer(&aacx,&aacy,&aacz);


    // ������ٶ�
    TickType_t current_time = xTaskGetTickCount();
    if (current_time - last_time >= pdMS_TO_TICKS(100)) {
      float delta_time = (current_time - last_time) / (float)configTICK_RATE_HZ;
      float angular_velocity = (pitch - last_pitch) / delta_time;

      // ������һ�εĽǶ�ֵ��ʱ���
      last_pitch = pitch;
      last_time = current_time;

      // ʹ�ü�����Ľ��ٶ�
      gyroz = (int)angular_velocity;
    }


    
    sprintf((char *)pitch_buf, "%d", (int)pitch);  
    sprintf((char *)gyroz_buf, "%d", (int)gyroz);

    //OLED_ShowString(1, 8, (char *)pitch_buf);
    //OLED_ShowString(2, 8, (char *)aacz_buf);

    

    Angle_Balance = pitch;
    //printf("pitch=%d,gyroz=%d\n",pitch,gyroz);
    //printf("angle_balance=%d",Angle_Balance);

    vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(100));

    xSemaphoreGive(SemSpeedPIDHandle);//��ȡ������������������ݺ��ͷ��ź���
  }
  /* USER CODE END DataGet */
}

/* USER CODE BEGIN Header_SpeedPID */
/**
* @brief Function implementing the TaskSpeedPID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SpeedPID */
void SpeedPID(void *argument)
{
  /* USER CODE BEGIN SpeedPID */
  //Ϊ����oled����ʾ����������������ת��Ϊ�ַ���
  // uint8_t Vertical_Kp_buf[20];
  // uint8_t Vertical_Kd_buf[20];
  // uint8_t Velocity_Kp_buf[20];
  // uint8_t Velocity_Ki_buf[20];
  
  /* Infinite loop */
  for(;;)
  {
    if (xSemaphoreTake(SemSpeedPIDHandle, pdMS_TO_TICKS(100)) == pdTRUE)
    {
      //PID��������

      //Target_Speed = 0;//�����ٶ�

      // if (P_value != 0.0)//���Pֵ��Ϊ0��˵��PID�����Ѿ�����λ������
      // {


      //   Vertical_Kp =  P_value;//ֱ���� ��������Kp��0~1000��Kd��0~10��PD������
      //   Vertical_Kd =  D_value;//ֱ���� ��������Kp��0~1000��Kd��0~10��PD������
      //   //Velocity_Kp =  P_value;//�ٶȻ� ��������Kp��0~1��             PI������
      //   //Velocity_Ki =  I_value;//�ٶȻ� ������             PI������

      // }

      // sprintf((char *)Vertical_Kp_buf, "%.3f", Vertical_Kp);
      // sprintf((char *)Vertical_Kd_buf, "%.3f", Vertical_Kd);
      // sprintf((char *)Velocity_Kp_buf, "%.3f", Velocity_Kp);
      // sprintf((char *)Velocity_Ki_buf, "%.3f", Velocity_Ki);

      // OLED_ShowString(1, 0, (char *)Vertical_Kp_buf);
      // OLED_ShowString(2, 0, (char *)Vertical_Kd_buf);
      // OLED_ShowString(3, 0, (char *)Velocity_Kp_buf);
      // OLED_ShowString(4, 0, (char *)Velocity_Ki_buf);

      //  OLED_ShowNum(1, 0, Vertical_Kp, 4); 
      //  OLED_ShowNum(2, 0, Vertical_Kd, 4);
      //  OLED_ShowNum(3, 0, Velocity_Kp, 4);
      //  OLED_ShowNum(4, 0, Velocity_Ki, 4);
    //  //OLED_ShowString(2, 0, ".");

      //�����ݴ���PID�������������������������ҵ��ת��ֵ
	    // Velocity_out=Velocity(Target_Speed,sLeft,sRight);
	    // Vertical_out=Vertical(Velocity_out+Med_Angle, pitch, gyroz);
	    
	    //���õ��ת��
      //set_speed(Velocity_out,Velocity_out);
	    //set_speed(Vertical_out,Vertical_out);
      //set_speed(100, 100);

      //�ڶ��ַ���

      
      Balance_Pwm = PID_Balance_Calc(&Balance_PID, pitch, gyroz);
      Balance_Pwm = Balance_Pwm * 0.8;
      //Balance_Pwm = PWM_Limit(Balance_Pwm, 40, -40);
      //printf("balance_pwm=%f\n",Balance_Pwm);


      //set_speed(-Balance_Pwm, -Balance_Pwm);

      velocity_Pwm=PID_Speed_Calc(&Speed_PID,sLeft,sRight);
      velocity_Pwm = velocity_Pwm * 0.8;
      //printf("velocity_Pwm=%f\n",velocity_Pwm);
      //set_speed(velocity_Pwm,velocity_Pwm);

      Motor_Left = velocity_Pwm - Balance_Pwm;
      Motor_Right = velocity_Pwm - Balance_Pwm;

      Motor_Left = PWM_Limit(Motor_Left, 40, -40);
      Motor_Right = PWM_Limit(Motor_Right, 40, -40);

      //printf("%d\n",Motor_Left);
      printf("balance_pwm=%f,velocity_pwm=%f,motor_left=%d\n",Balance_Pwm,velocity_Pwm,Motor_Left);

      set_speed(Motor_Left, Motor_Right);
      //set_speed(100, 100);

      


      
	   
    }
    osDelay(100);
  }
  /* USER CODE END SpeedPID */
}

/* USER CODE BEGIN Header_running_led_task */
  /**
   * @brief Function implementing the myTaskLedFlash thread.
   * @param argument: Not used
   * @retval None
   */
/* USER CODE END Header_running_led_task */
void running_led_task(void *argument)
{
  /* USER CODE BEGIN running_led_task */
    /* Infinite loop */
    for (;;)
    {
      led1_on();
      vTaskDelay(pdMS_TO_TICKS(100));
      led1_off();

      led2_on();
      vTaskDelay(pdMS_TO_TICKS(100));
      led2_off();

      led3_on();
      vTaskDelay(pdMS_TO_TICKS(100));
      led3_off();

      led4_on();
      vTaskDelay(pdMS_TO_TICKS(100));
      led4_off();
      osDelay(1);
    }
  /* USER CODE END running_led_task */
}

/* USER CODE BEGIN Header_OledDraw */
  /**
   * @brief Function implementing the myTaskDraw thread.
   * @param argument: Not used
   * @retval None
   */
/* USER CODE END Header_OledDraw */
void OledDraw(void *argument)
{
  /* USER CODE BEGIN OledDraw */
    KEYS keyValue; // ����ֵ
    /* Infinite loop */
    for (;;)
    {

      if (xQueueReceive(myQueueKeysHandle, &keyValue, pdMS_TO_TICKS(50)) != pdTRUE)
      {
        continue; // 
      }

      if (keyValue == KEY1)
      {
        OLED_ShowNum(4, 9, 1, 1);
      }
      else if (keyValue == KEY2)
      {
        OLED_ShowNum(4, 9, 2, 1);
      }
      else if (keyValue == KEY3)
      {
        OLED_ShowNum(4, 9, 3, 1);
      }
      else if (keyValue == KEY4)
      {
        OLED_ShowNum(4, 9, 4, 1);
      }
      vTaskDelay(400);
    }
  /* USER CODE END OledDraw */
}

/* USER CODE BEGIN Header_ScanKeys */
  /**
   * @brief Function implementing the myTaskScanKeys thread.
   * @param argument: Not used
   * @retval None
   */
/* USER CODE END Header_ScanKeys */
void ScanKeys(void *argument)
{
  /* USER CODE BEGIN ScanKeys */
    GPIO_PinState keyState = GPIO_PIN_SET;
    KEYS key = KEY_NONE;
    /* Infinite loop */
    for (;;)
    {
      key = KEY_NONE;

      keyState = HAL_GPIO_ReadPin(GPIOB, KEY1_Pin);
      if (keyState == GPIO_PIN_RESET)
      {

        key = KEY1;
      }

      keyState = HAL_GPIO_ReadPin(GPIOB, KEY2_Pin);
      if (keyState == GPIO_PIN_RESET)
      {
        key = KEY2;
      }

      keyState = HAL_GPIO_ReadPin(GPIOB, KEY3_Pin);
      if (keyState == GPIO_PIN_RESET)
      {
        key = KEY3;
      }

      keyState = HAL_GPIO_ReadPin(GPIOA, KEY4_Pin);
      if (keyState == GPIO_PIN_RESET)
      {
        key = KEY4;
      }

      // ����ֵ���
      if (key != KEY_NONE)
      {
        BaseType_t err = xQueueSendToBack(myQueueKeysHandle, &key, pdMS_TO_TICKS(50));
        if (err == errQUEUE_FULL)
        {
          xQueueReset(myQueueKeysHandle); //  �������ˣ���ն���
        }
        vTaskDelay(300);
      }

      else
      {  
        vTaskDelay(5);
      }
    }
  /* USER CODE END ScanKeys */
}

/* USER CODE BEGIN Header_PIDDataChange */
/**
* @brief Function implementing the TaskPIDDataChan thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PIDDataChange */
void PIDDataChange(void *argument)
{
  /* USER CODE BEGIN PIDDataChange */
  char *data_format = "P:%f, I:%f, D:%f"; // ���ڽ����ĸ�ʽ�ַ���
  float P_temp, I_temp, D_temp;           // ��ʱ�洢�������� PID ����
  /* Infinite loop */
  for(;;)
  {
    if (xSemaphoreTake(PIDDataGetHandle, pdMS_TO_TICKS(100)) == pdTRUE)
    {
      if (xUSART1.ReceiveNum) 
      {
        printf("Received Data: %s\r", (char *)xUSART1.ReceiveData); // ��ʾ���յ�������
        
        // ���Խ������յ����ַ����������ɹ�ʱ���ؽ����Ĳ�������
        if (sscanf((char *)xUSART1.ReceiveData, data_format, &P_temp, &I_temp, &D_temp) == 3)
        {
          // �ɹ������� PID ��������ֵ��ȫ�ֱ���
          // P_value = P_temp;
          // I_value = I_temp;
          // D_value = D_temp;

          // ��ӡ������� PID ֵ
          // printf("Parsed PID values: P = %f, I = %f, D = %f\r", P_value, I_value, D_value);
          
        }
        else
        {
          // �������ʧ�ܣ���ʾ������Ϣ
          printf("Error: Invalid PID data format\r");
        }

        xUSART1.ReceiveNum = 0; // ??0���ձ��
      }
     
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  /* USER CODE END PIDDataChange */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart1)                                                                    // �жϴ���
    {
        BaseType_t hightaskWoken = pdFALSE; 
        __HAL_UNLOCK(huart);                                                                 // ��������״̬
 
        xUSART1.ReceiveNum  = Size;                                                          // �ѽ����ֽ���������ṹ��xUSART1.ReceiveNum���Ա�ʹ��
        memset(xUSART1.ReceiveData, 0, sizeof(xUSART1.ReceiveData));                         // ��0ǰһ֡�Ľ�������
        memcpy(xUSART1.ReceiveData, xUSART1.BuffTemp, Size);                                 // �������ݣ�����ʱ�����У����Ƶ�xUSART1.ReceiveData[], �Ա�ʹ��
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, xUSART1.BuffTemp, sizeof(xUSART1.BuffTemp));   // �ٴο���DMA�����ж�; ÿ��������ָ�����ȣ����߲��������ж�ʱ���ͻ��������
        
        
        //DMA���һ�ν��պ��ͷ��ź�����֪ͨ����������
      if (PIDDataGetHandle != NULL)
      {
        xSemaphoreGiveFromISR(PIDDataGetHandle, &hightaskWoken);                               // �ͷ��ź�����֪ͨ����������
        portYIELD_FROM_ISR(hightaskWoken);                                                   // ���hightaskWokenΪpdTRUE�������һ���������
      }





    }
}

/* USER CODE END Application */

