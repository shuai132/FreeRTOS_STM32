/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include "adc.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId myTask_LEDHandle;
osThreadId myTask_UARTHandle;

/* USER CODE BEGIN Variables */
extern uint16_t ADC_Value[];

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartTask_LED(void const * argument);
void StartTask_UART(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
    
   HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
}
/* USER CODE END 2 */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  printf("MX_FREERTOS_Init...\r\n");
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

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask_LED */
  osThreadDef(myTask_LED, StartTask_LED, osPriorityIdle, 0, 128);
  myTask_LEDHandle = osThreadCreate(osThread(myTask_LED), NULL);

  /* definition and creation of myTask_UART */
  osThreadDef(myTask_UART, StartTask_UART, osPriorityIdle, 0, 128);
  myTask_UARTHandle = osThreadCreate(osThread(myTask_UART), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
    UBaseType_t FreeStack;
    float temperature;
    uint32_t GROUP=NVIC_PRIORITYGROUP_4, PP=0, SP=0;
  
    //测试优先级分配是否和预期一致
    HAL_NVIC_GetPriority(SysTick_IRQn, GROUP, &PP, &SP);
    printf("SysTick_IRQn    PP=%d,SP=%d\r\n", PP, SP);
    HAL_NVIC_GetPriority(PendSV_IRQn, GROUP, &PP, &SP);
    printf("PendSV_IRQn     PP=%d,SP=%d\r\n", PP, SP);
    HAL_NVIC_GetPriority(DMA1_Channel1_IRQn, GROUP, &PP, &SP);
    printf("DMA2_Stream0_IRQn   PP=%d,SP=%d\r\n", PP, SP);
    
  /* Infinite loop */
  for(;;)
  {
    FreeStack = uxTaskGetStackHighWaterMark(NULL);
    printf("TASK_%s FREE STACK:%ld\r\n", "Default", FreeStack);
      
    printf("ADC_Value[0] = %d\r\n", ADC_Value[0]);
    printf("ADC_Value[1] = %d\r\n", ADC_Value[1]);
    printf("ADC_Value[2] = %d\r\n", ADC_Value[2]);
      
    temperature = (float)ADC_Value[2]*(3.3f/4095);
    temperature=(1.43f - temperature)/0.0043f + 25;
    printf("CPU Temperature = %f\r\n", temperature);
    printf("C1:%f\r\n", temperature);
      
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* StartTask_LED function */
void StartTask_LED(void const * argument)
{
  /* USER CODE BEGIN StartTask_LED */
  UBaseType_t FreeStack;
  //uint32_t a[38]={0};
  /* Infinite loop */
  for(;;)
  {
    FreeStack = uxTaskGetStackHighWaterMark(NULL);
    printf("TASK_%s FREE STACK:%ld\r\n", "LED", FreeStack);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
    osDelay(1000);
  }
  /* USER CODE END StartTask_LED */
}

/* StartTask_UART function */
void StartTask_UART(void const * argument)
{
  /* USER CODE BEGIN StartTask_UART */
  UBaseType_t FreeStack;
  /* Infinite loop */
  for(;;)
  {
    FreeStack = uxTaskGetStackHighWaterMark(NULL);
    printf("TASK_%s FREE STACK:%ld\r\n", "UART", FreeStack);
    osDelay(1000);
  }
  /* USER CODE END StartTask_UART */
}

/* USER CODE BEGIN Application */
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
