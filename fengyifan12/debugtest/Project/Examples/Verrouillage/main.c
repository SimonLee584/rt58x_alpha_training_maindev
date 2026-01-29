/** @file main.c
 *
 * @brief FreeRTOS example main file.
 *        Demonstrate creating FreeRTOS task and use GPIO to wake up enter sleep task.
 *        FreeRTOS porting and basic operation.
 *
 */
/**
 * @defgroup freertos_example_group FreeRTOS
 * @ingroup examples_group
 * @{
 * @brief FreeRTOS example demonstrate
 */

/**************************************************************************************************
 *    INCLUDES
 *************************************************************************************************/
#include <stdio.h>
#include <string.h>
#include "cm3_mcu.h"
#include "project_config.h"
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "uart_drv.h"
#include "retarget.h"
#include "comm_subsystem_drv.h"
#include "rf_mcu_ahb.h"

#include "key_handler.h"

#include "semphr.h"
#include "mpu_wrappers.h"


#define NUM_TIMERS    5

#define SUBSYSTEM_CFG_PMU_MODE              0x4B0
#define SUBSYSTEM_CFG_LDO_MODE_DISABLE      0x02
#define GPIO20                              20
/*
 * Remark: UART_BAUDRATE_115200 is not 115200...Please don't use 115200 directly
 * Please use macro define  UART_BAUDRATE_XXXXXX
 */
#define PRINTF_BAUDRATE      UART_BAUDRATE_115200

/**
 * @ingroup freertos_example_group
 * @brief this is pin mux setting for message output
 *
 */
void Init_Default_Pin_Mux(void)
{
    int i;

    /*set all pin to gpio, except GPIO16, GPIO17 */
    for (i = 0; i < 32; i++)
    {
        if ((i != 16) && (i != 17) && (i != 20))
        {
            pin_set_mode(i, MODE_GPIO);
        }
    }

    /*uart0 pinmux*/
    pin_set_mode(16, MODE_UART);     /*GPIO16 as UART0 RX*/
    pin_set_mode(17, MODE_UART);     /*GPIO17 as UART0 TX*/

    gpio_cfg_output(GPIO20);
    return;
}
void Comm_Subsystem_Disable_LDO_Mode(void)
{
    uint8_t reg_buf[4];

    RfMcu_MemoryGetAhb(SUBSYSTEM_CFG_PMU_MODE, reg_buf, 4);
    reg_buf[0] &= ~SUBSYSTEM_CFG_LDO_MODE_DISABLE;
    RfMcu_MemorySetAhb(SUBSYSTEM_CFG_PMU_MODE, reg_buf, 4);
}

// 定义两个互斥锁
SemaphoreHandle_t MutexA = NULL;
SemaphoreHandle_t MutexB = NULL;

// 线程1：持有MutexA，等待MutexB
void Task1(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(MutexA, portMAX_DELAY) == pdPASS) {
            printf("Task1: get MutexA,wait MutexB...\n");
            vTaskDelay(10);

            xSemaphoreTake(MutexB, portMAX_DELAY); 
            printf("Task1: get MutexB(will not execute)\n");
            

            xSemaphoreGive(MutexB);
            xSemaphoreGive(MutexA);
        }
        vTaskDelay(1000);
    }
}

// 线程2：持有MutexB，等待MutexA
void Task2(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(MutexB, portMAX_DELAY) == pdPASS) {
            printf("Task2: get MutexB,wait MutexA...\n");
            vTaskDelay(10);

            xSemaphoreTake(MutexA, portMAX_DELAY); 
            printf("Task2: get MutexA(will not execute)\n");
            
            xSemaphoreGive(MutexA);
            xSemaphoreGive(MutexB);
        }
        vTaskDelay(1000);
    }
}

void Led_Task(void *pvParameters)
{
    while (1)
    {
        gpio_pin_set(GPIO20);
        Delay_ms(200);
        gpio_pin_clear(GPIO20);
        Delay_ms(200);
    }
}

void TaskMonitor(void *pvParameters)
{
    char pcWriteBuffer[256];

    while (1)
    {
        vTaskList(pcWriteBuffer);
        printf("\nTask List:\n%s\n", pcWriteBuffer);

        vTaskDelay(1000);
    }
}

int main(void)
{
#if (TIMER_CALLBACK_EXAMPLE == ENABLE)
    uint32_t x;
#endif

    /*we should set pinmux here or in SystemInit */
    Change_Ahb_System_Clk(SYS_48MHZ_CLK);

    Init_Default_Pin_Mux();

    /*init debug uart port for printf*/
    console_drv_init(PRINTF_BAUDRATE);

    Comm_Subsystem_Disable_LDO_Mode();//if don't load 569 FW, need to call the function.
    /*init delay function*/
    Key_Init();
    Delay_Init();

    printf("FreeRTOS Demo build %s %s \n", __DATE__, __TIME__);

    // 创建互斥锁
    MutexA = xSemaphoreCreateMutex();
    MutexB = xSemaphoreCreateMutex();

    // 创建线程
    xTaskCreate(Task1, "Task1", 128, NULL, 2, NULL);
    xTaskCreate(Task2, "Task2", 128, NULL, 2, NULL);
    xTaskCreate(Led_Task, "LedTask", 128, NULL, 1, NULL);
    xTaskCreate(TaskMonitor, "MonTask", 256, NULL, 2, NULL);

    /* Start the scheduler. */
    vTaskStartScheduler();

    while (1)
    {
    }
}

/* Static allocation functions for FreeRTOS */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
