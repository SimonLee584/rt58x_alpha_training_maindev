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

/* Recursive function to intentionally exhaust the task stack. */
static void Recursive_Overflow_Function(void)
{
    /* Use some stack space each call to speed up overflow. */
    volatile uint8_t dummy[256];

    dummy[0] = 0; /* Prevent optimization. */
    
    static uint32_t call_count = 0;
    call_count++;
    printf("Stack overflow test function call count: %lu\r\n", call_count);
    Recursive_Overflow_Function();
}

/* Task that will trigger a stack overflow via infinite recursion. */
void StackOverflow_Test_Task(void *pvParameters)
{
    (void)pvParameters;

    printf("Stack overflow test task start\r\n");

    /* This call should never return and will eventually overflow the stack. */
    Recursive_Overflow_Function();

    /* Should not reach here, but keep API usage correct. */
    vTaskDelete(NULL);
}

/* Task that deliberately dereferences a NULL pointer to generate a fault. */
void NullPointer_Test_Task(void *pvParameters)
{
    (void)pvParameters;

    printf("Now trigger NULL pointer fault\r\n");

    Delay_ms(10);
    /* 故意对空指针写入，制造硬件异常。 */
    volatile uint32_t *p = (uint32_t *)0x00000000u;
    *p = 0x11223344u;

    /* 理论上不会执行到这里，只是为了防止优化器合并/删除上面的访问。 */
    printf("This line should never be printed\r\n");
    vTaskDelete(NULL);
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

    xTaskCreate(Led_Task, "LedTask", 128, NULL, 1, NULL);

    /* Create a task with normal stack size to deliberately cause a stack overflow by recursion. */
    xTaskCreate(StackOverflow_Test_Task, "SOFTask", 128, NULL, 2, NULL);

    /* Create a task that will trigger a fault by dereferencing a NULL pointer. */
    // xTaskCreate(NullPointer_Test_Task, "NullTask", 128, NULL, 2, NULL);

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

/* FreeRTOS stack overflow hook: called when configCHECK_FOR_STACK_OVERFLOW != 0. */
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    (void)xTask;

    /* Report which task overflowed and halt here for debugging. */
    printf("*** Stack overflow detected in task: %s ***\r\n", pcTaskName);

    taskDISABLE_INTERRUPTS();
    for( ;; )
    {
    }
}
