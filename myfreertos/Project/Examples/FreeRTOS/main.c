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
#include "semaphore_handler.h"
#include "eventgrp_handler.h"
#include "pwm_control.h"
#include "task_config.h"
#include "tasks_and_timers.h"
#include "queue_task.h"

#include "semphr.h"
#include "mpu_wrappers.h"
/**************************************************************************************************
 *    MACROS
 *************************************************************************************************/

/**************************************************************************************************
 *    CONSTANTS AND DEFINES
 *************************************************************************************************/
#if configUSE_TICKLESS_IDLE             //Support Sleep (FreeRtosConfig.h)
#define TIMER_CALLBACK_EXAMPLE    ENABLE
#define SLEEP_WAKEUP_EXAMPLE      ENABLE
#endif

#define NUM_TIMERS    5

#define SUBSYSTEM_CFG_PMU_MODE              0x4B0
#define SUBSYSTEM_CFG_LDO_MODE_DISABLE      0x02
/*
 * Remark: UART_BAUDRATE_115200 is not 115200...Please don't use 115200 directly
 * Please use macro define  UART_BAUDRATE_XXXXXX
 */
#define PRINTF_BAUDRATE      UART_BAUDRATE_115200

/**************************************************************************************************
 *    GLOBAL VARIABLES
 *************************************************************************************************/
/* An array to hold handles to the created timers. */
TimerHandle_t timer_handle[NUM_TIMERS];

TaskHandle_t Rtos_Task1_Handle;
TaskHandle_t Rtos_Task2_Handle;
TaskHandle_t Rtos_Task3_Handle;
TaskHandle_t Led_Task_Handle;


#if (SLEEP_WAKEUP_EXAMPLE == ENABLE)
TaskHandle_t xSleepTaskHandle;
TimerHandle_t timer_task_resume_handle;
#endif


/**************************************************************************************************
 *    LOCAL FUNCTIONS
 *************************************************************************************************/
#if (SLEEP_WAKEUP_EXAMPLE == ENABLE)
static void Rtos_Sleep_Task(void *parameters_ptr);
#endif

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
        if ((i != 16) && (i != 17))
        {
            pin_set_mode(i, MODE_GPIO);
        }
    }

    /*uart0 pinmux*/
    pin_set_mode(16, MODE_UART);     /*GPIO16 as UART0 RX*/
    pin_set_mode(17, MODE_UART);     /*GPIO17 as UART0 TX*/

    return;
}
void Comm_Subsystem_Disable_LDO_Mode(void)
{
    uint8_t reg_buf[4];

    RfMcu_MemoryGetAhb(SUBSYSTEM_CFG_PMU_MODE, reg_buf, 4);
    reg_buf[0] &= ~SUBSYSTEM_CFG_LDO_MODE_DISABLE;
    RfMcu_MemorySetAhb(SUBSYSTEM_CFG_PMU_MODE, reg_buf, 4);
}
#if (TIMER_CALLBACK_EXAMPLE == ENABLE)
/* Define a callback function that will be used by multiple timer
   instances.  The callback function does nothing but count the number
   of times the associated timer expires, and stop the timer once the
   timer has expired 10 times.  The count is saved as the ID of the
   timer. */
void vTimerCallback(TimerHandle_t pxTimer)
{
    const uint32_t ulMaxExpiryCountBeforeStopping = 10;
    uint32_t ulCount;

    /* Optionally do something if the pxTimer parameter is NULL. */
    configASSERT(pxTimer);

    /* The number of times this timer has expired is saved as the timer's ID.  Obtain the count. */
    ulCount = (uint32_t) pvTimerGetTimerID(pxTimer);

    /* Increment the count, then test to see if the timer has expired ulMaxExpiryCountBeforeStopping yet. */
    ulCount++;

    /* If the timer has expired 10 times then stop it from running. */
    if (ulCount >= ulMaxExpiryCountBeforeStopping)
    {
        /* Do not use a block time if calling a timer API function
        from a timer callback function, as doing so could cause a
        deadlock! */
        xTimerStop(pxTimer, 0);
    }
    else
    {
        /* Store the incremented count back into the timer's ID field
        so it can be read back again the next time this software timer
        expires. */
        vTimerSetTimerID(pxTimer, (void *) ulCount);
    }
}

#else
/**
 * @ingroup freertos_example_group
 * @brief Timer 0 callback function
 * @param[in] timer_ptr
 * @return None
 */
void Timer0_Callback(TimerHandle_t timer_ptr)
{
    /* Optionally do something if the timer_ptr parameter is NULL. */
    configASSERT(timer_ptr);
}

/**
 * @ingroup freertos_example_group
 * @brief Timer 1 callback function
 * @param[in] timer_ptr
 * @return None
 */
void Timer1_Callback(TimerHandle_t timer_ptr)
{
    /* Optionally do something if the timer_ptr parameter is NULL. */
    configASSERT(timer_ptr);
}

/**
 * @ingroup freertos_example_group
 * @brief Timer 3 callback function
 * @param[in] timer_ptr
 * @return None
 */
void Timer2_Callback(TimerHandle_t timer_ptr)
{
    /* Optionally do something if the timer_ptr parameter is NULL. */
    configASSERT(timer_ptr);
}

#endif

#if (SLEEP_WAKEUP_EXAMPLE == ENABLE)
/**
 * @ingroup freertos_example_group
 * @brief Use gpin20 to wakeup the rtos
 * @param[in] pin
 * @param[in] isr_param
 * @return None
 */
void User_Gpio20_Isr_Handler(uint32_t pin, void *isr_param)
{
    //BaseType_t xYieldRequired;

    gpio_pin_clear(GPIO13);

    /* Resume the suspended task. */
    vTaskResume(xSleepTaskHandle);

    gpio_pin_set(GPIO13);
}

#endif

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


#if (SLEEP_WAKEUP_EXAMPLE == ENABLE)


    Comm_Subsystem_Sram_Deep_Sleep_Init();
    Comm_Subsystem_Disable_LDO_Mode();
    Lpm_Set_Low_Power_Level(LOW_POWER_LEVEL_SLEEP0);
    Lpm_Enable_Low_Power_Wakeup(LOW_POWER_WAKEUP_GPIO);

    gpio_set_debounce_time(DEBOUNCE_SLOWCLOCKS_64);

    gpio_cfg_input(GPIO20, GPIO_PIN_INT_EDGE_FALLING);
    gpio_debounce_enable(GPIO20);
    gpio_register_isr(GPIO20, User_Gpio20_Isr_Handler, NULL);
    gpio_int_enable(GPIO20);

#endif

#if (TIMER_CALLBACK_EXAMPLE == ENABLE)
    /* Create then start some timers.  Starting the timers before
    the RTOS scheduler has been started means the timers will start
    running immediately that the RTOS scheduler starts. */
    for (x = 0; x < NUM_TIMERS; x++)
    {
        if (x == 0)
        {
            timer_handle[x] = xTimerCreate
                              ( /* Just a text name, not used by the RTOS kernel. */
                                  "Timer",
                                  /* The timer period in ticks, must be greater than 0. */
                                  pdMS_TO_TICKS(100),
                                  /* The timers will auto-reload themselves when they expire. */
                                  pdTRUE,
                                  /* The ID is used to store a count of the number of times the timer has expired, which is initialised to 0. */
                                  (void *) 0,
                                  /* Each timer calls the same callback when it expires. */
                                  vTimerCallback
                              );
        }
        else
        {
            timer_handle[x] = xTimerCreate
                              ( /* Just a text name, not used by the RTOS kernel. */
                                  "Timer",
                                  /* The timer period in ticks, must be greater than 0. */
                                  (100 * x) + 100,
                                  /* The timers will auto-reload themselves when they expire. */
                                  pdTRUE,
                                  /* The ID is used to store a count of the number of times the timer has expired, which is initialised to 0. */
                                  (void *) 0,
                                  /* Each timer calls the same callback when it expires. */
                                  vTimerCallback
                              );
        }

        if (timer_handle[x] == NULL)
        {
            /* The timer was not created. */
        }
        else
        {
            /* Start the timer.  No block time is specified, and
            even if one was it would be ignored because the RTOS
            scheduler has not yet been started. */
            if (xTimerStart(timer_handle[x], 0) != pdPASS)
            {
                /* The timer could not be set into the Active state. */
            }
        }
    }

#else
    timer_handle[0] = xTimerCreate
                      ( /* Just a text name, not used by the RTOS kernel. */
                          "Timer0",
                          /* The timer period in ticks, must be greater than 0. */
                          pdMS_TO_TICKS(5000),
                          /* The timers will auto-reload themselves when they expire. */
                          pdTRUE,
                          /* The ID is used to store a count of the number of times the timer has expired, which is initialised to 0. */
                          (void *) 0,
                          /* Each timer calls the same callback when it expires. */
                          Timer0_Callback
                      );

    if (timer_handle[0] == NULL)
    {
        /* The timer was not created. */
    }
    else
    {
        /* Start the timer.  No block time is specified, and
        even if one was it would be ignored because the RTOS
        scheduler has not yet been started. */
        if (xTimerStart(timer_handle[0], 0) != pdPASS)
        {
            /* The timer could not be set into the Active state. */
        }
    }

    timer_handle[1] = xTimerCreate
                      ( /* Just a text name, not used by the RTOS kernel. */
                          "Timer1",
                          /* The timer period in ticks, must be greater than 0. */
                          5000,
                          /* The timers will auto-reload themselves when they expire. */
                          pdTRUE,
                          /* The ID is used to store a count of the number of times the timer has expired, which is initialised to 0. */
                          (void *) 0,
                          /* Each timer calls the same callback when it expires. */
                          Timer1_Callback
                      );

    if (timer_handle[1] == NULL)
    {
        /* The timer was not created. */
    }
    else
    {
        /* Start the timer.  No block time is specified, and
        even if one was it would be ignored because the RTOS
        scheduler has not yet been started. */
        if (xTimerStart(timer_handle[1], 0) != pdPASS)
        {
            /* The timer could not be set into the Active state. */
        }
    }

    timer_handle[2] = xTimerCreate
                      ( /* Just a text name, not used by the RTOS kernel. */
                          "Timer2",
                          /* The timer period in ticks, must be greater than 0. */
                          1000,
                          /* The timers will auto-reload themselves when they expire. */
                          pdFALSE,
                          /* The ID is used to store a count of the number of times the timer has expired, which is initialised to 0. */
                          (void *) 0,
                          /* Each timer calls the same callback when it expires. */
                          Timer2_Callback
                      );

    if (timer_handle[2] == NULL)
    {
        /* The timer was not created. */
    }
    else
    {
        /* Start the timer.  No block time is specified, and
        even if one was it would be ignored because the RTOS
        scheduler has not yet been started. */
        if (xTimerStart(timer_handle[2], 0) != pdPASS)
        {
            /* The timer could not be set into the Active state. */
        }
    }
#endif
    /* Start the tasks defined within this file/specific to this demo. */
#if (SLEEP_WAKEUP_EXAMPLE == ENABLE)
    xTaskCreate(Rtos_Sleep_Task, "SLEEP_TASK", configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1), &xSleepTaskHandle);
#else
    #if led_handle
    xTaskCreate(Led_Task, "LED_TASK", configMINIMAL_STACK_SIZE, NULL, tskMEDIUM_PRIORITY, &Led_Task_Handle);
    #endif
    #if semaphore_handle
    xTaskCreate(Semaphore_Task, "SEMAPHORE_TASK", configMINIMAL_STACK_SIZE, NULL, tskMEDIUM_PRIORITY, &Semaphore_Task_Handle);
    #endif
    #if eventgrp_handle
    xTaskCreate(Event_Task, "EVT_TASK", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &EventTask_Handle);
    #endif
    #if taskandtimer_handle
    xTaskCreate(TasksAndTimers_Task, "TasksAndTimers_Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    #endif
    #if queue_task_handle
    xTaskCreate(KeyQueue_Task, "KeyQueue_Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    #endif
#endif

    /* Start the scheduler. */
    vTaskStartScheduler();

    while (1)
    {
    }
}

#if (SLEEP_WAKEUP_EXAMPLE == ENABLE)
/**
 * @brief Freertos sleep task
 * @param[in] parameters_ptr
 * @return None
 */
static void Rtos_Sleep_Task(void *parameters_ptr)
{
    for (;;)
    {
        gpio_pin_set(GPIO0);
        gpio_pin_set(GPIO2);
        gpio_pin_set(GPIO3);
        gpio_pin_toggle(GPIO1);


        if (gpio_pin_get(GPIO20) == 1)
        {
            //if (xTimerReset(timer_task_resume_handle, 0) != pdPASS) {
            //    /* The reset command was not executed successfully. Take appropriate action here. */
            //}

            vTaskSuspend(xSleepTaskHandle);
        }
    }
}

#endif

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
