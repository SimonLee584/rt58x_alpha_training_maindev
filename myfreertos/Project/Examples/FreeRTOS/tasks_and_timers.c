#include "tasks_and_timers.h"
#include "timers.h"
#include <stdio.h>

static TaskHandle_t xTask1Handle = NULL;
static TaskHandle_t xTask2Handle = NULL;
static TimerHandle_t xSuspendTimer = NULL;
static TimerHandle_t xResumeTimer = NULL;

static void vTask1(void *pvParameters)
{
    (void)pvParameters;
    for (;;)
    {
        printf("Task1: tick\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void vTask2(void *pvParameters)
{
    (void)pvParameters;
    
    for (;;)
    {
        taskENTER_CRITICAL();
        printf("Task2: tick\r\n");
        vTaskDelay(pdMS_TO_TICKS(3000));
        taskEXIT_CRITICAL();
    }
    
}

static void prvSuspendTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    if (xTask1Handle) vTaskSuspend(xTask1Handle);
    if (xTask2Handle) vTaskSuspend(xTask2Handle);
    printf("Tasks suspended by timer\r\n");
}

static void prvResumeTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    if (xTask1Handle) vTaskResume(xTask1Handle);
    if (xTask2Handle) vTaskResume(xTask2Handle);
    printf("Tasks resumed by timer\r\n");
}

/* 该任务在调度器启动后运行一次，用来按偏移启动定时器，使挂起和恢复交替 */
static void vTimersStarterTask(void *pvParameters)
{
    (void)pvParameters;

    xSuspendTimer = xTimerCreate("SuspendTimer", pdMS_TO_TICKS(20000), pdTRUE, NULL, prvSuspendTimerCallback);
    xResumeTimer  = xTimerCreate("ResumeTimer",  pdMS_TO_TICKS(20000), pdTRUE, NULL, prvResumeTimerCallback);

    if (xSuspendTimer != NULL) xTimerStart(xSuspendTimer, 0);

    /* 偏移 10 秒，使恢复定时器在挂起之后触发（交替效果） */
    vTaskDelay(pdMS_TO_TICKS(10000));

    if (xResumeTimer != NULL) xTimerStart(xResumeTimer, 0);

    vTaskDelete(NULL);
}

void TasksAndTimers_Task(void *parameters_ptr)
{
    xTaskCreate(vTask1, "Task1", configMINIMAL_STACK_SIZE , NULL, tskIDLE_PRIORITY , &xTask1Handle);
    xTaskCreate(vTask2, "Task2", configMINIMAL_STACK_SIZE , NULL, tskIDLE_PRIORITY , &xTask2Handle);

    /* 创建一个短期任务来按需启动定时器（有偏移） */
    xTaskCreate(vTimersStarterTask, "TimersStarter", configMINIMAL_STACK_SIZE , NULL, tskMEDIUM_PRIORITY , NULL);
    vTaskDelete(NULL);
}
