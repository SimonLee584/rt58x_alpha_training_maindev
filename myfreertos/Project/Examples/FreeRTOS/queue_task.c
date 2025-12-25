#include "queue_task.h"
#include <stdio.h>

static QueueHandle_t xKeyQueue = NULL;

static void queue_task(void *pvParameters)
{
    uint8_t val = 0;
    for (;;)
    {
        if (xKeyQueue != NULL)
        {
            if (xQueueReceive(xKeyQueue, &val, portMAX_DELAY) == pdTRUE)
            {
                printf("Key Queue Received: %d\r\n", val);
            }
        }
    }
}

void KeyQueue_Task(void *pvParameters)
{
    if (xKeyQueue == NULL)
    {
        xKeyQueue = xQueueCreate(10, sizeof(uint8_t));
    }

    if (xKeyQueue != NULL)
    {
        xTaskCreate(queue_task, "queue_task", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
    }
    vTaskDelete(NULL);
}

BaseType_t queue_task_send(uint8_t val, TickType_t xTicksToWait)
{
    if (xKeyQueue == NULL)
        return errQUEUE_FULL;
    return xQueueSend(xKeyQueue, &val, xTicksToWait);
}

BaseType_t queue_task_send_from_isr(uint8_t val, BaseType_t *pxHigherPriorityTaskWoken)
{
    if (xKeyQueue == NULL)
        return pdFALSE;
    return xQueueSendFromISR(xKeyQueue, &val, pxHigherPriorityTaskWoken);
}
