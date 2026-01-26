/** @file eventgrp_handler.c
 *
 * @brief Event group handler implementation.
 */

/**************************************************************************************************
 *    INCLUDES
 *************************************************************************************************/
#include "eventgrp_handler.h"
#include <stdio.h>

/**************************************************************************************************
 *    GLOBALS
 *************************************************************************************************/
EventGroupHandle_t event_group = NULL;
TaskHandle_t EventTask_Handle = NULL;
TaskHandle_t EventTask1_Handle = NULL;
TaskHandle_t EventTask2_Handle = NULL;
TaskHandle_t EventTask3_Handle = NULL;

/**************************************************************************************************
 *    FUNCTIONS
 *************************************************************************************************/
void Event_Task(void *params)
{
    EventGroup_Init();
    xTaskCreate(EventTask1, "EVT_TASK1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &EventTask1_Handle);
    xTaskCreate(EventTask2, "EVT_TASK2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &EventTask2_Handle);
    xTaskCreate(EventTask3, "EVT_TASK3", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &EventTask3_Handle);
    vTaskDelete(NULL);
}
void EventGroup_Init(void)
{
    if (event_group == NULL)
    {
        event_group = xEventGroupCreate();
        if (event_group == NULL)
        {
            printf("Failed to create event group\n");
        }
    }
}

void EventTask1(void *params)
{
    for (;;)
    {
        if (event_group != NULL)
        {
            xEventGroupSetBits(event_group, EVENT_BIT_0);
            printf("EventTask1: set bit0\n");
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void EventTask2(void *params)
{
    for (;;)
    {
        if (event_group != NULL)
        {
            xEventGroupSetBits(event_group, EVENT_BIT_1);
            printf("EventTask2: set bit1\n");
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void EventTask3(void *params)
{
    const EventBits_t bits_to_wait = (EVENT_BIT_0 | EVENT_BIT_1);
    for (;;)
    {
        if (event_group != NULL)
        {
            /* Wait for both bits; clear on exit */
            EventBits_t bits = xEventGroupWaitBits(event_group, bits_to_wait, pdTRUE, pdTRUE, portMAX_DELAY);
            if ((bits & bits_to_wait) == bits_to_wait)
            {
                printf("EventTask3: both bits set -> executing\n");
                /* Perform action here */
            }
        }
    }
}
