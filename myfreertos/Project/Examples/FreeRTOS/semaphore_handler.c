/** @file semaphore_handler.c
 *
 * @brief Semaphore handler module implementation.
 */

/**************************************************************************************************
 *    INCLUDES
 *************************************************************************************************/
#include "semaphore_handler.h"
#include <stdio.h>
#include "project_config.h"

/**************************************************************************************************
 *    GLOBAL VARIABLES (definitions)
 *************************************************************************************************/
SemaphoreHandle_t gpio_semaphore = NULL;
SemaphoreHandle_t MutexSemaphore = NULL;
TaskHandle_t Semaphore_Task_Handle = NULL;
TaskHandle_t MutexSemaphore_Task_Handle = NULL;

/**************************************************************************************************
 *    LOCAL FUNCTIONS
 *************************************************************************************************/
static void High_Task(void *parameters_ptr);
static void Medium_Task(void *parameters_ptr);
static void Low_Task(void *parameters_ptr);

/**************************************************************************************************
 *    TASKS
 *************************************************************************************************/
void Semaphore_Task(void *parameters_ptr)
{
    xTaskCreate(BinarySemaphore_Task, "Binary_Sem_Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(MutexSemaphore_Task, "Mutex_Sem_Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    vTaskDelete(NULL);
}

void BinarySemaphore_Task(void *parameters_ptr)
{
    printf("Create GPIO Semaphore\n");
    #if Selected_FreeRTOS_Binary_Semaphore
    vSemaphoreCreateBinary(gpio_semaphore);
    #else
    gpio_semaphore = xSemaphoreCreateCounting(3, 0);
    #endif
    vTaskDelete(NULL);
}

void MutexSemaphore_Task(void *parameters_ptr)
{
    xTaskCreate(High_Task, "High_Task", configMINIMAL_STACK_SIZE, NULL, tskHIGH_PRIORITY, NULL);
    xTaskCreate(Medium_Task, "Medium_Task", configMINIMAL_STACK_SIZE, NULL, tskMEDIUM_PRIORITY, NULL);
    xTaskCreate(Low_Task, "Low_Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    MutexSemaphore = xSemaphoreCreateMutex();
    vTaskDelete(NULL);
}

/* Helper tasks used by mutex semaphore example */
static void High_Task(void *parameters_ptr)
{
    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));
        xSemaphoreTake(MutexSemaphore,portMAX_DELAY);
        printf("High_Task Running...\n");
        xSemaphoreGive(MutexSemaphore);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

static void Medium_Task(void *parameters_ptr)
{
    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
        printf("Medium_Task Running...\n");
        vTaskDelay(pdMS_TO_TICKS(10000));        
    }

}

static void Low_Task(void *parameters_ptr)
{
    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(20000));
        xSemaphoreTake(MutexSemaphore,portMAX_DELAY);
        printf("Low_Task Running...\n");
        xSemaphoreGive(MutexSemaphore);
        vTaskDelay(pdMS_TO_TICKS(20000));
    }
}
