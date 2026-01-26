/** @file semaphore_handler.h
 *
 * @brief Semaphore handler module header.
 */

/**************************************************************************************************
 *    INCLUDES
 *************************************************************************************************/
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
/**************************************************************************************************
 *    MACROS
 *************************************************************************************************/
#define Selected_FreeRTOS_Binary_Semaphore   1 

/**************************************************************************************************
 *    GLOBAL VARIABLES
 *************************************************************************************************/
extern SemaphoreHandle_t gpio_semaphore;
extern SemaphoreHandle_t MutexSemaphore;
extern TaskHandle_t Semaphore_Task_Handle;
extern TaskHandle_t MutexSemaphore_Task_Handle;

/**************************************************************************************************
 *    GLOBAL FUNCTIONS
 *************************************************************************************************/
void Semaphore_Task(void *parameters_ptr);
void MutexSemaphore_Task(void *parameters_ptr);
void BinarySemaphore_Task(void *parameters_ptr);
