/** @file key_handler.h
 *
 * @brief Key handler module header file.
 *        Handle GPIO key press events.
 *
 */

/**************************************************************************************************
 *    INCLUDES
 *************************************************************************************************/
#include "cm3_mcu.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "task_config.h"
/**************************************************************************************************
 *    MACROS
 *************************************************************************************************/


/**************************************************************************************************
 *    GLOBAL VARIABLES
 *************************************************************************************************/
extern TaskHandle_t Led_Task_Handle;
extern SemaphoreHandle_t gpio_semaphore;
extern TimerHandle_t timer_handle[];
extern TaskHandle_t Key_Task_Handle;

/* Provide GPIO number macros if not defined elsewhere (avoid redefinition warnings) */
#ifndef GPIO0
#define GPIO0   0
#define GPIO1   1
#define GPIO2   2
#define GPIO3   3
#endif

/**************************************************************************************************
 *    GLOBAL FUNCTIONS
 *************************************************************************************************/
void Key_Init(void);
void user_gpio0_isr_handler(uint32_t pin, void *isr_param);
void user_gpio1_isr_handler(uint32_t pin, void *isr_param);
void user_gpio2_isr_handler(uint32_t pin, void *isr_param);
void user_gpio3_isr_handler(uint32_t pin, void *isr_param);
