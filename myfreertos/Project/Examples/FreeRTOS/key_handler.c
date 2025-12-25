/** @file key_handler.c
 *
 * @brief Key handler module implementation.
 *        Handle GPIO key press events.
 *
 */

/**************************************************************************************************
 *    INCLUDES
 *************************************************************************************************/
#include "key_handler.h"
#include "project_config.h"
#include <stdio.h>
#include "timers.h"
#include "queue.h"
#include "uart_drv.h"
#include "retarget.h"
#include "pwm_control.h"
#include "queue_task.h"

/**************************************************************************************************
 *    GLOBALS
 *************************************************************************************************/
TaskHandle_t Key_Task_Handle = NULL;

/**************************************************************************************************
 *    TASKS / ISRs
 *************************************************************************************************/
void Key_Init(void)
{
    gpio_set_debounce_time(DEBOUNCE_SLOWCLOCKS_1024);
    gpio_debounce_enable(GPIO0);
    gpio_debounce_enable(GPIO1);
    gpio_debounce_enable(GPIO2);
    gpio_debounce_enable(GPIO3);

    pin_set_pullopt(GPIO0, MODE_PULLUP_100K);
    pin_set_pullopt(GPIO1, MODE_PULLUP_100K);
    pin_set_pullopt(GPIO2, MODE_PULLUP_100K);
    pin_set_pullopt(GPIO3, MODE_PULLUP_100K);

    gpio_cfg_input(GPIO0, GPIO_PIN_INT_EDGE_FALLING);
    gpio_register_isr(GPIO0, user_gpio0_isr_handler, NULL);
    gpio_int_enable(GPIO0);
    gpio_cfg_input(GPIO1, GPIO_PIN_INT_EDGE_FALLING);
    gpio_register_isr(GPIO1, user_gpio1_isr_handler, NULL);
    gpio_int_enable(GPIO1);
    gpio_cfg_input(GPIO2, GPIO_PIN_INT_EDGE_FALLING);
    gpio_register_isr(GPIO2, user_gpio2_isr_handler, NULL);
    gpio_int_enable(GPIO2);
    gpio_cfg_input(GPIO3, GPIO_PIN_INT_EDGE_FALLING);
    gpio_register_isr(GPIO3, user_gpio3_isr_handler, NULL);
    gpio_int_enable(GPIO3);

}

void user_gpio0_isr_handler(uint32_t pin, void *isr_param)
{
#if semaphore_handle
    if (gpio_semaphore != NULL)
    {
        if (xSemaphoreTakeFromISR(gpio_semaphore, NULL) == pdFALSE)
        {
            printf("Fail to Take GPIO Semaphore in ISR\n");
            return;
        }
    }
#endif
#if led_handle
    printf("LED_OFF\n");
    pwm1_breath_stop();
    gpio_pin_set(GPIO21);
#endif
#if queue_task_handle 
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    queue_task_send_from_isr(0, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#endif
}

void user_gpio1_isr_handler(uint32_t pin, void *isr_param)
{
#if led_handle
    printf("LED_ON\n");
    pwm1_breath_start();

    gpio_pin_clear(GPIO21);
#endif
#if queue_task_handle
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    queue_task_send_from_isr(1, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#endif
}

void user_gpio2_isr_handler(uint32_t pin, void *isr_param)
{
#if semaphore_handle
    printf("Give GPIO Semaphore\n");
    xSemaphoreGiveFromISR(gpio_semaphore, NULL);
#endif
}

void user_gpio3_isr_handler(uint32_t pin, void *isr_param)
{
#if eventgrp_handle
    xTimerResetFromISR(timer_handle[2], NULL);
#endif
}
