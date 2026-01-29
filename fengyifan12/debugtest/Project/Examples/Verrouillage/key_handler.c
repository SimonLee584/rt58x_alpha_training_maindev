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
}

void user_gpio1_isr_handler(uint32_t pin, void *isr_param)
{
}

void user_gpio2_isr_handler(uint32_t pin, void *isr_param)
{
}

void user_gpio3_isr_handler(uint32_t pin, void *isr_param)
{
}
