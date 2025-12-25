/* pwm_control.h - simple PWM1 breathing control API */
#ifndef __PWM_CONTROL_H__
#define __PWM_CONTROL_H__

/**************************************************************************************************
 *    INCLUDES
 *************************************************************************************************/
#include <stdint.h>
#include "pwm.h"
#include "FreeRTOS.h"

/**************************************************************************************************
 *    MACROS
 *************************************************************************************************/
#define PWM_PWM1_GPIO                  20  /* PWM1 pin */
#define GPIO21                         21  /* GPIO21 for LED toggle */

/**************************************************************************************************
 *    GLOBAL FUNCTIONS
 *************************************************************************************************/
/* 初始化 PWM1 呼吸灯
 * elements: RDMA 元素数（建议 32~256）
 * cnt_end_val: 计数结束值（例如 4000）
 * clk_div: PWM 时钟分频（PWM_CLK_DIV_8 等）
 * max_thd: 最大高电平计数（<= cnt_end_val）
 */
void pwm1_breath_init(uint32_t elements, uint32_t cnt_end_val, pwm_clk_div_t clk_div, uint32_t max_thd);
void pwm1_breath_start(void);
void pwm1_breath_stop(void);
void Led_Task(void *parameters_ptr);

#endif /* __PWM_CONTROL_H__ */
