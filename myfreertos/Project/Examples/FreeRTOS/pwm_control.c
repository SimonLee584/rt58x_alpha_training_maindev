/* pwm_control.c - implement PWM1 breathing (RDMA format1) */
#include "pwm_control.h"
#include "cm3_mcu.h"
#include "task.h"
#include <string.h>


#define PWM1_RDMA_MAX 256

static pwm_seq_para_head_t pwm1_cfg;
static uint32_t pwm1_rdma1[PWM1_RDMA_MAX];
static uint32_t pwm1_elements = 0;

void pwm1_breath_init(uint32_t elements, uint32_t cnt_end_val, pwm_clk_div_t clk_div, uint32_t max_thd)
{
    if (elements == 0 || elements > PWM1_RDMA_MAX)
        elements = 64;

    if (cnt_end_val == 0)
        cnt_end_val = 4000;

    if (max_thd == 0 || max_thd > cnt_end_val)
        max_thd = (cnt_end_val * 9) / 10; /* 默认 90% */

    memset(&pwm1_cfg, 0, sizeof(pwm1_cfg));
    memset(pwm1_rdma1, 0, sizeof(pwm1_rdma1));

    pwm1_cfg.pwm_id = PWM_ID_1;
    pwm1_cfg.pwm_play_cnt = 0; /* continuous */
    pwm1_cfg.pwm_seq_order = PWM_SEQ_ORDER_T; /* use seq1 */
    pwm1_cfg.pwm_seq_num = PWM_SEQ_NUM_1;
    pwm1_cfg.pwm_seq_mode = PWM_SEQ_MODE_CONTINUOUS;
    pwm1_cfg.pwm_triggered_src = PWM_TRIGGER_SRC_SELF;
    pwm1_cfg.pwm_clk_div = clk_div;
    pwm1_cfg.pwm_counter_mode = PWM_COUNTER_MODE_UP;
    pwm1_cfg.pwm_dma_smp_fmt = PWM_DMA_SMP_FMT_1;
    pwm1_cfg.pwm_count_end_val = 0; /* fmt1 uses embedded end value */

    pwm1_cfg.pwm_seq1.pwm_element_num = (uint16_t)elements;
    pwm1_cfg.pwm_seq1.pwm_repeat_num = 0;
    pwm1_cfg.pwm_seq1.pwm_delay_num = 0;
    pwm1_cfg.pwm_seq1.pwm_rdma_addr = (uint32_t)pwm1_rdma1;

    pwm1_elements = elements;

    /* fill a up-down ramp (breathing) */
    uint32_t half = (elements / 2) ? (elements / 2) : 1;
    for (uint32_t i = 0; i < elements; i++)
    {
        uint32_t thd;
        if (i < half)
            thd = (max_thd * i) / half;
        else
            thd = (max_thd * (elements - i)) / half;

        pwm1_rdma1[i] = PWM_FILL_SAMPLE_DATA_MODE1(0, thd, cnt_end_val);
    }

    /* initialize PWM driver with prepared config (do not start yet) */
    Pwm_Init(&pwm1_cfg);
}

void pwm1_breath_start(void)
{
    if (pwm1_elements == 0)
        return; /* not initialized */

    /* ensure pin muxed back to PWM function before starting */
    pin_set_mode(PWM_PWM1_GPIO, MODE_PWM1);
    Pwm_Start(&pwm1_cfg);
}

void pwm1_breath_stop(void)
{
    if (pwm1_elements == 0)
        return;

    Pwm_Stop(&pwm1_cfg);
    /* switch PWM pin back to GPIO and drive LED off */
    pin_set_mode(PWM_PWM1_GPIO, MODE_GPIO);
    gpio_cfg_output(PWM_PWM1_GPIO);
    gpio_pin_set(PWM_PWM1_GPIO); /* drive inactive (LED off) */
    /* also ensure LED indicator pin is off */
    gpio_cfg_output(GPIO21);
    gpio_pin_set(GPIO21);
}

/**
 * @brief Freertos Led Task
 * @param[in] parameters_ptr
 * @return None
 */
void Led_Task(void *parameters_ptr)
{
    pwm1_breath_init(128, 4000, PWM_CLK_DIV_256, 3600); /* 90% of 4000 */
    pwm1_breath_start();
    gpio_cfg_output(GPIO21); /* LED pin as output */
    gpio_pin_clear(GPIO21); /* LED on */
    vTaskDelete(NULL);    
}
