/** @file main.c
 *
 * @brief GPIO example main file.
 *
 *
 */
/**
* @defgroup GPIO_example_group  GPIO
* @ingroup examples_group
* @{
* @brief GPIO example demonstrate
*/
#include <stdio.h>
#include <string.h>
#include "cm3_mcu.h"
#include "project_config.h"

#include "uart_drv.h"
#include "retarget.h"
#include "rf_mcu_ahb.h"
/*
 * Remark: UART_BAUDRATE_115200 is not 115200...Please don't use 115200 directly
 * Please use macro define  UART_BAUDRATE_XXXXXX
 */

#define PRINTF_BAUDRATE      UART_BAUDRATE_115200

int main(void);

void SetClockFreq(void);

#define GPIO0   0
#define GPIO1   1
#define TIMER0_ID     0
#define TIMER1_ID     1
#define SUBSYSTEM_CFG_PMU_MODE              0x4B0
#define SUBSYSTEM_CFG_LDO_MODE_DISABLE      0x02

/* 按键扫描定义 */
#define KEY_DEBOUNCE_CNT                1       /* 按键去抖次数: 连续扫描1次确认 */
#define KEY_SCAN_INTERVAL               1      /* 按键扫描间隔: 1*50ms */
#define KEY_LONG_PRESS_TIME             20    /* 长按判定时间: 20*50ms */
#define KEY_DOUBLE_CLICK_INTERVAL       10     /* 双击判定间隔: 10*50ms */

/* 按键状态定义 */
typedef enum
{
    KEY_STATE_IDLE = 0,              /* 空闲状态 */
    KEY_STATE_DEBOUNCE = 1,          /* 去抖状态 */
    KEY_STATE_PRESSED = 2,           /* 按下状态 */
    KEY_STATE_RELEASED = 3           /* 释放状态 */
} key_state_t;

/* 按键事件类型 */
typedef enum
{
    KEY_EVENT_NONE = 0,              /* 无事件 */
    KEY_EVENT_SHORT_PRESS = 1,       /* 短按事件 */
    KEY_EVENT_LONG_PRESS = 2,        /* 长按事件 */
    KEY_EVENT_DOUBLE_CLICK = 3       /* 双击事件 */
} key_event_t;

/* 按键对象结构体 */
typedef struct
{
    uint8_t gpio_id;                 /* GPIO ID */
    uint8_t debounce_cnt;            /* 去抖计数器 */
    uint16_t press_time;             /* 按下时长计数器 */
    uint8_t state;                   /* 当前状态 */
    uint8_t last_press_event;        /* 上一次短按事件标志 */
    uint16_t last_release_time;      /* 上次释放的时间 */
    uint8_t click_count;             /* 点击计数 */
    uint16_t pending_timer;          /* 待输出事件的延迟计时器 */
} key_obj_t;

/**************************************************************************************************
 *    GLOBAL VARIABLES
 *************************************************************************************************/
volatile uint32_t   timer0_tick_count = 0;
/* 按键扫描触发标志（从中断置位，主循环处理中断不做打印） */
volatile uint8_t    key_scan_pending = 0;

/* 按键对象 */
key_obj_t key0 = {.gpio_id = GPIO0, .state = KEY_STATE_IDLE, .last_release_time = 0, .click_count = 0, .pending_timer = 0};
key_obj_t key1 = {.gpio_id = GPIO1, .state = KEY_STATE_IDLE, .last_release_time = 0, .click_count = 0, .pending_timer = 0};
/************************************************************/

/*this is pin mux setting*/
void Init_Default_Pin_Mux(void)
{
    int i;

    /*set all pin to gpio, except GPIO16, GPIO17 */
    for (i = 0; i < 32; i++)
    {
        if ((i != 16) && (i != 17))
        {
            pin_set_mode(i, MODE_GPIO);
        }
    }

    /*uart0 pinmux*/
    pin_set_mode(16, MODE_UART);     /*GPIO16 as UART0 RX*/
    pin_set_mode(17, MODE_UART);     /*GPIO17 as UART0 TX*/

    return;
}
void Comm_Subsystem_Disable_LDO_Mode(void)
{
    uint8_t reg_buf[4];

    RfMcu_MemoryGetAhb(SUBSYSTEM_CFG_PMU_MODE, reg_buf, 4);
    reg_buf[0] &= ~SUBSYSTEM_CFG_LDO_MODE_DISABLE;
    RfMcu_MemorySetAhb(SUBSYSTEM_CFG_PMU_MODE, reg_buf, 4);
}

/**
 * @brief 处理单个按键的状态机
 * @param[in] key 按键对象指针
 * @return 按键事件
 */
key_event_t key_process(key_obj_t *key)
{
    uint8_t pin_level = gpio_pin_get(key->gpio_id);
    key_event_t event = KEY_EVENT_NONE;

    /* 处理待输出事件的计时器 */
    if (key->pending_timer > 0)
    {
        key->pending_timer -= KEY_SCAN_INTERVAL;
        if (key->pending_timer <= 0)
        {
            /* 双击等待超时，输出短按事件 */
            if (key->click_count == 1)
            {
                event = KEY_EVENT_SHORT_PRESS;
            }
            key->click_count = 0;
        }
    }

    switch (key->state)
    {
        case KEY_STATE_IDLE:
            /* 空闲状态，等待按键按下 */
            if (pin_level == 0)
            {
                key->debounce_cnt = 1;
                key->state = KEY_STATE_DEBOUNCE;
            }
            break;

        case KEY_STATE_DEBOUNCE:
            /* 去抖状态 */
            if (pin_level == 0)
            {
                key->debounce_cnt++;
                if (key->debounce_cnt >= KEY_DEBOUNCE_CNT)
                {
                    /* 确认按键按下 */
                    key->press_time = 0;
                    key->state = KEY_STATE_PRESSED;
                }
            }
            else
            {
                /* 按键抖动，返回IDLE状态 */
                key->state = KEY_STATE_IDLE;
            }
            break;

        case KEY_STATE_PRESSED:
            /* 按下状态 */
            if (pin_level == 0)
            {
                key->press_time += KEY_SCAN_INTERVAL;
                /* 检查是否为长按 */
                if (key->press_time >= KEY_LONG_PRESS_TIME && key->last_press_event == 0)
                {
                    event = KEY_EVENT_LONG_PRESS;
                    key->last_press_event = 1;  /* 标记已输出长按事件 */
                    key->click_count = 0;  /* 长按后清除点击计数 */
                }
            }
            else
            {
                /* 按键释放 */
                if (key->press_time < KEY_LONG_PRESS_TIME && key->last_press_event == 0)
                {
                    /* 短按释放，增加点击计数 */
                    key->click_count++;
                    
                    if (key->click_count == 1)
                    {
                        /* 第一次点击，启动双击等待计时器 */
                        key->pending_timer = KEY_DOUBLE_CLICK_INTERVAL;
                    }
                    else if (key->click_count == 2)
                    {
                        /* 第二次点击，确认为双击 */
                        event = KEY_EVENT_DOUBLE_CLICK;
                        key->click_count = 0;
                        key->pending_timer = 0;
                    }
                }
                key->last_press_event = 0;
                key->state = KEY_STATE_IDLE;
            }
            break;

        default:
            key->state = KEY_STATE_IDLE;
            break;
    }

    return event;
}

/**
 * @brief 按键扫描和处理函数
 */
void key_scan(void)
{
    key_event_t event0, event1;

    event0 = key_process(&key0);
    event1 = key_process(&key1);

    /* 处理KEY0事件 */
    if (event0 == KEY_EVENT_SHORT_PRESS)
    {
        printf("KEY0 Short Press!\n");
    }
    else if (event0 == KEY_EVENT_LONG_PRESS)
    {
        printf("KEY0 Long Press!\n");
    }
    else if (event0 == KEY_EVENT_DOUBLE_CLICK)
    {
        printf("KEY0 Double Click!\n");
    }

    /* 处理KEY1事件 */
    if (event1 == KEY_EVENT_SHORT_PRESS)
    {
        printf("KEY1 Short Press!\n");
    }
    else if (event1 == KEY_EVENT_LONG_PRESS)
    {
        printf("KEY1 Long Press!\n");
    }
    else if (event1 == KEY_EVENT_DOUBLE_CLICK)
    {
        printf("KEY1 Double Click!\n");
    }
}

/**
 * @ingroup Timer_example_group
 * @brief Timer0 Interrupt callback handler
 * @param[in] timer_id Timer ID that triggered the timer interrupt
 * @return None
 */
void Timer0_Callback(uint32_t timer_id)
{
    /* 在中断上下文尽量不要调用 printf 等可能依赖中断的函数，改为置位标志由主循环打印 */
    key_scan_pending = 1;
}

/**
 * @ingroup Timer_example_group
 * @brief Timer configuration
 * @return None
 */
void Timer_Config(void)
{
    timer_config_mode_t cfg;

    cfg.int_en = ENABLE;
    cfg.mode = TIMER_PERIODIC_MODE;
    cfg.prescale = TIMER_PRESCALE_32;
    /* 设置定时器中断优先级为低，避免抢占串口中断导致打印被阻塞 */
    Timer_Int_Priority(TIMER0_ID, IRQ_PRIORITY_LOW);
    Timer_Open(TIMER0_ID, cfg, Timer0_Callback);
    Timer_Start(TIMER0_ID, 75000);  /* 48MHz / 32 = 1.5MHz, 75000 * (1/1.5MHz) = 50ms */
}

int main(void)
{
    /*we should set pinmux here or in SystemInit */
    Change_Ahb_System_Clk(SYS_48MHZ_CLK);

    Init_Default_Pin_Mux();

    /*init debug uart port for printf*/
    console_drv_init(PRINTF_BAUDRATE);

    Comm_Subsystem_Disable_LDO_Mode();//if don't load 569 FW, need to call the function.

    /*init delay function*/
    Delay_Init();
    /*set all pin for gpio input, and interrupt mode*/

    gpio_set_debounce_time(DEBOUNCE_SLOWCLOCKS_1024);

    gpio_cfg_input(GPIO0, GPIO_PIN_NOINT);
    gpio_cfg_input(GPIO1, GPIO_PIN_NOINT);

    Timer_Config();
    
    while (1)
    {
        if (key_scan_pending)
        {
            key_scan_pending = 0;
            key_scan();
        }
    }
}

void SetClockFreq(void)
{
    return;
}
/** @} */ /* end of examples group */
