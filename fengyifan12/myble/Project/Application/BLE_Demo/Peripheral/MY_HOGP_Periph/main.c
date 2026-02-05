/** @file
 *
 * @brief BLE HID peripheral role demo.
 *
 */

/**************************************************************************************************
 *    INCLUDES
 *************************************************************************************************/
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "bsp.h"
#include "bsp_console.h"
#include "util_log.h"
#include "util_printf.h"
#include "ble_app.h"

/**************************************************************************************************
 *    CONSTANTS AND DEFINES
 *************************************************************************************************/
uint8_t g_key_press_state = 0;          //key press state: 0=released, 1=pressed

#define GPIO0               0   // GPIO0 for KEY0 (HID 'A')
#define GPIO1               1   // GPIO1 for KEY1 (HID 'B')
#define GPIO2               2   // GPIO2 for KEY2 (HID 'C')
#define GPIO3               3   // GPIO3 for KEY3 (HID 'D')
#define GPIO4               4   // GPIO4 for KEY4 (HID 'E')
/**************************************************************************************************
 *    LOCAL FUNCTIONS
 *************************************************************************************************/
static void bsp_btn_event_handle(bsp_event_t event)
{
    switch (event)
    {
    case BSP_EVENT_BUTTONS_0:
        // disable sleep mode
        Lpm_Low_Power_Mask(LOW_POWER_MASK_BIT_TASK_BLE_APP);
        break;

    case BSP_EVENT_UART_RX_RECV:
    case BSP_EVENT_UART_RX_DONE:
    {
#if ((IO_CAPABILITY_SETTING == KEYBOARD_ONLY) || (IO_CAPABILITY_SETTING == KEYBOARD_DISPLAY) || (IO_CAPABILITY_SETTING == DISPLAY_YESNO) )
        static uint8_t rx_buffer[6];
        static uint8_t index = 0;
        char ch;

        bsp_console_stdin_str(&ch, 1);

        if ((ch == '\n') || (ch == '\r'))
        {
            // set passkey
            passkey_set(rx_buffer, index);

            // reset index
            index = 0;

            // enable sleep mode
            Lpm_Low_Power_Unmask(LOW_POWER_MASK_BIT_TASK_BLE_APP);
        }
        else
        {
            if (index == 6)
            {
                index = 0;
            }
            rx_buffer[index++] = ch;
        }
#endif
    }
    break;

    case BSP_EVENT_UART_BREAK:
        // diable sleep mode
        Lpm_Low_Power_Mask(LOW_POWER_MASK_BIT_TASK_BLE_APP);
        break;

    default:
        break;
    }
}

/* pin mux setting init*/
static void pin_mux_init(void)
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
    return;
}

/* GPIO key initialization */
static void gpio_key_init(void)
{
    // Configure GPIO0-4 as input (no interrupt, polling mode)
    gpio_cfg_input(GPIO0, GPIO_PIN_NOINT);
    gpio_cfg_input(GPIO1, GPIO_PIN_NOINT);
    gpio_cfg_input(GPIO2, GPIO_PIN_NOINT);
    gpio_cfg_input(GPIO3, GPIO_PIN_NOINT);
    gpio_cfg_input(GPIO4, GPIO_PIN_NOINT);
    
    // Enable pull-up resistors (active low buttons)
    pin_set_pullopt(GPIO0, MODE_PULLUP_100K);
    pin_set_pullopt(GPIO1, MODE_PULLUP_100K);
    pin_set_pullopt(GPIO2, MODE_PULLUP_100K);
    pin_set_pullopt(GPIO3, MODE_PULLUP_100K);
    pin_set_pullopt(GPIO4, MODE_PULLUP_100K);   

    printf("[GPIO] Keys initialized (GPIO0-4 for A-E)\n");
}

/* Scan GPIO keys and return key event code
 * Return: 0=no event, 1-6=key press, 0xFF=key release
 */
static uint8_t gpio_key_scan(void)
{
    static uint8_t last_gpio_state[5] = {1, 1, 1, 1, 1};  // 1=released (active low), init state
    static uint8_t last_pressed_key = 0;  // Record last pressed key for release detection
    uint8_t key_val = 0;
    uint8_t current_gpio;
    
    // Check GPIO0 (KEY A)
    current_gpio = gpio_pin_get(GPIO0);
    if (current_gpio == 0 && last_gpio_state[0] == 1)
    {
        key_val = 1;  // KEY0 -> 'A' pressed
        last_pressed_key = 1;
        g_key_press_state = 1;
    }
    else if (current_gpio == 1 && last_gpio_state[0] == 0)
    {
        key_val = 0xFF;  // KEY0 released
        last_pressed_key = 0;
        g_key_press_state = 1;
    }
    last_gpio_state[0] = current_gpio;
    if (key_val != 0) return key_val;
    
    // Check GPIO1 (KEY B)
    current_gpio = gpio_pin_get(GPIO1);
    if (current_gpio == 0 && last_gpio_state[1] == 1)
    {
        key_val = 2;  // KEY1 -> 'B' pressed
        last_pressed_key = 2;
        g_key_press_state = 1;
    }
    else if (current_gpio == 1 && last_gpio_state[1] == 0)
    {
        key_val = 0xFF;  // KEY1 released
        last_pressed_key = 0;
        g_key_press_state = 1;
    }
    last_gpio_state[1] = current_gpio;
    if (key_val != 0) return key_val;
    
    // Check GPIO2 (KEY C)
    current_gpio = gpio_pin_get(GPIO2);
    if (current_gpio == 0 && last_gpio_state[2] == 1)
    {
        key_val = 3;  // KEY2 -> 'C' pressed
        last_pressed_key = 3;
        g_key_press_state = 1;
    }
    else if (current_gpio == 1 && last_gpio_state[2] == 0)
    {
        key_val = 0xFF;  // KEY2 released
        last_pressed_key = 0;
        g_key_press_state = 1;
    }
    last_gpio_state[2] = current_gpio;
    if (key_val != 0) return key_val;
    
    // Check GPIO4 (Modifier key - Shift)
    current_gpio = gpio_pin_get(GPIO4);
    if (current_gpio == 0 && last_gpio_state[4] == 1)
    {
        key_val = 5;  // KEY4 -> 'Left_Shift_modifier' pressed
        last_pressed_key = 5;
        g_key_press_state = 1;
    }
    else if (current_gpio == 1 && last_gpio_state[4] == 0)
    {
        key_val = 6;  // KEY4 -> 'modifier_clear' (released)
        last_pressed_key = 0;
        g_key_press_state = 1;
    }
    last_gpio_state[4] = current_gpio;
    
    return key_val;
}

/* Key scan task - called periodically from app */
void gpio_key_scan_task(void)
{
    extern void app_key_event_handler(uint8_t key_code);
    uint8_t current_key = gpio_key_scan();
    
    if (g_key_press_state)
    {
        if (current_key != 0)  // Key pressed
        {
            app_key_event_handler(current_key);
        }
        g_key_press_state = 0;
    }
}

/**************************************************************************************************
 *    GLOBAL FUNCTIONS
 *************************************************************************************************/
int main(void)
{
    uint32_t status;

    /* Setting the System clock */
    status = Change_Ahb_System_Clk(SYS_48MHZ_CLK);
    if (status != STATUS_SUCCESS)
    {
        /* System clock cannot be switched correctly. */
        while (1);
    }

    /* pinmux init */
    pin_mux_init();
    
    /* GPIO keys init (KEY0-KEY4 for A-E) */
    gpio_key_init();

    /* delay init */
    Delay_Init();

    /* low power mode init */
    Lpm_Set_Low_Power_Level(LOW_POWER_LEVEL_SLEEP0);
    Lpm_Enable_Low_Power_Wakeup(LOW_POWER_WAKEUP_GPIO0);
    Lpm_Enable_Low_Power_Wakeup(LOW_POWER_WAKEUP_UART_RX);

    /* initil Button and press button0 to disable sleep mode & initil Console & UART */
    bsp_init((BSP_INIT_BUTTONS |
              BSP_INIT_DEBUG_CONSOLE |
              BSP_INIT_UART), bsp_btn_event_handle);

    /* retarget stdout for utility & initial utility logging */
    utility_register_stdout(bsp_console_stdout_char, bsp_console_stdout_string);
    util_log_init();

    NVIC_SetPriority(Uart0_IRQn, 5);
    NVIC_SetPriority(Uart1_IRQn, 5);

    /* enable protocol debug message */
    //util_log_on(UTIL_LOG_PROTOCOL);

    /* application init */
    app_init();

    /* scheduler start */
    vTaskStartScheduler();

    while (1)
    {
    }
}
