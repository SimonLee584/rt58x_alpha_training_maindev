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

void HardFault_Handler(void);

void SetClockFreq(void);

#define GPIO20  20
#define GPIO28  28
#define GPIO29  29
#define SUBSYSTEM_CFG_PMU_MODE              0x4B0
#define SUBSYSTEM_CFG_LDO_MODE_DISABLE      0x02
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

    gpio_cfg_output(GPIO20);

    return;
}
void Comm_Subsystem_Disable_LDO_Mode(void)
{
    uint8_t reg_buf[4];

    RfMcu_MemoryGetAhb(SUBSYSTEM_CFG_PMU_MODE, reg_buf, 4);
    reg_buf[0] &= ~SUBSYSTEM_CFG_LDO_MODE_DISABLE;
    RfMcu_MemorySetAhb(SUBSYSTEM_CFG_PMU_MODE, reg_buf, 4);
}
/*
 * this function is ISR. so it should be as short as possible
 * You SHOULD NOT call any function that will block the ISR
 *
 */

void user_gpio_isr_handler(uint32_t pin, void *isr_param)
{
    /*
     * This is a very STUPID idea --- call printf in ISR.
     * Here we use it just for showing debug information easily...
     * In normal code, you SHOULD AVOID this poor design...
     */

    //printf("pin %ld\n", pin);

    /*we also use gpio 31 to show interrupt happen*/
    gpio_pin_toggle(31);

    return;
}

void ArrayOutOfBounds_Test()
{
    int arr[5];
    int i;
    printf("Now trigger Array Out-of-Bounds fault\r\n");
    Delay_ms(10);
    for (i = 5; i <= 10; i++)
    {
        /* 故意访问数组边界之外的元素，制造硬件异常。 */
        arr[i] = i;
    }
    printf("This line should never be printed\r\n");
}

static void IllegalCodeAddress_Test(void)
{
    printf("Now trigger illegal code address branch (should HardFault)\r\n");
    Delay_ms(10);
    void (*bad_fn)(void) = (void (*)(void))0x00000000u;
    bad_fn();

    printf("This line should never be printed\r\n");
}

static void UndefinedInstruction_Test(void)
{
    printf("Now execute undefined instruction (UDF)\r\n");
    Delay_ms(10);
#if defined(__CC_ARM)
    extern void Execute_UDF_Once(void);
    Execute_UDF_Once();
#else
    __asm volatile ("UDF #0");
#endif

    printf("This line should never be printed\r\n");
}

#if defined(__CC_ARM)
__asm void Execute_UDF_Once(void)
{
    DCW 0xDE00
    BX lr
}
#endif

void HardFault_Handler(void)
{
    /* Make the fault obvious under debugger (halts on BKPT), then stop here. */
    __asm volatile ("BKPT #0");
    while (1)
    {
        /* Spin */
    }
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

    // ArrayOutOfBounds_Test();
    // IllegalCodeAddress_Test();
    UndefinedInstruction_Test();

    while (1)
    {
        gpio_pin_set(GPIO20);
        Delay_ms(200);
        gpio_pin_clear(GPIO20);
        Delay_ms(200);
    }
}

void SetClockFreq(void)
{
    return;
}
/** @} */ /* end of examples group */
