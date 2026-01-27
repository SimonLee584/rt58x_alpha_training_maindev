
#include <stdio.h>
#include <string.h>
#include "cm3_mcu.h"
#include "project_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "uart_drv.h"
#include "retarget.h"
#include "comm_subsystem_drv.h"
#include "rf_mcu_ahb.h"

#define PRINTF_BAUDRATE      UART_BAUDRATE_115200

int main(void);

#define GPIO0       0
#define GPIO1       1
#define GPIO2       2
#define GPIO20      20
#define GPIO21      21

#define SUBSYSTEM_CFG_PMU_MODE              0x4B0
#define SUBSYSTEM_CFG_LDO_MODE_DISABLE      0x02
/************************************************************/

#define EVENT_BIT0      0x01
#define EVENT_BIT1      0x02
#define EVENT_BIT2      0x04

TaskHandle_t Eventsetbit_task_handle;
TaskHandle_t Eventgroup_task_handle;

static void Eventsetbit_task(void *pvParameters);
static void Eventgroup_task(void *pvParameters);

#define DEBOUNCE_MS  20

static uint8_t key1_pressed = 0, key2_pressed = 0, key3_pressed = 0;
static TickType_t key1_tick = 0, key2_tick = 0, key3_tick = 0;

uint8_t Key_scan(void)
{
    // KEY1扫描
    if (gpio_pin_get(GPIO0) == 0) { 
        if (!key1_pressed) { 
            if (xTaskGetTickCount() - key1_tick >= pdMS_TO_TICKS(DEBOUNCE_MS)) {
                key1_pressed = 1;
                return 1;
            }
        }
    } else { 
        key1_pressed = 0;
        key1_tick = xTaskGetTickCount(); 
    }

    // KEY2扫描
    if (gpio_pin_get(GPIO1) == 0) {
        if (!key2_pressed) {
            if (xTaskGetTickCount() - key2_tick >= pdMS_TO_TICKS(DEBOUNCE_MS)) {
                key2_pressed = 1;
                return 2;
            }
        }
    } else {
        key2_pressed = 0;
        key2_tick = xTaskGetTickCount();
    }

    // KEY3扫描
    if (gpio_pin_get(GPIO2) == 0) {
        if (!key3_pressed) {
            if (xTaskGetTickCount() - key3_tick >= pdMS_TO_TICKS(DEBOUNCE_MS)) {
                key3_pressed = 1;
                return 3;
            }
        }
    } else {
        key3_pressed = 0;
        key3_tick = xTaskGetTickCount();
    }

    return 0; // 无按键触发
}

void Init_Default_Pin_Mux(void)
{
    int i;

    /*set all pin to gpio, except GPIO16, GPIO17 */
    for (i = 0; i < 32; i++)
    {
        if ((i != 16) && (i != 17) && (i != 28) && (i != 29))
        {
            pin_set_mode(i, MODE_GPIO);
        }
    }

    /*uart0 pinmux*/
    pin_set_mode(28, MODE_UART);     /*GPIO28 as UART1 TX*/
    pin_set_mode(29, MODE_UART);
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

void user_gpio2_isr_handler(uint32_t pin, void *isr_param){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    Delay_ms(50);
    if(gpio_pin_get(GPIO2) == 0){
        xTaskNotifyFromISR(Eventgroup_task_handle, EVENT_BIT2, eSetBits, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void Key_init(){
    gpio_set_debounce_time(DEBOUNCE_SLOWCLOCKS_1024);                       //set GPIO debounce time clock

    gpio_debounce_enable(GPIO0);                                                                //enable gpio0 debounce time.
    pin_set_pullopt(GPIO0, MODE_PULLUP_100K);                           //set the pin to default pull high 100K mode
    gpio_cfg_input(GPIO0, GPIO_PIN_NOINT); 
    
    gpio_debounce_enable(GPIO1);                                                                //enable gpio0 debounce time.
    pin_set_pullopt(GPIO1, MODE_PULLUP_100K);                           //set the pin to default pull high 100K mode
    gpio_cfg_input(GPIO1, GPIO_PIN_NOINT);  

    gpio_debounce_enable(GPIO2);                                                                //enable gpio0 debounce time.
    pin_set_pullopt(GPIO2, MODE_PULLUP_100K);                           /*set the pin to default pull high 100K mode*/
    gpio_cfg_input(GPIO2, GPIO_PIN_INT_EDGE_FALLING);                       /*set gpio 0 falling and rigger edge Interrupt Mode*/
    gpio_register_isr( GPIO2, user_gpio2_isr_handler, NULL);
    gpio_int_enable(GPIO2);
}

void LED_init(){ 
    gpio_pin_clear(GPIO20);
    gpio_cfg_output(GPIO20);
    gpio_pin_clear(GPIO20);      
    Delay_ms(10);

    gpio_pin_clear(GPIO21);
    gpio_cfg_output(GPIO21);
    gpio_pin_clear(GPIO21);     
    Delay_ms(10);
}

int main(void)
{
    Change_Ahb_System_Clk(SYS_48MHZ_CLK);

    Init_Default_Pin_Mux();

    /*init debug uart port for printf*/
    console_drv_init(PRINTF_BAUDRATE);

    Comm_Subsystem_Disable_LDO_Mode();//if don't load 569 FW, need to call the function.

    Delay_Init();
    Key_init();
    LED_init();

    xTaskCreate(Eventsetbit_task, "Eventsetbit_task", configMINIMAL_STACK_SIZE, NULL, 1, &Eventsetbit_task_handle);
    xTaskCreate(Eventgroup_task, "Eventgroup_task", configMINIMAL_STACK_SIZE, NULL, 2, &Eventgroup_task_handle);

    vTaskStartScheduler();
    while (1) {;}
}

static void Eventsetbit_task(void *pvParameters)
{
    uint8_t key;
    while(1){
        if(Eventgroup_task_handle != NULL){
            key = Key_scan();
            switch(key){
                case 1:
                    xTaskNotify(Eventgroup_task_handle, EVENT_BIT0,eSetBits);
                    break;
                    case 2:
                    xTaskNotify(Eventgroup_task_handle, EVENT_BIT1,eSetBits);
                    break;
            }
        }
        gpio_pin_toggle(GPIO20);
        vTaskDelay(60);
    }
}

static void Eventgroup_task(void *pvParameters)
{
    uint8_t eventvalue;
    static uint8_t event0flag,event1flag,event2flag;
    uint32_t NotifyValue;
    BaseType_t err;

    while(1){
        err = xTaskNotifyWait(0x00,0xffffff,&NotifyValue,portMAX_DELAY);
        if(err == pdPASS){
            if(NotifyValue & EVENT_BIT0){
                event0flag = 1;
            }
            else if(NotifyValue & EVENT_BIT1){
                event1flag = 1;
            }
            else if(NotifyValue & EVENT_BIT2){
                event2flag = 1;
            }
            eventvalue = event0flag |(event1flag << 1) |(event2flag << 2);
            printf("eventvalue = %d\r\n",eventvalue);
            if(event0flag && event1flag && event2flag){
                gpio_pin_toggle(GPIO21);
                event0flag = 0;
                event1flag = 0;
                event2flag = 0;
            }
        }
        else{
            printf("xTaskNotifyWait err\r\n");
        }
    }
}


