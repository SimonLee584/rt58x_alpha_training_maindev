
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

#define TIMER0_ID       0
#define TEST_UART_PORT 1

#define SUBSYSTEM_CFG_PMU_MODE              0x4B0
#define SUBSYSTEM_CFG_LDO_MODE_DISABLE      0x02
/************************************************************/
#define rtos_task1_priority 0
#define keyprocess_task_priority 0  

#define KEYMSG_Q_NUM 1
#define MSG_Q_NUM 4

TaskHandle_t rtos_task1_handele;
TaskHandle_t keyprocess_task_handele;
QueueHandle_t KEY_QUEUE;
QueueHandle_t MSG_QUEUE;

static void  rtos_task1(void *pvParameters);
static void keyprocess_task(void *pvParameters);

uint8_t rec_com_data;

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

void Key_init(){
    gpio_set_debounce_time(DEBOUNCE_SLOWCLOCKS_1024);                       //set GPIO debounce time clock

    gpio_debounce_enable(GPIO0);                                                                //enable gpio0 debounce time.
    pin_set_pullopt(GPIO0, MODE_PULLUP_100K);                           //set the pin to default pull high 100K mode
    gpio_cfg_input(GPIO0, GPIO_PIN_NOINT); 
    
    gpio_debounce_enable(GPIO1);                                                                //enable gpio0 debounce time.
    pin_set_pullopt(GPIO1, MODE_PULLUP_100K);                           //set the pin to default pull high 100K mode
    gpio_cfg_input(GPIO1, GPIO_PIN_NOINT);  

    gpio_debounce_enable(GPIO2);                                                                //enable gpio0 debounce time
    pin_set_pullopt(GPIO2, MODE_PULLUP_100K);                           //set the pin to default pull high 100K mode
    gpio_cfg_input(GPIO2, GPIO_PIN_NOINT);  
}

void Timer0_Callback(uint32_t timer_id){

    uint8_t rec_q_data;
    BaseType_t xHigherPriorityTaskWoken;

    BaseType_t xStatus = xQueueReceiveFromISR(MSG_QUEUE, &rec_q_data, &xHigherPriorityTaskWoken);
    if (xStatus == pdPASS){
            printf("receive data from queue:%d\n", rec_q_data);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}
void Timer_cfg(){

    timer_config_mode_t cfg;

    /** @ Timer interrupt enable, timerClk = 32MHz, clock is divided by 32, timer clock = 1MHz (1us), load = 999, timer interrupt = 1ms*/
    cfg.int_en = ENABLE;
    cfg.mode = TIMER_PERIODIC_MODE;
    cfg.prescale = TIMER_PRESCALE_128;
    Timer_Open(TIMER0_ID, cfg, Timer0_Callback);
    Timer_Start(TIMER0_ID, 250000-1);
}

void uart1_callback(uint32_t event, void *p_context)
{
    if (event & UART_EVENT_TX_DONE)
    {

    }

    if (event & UART_EVENT_RX_DONE)
    {
       
        BaseType_t xHigherPriorityTaskWoken;

        uart_rx(TEST_UART_PORT, &rec_com_data, 1);
        
        BaseType_t xStatus = xQueueSendFromISR(MSG_QUEUE, &rec_com_data, &xHigherPriorityTaskWoken);
        if (xStatus == pdPASS){
            printf("Successfully sent the data to the queue:%d\n", rec_com_data);
        }
        else{
            printf("Failed to send data to the queue:%d\n",rec_com_data);
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    if (event & (UART_EVENT_RX_OVERFLOW | UART_EVENT_RX_BREAK |
                 UART_EVENT_RX_FRAMING_ERROR | UART_EVENT_RX_PARITY_ERROR ))
    {
        printf("uart error event\n");
    }
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
    static uint32_t  handle;
    uart_config_t  uart1_drv_config;

    Change_Ahb_System_Clk(SYS_48MHZ_CLK);

    Init_Default_Pin_Mux();

    /*init debug uart port for printf*/
    console_drv_init(PRINTF_BAUDRATE);

    Comm_Subsystem_Disable_LDO_Mode();//if don't load 569 FW, need to call the function.

    Delay_Init();
    Key_init();
    Timer_cfg();
    LED_init();

    handle = 0;
    uart1_drv_config.baudrate = UART_BAUDRATE_115200;
    uart1_drv_config.databits = UART_DATA_BITS_8;
    uart1_drv_config.hwfc     = UART_HWFC_DISABLED;
    uart1_drv_config.parity   = UART_PARITY_NONE;
    uart1_drv_config.p_context = (void *) &handle;
    uart1_drv_config.stopbit  = UART_STOPBIT_ONE;
    uart1_drv_config.interrupt_priority = IRQ_PRIORITY_NORMAL;
    uart_init(TEST_UART_PORT, &uart1_drv_config, uart1_callback);

    uart_rx(TEST_UART_PORT, &rec_com_data, 1);

    KEY_QUEUE = xQueueCreate(KEYMSG_Q_NUM, sizeof(uint8_t));
    MSG_QUEUE = xQueueCreate(MSG_Q_NUM, sizeof(uint8_t));

    xTaskCreate(rtos_task1, "task1", configMINIMAL_STACK_SIZE, NULL, rtos_task1_priority, &rtos_task1_handele);
    xTaskCreate(keyprocess_task, "task2", configMINIMAL_STACK_SIZE, NULL, keyprocess_task_priority, &keyprocess_task_handele);
		
    vTaskStartScheduler();
    while (1) {;}
}

#if 1
static void rtos_task1(void *parameters_ptr)
{ 
    uint8_t key = 0;
    BaseType_t err;
    for(;;){
        key = Key_scan();                                          
        if(KEY_QUEUE != NULL && key){                               
            err = xQueueSend(KEY_QUEUE, &key, 10);                  
            if(err != pdTRUE){                                     
                printf("队列已满，发送失败！！！");                   
            }                                                       
        }                                                           
        vTaskDelay(10);
    }   
}

static void keyprocess_task(void *parameters_ptr)
{
    uint8_t key = 0;
     
    for(;;){
        if(xQueueReceive(KEY_QUEUE, &key, portMAX_DELAY) == pdTRUE){
            switch(key){
                case 1:
                    gpio_pin_toggle(GPIO20);
                    printf("KEY0 pressed\n");
                    break;
                case 2:
                    gpio_pin_toggle(GPIO21);
                    printf("KEY1 pressed\n");
                    break;
                case 3:
                    gpio_pin_toggle(GPIO20);
                    gpio_pin_toggle(GPIO21);
                    printf("KEY2 pressed\n");
                    break;
            }
        }

    }
}
#else
//使用任务通知模拟队列(只能模拟数量为1的队列)
static void rtos_task1(void *parameters_ptr)
{ 
    uint8_t key = 0;
    BaseType_t err;
    for(;;){
        key = Key_scan();                                           
        if(keyprocess_task_handele != NULL && key){   
        err = xTaskNotify(keyprocess_task_handele, key, eSetValueWithOverwrite);  
        if(err != pdTRUE){  
            printf("队列已满，发送失败！！！"); 
            }
        }                                   
        vTaskDelay(10);
    }
}

static void keyprocess_task(void *parameters_ptr)
{
    uint32_t NotifyValue;
    BaseType_t err;
    for(;;){
        err = xTaskNotifyWait(0, 0xFFFFFFFF, &NotifyValue, portMAX_DELAY);
        if(err == pdTRUE){
            switch(NotifyValue){
                case 1:
                    gpio_pin_toggle(GPIO20);
                    printf("KEY0 pressed\n");
                    break;
                case 2:
                    gpio_pin_toggle(GPIO21);
                    printf("KEY1 pressed\n");
                    break;
                case 3:
                    gpio_pin_toggle(GPIO20);
                    gpio_pin_toggle(GPIO21);
                    printf("KEY2 pressed\n");
                    break;
            }
        }
    }  
}
#endif
