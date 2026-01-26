
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
#define LED0ON 1
#define LED0OFF 2
#define LED1ON 3
#define LED1OFF 4
#define COMMANDERR 0XFF
#define COMMAND_LENGTH 10

TaskHandle_t data_process_task_handle;
TaskHandle_t SemapGive_task_handle;
TaskHandle_t SemapTake_task_handle;

static void SemapGive_task(void *pvParameters);
static void SemapTake_task(void *pvParameters);
static void Data_process_task(void *pvParameters);

uint8_t rec_com_data,length;
uint8_t control_msg[COMMAND_LENGTH];

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

void uart1_callback(uint32_t event, void *p_context)
{
    
    if (event & UART_EVENT_TX_DONE)
    {

    }

    if (event & UART_EVENT_RX_DONE)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        uart_rx(TEST_UART_PORT, &rec_com_data, 1);
        if(rec_com_data != 0x0a && rec_com_data != 0x0d){
            control_msg[length] = rec_com_data;
            length++;
            if(length > sizeof(control_msg)-1){
                length = 0;
            }
        }
        else if(rec_com_data == 0x0a && data_process_task_handle != NULL){
            vTaskNotifyGiveFromISR(data_process_task_handle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

       
    }

    if (event & (UART_EVENT_RX_OVERFLOW | UART_EVENT_RX_BREAK |
                 UART_EVENT_RX_FRAMING_ERROR | UART_EVENT_RX_PARITY_ERROR ))
    {
        printf("uart error event\n");
    }
}

void LowerToCap(uint8_t *str,uint8_t len)
{ 
    uint8_t i; 
    for(i=0;i<len;i++)
    { 
        if((96<str[i])&&(str[i]<123))
        str[i]=str[i]-32;
    } 
}

uint8_t CommandProcess(uint8_t *str)
{ 
    uint8_t CommandValue = COMMANDERR; 
    if(strcmp((char*)str,"LED0ON") == 0) CommandValue = LED0ON; 
    else if(strcmp((char*)str,"LED0OFF") == 0) CommandValue = LED0OFF; 
    else if(strcmp((char*)str,"LED1ON") == 0) CommandValue = LED1ON; 
    else if(strcmp((char*)str,"LED1OFF") == 0) CommandValue = LED1OFF; 
    return CommandValue; 
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

    gpio_pin_clear(GPIO20);
    gpio_cfg_output(GPIO20);
    gpio_pin_clear(GPIO20);      
    Delay_ms(10);

    gpio_pin_clear(GPIO21);
    gpio_cfg_output(GPIO21);
    gpio_pin_clear(GPIO21);     
    Delay_ms(10);

    uart_rx(TEST_UART_PORT, &rec_com_data, 1);

    xTaskCreate(Data_process_task, "Data_process", configMINIMAL_STACK_SIZE, NULL, 1, &data_process_task_handle);
    xTaskCreate(SemapGive_task,"SemapGive_task",configMINIMAL_STACK_SIZE,NULL,2,&SemapGive_task_handle);
    xTaskCreate(SemapTake_task,"SemapTake_task",configMINIMAL_STACK_SIZE,NULL,2,&SemapTake_task_handle);

    vTaskStartScheduler();
    while (1) {;}
}

static void Data_process_task(void *pvParameters)
{ 
    uint8_t CommandValue = COMMANDERR;
    uint32_t NotifyValue;
    uint8_t CommandStr[COMMAND_LENGTH+1];

    while(1){
        NotifyValue = ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
        if(NotifyValue == 1){
            sprintf((char*)CommandStr,"%s",control_msg);
            CommandStr[length] = '\0';
            LowerToCap(CommandStr,length);
            CommandValue = CommandProcess(CommandStr);
            if(CommandValue != COMMANDERR){
                printf("命令为 :%s\r\n",CommandStr);
                switch(CommandValue){
                    case LED0ON:
                        gpio_pin_clear(GPIO20);
                        break;
                    case LED0OFF:
                        gpio_pin_set(GPIO20);
                        break;
                    case LED1ON:
                        gpio_pin_clear(GPIO21);
                        break;
                    case LED1OFF:
                        gpio_pin_set(GPIO21);
                        break;
                }
            
            }
            else{
                printf("无效的命令，请重新输入 !!\r\n");
            }
        length = 0;
        memset(control_msg,0,sizeof(control_msg));
        }
    }
}

static void SemapGive_task(void *pvParameters){
    uint8_t key = 0;
    while(1){
        key = Key_scan();
        if(SemapTake_task_handle != NULL && key == 1){
            xTaskNotifyGive(SemapTake_task_handle);
        }
        vTaskDelay(50);
    }
}

static void SemapTake_task(void *pvParameters){
    uint32_t NotifyValue;

    while(1){
        NotifyValue = ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
        printf("SemapTake_task: %d\n",NotifyValue-1);
        gpio_pin_toggle(GPIO21);
        vTaskDelay(1000);
    }
}
