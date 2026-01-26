
#include <stdio.h>
#include <string.h>
#include "cm3_mcu.h"
#include "project_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "list.h"
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

List_t TestList; 
ListItem_t ListItem1;
ListItem_t ListItem2;
ListItem_t ListItem3;

TaskHandle_t LED0_task_handle;
TaskHandle_t List_task_handle;

static void LED0_task(void *pvParameters);
static void List_task(void *pvParameters);

#define DEBOUNCE_MS  20

static uint8_t key1_pressed = 0, key2_pressed = 0, key3_pressed = 0;
static TickType_t key1_tick = 0, key2_tick = 0, key3_tick = 0;

char InfoBuffer[1000];

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

    LED_init();

    xTaskCreate(LED0_task, "LED0_task", configMINIMAL_STACK_SIZE, NULL, 1, &LED0_task_handle);
    xTaskCreate(List_task, "List_task", configMINIMAL_STACK_SIZE, NULL, 1, &List_task_handle);

		
    vTaskStartScheduler();
    while (1) {;}
}

static void LED0_task(void *pvParameters)
{
    while (1) {
        gpio_pin_toggle(GPIO20);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void List_task(void *pvParameters)
{

    vListInitialise(&TestList); 
    vListInitialiseItem(&ListItem1); 
    vListInitialiseItem(&ListItem2); 
    vListInitialiseItem(&ListItem3);

    ListItem1.xItemValue=40; 
    ListItem2.xItemValue=60;
    ListItem3.xItemValue=50;
    printf("/*********** 列表和列表项地址 ************/ \r\n"); 
    printf("项目                            地址       \r\n"); 
    printf("TestList                        %#x       \r\n",(int)&TestList); 
    printf("TestList->pxIndex               %#x       \r\n",(int)TestList.pxIndex); 
    printf("TestList->xListEnd              %#x       \r\n",(int)(&TestList.xListEnd)); 
    printf("ListItem1                       %#x       \r\n",(int)&ListItem1); 
    printf("ListItem2                       %#x       \r\n",(int)&ListItem2); 
    printf("ListItem3                       %#x       \r\n",(int)&ListItem3); 
    printf("/**************** 结束 *****************/ \r\n");
    printf("按下KEY0键继续！\r\n\r\n\r\n");
    while(Key_scan() != 1) Delay_ms(10);

    vListInsert(&TestList,&ListItem1); 
    printf("/********** 添加列表ListItem1 ***********/ \r\n"); 
    printf("项目                            地址       \r\n"); 
    printf("TestList->xListEnd->pxNext      %#x       \r\n",(int)(TestList.xListEnd.pxNext)); 
    printf("ListItem1->pxNext               %#x       \r\n",(int)(ListItem1.pxNext)); 
    printf("/*********** 前后向连接分割线 ************/ \r\n"); 
    printf("TestList->xListEnd->pxPrevious  %#x       \r\n",(int)(TestList.xListEnd.pxPrevious)); 
    printf("ListItem1->pxPrevious           %#x       \r\n",(int)(ListItem1.pxPrevious)); 
    printf("/***************** 结束 ****************/ \r\n");
    printf("按下KEY0键继续！\r\n\r\n\r\n");
    while(Key_scan() != 1) Delay_ms(10);

    vListInsert(&TestList,&ListItem2); 
    printf("/********* 添加列表项ListItem2 **********/ \r\n");
    printf("项目                            地址       \r\n"); 
    printf("TestList->xListEnd->pxNext      %#x        \r\n",(int)(TestList.xListEnd.pxNext));
    printf("ListItem1->pxNext               %#x        \r\n",(int)(ListItem1.pxNext));
    printf("ListItem2->pxNext               %#x        \r\n",(int)(ListItem2.pxNext)); 
    printf("/************ 前后向连接分割线 ***********/ \r\n"); 
    printf("TestList->xListEnd->pxPrevious  %#x        \r\n",(int)(TestList.xListEnd.pxPrevious)); printf("ListItem1->pxPrevious %#x \r\n",(int)(ListItem1.pxPrevious));
    printf("ListItem2->pxPrevious           %#x        \r\n",(int)(ListItem2.pxPrevious)); 
    printf("/**************** 结束 *****************/   \r\n");
    printf("按下KEY0键继续！\r\n\r\n\r\n");
    while(Key_scan() != 1) Delay_ms(10);

    vListInsert(&TestList,&ListItem3); 
    printf("/********* 添加列表项ListItem3 **********/ \r\n");
    printf("项目                            地址       \r\n"); 
    printf("TestList->xListEnd->pxNext      %#x       \r\n",(int)(TestList.xListEnd.pxNext));
    printf("ListItem1->pxNext               %#x       \r\n",(int)(ListItem1.pxNext)); 
    printf("ListItem3->pxNext               %#x       \r\n",(int)(ListItem3.pxNext)); 
    printf("ListItem2->pxNext               %#x       \r\n",(int)(ListItem2.pxNext)); 
    printf("/*********** 前后向连接分割线 ************/ \r\n");
    printf("TestList->xListEnd->pxPrevious  %#x       \r\n",(int)(TestList.xListEnd.pxPrevious)); 
    printf("ListItem1->pxPrevious           %#x       \r\n",(int)(ListItem1.pxPrevious));
    printf("ListItem3->pxPrevious           %#x       \r\n",(int)(ListItem3.pxPrevious)); 
    printf("ListItem2->pxPrevious           %#x       \r\n",(int)(ListItem2.pxPrevious)); 
    printf("/***************** 结束 ****************/ \r\n");
    printf("按下KEY0键继续！\r\n\r\n\r\n");
    while(Key_scan() != 1) Delay_ms(10);

    uxListRemove(&ListItem2);
    printf("/********** 删除列表项ListItem2 *********/ \r\n"); 
    printf("项目                            地址       \r\n"); 
    printf("TestList->xListEnd->pxNext      %#x       \r\n",(int)(TestList.xListEnd.pxNext));
    printf("ListItem1->pxNext               %#x       \r\n",(int)(ListItem1.pxNext)); 
    printf("ListItem3->pxNext               %#x       \r\n",(int)(ListItem3.pxNext));
    printf("/************ 前后向连接分割线 ***********/ \r\n"); 
    printf("TestList->xListEnd->pxPrevious  %#x       \r\n",(int)(TestList.xListEnd.pxPrevious));
    printf("ListItem1->pxPrevious           %#x       \r\n",(int)(ListItem1.pxPrevious)); 
    printf("ListItem3->pxPrevious           %#x       \r\n",(int)(ListItem3.pxPrevious)); 
    printf("/**************** 结束 *****************/  \r\n");
    printf("按下KEY0键继续！\r\n\r\n\r\n");
    while(Key_scan() != 1) Delay_ms(10);

    TestList.pxIndex=TestList.pxIndex->pxNext;
    vListInsertEnd(&TestList,&ListItem2); 
    printf("/****** 在末尾添加列表项ListItem2 *******/ \r\n"); 
    printf("项目                            地址      \r\n"); 
    printf("TestList->pxIndex               %#x       \r\n",(int)TestList.pxIndex); 
    printf("TestList->xListEnd->pxNext      %#x       \r\n",(int)(TestList.xListEnd.pxNext));
    printf("ListItem2->pxNext               %#x       \r\n",(int)(ListItem2.pxNext)); 
    printf("ListItem1->pxNext               %#x       \r\n",(int)(ListItem1.pxNext)); 
    printf("ListItem3->pxNext               %#x       \r\n",(int)(ListItem3.pxNext));
    printf("/*********** 前后向连接分割线 **********/  \r\n"); 
    printf("TestList->xListEnd->pxPrevious  %#x      \r\n",(int)(TestList.xListEnd.pxPrevious));
    printf("ListItem2->pxPrevious           %#x      \r\n",(int)(ListItem2.pxPrevious)); 
    printf("ListItem1->pxPrevious           %#x      \r\n",(int)(ListItem1.pxPrevious)); 
    printf("ListItem3->pxPrevious           %#x      \r\n",(int)(ListItem3.pxPrevious)); 
    printf("/****************结束 *****************/  \r\n\r\n\r\n");

    while (1) { 
        gpio_pin_toggle(GPIO21);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

