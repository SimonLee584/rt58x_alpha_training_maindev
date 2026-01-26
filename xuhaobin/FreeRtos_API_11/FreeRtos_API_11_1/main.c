
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

TaskHandle_t LED0_task_handle;
TaskHandle_t Query_task_handle;

static void LED0_task(void *pvParameters);
static void Query_task(void *pvParameters);

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
    xTaskCreate(Query_task, "Query_task", configMINIMAL_STACK_SIZE, NULL, 1, &Query_task_handle);

		
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

static void Query_task(void *pvParameters)
{
    uint32_t totalruntime = 0;
    UBaseType_t ArraySize,x;
    TaskStatus_t *StatusArray;

    printf("********第一步:函数uxTaskGetSystemState()的使用********/\r\n");
    ArraySize = uxTaskGetNumberOfTasks();
    StatusArray = pvPortMalloc(ArraySize * sizeof(TaskStatus_t));
    if(StatusArray != NULL){
        ArraySize = uxTaskGetSystemState((TaskStatus_t *)StatusArray,(UBaseType_t)ArraySize,(uint32_t *)&totalruntime);
        printf("TaskName\t\tPriority\t\tTaskNumber\t\t\r\n");
        for(x=0; x<ArraySize; x++){
            printf("%s\t\t%d\t\t%d\t\t\r\n",StatusArray[x].pcTaskName,(int)StatusArray[x].uxCurrentPriority,(int)StatusArray[x].xTaskNumber);
        }
    }
    vPortFree(StatusArray);
    printf("/**************************结束**************************/\r\n");
    printf("按下KEY0键继续！\r\n\r\n\r\n");
    while(Key_scan() != 1) Delay_ms(10);

    TaskHandle_t TaskHandle;
    TaskStatus_t TaskStatus;
    printf("/************第二步：函数vtaskGetInfo()的使用************/\r\n");
    TaskHandle = xTaskGetHandle("LED0_task");
    vTaskGetInfo(TaskHandle, &TaskStatus, pdTRUE, eInvalid);
    printf("任务名：                   %s\r\n",TaskStatus.pcTaskName);
    printf("任务编号：                 %d\r\n",(int)TaskStatus.xTaskNumber);
    printf("任务状态：                 %d\r\n",TaskStatus.eCurrentState);
    printf("任务当前优先级：           %d\r\n",(int)TaskStatus.uxCurrentPriority);
    printf("任务基优先级：             %d\r\n",(int)TaskStatus.uxBasePriority);
    printf("任务堆栈基地址：           %#x\r\n",(int)TaskStatus.pxStackBase);
    printf("任务堆栈历史剩余最小值：   %d\r\n",TaskStatus.usStackHighWaterMark);
    printf("/**************************结束**************************/\r\n");
    printf("按下KEY0键继续！\r\n\r\n\r\n");
    while(Key_scan() != 1) Delay_ms(10);

    eTaskState Taskstate;
    char TaskInfo[10];
    printf("/************第三步:函数eTaskGetState()的使用************/\r\n");
    TaskHandle = xTaskGetHandle("Query_task");
    Taskstate = eTaskGetState(TaskHandle);
    memset(TaskInfo,0,sizeof(TaskInfo));
    switch((int)Taskstate){
        case 0:sprintf(TaskInfo,"Running");break;
        case 1:sprintf(TaskInfo,"Ready");break;
        case 2:sprintf(TaskInfo,"Suspend");break;
        case 3:sprintf(TaskInfo,"Delete");break;
        case 4:sprintf(TaskInfo,"Invaild");break;
    }
    printf("任务壮态值:%d,对应的壮态为 :%s\r\n",Taskstate,TaskInfo); 
    printf("/**************************结束 **************************/\r\n");
    printf("按下KEY0键继续！\r\n\r\n\r\n");
    while(Key_scan() != 1) Delay_ms(10);

    printf("/*************第四步:函数vTaskList()的使用*************/\r\n"); 
    vTaskList(InfoBuffer);
    printf("%s\r\n",InfoBuffer);
    
    while (1) { 
        gpio_pin_toggle(GPIO21);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

