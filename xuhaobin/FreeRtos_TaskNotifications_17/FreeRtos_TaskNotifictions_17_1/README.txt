第十七章 任务通知

说明：本章共有4个实验
         模拟二值信号量和计数型信号量的实验位于工程FreeRtos_TaskNotifictions_17_1
         模拟时间标志组实验位于工程FreeRtos_TaskNotifictions_17_2
         模拟队列实验在工程FreeRtos_Queue_13的main.c最后有体现，这里不再新建实验工程

实验一、二：任务通知模拟二值信号量和计数型信号量实验

main()：初始化UART1，注册回调函数，创建三个任务：Data_process_task、SemapGive_task、SemapTake_task

UART1回调函数：接收串口发送过来的数据，并在接收完成时释放二值信号量

Data_process_task：获取二值信号量成功后处理串口发送的指令，并根据指令执行相关操作
注：串口用于接收特定字符串，然后通过Data_process_task任务处理后来控制LED0、LED1的亮灭
(大小写无要求，程序自动转换)
LED0ON：点亮LED0  LED0OFF：熄灭LED0
LED1IN：点亮LED1  LED1OFF：熄灭LED1
其它：无效指令

SemapGive_task：每按下一次KEY0，释放一次计数型信号量

SemapTake_task：获取二值信号量并打印当前二值信号量的值（每秒获取一次）

实验三：任务通知模拟事件标志组实验

main()：注册KEY2的下降沿中断回调函数，创建两个任务Eventsetbit_task，Eventgroup_task

KEY2的回调函数：设置事件标志组的bit2

Eventsetbit_task：通过按键KEY0、KEY1分别设置事件标志组的bit0、bit1

Eventgroup_task：处理事件标志组事件
当KEY0、KEY1、KEY2都按下时，此时事件标志组的bit0~3都置1，翻转LED1的电平


