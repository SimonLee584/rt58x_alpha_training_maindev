第十一章 其他任务API函数

说明：本章分为两个实验，分别于两个工程实现

实验一：任务状态查询API函数实验（FreeRtos_API_11_1）

main()：创建两个任务：LED0_task、Query_task

LED0_task：控制LED0每五百秒翻转电平，提示系统正在运行

Query_task：任务状态查询操作，每完成一个步骤都需按下key0以便程序继续运行
步骤
(1)使用uxTaskGetSystemstate()获取当前所有任务的信息，并将任务名、优先级、任务编号通过串口打印出来
(2)使用vTaskInfo()函数获取任务LED0_task的任务信息并打印出来
(3)使用eTaskGetState()来获取任务query_task的运行状态并打印出来
(4)使用vTaskList()函数获取系统的任务的详细信息，并通过表格形式打印出来

实验二：任务运行时间信息统计实验（FreeRtos_API_11_2）

main()：创建三个任务：LED0_task、LED1_task、Runtimestats_task

LED0_task、LED1_task：分别控制LED0、LED1以不同周期闪烁

Runtimestats_task：使用vTaskGetRunTomeStats()函数获取任务运行时间

else：初始化TIM0，为FreeRTOS的时间统计提供时基，在文件timer.c中