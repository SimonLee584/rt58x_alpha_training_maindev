第十三章 队列

队列操作实验

main()：初始化GPIO（用于按键和LED）、TIMER0，URAT1，并注册TIMER0和UART1的中断回调函数
	    创建两个队列，分别用于传递键值（KEY_QUEUE，容量为1）和传递串口接收到的数据（MSG_QUEUE，容量为4）
	    创建两个任务rtos_task1、keyprocess_task

UART1中断回调uart1_callback：当UART1每次完成数据接收时触发，并将接收到的数据发送到队列MSG_QUEUE，发送成功或失败都会打印相关信息

TIMER0中断回调Timer0_Callback：每秒触发一个并从队列MSG_QUEUE读取一个值，读取成功则将该数值打印出来

rtos_task1：通过按下key0,key1,key2来获取不同的键值，并将键值发送到队列KEY_QUEUE中

keyprocess_task：读取队列中的键值，每个键值对应不同按键
key0：控制LED0翻转
key1：控制LED1翻转
key3：同时控制LED0、LED1翻转