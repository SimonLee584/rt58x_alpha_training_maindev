1、I2C Master and Slave

I2C Master
路径：./examples/i2c/i2c-master
说明：初始化为i2c master,将从UART1接收到的数据通过发送到i2c slave

I2C Slave
路径：./examples/i2c/i2c-slave
说明：初始化为i2c slave,将从i2c master接收到的数据通过UART0打印出来

引脚连接：GPIO22(master)--GPIO28(slave)
		GPIO23(master)--GPIO29(slave)
		
2、BLE KeyBoard
路径：./examples/KeyBoard
说明：main()：创建一个任务，负责初始化各类外设和ble stack，并处理键盘、ble服务等事件

键盘相关设置：
1、按键初始化：初始化KEY0-4，设置为内部上拉，双边沿触发中断
2、按键意义:KEY0——Shift键
		  KEY1——1键
		  KEY2——A键
		  KEY3——B键
		  KEY4——C键
2、实现逻辑：按下或松开任意按键，触发中断回调，发送相关键盘处理消息到队列中，
		  主任务获取队列并解析消息后，获取并发送报告给客户端。
