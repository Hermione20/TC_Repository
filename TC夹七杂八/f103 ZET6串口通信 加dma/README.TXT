实验器材:
	战舰STM32F103开发板V3版本
	
实验目的:
	学习串口的使用(接收与发送)
	
硬件资源:
	1,外接LED(连接在PB8、PF6、PF8、PF9)
	3,串口1(波特率:115200,PA9/PA10连接在板载USB转串口芯片CH340上面)
	
实验现象:
	本实验,STM32通过串口1和上位机对话，STM32在收到上位机发过来的字符串(以回车换
	行结束)后，完成指令相应的功能如下：

		字符串             功能现象
		LEDn Turn On	  对应的LEDn被点亮（n=1,2,3,4）
		LEDn Turn Off	  对应的LEDn被熄灭
		LEDn Flash m	  对应的LEDn闪烁m次
		LEDString         String理解为一串字符信息，每个字符对应的16进制表示后其二进制表示的四个位就是各LED灯的状态，0为灭，1为亮
						  例LED123F，即0001,0010,1111，现象为LED1亮；接着LED1灭、LED2亮；接着LED1~4均亮。
	
	
	下载后，同时每隔一定时间，通过串口1输出一段提示信息到电脑。 
	
注意事项:
	1,电脑端串口调试助手波特率必须是115200.
	2,请使用XCOM/SSCOM串口调试助手,其他串口助手可能控制DTR/RTS导致MCU复位/程序不运行
	3,串口输入字符串以回车换行结束.
	4,请用USB线连接在USB_232,找到USB转串口后测试本例程.
	5,P4的PA9/PA10必须通过跳线帽连接在RXD/TXD上.

更新日志:
2023.8.26  完成代码编写
2023.8.26  进行各功能的调试，存在问题：
			1.String功能尚未实现对大小写字母的兼容
			2.错误字符检测未完善
2023.8.27   代码更新			temp_rx_buf=(USART_RX_BUF[LED_count]-'0');
							
							if(temp_rx_buf>9&&temp_rx_buf<=96)temp_rx_buf-=7;
							else if(temp_rx_buf>96)temp_rx_buf-=39;
							
							USART_LED_STA=(temp_rx_buf<<4);	
			实现大小写敏感

2023.8.28  	LED_controller函数主要else if逻辑优化，新增else提高容错
			加入dma内容

			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			