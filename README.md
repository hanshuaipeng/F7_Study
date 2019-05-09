		usbd_conf.h修改 
		
		#define USBH_malloc(x)               mymalloc(SRAMIN,x)

		#define USBH_free(x)                 myfree(SRAMIN,x)
	
增加USB_HOST->MSC,读取U盘

增加iic驱动

增加IO扩展

增加us延时函数

更改usbh_conf.c  PCF8574_WriteBit(USB_PWR_IO,1); //开启USB HOST电源供电IC

更改fatfs    #define FF_VOLUMES		4 逻辑设备数量

5/19

加入fatfs的部分操作代码，exfuns.c

加入汉字，LCD_Chinese

修改fatfs中的ffunicode.c，将里边的大数组删除，通过SD卡写入到外部flash


