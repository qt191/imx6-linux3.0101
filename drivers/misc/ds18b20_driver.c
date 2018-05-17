#include <linux/init.h>  
#include <linux/module.h>  
#include <linux/delay.h>  
#include <linux/kernel.h>  
#include <linux/moduleparam.h>  
#include <linux/init.h>  
#include <linux/types.h>  
#include <linux/fs.h>  
#include <mach/gpio.h>  
#include <linux/gpio.h>  
#include <linux/device.h>  
#include <mach/hardware.h>  
#include <linux/cdev.h>  
#include <asm/uaccess.h>  
#include <linux/errno.h>  
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>

#include <linux/misc/ds18b20.h>


#define 	DEVICE_NAME   			   		"ds18b20"

#define 	DS18B20_MINOR						30

#define		DQ_PIN							ds18b20_data->ds18b20_pin_number


#define 	DQ_OUTP								1
#define		DQ_INP								0

#define		DQ_OUT							ds18b20_gpio_cfgpin(DQ_PIN,DQ_OUTP)
#define		DQ_IN							ds18b20_gpio_cfgpin(DQ_PIN,DQ_INP)
#define		DQ_STA							ds18b20_gpio_get_value(DQ_PIN)
#define 	DQ_L							ds18b20_gpio_set_value(DQ_PIN,0)
#define		DQ_H							ds18b20_gpio_set_value(DQ_PIN,1)


//定义DS18B20ROM指令
#define		DS18B20_ReadRom				0x33    	//读ROM
#define		DS18B20_MatchRom				0x55		//匹配ROM
#define		DS18B20_SkipRom				0xCC		//跳过ROM
#define		DS18B20_SearchRom				0xF0		//搜索ROM
#define		DS18B20_AlarmRom				0xEC		//报警搜索


//定义DS18B20存储器指令
#define		DS18B20_WriteSCR				0x4E		//写暂存存储器
#define		DS18B20_ReadSCR				0xBE		//读暂存存储器
#define		DS18B20_CopySCR				0x48		//复制暂存存储器
#define		DS18B20_ConvertTemp			0x44		//温度转换
#define		DS18B20_RecallEP				0xB8		//重新调出
#define		DS18B20_ReadPower				0xB4		//读电源



/*函数声明*/
void ds18b20_gpio_cfgpin(unsigned int pin,unsigned int dat);
int ds18b20_gpio_get_value(unsigned int pin);
void ds18b20_gpio_set_value(unsigned int pin,unsigned int value);
void write_char(unsigned char comd);
unsigned char read_char(void);
int read_temperature(void);

/*---------------------------------------------------------------------------------*/



struct  ds18b20_platform_data *ds18b20_data;


void ds18b20_gpio_cfgpin(unsigned int pin,unsigned int dat){
	if(dat==1){
		gpio_direction_output(pin,1);
	}else{
		gpio_direction_input(pin);
	}
}
int ds18b20_gpio_get_value(unsigned int pin){
	int value;
	value = gpio_get_value(pin);
	return value;
}
void ds18b20_gpio_set_value(unsigned int pin,unsigned int value){
	gpio_set_value(pin,value);
}

unsigned int init_ds18b20(void){

		unsigned int dat = 0;
		DQ_OUT;
		DQ_H;
		udelay(10);
		
		DQ_L;
		udelay(600);
		
		DQ_H;
		udelay(65);

		DQ_IN;
		dat = DQ_STA; //0:right 1:wrong
		udelay(160);		

		DQ_OUT;
		DQ_H;
		udelay(50);
		//printk("open dat:%d\n",dat);
		return dat; 
}

void write_char(unsigned char comd){
	unsigned int i = 0;
	DQ_OUT;
	for(i=8;i>0;i--){
		udelay(1);
		DQ_L;
		udelay(15);
		if(comd&0x01){
			DQ_H;
			//printk("%d",DQ_STA);
			udelay(45);
		}else{
			udelay(45);
		}
		DQ_H;
		//udelay(10);
		comd>>=1;
	}
}

unsigned char read_char(void){
	int i= 0;
	unsigned char dat = 0;
	
	for(i=8;i>0;i--){
		
		
		DQ_OUT;	
		udelay(1);
		DQ_L;
		udelay(5);
		dat>>=1;
		DQ_H;
		udelay(2);

		DQ_IN;
		udelay(10);
		
		if(DQ_STA){
			dat |= 0x80;
		}
		
		udelay(45);
	}
	return dat;
}

int read_temperature(void){
	unsigned int msb = 0;
	unsigned int lsb = 0;
	unsigned int sb = 0;
	if(init_ds18b20()){
		printk("ds18b20 init failed!\n");
		return -1;
	}
	
	write_char(DS18B20_SkipRom);   //跳过读取ROM
	
	write_char(DS18B20_ConvertTemp);//开始温度转换
	msleep_interruptible(720);
	//udelay(900);	
	
	if(init_ds18b20()){
		printk("ds18b20 init failed!\n");
		return -1;
	}
	write_char(DS18B20_SkipRom);   //跳过读取ROM
	write_char(DS18B20_ReadSCR);   //读取寄存器

	lsb = read_char();            //前两个寄存器的值分别为温度的低位和高位
	//printk("lsb:%x\n",lsb);
	msb = read_char();
	//printk("msb:%x\n",msb);
	
	
	msb <<=8;
	//printk("msb:%x\n",msb);
	return msb+lsb;
}
/*************************************************************************************************************/
static int ds18b20_driver_open(struct inode *inode,struct file *filp){
	//printk("***********************Welcome to use Ds18b20 Sensors*************************\n");
	int ret;	
	ret = gpio_request(DQ_PIN,"DQ");
	if(ret != 0){
		printk("ERROR:Can't open the DQ_PIN!\n");
	}
	/*while(1){
	printk("%d\n",init_ds18b20());
	}*/
	return 0;
}
		
static ssize_t ds18b20_driver_read(struct file *filp,char __user *buf,size_t count,loff_t *oppos){
	
	unsigned int temp,t;
	temp = read_temperature();
	if(temp<0){
		printk("ERROR:Read temperature failed!\n");
		return -EFAULT;
	}
	//printk("tem:%x\n",temp);
	
	if(temp>0x8000){ 
		temp = (~temp)+1;     //判断寄存器最四位是否为1，若为1则取反加1；
		//t = temp * 0.0625;
	}/*else{
		//t = temp * 0.0625;
	}*/
	
	if(copy_to_user(buf,&temp,sizeof(temp))){
		return -EFAULT;
	}

	return sizeof(temp);
}

static ssize_t ds18b20_driver_write(struct file *filp,const __user *buf,size_t count,loff_t *oppos){
	return 0;
}
static int ds18b20_driver_ioctl(struct file *filp,unsigned int cmd,unsigned long arg){
	return 0;
}
static int ds18b20_driver_close(struct inode *inode,struct file *filp){
	gpio_free(DQ_PIN);

	return 0;
}
static const struct file_operations ds18b20_driver_fops ={
	.owner 			= THIS_MODULE,
	.open			= ds18b20_driver_open,
	.read  			= ds18b20_driver_read,
	.write 			= ds18b20_driver_write,
	.unlocked_ioctl = ds18b20_driver_ioctl,
	.release		= ds18b20_driver_close,
};

static struct miscdevice ds18b20_driver_misdev = {
	.name = DEVICE_NAME,
	.minor= DS18B20_MINOR,
	.fops = &ds18b20_driver_fops,
};

static int __devinit ds18b20_driver_probe(struct platform_device *pdev){
	int ret;

	ds18b20_data = pdev->dev.platform_data;
	
	ret = misc_register(&ds18b20_driver_misdev);
	if(ret != 0){
		printk("DS18B20 register failed!!\n");
	}else{
		printk("DS18B20 register success!!\n");
	}
	
	return ret;
}
static int __devexit ds18b20_driver_remove(struct platform_device *pdev){
	int ret;
	ret = misc_deregister(&ds18b20_driver_misdev);
	if(ret != 0){
		printk("DS18B20 deregister failed!!\n");
	}else{
		printk("DS18B20 deregister success!!\n");
	}
	return ret;
}

static struct platform_driver ds18b20_driver = {
	.driver = {
		.name 	= DEVICE_NAME,
		.owner = THIS_MODULE,
	},
	.probe = ds18b20_driver_probe,
	.remove= ds18b20_driver_remove,
};

static int 	__init ds18b20_driver_init(void){

	return platform_driver_register(&ds18b20_driver);
	
}
static void __exit ds18b20_driver_exit(void){

	platform_driver_unregister(&ds18b20_driver);

}

module_init(ds18b20_driver_init);
module_exit(ds18b20_driver_exit);
MODULE_LICENSE("GPL");
