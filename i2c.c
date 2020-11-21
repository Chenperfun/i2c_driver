#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include "ap3216creg.h"

#define AP3216C_CNT	1
#define AP3216C_NAME	"ap3216c"

struct ap3216c_dev {
    int major;
    dev_t devid;
    struct cdev cdev;
    struct class *class;
    struct device *device;

    struct device_node *nd;
    unsigned short ir, als, ps;
    void *private_data;
};

static struct ap3216c_dev ap3216cdev;

static const struct file_operations ap3216c_ops = {
	.owner = THIS_MODULE,
	.open = ap3216c_open,
	.read = ap3216c_read,
	.release = ap3216c_release,
};

/* write one reg */
static void ap3216c_write_reg(struct ap3216c_dev *dev, u8 reg, u8 data)
{
    u8 *buf;
    buf = &data;
    ap3216c_write_regs(dev, reg, buf, 1);
}

/* write regs */
static s32 ap3216c_write_regs(struct ap3216c_dev *dev, u8 reg, u8 *buf, u8 len)
{
    u8 tmp[256];
    struct i2c_msg msg;
    struct i2c_client *client = (struct i2c_client *)dev->private_data;

    tmp[0] = reg;           
    memcpy(&b[1], buf, len);

    msg.addr = client->addr;    //pid
    msg.flag = 0;               //write
    msg.buf  = tmp;
    msg.len  = len + 1;

    return i2c_transfer(client->adapter, &msg, 1);
}

/* read one reg */
static u8 ap3216c_read_reg(struct ap3216c_dev *dev, u8 reg)	
{
    u8 data = 0;

    ap3216c_read_regs(dev, reg, &data, 1);
    return data;
}

/* read regs */
static s32 ap3216c_read_regs(struct ap3216c_dev *dev, u8 reg,void *val, u8 len)
{
    struct i2c_msg msg[2];
    struct i2c_client *client = (struct i2c_client *)dev->private_data;

    /* twice */
    msg[0].addr = client->addr;     //pid
    msg[0].flag = 0;                //write
    msg[0].buf  = &reg;
    msg[0].len  = 1;

    msg[1].addr = client->addr;     //pid
    msg[1].flag = 1;                //read
    msg[1].buf  = val;
    msg[1].len  = len;

    if(i2c_transfer(client->adapter, msg, 2) != 2)
    {
		printk("i2c rd failed=%d reg=%06x len=%d\n",ret, reg, len);
		ret = -EREMOTEIO;
    }
    return 0;
}

static int ap3216c_open(struct inode *inode, struct file *filp)
{
    filp->private_data = &ap3216cdev;

    /* hw init */
	ap3216c_write_reg(&ap3216cdev, AP3216C_SYSTEMCONG, 0x04);		/* 复位AP3216C 			*/
	mdelay(50);														/* AP3216C复位最少10ms 	*/
	ap3216c_write_reg(&ap3216cdev, AP3216C_SYSTEMCONG, 0X03);		/* 开启ALS、PS+IR 		*/
	
    return 0;
}

static int recv_data_from_dev(struct ap3216c_dev *dev)
{
	unsigned char i =0;
    unsigned char buf[6];

    for(i = 0; i < 6; i++)	
    {
        buf[i] = ap3216c_read_reg(dev, AP3216C_IRDATALOW + i);	
    }

    if(buf[0] & 0X80) 	/* IR_OF位为1,则数据无效 */
		dev->ir = 0;					
	else 				/* 读取IR传感器的数据   		*/
		dev->ir = ((unsigned short)buf[1] << 2) | (buf[0] & 0X03); 			
	
	dev->als = ((unsigned short)buf[3] << 8) | buf[2];	/* 读取ALS传感器的数据 */  
	
    if(buf[4] & 0x40)	/* IR_OF位为1,则数据无效  */
		dev->ps = 0;    													
	else 				/* 读取PS传感器的数据    */
		dev->ps = ((unsigned short)(buf[5] & 0X3F) << 4) | (buf[4] & 0X0F); 
    
    return 0;
}

static int ap3216c_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
    int iError;
    short data[3];
    struct ap3216c_dev *dev = (struct ap3216c_dev *)filp->private_data;

    iError = recv_data_from_dev(dev);
    if(iError){
		printk("recv data from dev error.\n");
        return -1;
    }

	data[0] = dev->ir;
	data[1] = dev->als;
	data[2] = dev->ps;

    iError = copy_to_user(buf, data, sizeof(data));
    if(iError)
		return -EFAULT;

    return 0;
}

static int ap3216c_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int ap3216c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    if(ap3216cdev.major){
        ap3216cdev.devid = MKDEV(ap3216cdev.major, 0);
        register_chrdev_region(ap3216cdev.devid, AP3216C_CNT, AP3216C_NAME);
    }
    else    //no major
    {
        alloc_chrdev_region(&ap3216cdev.devid, 0, AP3216C_CNT, AP3216C_NAME);
        ap3216cdev.major = MAJOR(ap3216cdev.devid);
    }

    cdev_init(&ap3216cdev.cdev, &ap3216c_ops);
    cdev_add(&ap3216cdev.cdev, ap3216cdev.devid, AP3216C_CNT);

    ap3216cdev.class  = class_create(THIS_MODULE, AP3216C_NAME);
	if (IS_ERR(ap3216cdev.class)) {
		return PTR_ERR(ap3216cdev.class);
	}

    ap3216cdev.device = device_create(ap3216cdev.class, NULL, ap3216cdev.devid, NULL, AP3216C_NAME); 
	if (IS_ERR(ap3216cdev.device)) {
		return PTR_ERR(ap3216cdev.device);
	}
    ap3216cdev.private_data = client;

    return 0;
}

static int ap3216c_remove(struct i2c_client *client)
{
	cdev_del(&ap3216cdev.cdev);
	unregister_chrdev_region(ap3216cdev.devid, AP3216C_CNT);

    device_destroy(ap3216cdev.class, ap3216cdev.devid);
    class_destroy(ap3216cdev.class);
    
    return 0;
}

static const struct i2c_device_id ap3216c_id[] = {
    {"imx6ull,ap3216c", 0},{}
};

static const struct of_match_id ap3216c_of_match[] = {
    { .compatible = "imx6ull,ap3216c" },{}
};

static struct i2c_driver ap3216c_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name  = "ap3216c",
        .of_match_table = ap3216c_of_match, 
    },
    .probe  = ap3216c_probe,
    .remove = ap3216c_remove,
    .id_table = ap3216c_id,
};

static int __init ap3216c_init(void)
{
    return i2c_add_driver(&ap3216c_driver);
}

static void __exit ap3216c_exit(void)
{
    i2c_del_driver(&ap3216c_driver);
}


//module_i2c_driver(ap3216c_driver);

module_init(ap3216c_init);
module_exit(ap3216c_exit);

MODULE_LICESE("GPL");