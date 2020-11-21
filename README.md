# 描述

该项目是基于NXP公司的IMX6ULL开发板，以及AP3216C 这个三合一环境光传感器，实现的I2C设备总线驱动。

开发板上通过 I2C1 连接了一个三合一环境传感器： AP3216C， AP3216C是由敦南科技推出的一款传感器，其支持环境光强度(ALS)、接近距离(PS)和红外线强度(IR)这三个环境参数检测。  



# 模块入口、出口

```c
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
```



# probe

当设备通过设备树或者i2c_device_id提供的ID比较，符合后，调用驱动的probe函数，主要为字符设备驱动的注册，添加驱动的ops，自动创建类与设备。

```c
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
```



# 设备私有数据

```c
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
```



# 字符设备的ops

```c
static const struct file_operations ap3216c_ops = {
	.owner = THIS_MODULE,
	.open = ap3216c_open,
	.read = ap3216c_read,
	.release = ap3216c_release,
};

static int ap3216c_open(struct inode *inode, struct file *filp)
{
    filp->private_data = &ap3216cdev;

    /* hw init */
	ap3216c_write_reg(&ap3216cdev, AP3216C_SYSTEMCONG, 0x04);		/* 复位AP3216C 			*/
	mdelay(50);														/* AP3216C复位最少10ms 	*/
	ap3216c_write_reg(&ap3216cdev, AP3216C_SYSTEMCONG, 0X03);		/* 开启ALS、PS+IR 		*/
	
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
```

调用open函数，对硬件设备进行初始化。

调用read函数，将读到的数据传输到用户态。通过函数调用recv_data_from_dev实现。



# 读写操作

需要构建低层的寄存器读写函数，通过i2c总线平台发送

read:

```c
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
```



write:

```c
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
```



# 总线传输原理

i2c总线通过构造i2c_msg结构体实现。在i2c驱动中，通过函数i2c_transfer，将构造好的msg包，绑定到i2c_adapter适配器，可理解为主机，在i2c总线驱动中，i2c_adapter为传输实现了算法传输结构：i2c_algorithm  *algo ，algo包含master_xfer （适配器传输函数） 以及  smbus_xfer（总线传输函数）

```c
struct i2c_adapter {
    ...
    unsigned int class; /* classes to allow probing for */    
  	const struct i2c_algorithm *algo; /* 总线访问算法 */       
    ...
}
    
struct i2c_algorithm {
	...
	int (*master_xfer)(struct i2c_adapter *adap, struct i2c_msg *msgs, int num);
    int (*smbus_xfer) (struct i2c_adapter *adap, u16 addr, 401 unsigned short flags, char read_write, u8 command, int size, union i2c_smbus_data *data);
    ...
}
```

