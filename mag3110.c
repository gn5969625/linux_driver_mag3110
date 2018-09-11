#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <asm/mach/map.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include "mag3110.h"
#include <linux/sysfs.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/mutex.h>
//#include <linux/mod_devicetable.h>
#define DRV_VERSION "V1.0"
#define FREESCALE_MAG3110_MAX_RANGE 30000

//static struct i2c_client *mag3110_client;
typedef struct mag3110_data {
       struct i2c_client *client;
       struct input_dev *input_dev;
       struct work_struct work;
       unsigned short offset[3]; //get Sensitivity Adjustment Values from fuse rom
       struct mutex lock;
       unsigned char mode;
}mag3110_data;

static void set_mag3110_mode(mag3110_data *data,unsigned char mode) {
     int ret;
     data->mode = mode;
     ret = i2c_smbus_write_byte_data(data->client,MAG3110_CTRL_REG1,data->mode);
}

static int i2c_mag3110_read_len(struct i2c_client *client,unsigned char reg_addr,unsigned char len,unsigned char *buf)
{
        int ret;
        unsigned char txbuf = reg_addr;
        struct i2c_msg msg[] = {
                {client->addr,0,1,&txbuf},
                {client->addr,1,len,buf}
        };
        ret = i2c_transfer(client->adapter,msg,2);
        if(ret < 0) {
                printk("i2c_transfer read len error\n");
                return -1;
        }
        return 0;
}

//static mag3110_data akm_data;
static void get_mag3110_raw_data(mag3110_data *data,unsigned char raw_data[]) {
     int ret;
     do {
       ret = i2c_smbus_read_byte_data(data->client,MAG3110_STATUS);
     }while(!(ret & MAG3110_STATUS_DRDY ));
     i2c_mag3110_read_len(data->client,MAG3110_OUT_X,6,raw_data);
}

static ssize_t mag3110_data_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    unsigned char raw_buf[6];
    mag3110_data *data = dev_get_drvdata(dev);
    get_mag3110_raw_data(data,raw_buf);
    input_report_abs(data->input_dev, ABS_X, (raw_buf[0]<<8) | raw_buf[1]);
    input_report_abs(data->input_dev, ABS_Y, (raw_buf[2]<<8) | raw_buf[3]);
    input_report_abs(data->input_dev, ABS_Z, (raw_buf[4]<<8) | raw_buf[5]);
    input_sync(data->input_dev);
    return sprintf(buf,"x=%x,y=%x,z=%x\n",(raw_buf[0]<<8) | raw_buf[1],(raw_buf[2]<<8) | raw_buf[3], (raw_buf[4]<<8) | raw_buf[5]);
}
static DEVICE_ATTR(mag3110_data, 0644 , mag3110_data_show, NULL);

static ssize_t mag3110_mode_show(struct device *dev,struct device_attribute *attr, char *buf)
{
        mag3110_data *data = dev_get_drvdata(dev);
	return sprintf(buf ,"mode = %x\n",data->mode);
}

static ssize_t mag3110_mode_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
        mag3110_data *data = dev_get_drvdata(dev);
        data->mode =  (unsigned char)simple_strtoul(buf,NULL, 10);
	set_mag3110_mode(data,data->mode);
        return count;
}

static DEVICE_ATTR(mag3110_mode, 0644 , mag3110_mode_show, mag3110_mode_store);
static struct attribute *mag3110_attrs[] = {
    &dev_attr_mag3110_mode.attr,
    &dev_attr_mag3110_data.attr,
    NULL
};
static struct attribute_group mag3110_attr_group = {
    .name = "mag3110",
    .attrs = mag3110_attrs,
};
static void set_mag3110_retset(mag3110_data *data) {
     int ret;
     ret = i2c_smbus_write_byte_data(data->client,MAG3110_CTRL_REG2,0);
}

static void get_mag3110_offset(mag3110_data *data) {
     int ret;
     //enter fuse rom mode
     mdelay(100);
     data->offset[0] = (i2c_smbus_read_word_data(data->client,MAG3110_OFF_X) << 8);
     data->offset[0] = i2c_smbus_read_word_data(data->client,MAG3110_OFF_X+1);
     data->offset[1] = (i2c_smbus_read_word_data(data->client,MAG3110_OFF_Y) << 8);            
     data->offset[1] = i2c_smbus_read_word_data(data->client,MAG3110_OFF_Y+1);
     data->offset[2] = (i2c_smbus_read_word_data(data->client,MAG3110_OFF_Z) << 8);            
     data->offset[2] = i2c_smbus_read_word_data(data->client,MAG3110_OFF_Z+1);
}
static void set_mag3110_input_dev(mag3110_data *data) {
        int ret;
        data->input_dev = input_allocate_device();
        set_bit(EV_ABS, data->input_dev->evbit);
        /* x-axis magnetic */
        input_set_abs_params(data->input_dev, ABS_X, -FREESCALE_MAG3110_MAX_RANGE, FREESCALE_MAG3110_MAX_RANGE, 0, 0);
        /* y-axis magnetic */
        input_set_abs_params(data->input_dev, ABS_Y, -FREESCALE_MAG3110_MAX_RANGE, FREESCALE_MAG3110_MAX_RANGE, 0, 0);
        /* z-axis magnetic */
        input_set_abs_params(data->input_dev, ABS_Z, -FREESCALE_MAG3110_MAX_RANGE, FREESCALE_MAG3110_MAX_RANGE, 0, 0);
        data->input_dev->name = "mag3110";
        data->input_dev->dev.parent = &data->client->dev;
        ret = input_register_device(data->input_dev);
}
static int mag3110_dev_init(mag3110_data *data) {
	unsigned char ret;
	printk("%s called\n", __func__);
 
	ret = i2c_smbus_read_byte_data(data->client, MAG3110_WHO_AM_I);
        printk("%s,WIA ID:%x\n",__func__,ret);

        get_mag3110_offset(data);
        set_mag3110_mode(data,0);

        set_mag3110_input_dev(data);

}

static int mag3110_probe(struct i2c_client *i2c, const struct i2c_device_id *id) {
	int ret;
        mag3110_data *data = NULL;
	dev_dbg(&i2c->dev, "%s\n", __func__);
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL))
                return -ENODEV;
	dev_info(&i2c->dev, "chip found, driver version " DRV_VERSION "\n");

        data = kzalloc(sizeof(mag3110_data),GFP_KERNEL);
	//mag3110_client = i2c;
        data->client = i2c;
        mutex_init(&data->lock);
	mag3110_dev_init(data);
	printk("mag3110 device component found!~\n");
	ret = sysfs_create_group(&i2c->dev.kobj, &mag3110_attr_group);
        i2c_set_clientdata(i2c,data);
	return 0;
}

static int mag3110_remove(struct i2c_client *i2c) {
        mag3110_data *data = i2c_get_clientdata(i2c);
        input_unregister_device(data->input_dev);
        input_free_device(data->input_dev);
        mutex_destroy(&data->lock);
        kfree(data);
	sysfs_remove_group(&i2c->dev.kobj, &mag3110_attr_group);
	return 0;
}
/*
static int mag3110_open(struct inode *inode, struct file *file) {
        return 0;
}

static int mag3110_release(struct inode *inode, struct file *file) {
        return 0;
}

static long mag3110_ioctl( struct file *file, unsigned int cmd,unsigned long arg) {
        return 0;
}

static struct file_operations mag3110_fops = {
        .owner = THIS_MODULE,
        .open = mag3110_open,
        .release = mag3110_release,
        .ioctl = mag3110_ioctl,
};

static struct miscdevice mag3110_device = {
        .minor = MISC_DYNAMIC_MINOR,
        .name = "mag3110_device",
        .fops = &mag3110_fops,
};
*/
static const struct i2c_device_id mag3110_id[] = {  
    { "kevin,mag3110",0},
    {}
};
MODULE_DEVICE_TABLE(i2c, mag3110_id);

static struct of_device_id mag3110_of_match[] = {
        { .compatible = "kevin,mag3110" },
        { },
};
MODULE_DEVICE_TABLE(of, mag3110_of_match);

static struct i2c_driver mag3110_driver = {
    .driver = {
        .name           = "kevin,mag3110",
        .owner          = THIS_MODULE,
        .of_match_table = of_match_ptr(mag3110_of_match),
    },
    .probe      = mag3110_probe,
    .remove     = mag3110_remove,
    .id_table   = mag3110_id,
};
module_i2c_driver(mag3110_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin.Shen");
MODULE_DESCRIPTION("A i2c-mag3110 driver for testing module ");
MODULE_VERSION(DRV_VERSION);
