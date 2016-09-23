/*
* @Author: emlslxl
* @Date:   2016-09-07 13:59:00
* @Last Modified by:   emlslxl
* @Last Modified time: 2016-09-23 13:54:32
*/

#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <mach/platform.h>
#include <linux/pwm.h>
#include <linux/ctype.h>
#include <linux/limits.h>
#include <linux/pwm.h>
#include <linux/kdev_t.h>
#include <plat/system.h>
#include <plat/sys_config.h>
#include <linux/miscdevice.h>
#include <linux/device.h>

#define GPIO_PWM_PORT 9 //port_I -> 9
#define GPIO_PWM_NUM  3 //PI03

static void *v_gpio_base_addr = NULL;

#define PY_GPIO_REGS_BASE_ADDR 0x01C20800
#define V_GPIO_REG_BASE v_gpio_base_addr

#define __REG(x)                        (*(volatile unsigned int *)(x))

#define V_PIO_REG_CFG(n, i)               ((volatile unsigned int *)(V_GPIO_REG_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x00))
#define V_PIO_REG_DLEVEL(n, i)            ((volatile unsigned int *)(V_GPIO_REG_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x14))
#define V_PIO_REG_PULL(n, i)              ((volatile unsigned int *)(V_GPIO_REG_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x1C))
#define V_PIO_REG_DATA(n)                   ((volatile unsigned int *)(V_GPIO_REG_BASE + ((n)-1)*0x24 + 0x10))

#define V_PIO_REG_CFG_VALUE(n, i)          __REG(V_GPIO_REG_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x00)
#define V_PIO_REG_DLEVEL_VALUE(n, i)       __REG(V_GPIO_REG_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x14)
#define V_PIO_REG_PULL_VALUE(n, i)         __REG(V_GPIO_REG_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x1C)
#define V_PIO_REG_DATA_VALUE(n)            __REG(V_GPIO_REG_BASE + ((n)-1)*0x24 + 0x10)

static void *v_pwm_base_addr = NULL;
#define PY_PWM_REGS_BASE_ADDR  0x01C20C00
#define V_PWM_REG_BASE v_pwm_base_addr

static __u32 pwm_read_reg(__u32 offset)
{
  __u32 value = 0;

  value = readl(v_pwm_base_addr + offset);

  return value;
}

static __s32 pwm_write_reg(__u32 offset, __u32 value)
{
  writel(value, v_pwm_base_addr + offset);

  return 0;
}

static void pwm_set_duty(__u32 duty)
{
  __u32 active_cycle = 0;
  __u32 entire_cycle = 0;
  __u32 tmp;
  
  tmp = pwm_read_reg(0x208);
  entire_cycle = tmp >> 16;

  active_cycle = ((duty > 100 ? 100 : duty) * entire_cycle + (100 / 2)) / 100;

  tmp = pwm_read_reg(0x208);

  pwm_write_reg(0x208, (tmp & 0xffff0000) | active_cycle);
}

static void pwm_set_freq_hz(__u32 freq_hz)
{
  __u32 entire_cycle = 0;
  __u32 tmp;

  entire_cycle = 24000000/(freq_hz > 24000000 ? 24000000 : freq_hz);

  tmp = pwm_read_reg(0x208);

  pwm_write_reg(0x208, (tmp & 0x0000ffff) | entire_cycle << 16);
}

static void pwm1_enable(__u32 b_en)
{
  __u32 tmp = 0;

  tmp = pwm_read_reg(0x200);
  if (b_en)
    tmp |= (1 << 19);
  else
    tmp &= (~(1 << 19));

  pwm_write_reg(0x200, tmp);
}

static int pwm_init(void)
{
  __u32 tmp;

  volatile __u32  *tmp_group_func_addr = NULL;

  __u32 tmp_group_func_data = 0;

  if(!request_mem_region(PY_GPIO_REGS_BASE_ADDR,0x218,"pwm1_gpio"))
  {
    printk("lxl pwm1_gpio request_mem_region failed!\n");
  }

  v_gpio_base_addr = ioremap(PY_GPIO_REGS_BASE_ADDR,0x218);
  if(v_gpio_base_addr == NULL)
    printk("lxl v_gpio_base_addr ioremap failed!\n");

  if(!request_mem_region(PY_PWM_REGS_BASE_ADDR,0x20C,"pwm1"))
  {
    printk("lxl pwm1 request_mem_region failed!\n");
  }

  v_pwm_base_addr = ioremap(PY_PWM_REGS_BASE_ADDR,0x20C);
  if(v_pwm_base_addr == NULL)
    printk("lxl v_pwm_base_addr ioremap failed!\n");

  tmp_group_func_addr    = V_PIO_REG_CFG(GPIO_PWM_PORT, 0); /* 更新功能寄存器地址 */
  tmp_group_func_data = *tmp_group_func_addr;
  tmp_group_func_data |= 2 << 12;
  *tmp_group_func_addr = tmp_group_func_data; //config pwm function

  pwm_write_reg(0x208, ((1200 - 1) << 16) | 900); //freq=20KHz
  tmp = pwm_read_reg(0x208);

  tmp = pwm_read_reg(0x200) & 0xff807fff;
  tmp |= ((1 << 21) | (1 << 20) | (0xf << 15));
  pwm_write_reg(0x200, tmp);

  pwm1_enable(1);
  
  return 0;
}
/***********************************************************/

static int em6057_pwm_open(struct inode *inode, struct file *file) {
  // printk("lxl em6057_pwm_open\n");
  return 0;
}

ssize_t em6057_pwm_write(struct file *file, const char __user *buf, 
        size_t size, loff_t *offset) {
  // printk("lxl em6057_pwm_write\n");
  return 0;
}

static long em6057_pwm_ioctl(struct file *filp,unsigned int cmd,
        unsigned long data)
{
  __u32 tmp = 0;
  // printk("lxl em6057_pwm_ioctl\n");
  pwm_set_duty(cmd);
  tmp = pwm_read_reg(0x208);

  return 0;
}

static int em6057_pwm_release(struct inode *inode, struct file *file) {
  // printk("lxl em6057_pwm_release\n");
  return 0;
}

static const struct file_operations em6057_pwm_fops = {
  .open    = em6057_pwm_open,
  .write    = em6057_pwm_write,
  .unlocked_ioctl = em6057_pwm_ioctl,
  .release  = em6057_pwm_release
};


static struct miscdevice em6057_pwm_dev = {    //创建杂项设备
  .minor =  255,
  .name =    "em6057_pwm",
  .fops =    &em6057_pwm_fops
};

/****************************************************************************/

static struct device_attribute *pattr;

static struct attribute *em6057_pwm_attributes[4] = {
  NULL
};

static struct attribute_group em6057_pwm_attribute_group = {
  .name = "pwm1",
  .attrs = em6057_pwm_attributes,
};

static ssize_t em6057_pwm_duty_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
  unsigned long data;
  int error;

  error = strict_strtoul(buf, 10, &data);
  if (error)
    return error;

  // printk("lxl em6057_pwm_store\n");
  pwm_set_duty(data);
  

  return count;
}

static ssize_t em6057_pwm_duty_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
  unsigned int data,tmp;

  // printk("lxl em6057_pwm_show\n");

  tmp = pwm_read_reg(0x208);
  data = (tmp&0xffff)*100/(tmp >> 16);

  if(data != EGPIO_FAIL) {
    return sprintf(buf, "%d\n", data);
  } else {
    return sprintf(buf, "error\n");
  }
}

static ssize_t em6057_pwm_freq_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
  unsigned long data;
  int error;

  error = strict_strtoul(buf, 10, &data);
  if (error)
    return error;

  // printk("lxl em6057_pwm_freq_store\n");
  pwm_set_freq_hz(data);
  
  return count;
}

static ssize_t em6057_pwm_freq_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
  unsigned int data,tmp;

  // printk("lxl em6057_pwm_freq_show\n");

  tmp = pwm_read_reg(0x208);
  data = 24000000/(tmp >> 16);

  if(data != EGPIO_FAIL) {
    return sprintf(buf, "%d\n", data);
  } else {
    return sprintf(buf, "error\n");
  }
}


static int __init em6057_pwm_init(void)
{
  int err;
  struct device_attribute *attr_i;

  pwm_init();

  err = misc_register(&em6057_pwm_dev);
  if(err) {
    pr_err("%s register em6057_pwm_dev as misc device error\n", __FUNCTION__);
    goto exit;
  }

  pattr = kzalloc(sizeof(struct device_attribute)*2, GFP_KERNEL);
  attr_i = pattr;

  /* Add attributes to the group */
  sysfs_attr_init(&attr_i->attr);
  attr_i->attr.name = "duty";
  attr_i->attr.mode = 0644;
  attr_i->show = em6057_pwm_duty_show;
  attr_i->store = em6057_pwm_duty_store;
  em6057_pwm_attributes[0] = &attr_i->attr;

  attr_i++;

  sysfs_attr_init(&attr_i->attr); 
  attr_i->attr.name = "freq";
  attr_i->attr.mode = 0644;
  attr_i->show = em6057_pwm_freq_show;
  attr_i->store = em6057_pwm_freq_store;
  em6057_pwm_attributes[1] = &attr_i->attr;
  
  sysfs_create_group(&em6057_pwm_dev.this_device->kobj,
             &em6057_pwm_attribute_group);
exit:
  return err;
}

static void em6057_pwm_exit(void)
{
  sysfs_remove_group(&em6057_pwm_dev.this_device->kobj,
             &em6057_pwm_attribute_group);
  kfree(pattr);

  misc_deregister(&em6057_pwm_dev);

  iounmap(v_pwm_base_addr);
  release_mem_region(PY_PWM_REGS_BASE_ADDR,0x20C);

  iounmap(v_gpio_base_addr);
  release_mem_region(PY_GPIO_REGS_BASE_ADDR,0x218);
}

late_initcall(em6057_pwm_init);
module_exit(em6057_pwm_exit);

MODULE_DESCRIPTION("a simple pwm driver");
MODULE_AUTHOR("emlslxl");
MODULE_LICENSE("GPL");
