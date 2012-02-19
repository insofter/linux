#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#include <linux/hd44780drv.h>
#include "icdcommon.h"

struct hd44780dev
{
  struct platform_device *pdev;
  struct cdev cdev;
  spinlock_t spinlock;
};

static int hd44780dev_init(struct hd44780dev *lcd,
  struct platform_device *pdev, dev_t dev_nr);
static void hd44780dev_free(struct hd44780dev *lcd);
static int hd44780dev_cdev_open(struct inode *inode, struct file *filp);
static ssize_t hd44780dev_cdev_write(struct file *filp,
  const char __user *buf, size_t count, loff_t *offp);
static int hd44780dev_cdev_release(struct inode *inode, struct file *filp);
static ssize_t hd44780dev_cmd_store(struct device *dev,
  struct device_attribute *attr, const char *buf, size_t count);
static void __hd44780dev_write_4bit(struct hd44780dev *lcd, int rs, unsigned char d);
static void __hd44780dev_write_char(struct hd44780dev *lcd, unsigned char d);
static void __hd44780dev_write_cmd(struct hd44780dev *lcd, unsigned char d);

static DEVICE_ATTR(cmd, S_IWUSR, NULL, hd44780dev_cmd_store);

static const struct file_operations hd44780dev_cdev_operations = {
  .owner = THIS_MODULE,
  .open = hd44780dev_cdev_open,
  .llseek = no_llseek,
  .write = hd44780dev_cdev_write,
  .release = hd44780dev_cdev_release,
};

struct hd44780drv
{
  dev_t base_dev_number;
  struct platform_driver driver;
};

static int __init hd44780drv_init(void);
static void __exit hd44780drv_exit(void);
static int __devinit hd44780drv_probe(struct platform_device *pdev);
static int __devexit hd44780drv_remove(struct platform_device *pdev);

static struct hd44780drv lcd_driver = {
  .driver = {
    .probe = hd44780drv_probe,
    .remove = __devexit_p(hd44780drv_remove),
    .driver = {
      .name =  "gpio-hd44780",
      .owner = THIS_MODULE,
    }
  }
};

//------------------------------------------------------------------------------

static int hd44780dev_init(struct hd44780dev *lcd, struct platform_device *pdev, dev_t dev_nr)
{
  int err = 0;

  lcd->pdev = pdev;

  cdev_init(&lcd->cdev, &hd44780dev_cdev_operations);
  lcd->cdev.owner = THIS_MODULE;
  err = cdev_add(&lcd->cdev, dev_nr, 1);
  CHECK_ERR(err, fail, "Adding character device for %s failed", pdev->name);

  spin_lock_init(&lcd->spinlock);

  err = device_create_file(&pdev->dev, &dev_attr_cmd);
  CHECK_ERR(err, fail_free_cdev, "Failed to register device attribute 'cmd'");

  // Basic init sequence 
  __hd44780dev_write_4bit(lcd, 0, 0x3);
  mdelay(5); 
  __hd44780dev_write_4bit(lcd, 0, 0x3);
  udelay(200);
  __hd44780dev_write_4bit(lcd, 0, 0x3);
  udelay(50);
  // Set to 4 bit mode
  __hd44780dev_write_4bit(lcd, 0, 0x2);
  udelay(50); 
  // Function set; 2 lines, 5x7 dot format
  __hd44780dev_write_cmd(lcd, 0x28);
  udelay(50);
  // Display off
  __hd44780dev_write_cmd(lcd, 0x08);
  udelay(50);
  // Display clear
  __hd44780dev_write_cmd(lcd, 0x01);
  udelay(1700);
  // Entry mode; increment, display shift off
  __hd44780dev_write_cmd(lcd, 0x06);
  udelay(50);
  return 0; 

fail_free_cdev:
  cdev_del(&lcd->cdev);

fail:
  return err;
}

static void hd44780dev_free(struct hd44780dev *lcd)
{
  device_remove_file(&lcd->pdev->dev, &dev_attr_cmd);

  cdev_del(&lcd->cdev);
}

static int hd44780dev_cdev_open(struct inode *inode, struct file *filp)
{
  struct hd44780dev *lcd = (struct hd44780dev *)
    container_of(inode->i_cdev, struct hd44780dev, cdev);
  filp->private_data = lcd;
  return 0;
}

static ssize_t hd44780dev_cdev_write(struct file *filp, const char __user *buf, size_t count, loff_t *offp)
{
  struct hd44780dev *lcd = (struct hd44780dev *)filp->private_data;
  int i = 0;
  char c = 0;

  spin_lock(&lcd->spinlock);
  for (i = 0; i < count; i++)
  {
    get_user(c, buf + i);
    if (c != '\n')
      __hd44780dev_write_char(lcd, c);
    udelay(100);
  }
  spin_unlock(&lcd->spinlock);
  
  return count;
}

static int hd44780dev_cdev_release(struct inode *inode, struct file *filp)
{
  return 0;
}

static ssize_t hd44780dev_cmd_store(struct device *dev,
  struct device_attribute *attr, const char *buf, size_t count)
{
  int err = 0;
  struct platform_device *pdev = container_of(dev, struct platform_device, dev);
  struct hd44780dev *lcd = platform_get_drvdata(pdev);
  long value = 0;

  spin_lock(&lcd->spinlock);
  err = strict_strtol(buf, 0, &value);
  CHECK_ERR(err, fail, "Invalid cmd value");
  CHECK(value >= 0 && value < 256, err, -EINVAL, fail, "cmd value out of range");
  __hd44780dev_write_cmd(lcd, (unsigned char)value);
  if (value == 0x01 || (value & 0xFE) == 0x02)
    mdelay(5);
  else
    udelay(100);
  spin_unlock(&lcd->spinlock);
  return count;

fail:
  spin_unlock(&lcd->spinlock);
  return err;
}

static void __hd44780dev_write_4bit(struct hd44780dev *lcd, int rs, unsigned char d)
{
  struct hd44780dev_data *device_data = lcd->pdev->dev.platform_data;
  gpio_set_value(device_data->gpio_rs, rs);
  gpio_set_value(device_data->gpio_d4, d & 0x01);
  gpio_set_value(device_data->gpio_d5, (d >> 1) & 0x01);
  gpio_set_value(device_data->gpio_d6, (d >> 2) & 0x01);
  gpio_set_value(device_data->gpio_d7, (d >> 3) & 0x01);
  udelay(2);
  gpio_set_value(device_data->gpio_e, 1);
  udelay(5);
  gpio_set_value(device_data->gpio_e, 0);
}

static void __hd44780dev_write_char(struct hd44780dev *lcd, unsigned char d)
{
  __hd44780dev_write_4bit(lcd, 1, d >> 4);
  __hd44780dev_write_4bit(lcd, 1, d);
}

static void __hd44780dev_write_cmd(struct hd44780dev *lcd, unsigned char d)
{
  __hd44780dev_write_4bit(lcd, 0, d >> 4);
  __hd44780dev_write_4bit(lcd, 0, d);
}

//------------------------------------------------------------------------------

static int __init hd44780drv_init(void)
{
  int err = alloc_chrdev_region(&lcd_driver.base_dev_number, 0, 1, "lcd");
  CHECK_ERR(err, fail, "allocating chrdev region failed");

  err = platform_driver_register(&lcd_driver.driver);
  CHECK_ERR(err, fail_regunreg, "registering platform device failed");

  DBG_TRACE("module initialize succeddfully");
  return 0;

fail_regunreg:
  unregister_chrdev_region(lcd_driver.base_dev_number, 1);

fail:
  return err;
}

static void __exit hd44780drv_exit(void)
{
  platform_driver_unregister(&lcd_driver.driver);

  unregister_chrdev_region(lcd_driver.base_dev_number, 1);

  DBG_TRACE("module exit");
}

static int __devinit hd44780drv_probe(struct platform_device *pdev)
{
  int err = 0;
  dev_t dev_nr = lcd_driver.base_dev_number;
  struct hd44780dev *lcd = NULL;

  lcd = kzalloc(sizeof(struct hd44780dev), GFP_KERNEL);
  CHECK_PTR(lcd, err, fail, "Memory allocation of hd44780dev failed");

  platform_set_drvdata(pdev, lcd);

  err = hd44780dev_init(lcd, pdev, dev_nr);
  CHECK_ERR(err, fail_free, "Initalization of hd44780dev failed");

  DBG_TRACE("Device %s added", pdev->name);
  return 0;

fail_free:
  kfree(lcd);

fail:
  return err;
}

static int __devexit hd44780drv_remove(struct platform_device *pdev)
{
  struct hd44780dev *lcd = platform_get_drvdata(pdev);
  hd44780dev_free(lcd);
  kfree(lcd);

  DBG_TRACE("Device %s removed", pdev->name);

  return 0;
}

module_init(hd44780drv_init);
module_exit(hd44780drv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomasz Rozensztrauch <t.rozensztrauch@gmail.com>");
MODULE_DESCRIPTION("HD44780-compatible lcd device driver");

