#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/sched.h>

#include <linux/itddrv.h>
#include "icdcommon.h"

#define ITDDEV_CLEAR (0)
#define ITDDEV_BLOCKED (1)
#define ITDDEV_UNINITIALIZED (-1)

#define ITDDEV_IOBUF_SIZE (1024)
#define ITDDEV_EVENT_QUEUE_LEN (256)

struct itddev_event
{
  unsigned long sec;
  unsigned long usec;
  int state;
};

struct itddev_queue
{
  struct itddev_event* buffer;
  unsigned int len;
  unsigned int reading_head;
  unsigned int writting_head;
  wait_queue_head_t ready;
};

static inline void __itddev_queue_advance_reading_head(struct itddev_queue* queue)
{
  if (queue->reading_head + 1 < queue->len)
    queue->reading_head += 1;
  else
    queue->reading_head = 0;
}

static inline void __itddev_queue_advance_writting_head(struct itddev_queue* queue)
{
  if (queue->writting_head + 1 < queue->len)
    queue->writting_head += 1;
  else
    queue->writting_head = 0;
}

struct itddev 
{
  struct platform_device *pdev;
  struct cdev cdev;
  char *iobuf;
  char *iobuf_ptr;
 
  struct timer_list timer;
  spinlock_t spinlock;

  int last_irq_value;
  struct timeval last_irq_timeval;
  struct itddev_queue queue;
  int state;
  int timer_running;
  long engage_delay_usec;
  long release_delay_usec;
  int active_low;
  int led_ctrl;
};


static inline void  __itddev_set_led_on(struct itddev *itd)
{
  struct itddev_data *device_data = itd->pdev->dev.platform_data;
  gpio_set_value(device_data->gpio_led, 0);
}

static inline void  __itddev_set_led_off(struct itddev *itd)
{
  struct itddev_data *device_data = itd->pdev->dev.platform_data;
  gpio_set_value(device_data->gpio_led, 1);
}

static void __itddev_handle_irq(struct itddev *itd);

#define ITDDRV_NUM_OF_DEVS (4)

struct itddrv
{
  dev_t base_dev_number;
  int gpio_test;
  long test_time_usec;
  struct itddev *itd[ITDDRV_NUM_OF_DEVS];
  spinlock_t spinlock;
  struct platform_driver driver;
};

static void __itddrv_test_delay(struct itddrv *itd_driver);

static inline void __itddrv_set_test_on(struct itddrv *itd_driver)
{
  gpio_set_value(itd_driver->gpio_test, 0);
}

static inline void __itddrv_set_test_off(struct itddrv *itd_driver)
{
  gpio_set_value(itd_driver->gpio_test, 1);
}

//------------------------------------------------------------------------------

static int __itddev_read_value(struct itddev *itd)
{
  struct itddev_data *device_data = itd->pdev->dev.platform_data;
  int value = gpio_get_value(device_data->gpio_in);
  return ((!value && itd->active_low) || (value && !itd->active_low)) ?
    ITDDEV_BLOCKED : ITDDEV_CLEAR;
}

static ssize_t itddev_engage_delay_usec_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct platform_device *pdev = container_of(dev, struct platform_device, dev);
  struct itddev *itd = platform_get_drvdata(pdev);
  long engage_delay_usec;
  unsigned long flags;

  spin_lock_irqsave(&itd->spinlock, flags);
  engage_delay_usec = itd->engage_delay_usec;
  spin_unlock_irqrestore(&itd->spinlock, flags);
  return sprintf(buf, "%li\n", engage_delay_usec);
}

static ssize_t itddev_engage_delay_usec_store(struct device *dev,
  struct device_attribute *attr, const char *buf, size_t count)
{
  int err = 0;
  struct platform_device *pdev = container_of(dev, struct platform_device, dev);
  struct itddev *itd = platform_get_drvdata(pdev);
  long value = 0;
  unsigned long flags;

  err = strict_strtol(buf, 0, &value);
  CHECK_ERR(err, fail, "Invalid value for engage_delay_usec");
  spin_lock_irqsave(&itd->spinlock, flags);
  itd->engage_delay_usec = value;
  spin_unlock_irqrestore(&itd->spinlock, flags);
  return count;

fail:
  return err;
}

static ssize_t itddev_release_delay_usec_show(struct device *dev,
  struct device_attribute *attr, char *buf)
{
  struct platform_device *pdev = container_of(dev, struct platform_device, dev);
  struct itddev *itd = platform_get_drvdata(pdev);
  long release_delay_usec;
  unsigned long flags;

  spin_lock_irqsave(&itd->spinlock, flags);
  release_delay_usec = itd->release_delay_usec;
  spin_unlock_irqrestore(&itd->spinlock, flags);
  return sprintf(buf, "%li\n", release_delay_usec);
}

static ssize_t itddev_release_delay_usec_store(struct device *dev,
  struct device_attribute *attr, const char *buf, size_t count)
{
  int err = 0;
  struct platform_device *pdev = container_of(dev, struct platform_device, dev);
  struct itddev *itd = platform_get_drvdata(pdev);
  long value = 0;
  unsigned long flags;

  err = strict_strtol(buf, 0, &value);
  CHECK_ERR(err, fail, "Invalid value for release_delay_usec");
  spin_lock_irqsave(&itd->spinlock, flags);
  itd->release_delay_usec = value;
  spin_unlock_irqrestore(&itd->spinlock, flags);
  return count;

fail:
  return err;
}

static ssize_t itddev_active_low_show(struct device *dev,
  struct device_attribute *attr, char *buf)
{
  struct platform_device *pdev = container_of(dev, struct platform_device, dev);
  struct itddev *itd = platform_get_drvdata(pdev);
  int active_low;
  unsigned long flags;

  spin_lock_irqsave(&itd->spinlock, flags);
  active_low = itd->active_low;
  spin_unlock_irqrestore(&itd->spinlock, flags);
  return sprintf(buf, "%i\n", active_low);
}

static ssize_t itddev_active_low_store(struct device *dev,
  struct device_attribute *attr, const char *buf, size_t count)
{
  int err = 0;
  struct platform_device *pdev = container_of(dev, struct platform_device, dev);
  struct itddev *itd = platform_get_drvdata(pdev);
  long value = 0;
  unsigned long flags;

  err = strict_strtol(buf, 0, &value);
  CHECK_ERR(err, fail, "Invalid value for active_low");
  CHECK(value == 0 || value == 1, err, -EINVAL, fail,"Invalid value for active_low"); 
  spin_lock_irqsave(&itd->spinlock, flags);
  itd->active_low = value ? 1 : 0;
  // After changinf active_low attribute reset the device int uninitialized state
  itd->state = ITDDEV_UNINITIALIZED;
  itd->queue.reading_head = 0;
  itd->queue.writting_head = 0;
  spin_unlock_irqrestore(&itd->spinlock, flags);
  return count;

fail:
  return err;
}

static ssize_t itddev_queue_len_show(struct device *dev,
  struct device_attribute *attr, char *buf)
{
  struct platform_device *pdev = container_of(dev, struct platform_device, dev);
  struct itddev *itd = platform_get_drvdata(pdev);
  unsigned int queue_len;
  unsigned long flags;

  spin_lock_irqsave(&itd->spinlock, flags);
  queue_len = itd->queue.len;
  spin_unlock_irqrestore(&itd->spinlock, flags);
  return sprintf(buf, "%u\n", queue_len);
}

static ssize_t itddev_queue_len_store(struct device *dev,
  struct device_attribute *attr, const char *buf, size_t count)
{
  int err = 0;
  struct platform_device *pdev = container_of(dev, struct platform_device, dev);
  struct itddev *itd = platform_get_drvdata(pdev);
  unsigned long value = 0;
  unsigned long flags;
  struct itddev_event *buffer, *old_buffer;
  int queue_len_bytes;

  err = strict_strtoul(buf, 0, &value);
  CHECK_ERR(err, fail, "Invalid value for queue_len");

  // Alloc new buffer
  queue_len_bytes = sizeof(struct itddev_event) * value;
  buffer = kmalloc(queue_len_bytes, GFP_KERNEL);
  CHECK_PTR(buffer, err, fail, "Memory allocation of queue.buffer failed");

  // Exchange old buffer with the newly allocated one
  spin_lock_irqsave(&itd->spinlock, flags);
  old_buffer = itd->queue.buffer;
  itd->queue.buffer = buffer;
  itd->queue.len = value;
  itd->queue.reading_head = 0;
  itd->queue.writting_head = 0;
  spin_unlock_irqrestore(&itd->spinlock, flags);

  // Get rid of the old one
  kfree(old_buffer);

  return count;

fail:
  return err;
}

static ssize_t itddev_led_ctrl_show(struct device *dev,
  struct device_attribute *attr, char *buf)
{
  struct platform_device *pdev = container_of(dev, struct platform_device, dev);
  struct itddev *itd = platform_get_drvdata(pdev);
  int led_ctrl;
  unsigned long flags;

  spin_lock_irqsave(&itd->spinlock, flags);
  led_ctrl = itd->led_ctrl;
  spin_unlock_irqrestore(&itd->spinlock, flags);

  return sprintf(buf, "%c\n", led_ctrl ? 'A' : 'M');
}

static ssize_t itddev_led_ctrl_store(struct device *dev,
  struct device_attribute *attr, const char *buf, size_t count)
{
  int err = 0;
  struct platform_device *pdev = container_of(dev, struct platform_device, dev);
  struct itddev *itd = platform_get_drvdata(pdev);
  int value = -1;
  char str[20];
  int res;
  unsigned long flags;

  res = sscanf(buf,"%10s", str);
  CHECK(res == 1, err, -EINVAL, fail, "Invalid value for led_ctrl");
  
  if (strcmp(str, "A") == 0)
    value = 2;
  else if (strcmp(str, "0") == 0)
    value = 0;
  else if (strcmp(str, "1") == 0)
    value = 1;

  CHECK(value >= 0, err, -EINVAL, fail, "Invalid value for led_ctrl");

  spin_lock_irqsave(&itd->spinlock, flags);
  if (value == 2)
  {
    itd->led_ctrl = 1;
    if (itd->state == ITDDEV_CLEAR)
      __itddev_set_led_off(itd);
    else
      __itddev_set_led_on(itd);
  }
  else
  {
    itd->led_ctrl = 0;
    if (value == 1) 
      __itddev_set_led_on(itd);
    else
      __itddev_set_led_off(itd);
  }
  spin_unlock_irqrestore(&itd->spinlock, flags);

  return count;

fail:
  return err;
}

static ssize_t itddev_restart_store(struct device *dev,
  struct device_attribute *attr, const char *buf, size_t count)
{
  int err = 0;
  struct platform_device *pdev = container_of(dev, struct platform_device, dev);
  struct itddev *itd = platform_get_drvdata(pdev);
  long value = 0;
  unsigned long flags;

  err = strict_strtol(buf, 0, &value);
  CHECK_ERR(err, fail, "Invalid value for restart");
  CHECK(value == 1, err, -EINVAL, fail,"Invalid value for restart"); 
  spin_lock_irqsave(&itd->spinlock, flags);
  itd->state = ITDDEV_UNINITIALIZED;
  itd->queue.reading_head = 0;
  itd->queue.writting_head = 0;
  __itddev_handle_irq(itd);
  spin_unlock_irqrestore(&itd->spinlock, flags);
  return count;

fail:
  return err;
}

static ssize_t itddev_test_show(struct device *dev,
  struct device_attribute *attr, char *buf)
{
  int err = 0;
  struct platform_device *pdev = container_of(dev, struct platform_device, dev);
  struct platform_driver *driver = container_of(pdev->dev.driver,
    struct platform_driver, driver);
  struct itddrv *itd_driver = container_of(driver,
    struct itddrv, driver);
  struct itddev *itd = platform_get_drvdata(pdev);
  long value = 0;
  int test = 0;
  unsigned long flags;

  CHECK(itd_driver->gpio_test > 0, err, -EINVAL, fail, "Test gpio pin not set");

  spin_lock_bh(&itd_driver->spinlock);

  spin_lock_irqsave(&itd->spinlock, flags);
  value = __itddev_read_value(itd);
  spin_unlock_irqrestore(&itd->spinlock, flags);
  if (value == ITDDEV_BLOCKED)
    test += 1;

  __itddrv_set_test_on(itd_driver);
  __itddrv_test_delay(itd_driver);

  spin_lock_irqsave(&itd->spinlock, flags);
  value = __itddev_read_value(itd);
  spin_unlock_irqrestore(&itd->spinlock, flags);
  if (value == ITDDEV_CLEAR)
    test += 2;

  __itddrv_set_test_off(itd_driver);

  spin_unlock_bh(&itd_driver->spinlock);

  return sprintf(buf, "%i\n", test);

fail:
  return err;
}

static DEVICE_ATTR(engage_delay_usec, S_IRUGO | S_IWUSR,
  itddev_engage_delay_usec_show, itddev_engage_delay_usec_store);
static DEVICE_ATTR(release_delay_usec, S_IRUGO | S_IWUSR,
  itddev_release_delay_usec_show, itddev_release_delay_usec_store);
static DEVICE_ATTR(active_low, S_IRUGO | S_IWUSR,
  itddev_active_low_show, itddev_active_low_store);
static DEVICE_ATTR(queue_len, S_IRUGO | S_IWUSR,
  itddev_queue_len_show, itddev_queue_len_store);
static DEVICE_ATTR(led_ctrl, S_IRUGO | S_IWUSR,
  itddev_led_ctrl_show, itddev_led_ctrl_store);
static DEVICE_ATTR(restart, S_IRUGO | S_IWUSR, NULL, itddev_restart_store);
static DEVICE_ATTR(test, S_IRUGO, itddev_test_show, NULL);

static int __itddev_create_dev_attr(struct itddev *itd)
{
  int err = 0;
  int attr_engage_delay_usec_created = 0;
  int attr_release_delay_usec_created = 0;
  int attr_active_low_created = 0;
  int attr_queue_len_created = 0;
  int attr_led_ctrl_created = 0;
  int attr_restart_created = 0;
  int attr_test_created = 0;

  err = device_create_file(&itd->pdev->dev, &dev_attr_engage_delay_usec);
  CHECK_ERR(err, fail, "Failed to register device attribute 'engage_delay_usec'");
  attr_engage_delay_usec_created = 1;

  err = device_create_file(&itd->pdev->dev, &dev_attr_release_delay_usec);
  CHECK_ERR(err, fail, "Failed to register device attribute 'release_delay_usec'");
  attr_release_delay_usec_created = 1;

  err = device_create_file(&itd->pdev->dev, &dev_attr_active_low);
  CHECK_ERR(err, fail, "Failed to register device attribute 'active_low'");
  attr_active_low_created = 1;

  err = device_create_file(&itd->pdev->dev, &dev_attr_queue_len);
  CHECK_ERR(err, fail, "Failed to register device attribute 'queue_len'");
  attr_queue_len_created = 1;

  err = device_create_file(&itd->pdev->dev, &dev_attr_led_ctrl);
  CHECK_ERR(err, fail, "Failed to register device attribute 'led_ctrl'");
  attr_led_ctrl_created = 1;

  err = device_create_file(&itd->pdev->dev, &dev_attr_restart);
  CHECK_ERR(err, fail, "Failed to register device attribute 'restart'");
  attr_restart_created = 1;

  err = device_create_file(&itd->pdev->dev, &dev_attr_test);
  CHECK_ERR(err, fail, "Failed to register device attribute 'test'");
  attr_test_created = 1;

  return 0;

fail:
  if (attr_test_created)
    device_remove_file(&itd->pdev->dev, &dev_attr_test);
  if (attr_restart_created)
    device_remove_file(&itd->pdev->dev, &dev_attr_restart);
  if (attr_led_ctrl_created)
    device_remove_file(&itd->pdev->dev, &dev_attr_led_ctrl);
  if (attr_queue_len_created)
    device_remove_file(&itd->pdev->dev, &dev_attr_queue_len);
  if (attr_active_low_created)
    device_remove_file(&itd->pdev->dev, &dev_attr_active_low);
  if (attr_release_delay_usec_created)
    device_remove_file(&itd->pdev->dev, &dev_attr_release_delay_usec);
  if (attr_engage_delay_usec_created)
    device_remove_file(&itd->pdev->dev, &dev_attr_engage_delay_usec);
  return err;
}

static void __itddev_remove_dev_attr(struct itddev *itd)
{
  device_remove_file(&itd->pdev->dev, &dev_attr_test);
  device_remove_file(&itd->pdev->dev, &dev_attr_restart);
  device_remove_file(&itd->pdev->dev, &dev_attr_led_ctrl);
  device_remove_file(&itd->pdev->dev, &dev_attr_queue_len);
  device_remove_file(&itd->pdev->dev, &dev_attr_active_low);
  device_remove_file(&itd->pdev->dev, &dev_attr_release_delay_usec);
  device_remove_file(&itd->pdev->dev, &dev_attr_engage_delay_usec);
}

static void __itddev_process_event(struct itddev *itd)
{
  struct itddev_event *ev;

  itd->state = itd->last_irq_value;

  // Queue itd event
  ev = &itd->queue.buffer[itd->queue.writting_head];
  ev->sec = itd->last_irq_timeval.tv_sec;
  ev->usec = itd->last_irq_timeval.tv_usec;
  ev->state = itd->last_irq_value;

  __itddev_queue_advance_writting_head(&itd->queue);

  // If the queu was full then we have just overwritten
  // the oldest event and we must also advance reading_head
  if (itd->queue.reading_head == itd->queue.writting_head)
    __itddev_queue_advance_reading_head(&itd->queue);

  // Update led (if the driver has been made responsible for led controll)
  if (itd->led_ctrl)
  {
    if( itd->state == ITDDEV_CLEAR)
        __itddev_set_led_off(itd);
    else
        __itddev_set_led_on(itd);
  }

  // Wake up cdev read, if blocked
  wake_up_interruptible(&itd->queue.ready);
}

static void itddev_timer_fn(unsigned long data)
{
  struct itddev *itd = (struct itddev *)data;
  unsigned long flags;

  spin_lock_irqsave(&itd->spinlock, flags);

  // This is a legitimate case - it may happen that the timer has been cancelled
  // but it happened "too late" so this handler was called, but should do nothing
  if (!itd->timer_running)
    goto unlock;

  __itddev_process_event(itd);

  itd->timer_running = 0;

unlock:
  spin_unlock_irqrestore(&itd->spinlock, flags);
}

static void __itddev_handle_irq(struct itddev *itd)
{
  itd->last_irq_value = __itddev_read_value(itd);

  if ((itd->state == ITDDEV_CLEAR && itd->last_irq_value == ITDDEV_BLOCKED)
    || (itd->state == ITDDEV_BLOCKED && itd->last_irq_value == ITDDEV_CLEAR)
    || itd->state == ITDDEV_UNINITIALIZED)
  {
    if (!itd->timer_running)
    {
      do_gettimeofday(&itd->last_irq_timeval);

      if (itd->last_irq_value == ITDDEV_BLOCKED && itd->engage_delay_usec > 0)
      {
        mod_timer(&itd->timer, jiffies + usecs_to_jiffies(itd->engage_delay_usec));
        itd->timer_running = 1;
      }
      else if (itd->last_irq_value == ITDDEV_CLEAR && itd->release_delay_usec > 0)
      {
        mod_timer(&itd->timer, jiffies + usecs_to_jiffies(itd->release_delay_usec));
        itd->timer_running = 1;
      }
      else
        __itddev_process_event(itd);
    }
  }
  else
  {
    if (itd->timer_running)
    {
      itd->timer_running = 0;
      del_timer(&itd->timer);
    }
  }
}

static irqreturn_t itddev_irq_handler_fn(int irq, void *dev)
{
  struct itddev *itd = (struct itddev *)dev;
  unsigned long flags;

  spin_lock_irqsave(&itd->spinlock, flags);
  __itddev_handle_irq(itd);
  spin_unlock_irqrestore(&itd->spinlock, flags);
  return IRQ_HANDLED;
}

static int itddev_cdev_open(struct inode *inode, struct file *filp)
{
  struct itddev *itd = (struct itddev *)
    container_of(inode->i_cdev, struct itddev, cdev);
  filp->private_data = itd;
  return 0;
}

static int itddev_queue_empty(struct itddev *itd)
{
  unsigned long flags;
  int queue_empty;
  spin_lock_irqsave(&itd->spinlock, flags);
  if (itd->queue.reading_head != itd->queue.writting_head)
    queue_empty = 0;
  else
    queue_empty = 1;
  spin_unlock_irqrestore(&itd->spinlock, flags);
  return queue_empty;
}

static int itddev_cdev_read(struct file *filp, __user char *buf, size_t count, loff_t *offp)
{
  int err = 0;
  struct itddev *itd = (struct itddev *)filp->private_data;
  struct itddev_event* ev;
  unsigned long sec = 0;
  unsigned long usec = 0;
  int state = 0;
  int queue_empty = 1;
  size_t copied = 0;
  size_t len = 0;
  unsigned long res = 0;
  unsigned long flags;

  // In case some data has left in the buffer from previous read operation
  if (itd->iobuf_ptr != 0 && itd->iobuf_ptr[0] != 0)
  {
    len = min(count, strlen(itd->iobuf_ptr));
    res = copy_to_user(buf, itd->iobuf_ptr, len);
    CHECK(res == 0, err, -EINVAL, fail, "Copying data to userspace buffer failed");
    itd->iobuf_ptr += len;
    copied += len;
  }

  while(copied < count)
  {
    while (1)
    {
      spin_lock_irqsave(&itd->spinlock, flags);
      if (itd->queue.reading_head != itd->queue.writting_head)
      {
        ev = &itd->queue.buffer[itd->queue.reading_head];
        sec = ev->sec;
        usec = ev->usec;
        state = ev->state;
        __itddev_queue_advance_reading_head(&itd->queue);
        queue_empty = 0;
      }
      else
        queue_empty = 1;
      spin_unlock_irqrestore(&itd->spinlock, flags);

      if (queue_empty)
        break;

      snprintf(itd->iobuf, ITDDEV_IOBUF_SIZE, "%lu %lu %i\n", sec, usec, state);
      len = min(count - copied, strlen(itd->iobuf));
      res = copy_to_user(buf + copied, itd->iobuf, len);
      CHECK(res == 0, err, -EINVAL, fail, "Copying data to userspace buffer failed");
      itd->iobuf_ptr = itd->iobuf + len;
      copied += len;

      if (copied == count)
        break;
    }

    if (copied > 0)
      break;

    err = wait_event_interruptible(itd->queue.ready, !itddev_queue_empty(itd));
    CHECK(err != -ERESTARTSYS, err, -EINTR, fail, "Wait for queue interrupted");
    CHECK_ERR(err, fail, "Wait for queue ready failed");
  }
  return copied;

fail:
  return err;
}

static int itddev_cdev_release(struct inode *inode, struct file *filp)
{
  return 0;
}

const struct file_operations itddev_cdev_operations = {
  .owner = THIS_MODULE,
  .open = itddev_cdev_open,
  .llseek = no_llseek,
  .read = itddev_cdev_read,
  .release = itddev_cdev_release,
};


static int itddev_init(struct itddev *itd, struct platform_device *pdev, dev_t dev_nr)
{
  int err = 0;
  struct itddev_data *device_data = pdev->dev.platform_data;
  unsigned long flags;
  int queue_len_bytes = sizeof(struct itddev_event) * ITDDEV_EVENT_QUEUE_LEN; 
  itd->queue.buffer = NULL;
  itd->iobuf = NULL;

  itd->pdev = pdev;

  itd->queue.buffer = kmalloc(queue_len_bytes, GFP_KERNEL);
  CHECK_PTR(itd->queue.buffer, err, fail, "Memory allocation of queue.buffer failed");
  itd->queue.len = ITDDEV_EVENT_QUEUE_LEN;
  itd->queue.reading_head = 0;
  itd->queue.writting_head = 0;
  init_waitqueue_head(&itd->queue.ready);

  itd->iobuf = kmalloc(ITDDEV_IOBUF_SIZE, GFP_KERNEL);
  CHECK_PTR(itd->iobuf, err, fail, "Memory allocation of iobuf failed");
  itd->iobuf[0] = 0;
  itd->iobuf_ptr = itd->iobuf;

  spin_lock_init(&itd->spinlock);

  itd->last_irq_value = 0;
  itd->last_irq_timeval.tv_sec = 0;
  itd->last_irq_timeval.tv_usec = 0;
  
  itd->state = ITDDEV_UNINITIALIZED;
  itd->timer_running = 0;

  itd->engage_delay_usec = 0;
  itd->release_delay_usec = 0;
  itd->active_low = 0;
  itd->led_ctrl = 0;

  init_timer(&itd->timer);
  itd->timer.data = (unsigned long)itd;
  itd->timer.function = itddev_timer_fn;

  cdev_init(&itd->cdev, &itddev_cdev_operations);
  itd->cdev.owner = THIS_MODULE;

  spin_lock_irqsave(&itd->spinlock, flags);

  err = __itddev_create_dev_attr(itd);
  CHECK_ERR(err, fail_unlock, "Failed to register device attributes");

  err = cdev_add(&itd->cdev, dev_nr, 1);
  CHECK_ERR(err, fail_free_dev_attr, "Adding character device for %s.%i failed",
    pdev->name, pdev->id);

  err = request_irq(gpio_to_irq(device_data->gpio_in), itddev_irq_handler_fn,
    IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "itd-irq", (void*)itd);
  CHECK_ERR(err, fail_free_cdev, "Requesting irq for %s.%i failed",
    pdev->name, pdev->id);

  __itddev_set_led_on(itd);

  spin_unlock_irqrestore(&itd->spinlock, flags);
  return 0;

fail_free_cdev:
  cdev_del(&itd->cdev);
fail_free_dev_attr:
  __itddev_remove_dev_attr(itd);
fail_unlock:
  spin_unlock_irqrestore(&itd->spinlock, flags);
fail:
  if (itd->iobuf != NULL)
    kfree(itd->iobuf);
  if (itd->queue.buffer != NULL)
    kfree(itd->queue.buffer);
  return err;
}

static void itddev_free(struct itddev *itd)
{
  struct itddev_data *device_data = itd->pdev->dev.platform_data;

  free_irq(gpio_to_irq(device_data->gpio_in), (void*)itd);
  cdev_del(&itd->cdev);
  __itddev_remove_dev_attr(itd);
  kfree(itd->iobuf);
  kfree(itd->queue.buffer);
}

//------------------------------------------------------------------------------

static void __itddrv_test_delay(struct itddrv *itd_driver)
{
  long n = itd_driver->test_time_usec;  
  if (n <= MAX_UDELAY_MS * 1000)
  {
    udelay(n);
  }
  else
  {
    mdelay(n / 1000);
    udelay(n % 1000);
  }
}

static int __devinit itddrv_probe(struct platform_device *pdev)
{
  int err = 0;
  struct itddev_data *device_data = pdev->dev.platform_data;
  struct platform_driver *driver = container_of(pdev->dev.driver,
    struct platform_driver, driver);
  struct itddrv *itd_driver = container_of(driver,
    struct itddrv, driver);
  dev_t dev_nr = itd_driver->base_dev_number + pdev->id;
  struct itddev *itd = NULL;
  
  CHECK(pdev->id >= 0 && pdev->id < ITDDRV_NUM_OF_DEVS, err, -EINVAL, fail,
    "Platform device id out of supported range");

  itd = kzalloc(sizeof(struct itddev), GFP_KERNEL);
  CHECK_PTR(itd, err, fail, "Memory allocation of itddev failed");

  spin_lock_bh(&itd_driver->spinlock);

  if (itd_driver->gpio_test == 0)
  {
    itd_driver->gpio_test = device_data->gpio_test;
    __itddrv_set_test_off(itd_driver);
  }

  platform_set_drvdata(pdev, itd);
  itd_driver->itd[pdev->id] = itd;
  
  err = itddev_init(itd, pdev, dev_nr);
  CHECK_ERR(err, fail_unlock_free, "Initalization of itddev failed");

  spin_unlock_bh(&itd_driver->spinlock);

  DBG_TRACE("Device %s.%d added", pdev->name, pdev->id);
  return 0;

fail_unlock_free:
  spin_unlock_bh(&itd_driver->spinlock);
  kfree(itd);

fail:
  return err;
}

static int __devexit itddrv_remove(struct platform_device *pdev)
{
  struct itddev *itd = platform_get_drvdata(pdev);
  struct platform_driver *driver = container_of(itd->pdev->dev.driver,
    struct platform_driver, driver);
  struct itddrv *itd_driver = container_of(driver, struct itddrv, driver);

  spin_lock_bh(&itd_driver->spinlock);
  itd_driver->itd[pdev->id] = NULL;
  itddev_free(itd);
  spin_unlock_bh(&itd_driver->spinlock);

  kfree(itd);

  DBG_TRACE("Device %s.%d removed", pdev->name, pdev->id);
  return 0;
}

static struct itddrv itd_driver = {
  .gpio_test = 0,
  .test_time_usec = 1000000,
  .spinlock = __SPIN_LOCK_UNLOCKED(itd_driver.spinlock),
  .driver = {
    .probe = itddrv_probe,
    .remove = __devexit_p(itddrv_remove),
    .driver = { 
      .name = "gpio-itd",
      .owner = THIS_MODULE,
    }
  }
};

static ssize_t itddrv_test_time_usec_show(struct device_driver *driver, char *buf)
{
  struct platform_driver *pdrv = container_of(driver, struct platform_driver, driver);
  struct itddrv *itd_driver = container_of(pdrv, struct itddrv, driver);
  long test_time_usec;
  spin_lock_bh(&itd_driver->spinlock);
  test_time_usec = itd_driver->test_time_usec;
  spin_unlock_bh(&itd_driver->spinlock);
  return sprintf(buf, "%li\n", test_time_usec);
}

static ssize_t itddrv_test_time_usec_store(struct device_driver *driver,
  const char *buf, size_t count)
{
  int err = 0;
  struct platform_driver *pdrv = container_of(driver, struct platform_driver, driver);
  struct itddrv *itd_driver = container_of(pdrv, struct itddrv, driver);
  long value = 0;

  err = strict_strtol(buf, 0, &value);
  CHECK_ERR(err, fail, "Invalid value for 'test_time_usec'");
  spin_lock_bh(&itd_driver->spinlock);
  itd_driver->test_time_usec = value;
  spin_unlock_bh(&itd_driver->spinlock);
  return count;

fail:
  return err;
}

static ssize_t itddrv_test_show(struct device_driver *driver, char *buf)
{
  int err = 0;
  struct platform_driver *pdrv = container_of(driver, struct platform_driver, driver);
  struct itddrv *itd_driver = container_of(pdrv, struct itddrv, driver);
  long value = 0;
  int test[ITDDRV_NUM_OF_DEVS] = { 0 };
  int i = 0;
  int count = 0;
  unsigned long flags;

  CHECK(itd_driver->gpio_test > 0, err, -EINVAL, fail, "Test gpio pin not set");

  spin_lock_bh(&itd_driver->spinlock);

  for (i = 0; i < ITDDRV_NUM_OF_DEVS; i++)
  {
    if (itd_driver->itd[i] == NULL)
      continue;
    spin_lock_irqsave(&itd_driver->itd[i]->spinlock, flags);
    value = __itddev_read_value(itd_driver->itd[i]);
    spin_unlock_irqrestore(&itd_driver->itd[i]->spinlock, flags);
    if (value == ITDDEV_BLOCKED)
      test[i] += 1;
  }

  __itddrv_set_test_on(itd_driver);
  __itddrv_test_delay(itd_driver);

  for (i = 0; i < ITDDRV_NUM_OF_DEVS; i++)
  {
    if (itd_driver->itd[i] == NULL)
      continue;
    spin_lock_irqsave(&itd_driver->itd[i]->spinlock, flags);
    value = __itddev_read_value(itd_driver->itd[i]);
    spin_unlock_irqrestore(&itd_driver->itd[i]->spinlock, flags);
    if (value == ITDDEV_CLEAR)
      test[i] += 2;
  }

  __itddrv_set_test_off(itd_driver);

  spin_unlock_bh(&itd_driver->spinlock);

  for (i = 0; i < ITDDRV_NUM_OF_DEVS - 1; i++)
    count += sprintf(buf + count, "%i ", test[i]);
  return count + sprintf(buf + count, "%i\n", test[ITDDRV_NUM_OF_DEVS - 1]);

fail:
  return err;
}

static DRIVER_ATTR(test_time_usec, S_IRUGO | S_IWUSR,
  itddrv_test_time_usec_show, itddrv_test_time_usec_store);
static DRIVER_ATTR(test, S_IRUGO, itddrv_test_show, NULL);

static int __itddrv_create_driver_attr(struct itddrv *itd_driver)
{
  int err = 0;
  int attr_test_time_usec_created = 0;
  int attr_test_created = 0;

  err = driver_create_file(&itd_driver->driver.driver, &driver_attr_test_time_usec);
  CHECK_ERR(err, fail, "Failed to register driver attribute 'test_time_usec'");
  attr_test_time_usec_created = 1;

  err = driver_create_file(&itd_driver->driver.driver, &driver_attr_test);
  CHECK_ERR(err, fail, "Failed to register driver attribute 'test'");
  attr_test_created = 1;

  return 0;

fail:
  if (attr_test_created)
    driver_remove_file(&itd_driver->driver.driver, &driver_attr_test);
  if (attr_test_time_usec_created)
    driver_remove_file(&itd_driver->driver.driver, &driver_attr_test_time_usec);
  return err;
}

static void __itddrv_remove_driver_attr(struct itddrv *itd_driver)
{
  driver_remove_file(&itd_driver->driver.driver, &driver_attr_test);
  driver_remove_file(&itd_driver->driver.driver, &driver_attr_test_time_usec);
}

static int __init itddrv_init(void)
{
  int err = 0;

  err = alloc_chrdev_region(&itd_driver.base_dev_number, 0,
    ITDDRV_NUM_OF_DEVS, "itd");
  CHECK_ERR(err, fail, "Failed allocating chrdev region");

  err = platform_driver_register(&itd_driver.driver);
  CHECK_ERR(err, fail_regunreg, "Failed to register platform driver");

  err = __itddrv_create_driver_attr(&itd_driver);
  CHECK_ERR(err, fail_drvunreg, "Failed to register driver attributes");

  DBG_TRACE("Module initialized successfully");
  return 0;

fail_drvunreg:
  platform_driver_unregister(&itd_driver.driver);
fail_regunreg:
  unregister_chrdev_region(itd_driver.base_dev_number, ITDDRV_NUM_OF_DEVS);
fail:
  return err;
}

static void __exit itddrv_exit(void)
{
  __itddrv_remove_driver_attr(&itd_driver);
  platform_driver_unregister(&itd_driver.driver);
  unregister_chrdev_region(itd_driver.base_dev_number, ITDDRV_NUM_OF_DEVS);

  DBG_TRACE("Module exited");
}

module_init(itddrv_init);
module_exit(itddrv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomasz Rozensztrauch <t.rozensztrauch@gmail.com>");
MODULE_DESCRIPTION("Itd (input transition detector) device driver");

