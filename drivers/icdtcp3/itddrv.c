#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#include <linux/itddrv.h>
#include "icdcommon.h"

#define ITDDEV_CLEAR (0)
#define ITDDEV_BLOCKED (1)
#define ITDDEV_UNINITIALIZED (-1)

#define ITDDEV_IOBUF_SIZE (1024)

struct itddev 
{
  struct platform_device *pdev;
  struct cdev cdev;
  char *iobuf;
  char *iobuf_ptr;
 
  struct timer_list timer;
  struct tasklet_struct irq_tasklet;
  spinlock_t spinlock;
  struct list_head queue;
  atomic_t queue_count;
  wait_queue_head_t queue_ready;
  int state;
  int timer_running;

  long engage_delay_usec;
  long release_delay_usec;
  int active_low;
};

struct itddev_event
{
  struct list_head list;
  unsigned long sec;
  unsigned long usec;
  int state;
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
  gpio_set_value(itd_driver->gpio_test, 1);
}

static inline void __itddrv_set_test_off(struct itddrv *itd_driver)
{
  gpio_set_value(itd_driver->gpio_test, 0);
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
  return sprintf(buf, "%li\n", itd->engage_delay_usec);
}

static ssize_t itddev_engage_delay_usec_store(struct device *dev,
  struct device_attribute *attr, const char *buf, size_t count)
{
  int err = 0;
  struct platform_device *pdev = container_of(dev, struct platform_device, dev);
  struct itddev *itd = platform_get_drvdata(pdev);
  long value = 0;

  spin_lock_bh(&itd->spinlock);
  err = strict_strtol(buf, 0, &value);
  CHECK_ERR(err, fail, "Invalid value for engage_delay_usec");
  itd->engage_delay_usec = value;
  spin_unlock_bh(&itd->spinlock);
  return count;

fail:
  spin_unlock_bh(&itd->spinlock);
  return 0;
}

static ssize_t itddev_release_delay_usec_show(struct device *dev,
  struct device_attribute *attr, char *buf)
{
  struct platform_device *pdev = container_of(dev, struct platform_device, dev);
  struct itddev *itd = platform_get_drvdata(pdev);
  return sprintf(buf, "%li\n", itd->release_delay_usec);
}

static ssize_t itddev_release_delay_usec_store(struct device *dev,
  struct device_attribute *attr, const char *buf, size_t count)
{
  int err = 0;
  struct platform_device *pdev = container_of(dev, struct platform_device, dev);
  struct itddev *itd = platform_get_drvdata(pdev);
  long value = 0;

  spin_lock_bh(&itd->spinlock);
  err = strict_strtol(buf, 0, &value);
  CHECK_ERR(err, fail, "Invalid value for release_delay_usec");
  itd->release_delay_usec = value;
  spin_unlock_bh(&itd->spinlock);
  return count;

fail:
  spin_unlock_bh(&itd->spinlock);
  return 0;
}

static ssize_t itddev_active_low_show(struct device *dev,
  struct device_attribute *attr, char *buf)
{
  struct platform_device *pdev = container_of(dev, struct platform_device, dev);
  struct itddev *itd = platform_get_drvdata(pdev);
  return sprintf(buf, "%i\n", itd->active_low);
}

static ssize_t itddev_active_low_store(struct device *dev,
  struct device_attribute *attr, const char *buf, size_t count)
{
  int err = 0;
  struct platform_device *pdev = container_of(dev, struct platform_device, dev);
  struct itddev *itd = platform_get_drvdata(pdev);
  long value = 0;

  spin_lock_bh(&itd->spinlock);
  err = strict_strtol(buf, 0, &value);
  CHECK_ERR(err, fail, "Invalid value for active_low");
  itd->active_low = value ? 1 : 0;

  // After changinf active_low attribute reset
  // the device int uninitialized state
  itd->state = ITDDEV_UNINITIALIZED;
  tasklet_schedule(&itd->irq_tasklet);

  spin_unlock_bh(&itd->spinlock);
  return count;

fail:
  spin_unlock_bh(&itd->spinlock);
  return 0;
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

  CHECK(itd_driver->gpio_test > 0, err, -EINVAL, fail, "Test gpio pin not set");

  spin_lock_bh(&itd_driver->spinlock);
  spin_lock_bh(&itd->spinlock);

  value = __itddev_read_value(itd);
  if (value == ITDDEV_BLOCKED)
    test += 1;

  __itddrv_set_test_on(itd_driver);

  __itddrv_test_delay(itd_driver);

  value = __itddev_read_value(itd);
  if (value == ITDDEV_CLEAR)
    test += 2;

  __itddrv_set_test_off(itd_driver);

  spin_unlock_bh(&itd->spinlock);
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
static DEVICE_ATTR(test, S_IRUGO, itddev_test_show, NULL);

static inline void timeval_offset_usec(struct timeval *tm, long offset_usec)
{
  long sign = (offset_usec < 0) ? -1 : 1;
  long abs = (offset_usec < 0) ? -offset_usec : offset_usec;
  tm->tv_sec += sign * (abs / USEC_PER_SEC);
  tm->tv_usec += sign * (abs % USEC_PER_SEC);
  if (tm->tv_usec < 0)
  {
    tm->tv_sec--;
    tm->tv_usec += USEC_PER_SEC;
  }
}

static int __itddev_queue_event(struct itddev *itd,
  struct timeval *tm, int state)
{
  // queue itd event & wake up waiting processes
  int err = 0;
  struct itddev_event *ev = NULL;

  ev = kmalloc(sizeof(struct itddev_event), GFP_ATOMIC);
  CHECK_PTR(ev, err, fail, "Memory allocation of itddev_event failed");

  ev->sec = tm->tv_sec;
  ev->usec = tm->tv_usec;
  ev->state = state;

  list_add_tail(&ev->list, &itd->queue);
       
  DBG_TRACE("%s.%d event: sec=%lu,usec=%lu,state=%i\n",
    itd->pdev->name, itd->pdev->id, ev->sec, ev->usec, ev->state);

  atomic_inc(&itd->queue_count);
  wake_up_interruptible(&itd->queue_ready);

fail:
  return err;
}

static void itddev_timer_fn(unsigned long data)
{
  struct itddev *itd = (struct itddev *)data;
  struct timeval tm;

  do_gettimeofday(&tm);

  spin_lock_bh(&itd->spinlock);

  // This is legitimate case - it may happen that the timer has been cancelled
  // but it happened "too late" so this handler was called, but should do nothing
  if (!itd->timer_running)
    goto unlock;

  switch(itd->state)
  {
    case ITDDEV_UNINITIALIZED:
      itd->state = ITDDEV_CLEAR;
      DBG_TRACE("%s.%d state UNINITIALIZED -> CLEAR", itd->pdev->name, itd->pdev->id);
      __itddev_set_led_off(itd);
      break;

    case ITDDEV_CLEAR:
      DBG_TRACE("%s.%d state CLEAR -> BLOCKED", itd->pdev->name, itd->pdev->id);
      itd->state = ITDDEV_BLOCKED;
      timeval_offset_usec(&tm, -itd->engage_delay_usec);
      __itddev_queue_event(itd, &tm, ITDDEV_BLOCKED);
      __itddev_set_led_on(itd);
      break;

    case ITDDEV_BLOCKED:
      itd->state = ITDDEV_CLEAR;
      DBG_TRACE("%s.%d state BLOCKED -> CLEAR", itd->pdev->name, itd->pdev->id);
      timeval_offset_usec(&tm, -itd->release_delay_usec);
      __itddev_queue_event(itd, &tm, ITDDEV_CLEAR);
      __itddev_set_led_off(itd);
      break;

    default:
      DBG_REPORT(-EINVAL, "%s.%d Invalid state; ? -> UNINITIALIZED",
        itd->pdev->name, itd->pdev->id);
      itd->state = ITDDEV_UNINITIALIZED;
      tasklet_schedule(&itd->irq_tasklet);
      __itddev_set_led_off(itd);
  }

  itd->timer_running = 0;

unlock:
  spin_unlock_bh(&itd->spinlock);
}

static void itddev_irq_tasklet_fn(unsigned long data)
{
  // state value action
  // -1     0    (re)start timer
  // -1     1    cancel timer if running
  //  0     0    cancel timer if running
  //  0     1    (re)start timer
  //  1     0    (re)start timer
  //  1     1    cancel timer if running

  struct itddev *itd = (struct itddev *)data;
  unsigned long delay_usec;
  int value = 0;

  spin_lock_bh(&itd->spinlock);

  value = __itddev_read_value(itd);

  if ((itd->state == ITDDEV_UNINITIALIZED && value == ITDDEV_CLEAR)
    || (itd->state == ITDDEV_CLEAR && value == ITDDEV_BLOCKED)
    || (itd->state == ITDDEV_BLOCKED && value == ITDDEV_CLEAR))
  {
    delay_usec = (value == ITDDEV_BLOCKED) ? itd->engage_delay_usec :
      itd->release_delay_usec;
    mod_timer(&itd->timer, jiffies + usecs_to_jiffies(delay_usec));
    itd->timer_running = 1;
    DBG_TRACE("%s.%d irq: value=%i, TIMER (RE)STARTED",
      itd->pdev->name, itd->pdev->id, value);
  }
  else
  {
    itd->timer_running = 0;
    del_timer(&itd->timer);
    DBG_TRACE("%s.%d irq: value=%i, TIMER CANCELED",
      itd->pdev->name, itd->pdev->id, value);
  }

  spin_unlock_bh(&itd->spinlock);
}

static irqreturn_t itddev_irq_handler_fn(int irq, void *dev)
{
  struct itddev *itd = (struct itddev *)dev;
  tasklet_schedule(&itd->irq_tasklet);
  return IRQ_HANDLED;
}

static int itddev_cdev_open(struct inode *inode, struct file *filp)
{
  struct itddev *itd = (struct itddev *)
    container_of(inode->i_cdev, struct itddev, cdev);
  filp->private_data = itd;
  return 0;
}

static int itddev_cdev_read(struct file *filp, __user char *buf, size_t count, loff_t *offp)
{
  int err = 0;
  struct itddev *itd = (struct itddev *)filp->private_data;
  struct itddev_event* entry = NULL;
  size_t copied = 0;
  size_t len = 0;
  unsigned long res = 0;

  // In case some data has left in the buffer from previous read operation
  if (itd->iobuf_ptr != 0 && itd->iobuf_ptr[0] != 0)
  {
    len = min(count - copied, strlen(itd->iobuf_ptr));
    res = copy_to_user(buf + copied, itd->iobuf_ptr, len);
    CHECK(res == 0, err, -EINVAL, fail, "Copying data to userspace buffer failed");
    itd->iobuf_ptr += len;
    copied += len;
  }

  while(copied < count)
  {
    while (atomic_read(&itd->queue_count) > 0)
    {
      spin_lock_bh(&itd->spinlock);

      atomic_dec(&itd->queue_count);
      CHECK(!list_empty(&itd->queue), err, -EINVAL, fail, "Corrupt event queue");
      entry = list_first_entry(&itd->queue, struct itddev_event, list);
      snprintf(itd->iobuf, ITDDEV_IOBUF_SIZE, "%lu %lu %i\n",
        entry->sec, entry->usec, entry->state);
      list_del(&entry->list);
      kfree(entry);

      spin_unlock_bh(&itd->spinlock);

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

    err = wait_event_interruptible(itd->queue_ready,
      atomic_read(&itd->queue_count) > 0);
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

  itd->pdev = pdev;

  itd->iobuf = kmalloc(ITDDEV_IOBUF_SIZE, GFP_KERNEL);
  CHECK_PTR(itd->iobuf, err, fail, "Memory allocation of iobuf failed");
  itd->iobuf[0] = 0;
  itd->iobuf_ptr = itd->iobuf;

  cdev_init(&itd->cdev, &itddev_cdev_operations);
  itd->cdev.owner = THIS_MODULE;

  err = cdev_add(&itd->cdev, dev_nr, 1);
  CHECK_ERR(err, fail_free_iobuf, "Adding character device for %s.%i failed",
    pdev->name, pdev->id);

  err = request_irq(gpio_to_irq(device_data->gpio_in), itddev_irq_handler_fn,
    IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "itd-irq", (void*)itd);
  CHECK_ERR(err, fail_free_cdev, "Requesting irq for %s.%i failed",
    pdev->name, pdev->id);

  itd->state = ITDDEV_UNINITIALIZED;
  itd->timer_running = 0;

  itd->engage_delay_usec = 2000000;
  itd->release_delay_usec = 2000000;
  itd->active_low = 0;

  INIT_LIST_HEAD(&itd->queue);
  atomic_set(&itd->queue_count, 0);
  init_waitqueue_head(&itd->queue_ready);
  spin_lock_init(&itd->spinlock);
  tasklet_init(&itd->irq_tasklet, itddev_irq_tasklet_fn, (unsigned long)itd);

  init_timer(&itd->timer);
  itd->timer.data = (unsigned long)itd;
  itd->timer.function = itddev_timer_fn;

  err = device_create_file(&pdev->dev, &dev_attr_engage_delay_usec);
  CHECK_ERR(err, fail_free_irq, "Failed to register device attribute 'engage_delay_usec'");
  err = device_create_file(&pdev->dev, &dev_attr_release_delay_usec);
  CHECK_ERR(err, fail_free_irq, "Failed to register device attribute 'release_delay_usec'");
  err = device_create_file(&pdev->dev, &dev_attr_active_low);
  CHECK_ERR(err, fail_free_irq, "Failed to register device attribute 'active_low'");
  err = device_create_file(&pdev->dev, &dev_attr_test);
  CHECK_ERR(err, fail_free_irq, "Failed to register device attribute 'test'");

  __itddev_set_led_off(itd);
  tasklet_schedule(&itd->irq_tasklet);
  return 0;

fail_free_irq:
  free_irq(gpio_to_irq(device_data->gpio_in), (void*)itd);

fail_free_cdev:
  cdev_del(&itd->cdev);

fail_free_iobuf:
  kfree(itd->iobuf);

fail:
  return err;
}

static void itddev_free(struct itddev *itd)
{
  struct itddev_data *device_data = itd->pdev->dev.platform_data;

  device_remove_file(&itd->pdev->dev, &dev_attr_test);
  device_remove_file(&itd->pdev->dev, &dev_attr_active_low);
  device_remove_file(&itd->pdev->dev, &dev_attr_release_delay_usec);
  device_remove_file(&itd->pdev->dev, &dev_attr_engage_delay_usec);

  free_irq(gpio_to_irq(device_data->gpio_in), (void*)itd);

  cdev_del(&itd->cdev);

  kfree(itd->iobuf);
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

  spin_lock_bh(&itd_driver->spinlock);

  if (itd_driver->gpio_test == 0)
    itd_driver->gpio_test = device_data->gpio_test;

  itd = kzalloc(sizeof(struct itddev), GFP_KERNEL);
  CHECK_PTR(itd, err, fail_unlock, "Memory allocation of itddev failed");

  platform_set_drvdata(pdev, itd);
  itd_driver->itd[pdev->id] = itd;
  
  err = itddev_init(itd, pdev, dev_nr);
  CHECK_ERR(err, fail_free, "Initalization of itddev failed");

  spin_unlock_bh(&itd_driver->spinlock);

  DBG_TRACE("Device %s.%d added", pdev->name, pdev->id);
  return 0;

fail_free:
  kfree(itd);

fail_unlock:
  spin_unlock_bh(&itd_driver->spinlock);

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
  kfree(itd);
  spin_unlock_bh(&itd_driver->spinlock);

  DBG_TRACE("Device %s.%d removed", pdev->name, pdev->id);
  return 0;
}

static struct itddrv itd_driver = {
  .gpio_test = 0,
  .test_time_usec = 1000000,
  .spinlock = SPIN_LOCK_UNLOCKED,
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
  return sprintf(buf, "%li\n", itd_driver->test_time_usec);
}

static ssize_t itddrv_test_time_usec_store(struct device_driver *driver,
  const char *buf, size_t count)
{
  int err = 0;
  struct platform_driver *pdrv = container_of(driver, struct platform_driver, driver);
  struct itddrv *itd_driver = container_of(pdrv, struct itddrv, driver);
  long value = 0;

  spin_lock_bh(&itd_driver->spinlock);
  err = strict_strtol(buf, 0, &value);
  CHECK_ERR(err, fail, "Invalid value for 'test_time_usec'");
  itd_driver->test_time_usec = value;
  spin_unlock_bh(&itd_driver->spinlock);
  return count;

fail:
  spin_unlock_bh(&itd_driver->spinlock);
  return 0;
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

  CHECK(itd_driver->gpio_test > 0, err, -EINVAL, fail, "Test gpio pin not set");

  spin_lock_bh(&itd_driver->spinlock);
  for (i = 0; i < ITDDRV_NUM_OF_DEVS; i++)
    if (itd_driver->itd[i] != NULL)
      spin_lock_bh(&itd_driver->itd[i]->spinlock);

  for (i = 0; i < ITDDRV_NUM_OF_DEVS; i++)
  {
    if (itd_driver->itd[i] == NULL)
      continue;
    value = __itddev_read_value(itd_driver->itd[i]);
    if (value == ITDDEV_BLOCKED)
      test[i] += 1;
  }

  __itddrv_set_test_on(itd_driver);

  __itddrv_test_delay(itd_driver);

  for (i = 0; i < ITDDRV_NUM_OF_DEVS; i++)
  {
    if (itd_driver->itd[i] == NULL)
      continue;
    value = __itddev_read_value(itd_driver->itd[i]);
    if (value == ITDDEV_CLEAR)
      test[i] += 2;
  }

  __itddrv_set_test_off(itd_driver);

  for (i = ITDDRV_NUM_OF_DEVS - 1; i >= 0; i--)
    if (itd_driver->itd[i] != NULL)
      spin_unlock_bh(&itd_driver->itd[i]->spinlock);
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

static int __init itddrv_init(void)
{
  int err = alloc_chrdev_region(&itd_driver.base_dev_number, 0,
    ITDDRV_NUM_OF_DEVS, "itd");
  CHECK_ERR(err, fail, "allocating chrdev region failed");

  err = platform_driver_register(&itd_driver.driver);
  CHECK_ERR(err, fail_regunreg, "registering platform device failed");

  err = driver_create_file(&itd_driver.driver.driver, &driver_attr_test_time_usec);
  CHECK_ERR(err, fail_drvunreg, "Failed to register driver attribute 'test_time_usec_usec'");
  err = driver_create_file(&itd_driver.driver.driver, &driver_attr_test);
  CHECK_ERR(err, fail_drvunreg, "Failed to register driver attribute 'test'");

  DBG_TRACE("module initialize succeddfully");
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
  driver_remove_file(&itd_driver.driver.driver, &driver_attr_test_time_usec);
  driver_remove_file(&itd_driver.driver.driver, &driver_attr_test);

  platform_driver_unregister(&itd_driver.driver);

  unregister_chrdev_region(itd_driver.base_dev_number, ITDDRV_NUM_OF_DEVS);

  DBG_TRACE("module exit");
}

module_init(itddrv_init);
module_exit(itddrv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomasz Rozensztrauch <t.rozensztrauch@gmail.com>")
MODULE_DESCRIPTION("Itd (input transition detector) device driver");

