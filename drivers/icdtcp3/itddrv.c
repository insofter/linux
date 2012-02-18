#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/itddrv.h>

#include <linux/fs.h>
#include <linux/cdev.h>

#include <linux/uaccess.h>

#define PHOTOCELL_CLEAR (0)
#define PHOTOCELL_BLOCKED (1)
#define PHOTOCELL_UNINITIALIZED (-1)

#define PHOTOCELL_IOBUFF_SIZE (1024)

struct photocell_device 
{
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
  unsigned long start_time;

  int engage_delay;
  int release_delay;
};

struct photocell_entry
{
  struct list_head list;
  unsigned long start_time;
  unsigned long end_time;
};

static struct photocell_device *phc;

void photocell_timer_fn(unsigned long data)
{
  struct photocell_device *photocell = (struct photocell_device *)data;
  struct photocell_entry *entry = NULL;

  spin_lock_bh(&photocell->spinlock);
  if (!photocell->timer_running)
    goto unlock;

  switch(photocell->state)
  {
    case PHOTOCELL_UNINITIALIZED:
      photocell->state = PHOTOCELL_CLEAR;
      printk(KERN_ALERT "TM: %lu, UNINITIALIZED -> CLEAR\n", jiffies);
      gpio_set_value(AT91_PIN_PB0, 1);
      break;

    case PHOTOCELL_CLEAR:
      photocell->state = PHOTOCELL_BLOCKED;
      photocell->start_time = jiffies - photocell->engage_delay;
      printk(KERN_ALERT "TM: %lu, CLEAR -> BLOCKED\n", jiffies);
      gpio_set_value(AT91_PIN_PB0, 0);
      break;

    case PHOTOCELL_BLOCKED:
      photocell->state = PHOTOCELL_CLEAR;
      printk(KERN_ALERT "TM: %lu, BLOCKED -> CLEAR\n", jiffies);
      // queue photocell event & wake up waiting processes
      entry = kmalloc(sizeof(struct photocell_entry), GFP_ATOMIC);
      if (entry == NULL)
        printk(KERN_ALERT "kmalloc failed\n");
      if (entry)
      {
        entry->start_time = photocell->start_time;
        entry->end_time = jiffies - photocell->release_delay;
        list_add_tail(&entry->list, &photocell->queue);
       
        printk(KERN_ALERT "TM: Queued: dt=%lu\n",
          entry->end_time - entry->start_time);

        atomic_inc(&photocell->queue_count);
        wake_up_interruptible(&photocell->queue_ready);
      }
      gpio_set_value(AT91_PIN_PB0, 1);
      break;

    default:
      printk(KERN_ALERT "TM: %lu, Error! Invalid value of state! ? -> UNINITIALIZED\n", jiffies);
      photocell->state = PHOTOCELL_UNINITIALIZED;
  }

  photocell->timer_running = 0;

unlock:
  spin_unlock_bh(&photocell->spinlock);
}

void photocell_irq_tasklet_fn(unsigned long data)
{
  struct photocell_device *photocell = (struct photocell_device *)data;
  unsigned long delay;
//  unsigned int gpio_nr1 = AT91_PIN_PB0;
  unsigned int gpio_nr2 = AT91_PIN_PB2;
  int gpio = 0;

  gpio = gpio_get_value(gpio_nr2) ? 0 : 1; // Hardcoded reversion for now
//  gpio_set_value(gpio_nr1, gpio);

  // state gpio action
  // -1     0    (re)start timer
  // -1     1    cancel timer if running
  //  0     0    cancel timer if running
  //  0     1    (re)start timer
  //  1     0    (re)start timer
  //  1     1    cancel timer if running

  spin_lock_bh(&photocell->spinlock);

  if ((photocell->state == PHOTOCELL_UNINITIALIZED && !gpio)
    || (photocell->state == PHOTOCELL_CLEAR && gpio)
    || (photocell->state == PHOTOCELL_BLOCKED && !gpio))
  {
    delay = gpio ? photocell->engage_delay : photocell->release_delay;
    mod_timer(&photocell->timer, jiffies + delay);
    photocell->timer_running = 1;
    printk(KERN_ALERT "IRQ: gpio=%u, jiffies=%lu TIMER (RE)STARTED\n", gpio, jiffies);
  }
  else
  {
    photocell->timer_running = 0;
    del_timer(&photocell->timer);
    printk(KERN_ALERT "IRQ: gpio=%u, jiffies=%lu TIMER CANCELED\n", gpio, jiffies);
  }

  spin_unlock_bh(&photocell->spinlock);
}


irqreturn_t photocell_irq_handler(int irq, void *dev)
{
  struct photocell_device *photocell = (struct photocell_device *)dev;
  tasklet_schedule(&photocell->irq_tasklet);
  return IRQ_HANDLED;
}

static int phc_cdev_open(struct inode *inode, struct file *filp)
{
  struct photocell_device *photocell = (struct photocell_device *)
    container_of(inode->i_cdev, struct photocell_device, cdev);
  filp->private_data = photocell;
  return 0;
}

static int phc_cdev_read(struct file *filp, __user char *buf, size_t count, loff_t *offp)
{
  int err = 0;
  struct photocell_device *dev = (struct photocell_device *)filp->private_data;
  struct photocell_entry* entry = NULL;
  size_t copied = 0;
  size_t len = 0;
  unsigned long res = 0;

  // In case some data has left in the buffer from previous read operation
  if (dev->iobuf_ptr != 0 && dev->iobuf_ptr[0] != 0)
  {
    len = min(count - copied, strlen(dev->iobuf_ptr));
    res = copy_to_user(buf + copied, dev->iobuf_ptr, len);
    //ICD2_CHECK(res == 0);
    dev->iobuf_ptr += len;
    copied += len;
  }

  while(copied < count)
  {
    while (atomic_read(&dev->queue_count) > 0)
    {
      spin_lock_bh(&dev->spinlock);

      atomic_dec(&dev->queue_count);
      //ICD2_CHECK(!list_empty(&photocell->queue));
      entry = list_first_entry(&dev->queue, struct photocell_entry, list);
      snprintf(dev->iobuf, PHOTOCELL_IOBUFF_SIZE, "%lu %lu\n",
        entry->start_time, entry->end_time);
      list_del(&entry->list);
      kfree(entry);

      spin_unlock_bh(&dev->spinlock);

      len = min(count - copied, strlen(dev->iobuf));
      res = copy_to_user(buf + copied, dev->iobuf, len);
      //ICD2_CHECK(res == 0);
      dev->iobuf_ptr = dev->iobuf + len;
      copied += len;

      if (copied == count)
        break;
    }

    if (copied > 0)
      break;

    err = wait_event_interruptible(dev->queue_ready,
      atomic_read(&dev->queue_count) > 0);
    //ICD2_CHECK_ERR()
    if (err)
      break;
  }

  return copied;
}

static int phc_cdev_release(struct inode *inode, struct file *filp)
{
  return 0;
}

const struct file_operations phc_cdev_operations = {
  .owner = THIS_MODULE,
  .open = phc_cdev_open,
  .llseek = no_llseek,
  .read = phc_cdev_read,
  .release = phc_cdev_release,
};


static int init_phc_devices(void)
{
  int err;
  dev_t dev;

  err = alloc_chrdev_region(&dev, 0, 1, "photocell");
  if (err)
  {
    printk(KERN_ALERT "alloc_chrdev_region failed!\n");
    return err;
  }

  phc = kzalloc(sizeof(struct photocell_device), GFP_KERNEL);
  if (!phc)
    return -ENOMEM;
  
  phc->iobuf = kmalloc(PHOTOCELL_IOBUFF_SIZE, GFP_KERNEL);
  if (!phc->iobuf)
  {
    kfree(phc);
    return -ENOMEM;
  }
  phc->iobuf[0] = 0;
  phc->iobuf_ptr = phc->iobuf;

  cdev_init(&phc->cdev, &phc_cdev_operations);
  phc->cdev.owner = THIS_MODULE;

  err = cdev_add(&phc->cdev, dev, 1);
  if (err)
  {
    printk(KERN_ALERT "cdev_add failed!\n");
    return err;
  }

  printk(KERN_ALERT "HZ=%d\n", HZ);

  phc->state = PHOTOCELL_UNINITIALIZED;
  phc->timer_running = 0;
  phc->start_time = 0;
  phc->engage_delay = 200;
  phc->release_delay = 200;
  INIT_LIST_HEAD(&phc->queue);
  atomic_set(&phc->queue_count, 0);
  init_waitqueue_head(&phc->queue_ready);
  spin_lock_init(&phc->spinlock);
  tasklet_init(&phc->irq_tasklet, photocell_irq_tasklet_fn, (unsigned long)phc);
  init_timer(&phc->timer);
  phc->timer.data = (unsigned long)phc;
  phc->timer.function = photocell_timer_fn;

  tasklet_schedule(&phc->irq_tasklet);

  return 0;
}

static void free_phc_devices(void)
{
  cdev_del(&phc->cdev);
  kfree(phc->iobuf);
  kfree(phc);
  unregister_chrdev_region(phc->cdev.dev, 1);
}

/*static int __init hello_init(void)
{
  unsigned int gpio_nr1 = AT91_PIN_PB0;
  unsigned int gpio_nr2 = AT91_PIN_PB2;
  int irq = 0;
//  int value = 0;



  printk(KERN_ALERT "Hello world!\n");

  if (init_phc_devices() != 0)
  {
    printk(KERN_ALERT "init_phc_devices failed!\n");
    return 0;
  }
 
  // Output gpio
  
  if (gpio_request(gpio_nr1, "ICD2_PHOTOCELL OUT") != 0)
  {
    printk(KERN_ALERT "gpio_request failed!\n");
    return 0;
  }
  
  if (gpio_direction_output(gpio_nr1, 0) != 0)
  {
    printk(KERN_ALERT "gpio_direction_output failed!\n");
    return 0;
  }

  gpio_set_value(gpio_nr1, 0);

  // Input gpio

  if (gpio_request(gpio_nr2, "ICD2_PHOTOCELL_IN") != 0)
  {
    printk(KERN_ALERT "gpio_request failed!\n");
    return 0;
  }
  if (gpio_direction_input(gpio_nr2) != 0)
  {
    printk(KERN_ALERT "gpio_direction_input failed!\n");
    return 0;
  }
 
  if (at91_set_deglitch(gpio_nr2, 1) != 0)
  {
    printk(KERN_ALERT "at91_set_deglitch failed!\n");
    return 0;
  }

//  value = gpio_get_value(gpio_nr2);
//  printk(KERN_ALERT "gpio_get_value returned %i\n", value);

  irq = gpio_to_irq(gpio_nr2);

  if (request_irq(irq, photocell_irq_handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
    "icd2-photocell-irq", (void*)phc) != 0)
  {
    printk(KERN_ALERT "request_irq failed!\n");
    return 0;
  }

  return 0;
}

static void __exit hello_exit(void)
{
  unsigned int gpio_nr1 = AT91_PIN_PB0;
  unsigned int gpio_nr2 = AT91_PIN_PB2;
  int irq;

  irq = gpio_to_irq(gpio_nr2);
  free_irq(irq, (void*)phc);

  gpio_free(gpio_nr2);
  gpio_free(gpio_nr1);

  free_phc_devices();

  printk(KERN_ALERT "Goodbye, cruel world\n");
}
*/
//-----------------------

static int __devinit itddrv_probe(struct platform_device *pdev)
{
  struct icdtcp3_itd_data *data = pdev->dev.platform_data;

  printk(KERN_ALERT"itddrv_probe\n");

  printk(KERN_ALERT"gpio_in = %i\n", data->gpio_in);
  printk(KERN_ALERT"gpio_led = %i\n", data->gpio_led);
  printk(KERN_ALERT"gpio_test = %i\n", data->gpio_test);
  printk(KERN_ALERT"descr = %s\n", data->desc);

  return 0;
}

static int __devexit itddrv_remove(struct platform_device *pdev)
{
  struct icdtcp3_itd_data *data = pdev->dev.platform_data;

  printk(KERN_ALERT "itddrv_remove\n");

  printk(KERN_ALERT"gpio_in = %i\n", data->gpio_in);
  printk(KERN_ALERT"gpio_led = %i\n", data->gpio_led);
  printk(KERN_ALERT"gpio_test = %i\n", data->gpio_test);
  printk(KERN_ALERT"descr = %s\n", data->desc);

  return 0;
}

static struct platform_driver gpio_itd_driver = {
  .probe = itddrv_probe,
  .remove = __devexit_p(itddrv_remove),
  .driver = { 
    .name = "gpio-itd",
    .owner = THIS_MODULE,
  }
};

static int __init itddrv_init(void)
{
  printk(KERN_ALERT "itddrv_init\n");
  return platform_driver_register(&gpio_itd_driver);
}

static void __exit itddrv_exit(void)
{
  printk(KERN_ALERT "itddrv_exit\n");
  platform_driver_unregister(&gpio_itd_driver);
}

module_init(itddrv_init);
module_exit(itddrv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomasz Rozensztrauch <t.rozensztrauch@gmail.com>")
MODULE_DESCRIPTION("Itd (input transition detector) device driver for icdtcp3");

