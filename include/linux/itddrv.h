#ifndef __ITDDRV_H__
#define __ITDDRV_H__

struct itddev_data
{
  int gpio_in;
  int gpio_led;
  int gpio_test;
  char* desc;
};

#endif // __ITDDRV_H__
