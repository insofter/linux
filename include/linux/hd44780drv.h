#ifndef __HD44780DRV_H__
#define __HD44780DRV_H__

struct hd44780dev_data
{
  int gpio_rs;
  int gpio_e;
  int gpio_d4;
  int gpio_d5;
  int gpio_d6;
  int gpio_d7;
  int gpio_backlight;
  char* descr;
};

#endif //__HD44780DRV_H__
