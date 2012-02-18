#ifndef __ICD_H__
#define __ICD_H__

extern void icd_printk(int priority, const char *fmt, ...);

/**
 * DBG_TRACE - logs a message (printk with KERN_DEBUG prefix); the message
 *   is only printed it verbose option is greather that 1
 */
#define DBG_TRACE(s, args...) \
  icd_printk(0, KERN_DEBUG "%s: "s"\n", __func__, ## args)

/**
 * DBG_REPORT - like DBG_TRACE; but in case of error the message is logged
 *   with higher priority, so if verbose option is only greather than 0
 *   (= print errors) then it gets printed
 */
#define DBG_REPORT(err, s, args...) \
  icd_printk(((int)err < 0), "%s%s,%s: "s"\n", \
    ((int)err < 0) ? KERN_ERR : KERN_DEBUG , __func__, \
    ((int)err < 0) ? "failed" : "succeeded", ## args)

/**
 * CHECK - if condition evaluates to 0 then it logs provided error message
 *   (if verbose option is greather than 0)
 */
#define CHECK(cond, errvar, value, label, s, args...) \
  if (!(cond)) \
  { \
    icd_printk(1, KERN_ERR "%s(%s:%d): "s"\n", __func__, \
      __FILE__, __LINE__, ## args); \
    errvar = value; \
    goto label; \
  }

/**
 * CHECK_ERR - if en error has occurred (err < 0) then it logs provided error message
 *   (if verbose option is greather than 0)
 */
#define CHECK_ERR(err, label, s, args...) \
  if ((int)err < 0) \
  { \
    icd_printk(1, KERN_ERR "%s(%s:%d),err=%d: "s"\n", __func__, \
      __FILE__, __LINE__, err, ## args); \
    goto label; \
  }

/**
 * CHECK_PTR - if en error occurred (ptr is NULL or IS_ERR(ptr)) then it logs
 *   provided error message (if verbose option is greather than 0);
 *   alos set provided err variable and zeroes ptr
 */
#define CHECK_PTR(ptr, errvar, label, s, args...) \
  if (ptr == NULL || IS_ERR(ptr)) \
  { \
    errvar = (ptr == NULL) ? -ENOMEM :  PTR_ERR(ptr); \
    ptr = NULL; \
    icd_printk(1, KERN_ERR "%s(%s:%d),err=%d: "s"\n", __func__, \
      __FILE__, __LINE__, errvar, ## args); \
    goto label; \
  }

#endif // __ICD_H__

