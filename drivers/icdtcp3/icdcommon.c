#include <linux/module.h>
/**
 * icd_printk - logs a message, depending on its priority and current
 *   verbosity level (based on verbose option)
 * @priority - message priority 0 (diagnostics) or 1 (errors)
 * @fmt, ... - message format and parameters as for printk
 *
 * We have 3 verbosity levels:
 *   0 - no messages at all
 *   1 - errors (priority 1 messages)
 *   2 - errors and diagnostics (priority 0 and 1 messages)
 */ 
void icd_printk(int priority, const char *fmt, ...)
{
  va_list args;
  int verbose;

  verbose = 2; //TODO
  if (verbose > 1 || (verbose == 1 && priority))
  {
    va_start(args, fmt);
    vprintk(fmt, args);
    va_end(args);
  }
}

