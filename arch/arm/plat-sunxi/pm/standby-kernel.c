/* The standby code is compiled with different calling convention so include another copy for use in kernel. */

#include "standby/common.c"
#include "standby/standby_power.c"
#include "standby/standby_twi.c"
#include "standby/standby_int.c"
#include "standby/standby_clock.c"
#include "standby/standby_key.c"
#include "standby/standby_usb.c"
#include "standby/standby_ir.c"
#include "standby/standby_tmr.c"
#include "standby/standby_wakeup.c"
