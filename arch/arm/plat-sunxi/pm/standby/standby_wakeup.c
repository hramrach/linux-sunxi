#include "standby_i.h"
#include "standby_wakeup.h"

void standby_wakeup_init(void)
{
	/* initialise standby modules */
	standby_clk_init();
	standby_int_init();
	standby_tmr_init();
	standby_power_init();
	/* init some system wake source */
	if(pm_info.standby_para.event & SUSPEND_WAKEUP_SRC_EXINT){
		standby_enable_int(INT_SOURCE_EXTNMI);
	}
	if(pm_info.standby_para.event & SUSPEND_WAKEUP_SRC_KEY){
		standby_key_init();
		standby_enable_int(INT_SOURCE_LRADC);
	}
	if(pm_info.standby_para.event & SUSPEND_WAKEUP_SRC_IR){
		standby_ir_init();
		standby_enable_int(INT_SOURCE_IR0);
		standby_enable_int(INT_SOURCE_IR1);
	}
	if(pm_info.standby_para.event & SUSPEND_WAKEUP_SRC_ALARM){
		//standby_alarm_init();???
		standby_enable_int(INT_SOURCE_ALARM);
	}
	if(pm_info.standby_para.event & SUSPEND_WAKEUP_SRC_USB){
		standby_usb_init();
		standby_enable_int(INT_SOURCE_USB0);
	}
	if(pm_info.standby_para.event & SUSPEND_WAKEUP_SRC_TIMEOFF){
		/* set timer for power off */
		if(pm_info.standby_para.time_off) {
			standby_tmr_set(pm_info.standby_para.time_off);
			standby_enable_int(INT_SOURCE_TIMER0);
		}
	}
}

void standby_wakeup_fini(void)
{
	if(pm_info.standby_para.event & SUSPEND_WAKEUP_SRC_USB){
		standby_usb_exit();
	}
	if(pm_info.standby_para.event & SUSPEND_WAKEUP_SRC_IR){
		standby_ir_exit();
	}
	if(pm_info.standby_para.event & SUSPEND_WAKEUP_SRC_ALARM){
		//standby_alarm_exit();
	}
	if(pm_info.standby_para.event & SUSPEND_WAKEUP_SRC_KEY){
		standby_key_exit();
	}
	standby_power_exit();
	standby_tmr_exit();
	standby_int_exit();
	standby_clk_exit();
}
