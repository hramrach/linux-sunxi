#ifndef __LEDTRIG_CAMERA_H_INCLUDED
#define __LEDTRIG_CAMERA_H_INCLUDED

#if defined(CONFIG_LEDS_TRIGGER_CAMERA) || defined(CONFIG_LEDS_TRIGGER_CAMERA_MODULE)
extern void ledtrig_flash_ctrl(bool on);
extern void ledtrig_torch_ctrl(bool on);
#else
static inline void ledtrig_flash_ctrl(bool on) {}
static inline void ledtrig_torch_ctrl(bool on) {}
#endif

#endif
