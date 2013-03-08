#ifndef __LEDTRIG_BLOCK_H_INCLUDED
#define __LEDTRIG_BLOCK_H_INCLUDED
struct gendisk;
extern struct device_type disk_type;
extern struct class block_class;
typedef void (*disk_func)(struct gendisk *);
extern disk_func ledtrig_block_add;
extern disk_func ledtrig_block_del;
extern struct mutex block_class_lock;
#endif
