/*
 * Block device activity LED trigger
 *
 * Copyright 2013
 *
 * Author: Michal Suchanek <hramrach@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/genhd.h>
#include <linux/timer.h>
#include <linux/leds.h>
#include <linux/list.h>
#include <linux/slab.h>

#include "ledtrig-block.h"

static void ledtrig_block_timerfunc(unsigned long data);

static DEFINE_TIMER(ledtrig_block_timer, ledtrig_block_timerfunc, 0, 0);
static DECLARE_RWSEM(trigger_list_lock);
static LIST_HEAD(trigger_list);
static DECLARE_RWSEM(active_list_lock);
static LIST_HEAD(active_list);
static unsigned long trigger_interval = 10;

module_param(trigger_interval, ulong, 0);
MODULE_PARM_DESC(trigger_interval, "Interval in ms after which block device statistics are checked");

static ssize_t show_trigger_interval(struct device *dev, struct device_attribute *attr,
		char *buf);
static ssize_t store_trigger_interval(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count);
static DEVICE_ATTR(trigger_interval, S_IWUSR | S_IRUGO, show_trigger_interval, store_trigger_interval);

typedef enum {
	BLOCKTRIG_TYPE_MIN = 1,
	BLOCKTRIG_IO = BLOCKTRIG_TYPE_MIN,
	BLOCKTRIG_READ,
	BLOCKTRIG_WRITE,
	BLOCKTRIG_TYPE_MAX = BLOCKTRIG_WRITE
} block_trigger_type_t;

struct block_trigger {
	char * name;
	sector_t data;
	block_trigger_type_t type;
	struct gendisk		*disk;
	struct led_trigger	 trigger;
	struct list_head	 trigger_list;
	struct list_head	 active_list;
};

static ssize_t show_trigger_interval(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	ssize_t ret;
	down_read(&active_list_lock);
	ret = scnprintf(buf, PAGE_SIZE, "%lu\n", trigger_interval);
	up_read(&active_list_lock);
	return ret;
}

static ssize_t store_trigger_interval(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	ssize_t ret;
	unsigned long tmp;
	ret = sscanf(buf, "%lu", &tmp);
	if (!tmp) return -EINVAL;
	down_write(&active_list_lock);
	trigger_interval = tmp;
	up_write(&active_list_lock);
	return ret;
}

static void inline ledtrig_block_schedule_timer(int check)
{
	if (check && timer_pending(&ledtrig_block_timer))
		return;
	if (list_empty(&active_list)) /* Should be locked by caller. */
		return;
	if(!trigger_interval) trigger_interval = 1;
	mod_timer(&ledtrig_block_timer, jiffies + msecs_to_jiffies(trigger_interval));
}

static sector_t get_disk_data(struct gendisk *disk, block_trigger_type_t type)
{
	switch(type) {
		case BLOCKTRIG_READ:
			return part_stat_read(disk_get_part(disk,0), sectors[READ]);
		case BLOCKTRIG_WRITE:
			return part_stat_read(disk_get_part(disk,0), sectors[WRITE]);
		case BLOCKTRIG_IO:
			return part_stat_read(disk_get_part(disk,0), sectors[READ])+
				part_stat_read(disk_get_part(disk,0), sectors[WRITE]);
	}
	return 0; /* not reached */
}

static void ledtrig_block_active(struct led_trigger * trigger)
{
	struct block_trigger *entry = container_of(trigger, struct block_trigger, trigger);

	down_write(&active_list_lock);
	entry->data = get_disk_data(entry->disk,entry->type);
	list_add_tail(&entry->active_list, &active_list);
	ledtrig_block_schedule_timer(1);
	up_write(&active_list_lock);
}

static void ledtrig_block_inactive(struct led_trigger * trigger)
{
	struct block_trigger *entry = container_of(trigger, struct block_trigger, trigger);

	down_write(&active_list_lock);
	list_del(&entry->active_list);
	up_write(&active_list_lock);
}

static void __ledtrig_block_add(struct gendisk *disk)
{
	struct block_trigger * entry;
	block_trigger_type_t type;
	static const char _read[] = "-read";
	static const char _write[] = "-write";
	static const char _io[] = "";
	const char * suffix;
	struct device *ddev = disk_to_dev(disk);
	const char * devname = dev_name(ddev);
	
	down_write(&trigger_list_lock);
	for (type = BLOCKTRIG_TYPE_MIN; type <= BLOCKTRIG_TYPE_MAX; type++) {

		entry = kzalloc(sizeof(struct block_trigger), GFP_KERNEL);
		if (!entry) {
			pr_warn("LED triggers for device %s failed to register (no memory)\n",
					devname);
			goto add_end;
		}

		switch (type) {
			case BLOCKTRIG_READ:
				suffix = _read;
				break;
			case BLOCKTRIG_WRITE:
				suffix = _write;
				break;
			case BLOCKTRIG_IO:
				suffix = _io;
				break;
		}

		entry->name = kasprintf(GFP_KERNEL, "%s%s", devname, suffix);
		if (!entry->name) {
			kfree(entry);
			pr_warn("LED triggers for device %s failed to register (no memory)\n",
					devname);
			goto add_end;
		}
		entry->type = type;
		entry->disk = disk;
		entry->trigger.active = &ledtrig_block_active;
		entry->trigger.inactive = &ledtrig_block_inactive;
		led_trigger_register(&entry->trigger);
		list_add_tail(&entry->trigger_list, &trigger_list);
	}
add_end:
	up_write(&trigger_list_lock);
}

static void ledtrig_do_block_del(struct block_trigger * entry)
{
	led_trigger_unregister(&entry->trigger);
	kfree(entry->name);
	list_del(&entry->trigger_list);
	kfree(entry);
}

static void __ledtrig_block_del(struct gendisk *disk)
{
	struct block_trigger * entry, *tmp;
	down_write(&trigger_list_lock);
	list_for_each_entry_safe(entry, tmp, &trigger_list, trigger_list) {
		if (entry->disk == disk) /* multiple entries exist - r/w */
			ledtrig_do_block_del(entry);
	}
	up_write(&trigger_list_lock);
}

static void ledtrig_block_timerfunc(unsigned long data)
{
	struct block_trigger * entry;
	down_read(&active_list_lock);
	ledtrig_block_schedule_timer(0); /*FIXME is timer pending in timer func? */
	list_for_each_entry(entry, &active_list, active_list) {
		sector_t new_data = get_disk_data(entry->disk, entry->type);

		if (entry->data != new_data) {
			entry->data = new_data;
			led_trigger_event(&entry->trigger, LED_FULL);
		} else {
			led_trigger_event(&entry->trigger, LED_OFF);
		}
	}
	up_read(&active_list_lock);
}

static int ledtrig_block_probe(struct platform_device *pdev)
{
	struct class_dev_iter iter;
	struct device *dev;
	device_create_file(&pdev->dev, &dev_attr_trigger_interval);
	mutex_lock(&block_class_lock);
	ledtrig_block_add = __ledtrig_block_add;
	ledtrig_block_del = __ledtrig_block_del;
	class_dev_iter_init(&iter, &block_class, NULL, &disk_type);
	printk("Block device LED trigget init.\nAdding existing devices:");
	while ((dev = class_dev_iter_next(&iter))) {
		struct gendisk *disk = dev_to_disk(dev);
		printk(" %s", dev_name(dev));
		ledtrig_block_add(disk);
	}
	printk(".\n");
	class_dev_iter_exit(&iter);
	mutex_unlock(&block_class_lock);

	return 0;
}

static int __devexit ledtrig_block_remove(struct platform_device *pdev)
{
	mutex_lock(&block_class_lock);
	ledtrig_block_add = NULL;
	ledtrig_block_del = NULL;
	mutex_unlock(&block_class_lock);

	down_write(&trigger_list_lock);
	while (!list_empty(&trigger_list))
		ledtrig_do_block_del(
				list_first_entry(&trigger_list, struct block_trigger, trigger_list));
	up_write(&trigger_list_lock);
	del_timer_sync(&ledtrig_block_timer);
	device_remove_file(&pdev->dev, &dev_attr_trigger_interval);
	return 0;
}

static const struct of_device_id of_ledtrig_block_match[] = {
	{ .compatible = "ledtrig-block", },
	{},
};

static struct platform_driver ledtrig_block_driver = {
	.probe = ledtrig_block_probe,
	.remove = ledtrig_block_remove,
	.driver = {
		.name = "ledtrig-block",
		.owner = THIS_MODULE,
		.of_match_table = of_ledtrig_block_match,
	},
};

module_platform_driver(ledtrig_block_driver);

MODULE_AUTHOR("Michal Suchanek <hramrach@gmail.com>");
MODULE_DESCRIPTION("LED Block Device Activity Trigger\nset led based on block device statistics.");
MODULE_LICENSE("GPL");
