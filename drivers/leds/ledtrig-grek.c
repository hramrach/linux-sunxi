#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/printk.h>

#include "leds.h"

MODULE_AUTHOR("Michal Suchanek <hramrach@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LED trigger aggregator");

static DECLARE_RWSEM(trigger_list_lock);
static LIST_HEAD(trigger_list);
static unsigned ngrek;

typedef struct grek_led_data {
	struct led_classdev  led;
	struct list_head     node;
	int i;
	/* stack for traversing trigger tree to eliminate posibility of cycles */
	/* the statck is protected by trigger list lock in led_classdev */
	int stack_level;
	struct led_trigger ** stack;
} grek_led_data;

typedef enum {
	GREK_OR,
	GREK_AND,
} grek_function;

struct ledtrig_aggregator {
    char * name;
    int nleds;
    grek_function function;
    bool negate;
    struct led_trigger out;
    struct list_head ins;
};

static struct ledtrig_aggregator * greks = NULL;
static struct device * ledtrig_grek_device;

module_param(ngrek, uint, 0444);
MODULE_PARM_DESC(ngrek, "Number of aggregators (1-99)");

static void ledtrig_grek_update(int index)
{
	struct led_classdev * led;
	bool on = 0;
	down_read(&trigger_list_lock);
	if (!list_empty(&greks[index].ins)) {
		switch (greks[index].function) {
			case GREK_AND: on = 1; break;
			case GREK_OR:  on = 0 ; break;
		}
		list_for_each_entry(led, &greks[index].ins, node) {
			bool led_on = (led_get_brightness(led) != LED_OFF);
			switch (greks[index].function) {
				case GREK_AND: on = on && led_on; break;
				case GREK_OR:  on = on || led_on; break;
			}
		}
	}
	up_read(&trigger_list_lock);
	if (greks[index].negate)
		on = ! on;
	led_trigger_event(&greks[index].out, on ? LED_FULL : LED_OFF);
	down_read(&trigger_list_lock);
	up_read(&trigger_list_lock);
}

static void ledtrig_grek_do_quench(struct grek_led_data * data)
{
	list_del(&data->node);
	greks[data->i].nleds--;
	led_classdev_unregister(&data->led);
	kfree(data);
	return;
}

static void ledtrig_grek_quench(struct grek_led_data * data)
{
	int i = data->i;
	down_write(&trigger_list_lock);
	ledtrig_grek_do_quench(data);
	up_write(&trigger_list_lock);
	ledtrig_grek_update(i);
}

static void led_grek_set(struct led_classdev *led, enum led_brightness value)
{
	struct grek_led_data *data = container_of(led, struct grek_led_data, led);
	int i = data->i;
	led->brightness = value;
	ledtrig_grek_update(i);
}

/* note: the trigger passed in may not be set on the led yet */
static int led_grek_check_trig(struct led_classdev *led, struct led_trigger * trig);
static int led_grek_do_check_trig(struct grek_led_data *data, struct led_classdev *led, struct led_trigger * trig)
{
	struct list_head * led_list;
	int i;
	if (!trig)
		return 0;
	
	led_list = &trig->led_cdevs;
	for( i = 0; i < data->stack_level ; i++)
		if(data->stack[i] == trig)
			return -1; /* cycle */
	data->stack[data->stack_level++] = trig;
	list_for_each_entry(led, led_list, node) {
		if (led->set_trigger_hook == led_grek_check_trig) { /* aggregator led */
			read_lock(&led->trigger_lock);
			trig = led->trigger;
			read_lock(&trig->leddev_list_lock);
			if (led_grek_do_check_trig(data, led, trig))
				return -1;
			read_unlock(&trig->leddev_list_lock);
			read_unlock(&led->trigger_lock);
		}
	}
	return 0; /* no issue found */
}

/* prevent agregator cycles - called with ledlist locked*/
static int led_grek_check_trig(struct led_classdev *led, struct led_trigger * trig)
{
	struct grek_led_data *data = container_of(led, struct grek_led_data, led);
	return led_grek_do_check_trig(data, led, trig);
}

static int ledtrig_grek_do_add_led(int i, char * default_trigger)
{
	int ret = -ENOMEM;
	const char * name_format = "%s::led%2i";
	struct grek_led_data * data;
	struct led_classdev *led;

	if (default_trigger) {
		list_for_each_entry(led, &greks[i].ins, node) {
			down_read(&led->trigger_lock);
			if(led->trigger->name == default_trigger)
				return -EEXIST;
			up_read(&led->trigger_lock);
		}
	}

	data = kzalloc(sizeof(struct grek_led_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->led.name = kasprintf(GFP_KERNEL, name_format, greks[i].name, i);
	if (!data->led.name)
		goto err;
	/* The stack can contain only aggregators and 1 other trigger
	 * (aggreagator input) before a trigger repeats so ngrek + 2 should be
	 * more than enough. */
	data->stack = kzalloc(sizeof(struct led_trigger *) * (ngrek + 2),
			GFP_KERNEL);
	data->i = i;
	data->led.brightness_set = led_grek_set;
	data->led.set_trigger_hook = led_grek_check_trig;
	if (list_empty(&greks[i].ins)) /* default to off on fresh aggregator */
		data->led.brightness = greks[i].negate ? LED_FULL : LED_OFF;
	else switch (greks[i].function) {/* choose value that does not disturb function */
		case GREK_AND: data->led.brightness = LED_FULL; break;
		case GREK_OR: data->led.brightness = LED_OFF; break;
	}
	ret = led_classdev_register(ledtrig_grek_device, &data->led);
	if (ret < 0)
		goto err;

	list_add_tail(&data->node, &greks[i].ins);

	greks[i].nleds++;
	return 0;
err:
	kfree(data->stack);
	kfree(data->led.name);
	kfree(data);
	return ret;
}

static int ledtrig_grek_add_led(int i, char * default_trigger)
{
	int ret;
	down_write(&trigger_list_lock);
	ret = ledtrig_grek_do_add_led(i, default_trigger);
	up_write(&trigger_list_lock);
	return ret;
}

static int __devexit ledtrig_grek_remove(struct platform_device *pdev)
{
	unsigned i;
	if ( !greks ) return 0;
	down_write(&trigger_list_lock);
	for (i = 0 ; i < ngrek ; i++) {
		while (greks[i].name && !list_empty(&greks[i].ins))
			ledtrig_grek_do_quench(list_first_entry(&greks[i].ins, grek_led_data, node));
		kfree(greks[i].name);
	}
	kfree(greks);
	ledtrig_grek_device = NULL;
	up_write(&trigger_list_lock);
	return 0;
}

static int ledtrig_grek_probe(struct platform_device *pdev)
{
	unsigned i;
	const char * name_format = "grek%2i";
	if ( !ngrek || ngrek > 99 ) return EINVAL;
	greks = kzalloc(sizeof(struct ledtrig_aggregator) * ngrek, GFP_KERNEL);
	if ( !greks) {
		kfree(greks);
		return -ENOMEM;
	}
	for (i = 0 ; i < ngrek ; i++) {
		int ret;
		int nlen = sprintf(NULL, name_format, i);
		greks[i].name = kzalloc(nlen + 1, GFP_KERNEL);
		if (!greks[i].name) {
			ledtrig_grek_remove(pdev);
			return -ENOMEM;
		}
		sprintf(greks[i].name, name_format, i);
		INIT_LIST_HEAD(&greks[i].ins);
		ret = ledtrig_grek_do_add_led(i, NULL);
		if (ret) {
			ledtrig_grek_remove(pdev);
			return ret;
		}
	}
	ledtrig_grek_device = &pdev->dev;
	return 0;
}

static const struct of_device_id of_ledtrig_grek_match[] = {
	{ .compatible = "ledtrig-grek", },
	{},
};

static struct platform_driver ledtrig_grek_driver = {
	.probe = ledtrig_grek_probe,
	.remove = ledtrig_grek_remove,
	.driver = {
		.name = "ledtrig-grek",
		.owner = THIS_MODULE,
		.of_match_table = of_ledtrig_grek_match,
	},
};

module_platform_driver(ledtrig_grek_driver);

