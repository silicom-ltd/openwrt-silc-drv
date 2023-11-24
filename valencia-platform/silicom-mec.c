/*
 * silicom-mec.c - Silicom MEC17xx ECFW led and hotkeys Driver
 *
 * Copyright (C) 2023 Jeff Daly <jeffd@silicom-usa.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/acpi.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/led-class-multicolor.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/input/sparse-keymap.h>

MODULE_AUTHOR("Jeff Daly");
MODULE_DESCRIPTION("Silicom MEC17xx ECFW LEDs and such");
MODULE_VERSION("1.5");
MODULE_LICENSE("GPL");

#define ACPI_BUTTON_NOTIFY_SWBTN_RELEASE        0xc6
#define ACPI_BUTTON_NOTIFY_SWBTN_PRESSED        0xc5

struct ecfw_button {
	struct input_dev *input;
	char phys[32];
};

static const struct key_entry silicom_array_keymap[] = {
	{ KE_KEY,    0xC5, { KEY_PROG1 } },                /* Press */
	{ KE_KEY,    0xC6, { KEY_PROG1 } },                /* Release */
	{ KE_END },
};

struct ecfw_led {
	int index;
	unsigned int host_owned;
	struct device dev;
	struct led_classdev cdev;
	struct led_classdev_mc mc_cdev;
	struct silicom_data *data;
};

struct ecfw_fan_entry {
	u64 temp;
	u64 rpm;
};

struct silicom_data {
	acpi_handle handle;

	struct ecfw_button *button;
	struct platform_device *platform_device;
	struct acpi_device *acpi_dev;
	struct gpio_chip *gpiochip;
	struct ecfw_led *mc_led_array[4];
	struct ecfw_led *gpio_led_array[12];
	struct mutex lock;

	struct input_dev *input;

	/* set LED brightness */
	int (*brightness_fn)(struct ecfw_led *led);

	/* set multicolor LED brightness */
	int (*multicolor_brightness_fn)(struct ecfw_led *led);
	u8 *gpio_channels;
	u16 ngpio;
};

static const char * const plat_gpio_names[] = {
	"SIM_M2_SLOT1A_DET_N",
	"SIM_M2_SLOT1B_DET_N",
	"SIM_M2_SLOT2A_DET_N",
	"SIM_M2_SLOT2B_DET_N",
	"SIM_M2_SLOT1_MUX_SEL",
	"SIM_M2_SLOT2_MUX_SEL",
	"W_DISABLE_M2_SLOT1_N",
	"W_DISABLE_M2_SLOT2_N",
	"RST_CTL_M2_SLOT1_N",
	"RST_CTL_M2_SLOT2_N",
	"SLOT3_SSD_PWRDIS",
};
static u8 plat_gpio_channels[] = {0,1,2,3,4,5,6,7,8,9,10};

static int silicom_gpio_get_direction(struct gpio_chip *gc, unsigned int offset);
static int silicom_gpio_direction_input(struct gpio_chip *gc, unsigned int offset);
static int silicom_gpio_direction_output(struct gpio_chip *gc, unsigned int offset, int value);
static int silicom_gpio_get(struct gpio_chip *gc, unsigned int offset);
static void silicom_gpio_set(struct gpio_chip *gc, unsigned int offset, int value);
static struct silicom_data *silicom_data_ptr;

static struct gpio_chip silicom_gpio_chip = {
	.label = "silicom-gpio",
	.get_direction = silicom_gpio_get_direction,
	.direction_input = silicom_gpio_direction_input,
	.direction_output = silicom_gpio_direction_output,
	.get = silicom_gpio_get,
	.set = silicom_gpio_set,
	.base = -1,
	.ngpio = ARRAY_SIZE(plat_gpio_channels),
	.names = plat_gpio_names,
	/* We're using a mutex to protect the indirect access, so we can sleep if the lock blocks */
	.can_sleep = true,
};

static int silicom_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	if (offset <= 3)
		return GPIO_LINE_DIRECTION_IN;

	return GPIO_LINE_DIRECTION_OUT;
}

static int silicom_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	int direction = silicom_gpio_get_direction(gc, offset);

	return direction == GPIO_LINE_DIRECTION_IN ? 0 : -EINVAL;
}

static void silicom_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	int direction = silicom_gpio_get_direction(gc, offset);
	acpi_handle handle = acpi_device_handle(silicom_data_ptr->acpi_dev);
	union acpi_object args[] = {
		{ .type = ACPI_TYPE_INTEGER, },
		{ .type = ACPI_TYPE_INTEGER, },
	};
	struct acpi_object_list arg_list = {
		.pointer = args,
		.count = ARRAY_SIZE(args),
	};

	acpi_status status;

	if (direction == GPIO_LINE_DIRECTION_IN)
		return;

	if(!silicom_data_ptr)
		return;

	args[0].integer.value = offset;
	if (value)
		args[1].integer.value |= (0x1 << offset);
	else
		args[1].integer.value = value;

	mutex_lock(&silicom_data_ptr->lock);
	status = acpi_evaluate_object(handle, "LDBM", &arg_list, NULL);

	if (!ACPI_SUCCESS(status))
		pr_err("ACPI_SUCCESS_failed\n");

	mutex_unlock(&silicom_data_ptr->lock);
}

static int silicom_gpio_direction_output(struct gpio_chip *gc, unsigned int offset, int value)
{
	int direction = silicom_gpio_get_direction(gc, offset);

	if (direction == GPIO_LINE_DIRECTION_IN)
		return -EINVAL;

	silicom_gpio_set(gc, offset, value);

	return 0;
}

static int silicom_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	uint8_t value, ret=0;

	if(!silicom_data_ptr)
		return -1;

	mutex_lock(&silicom_data_ptr->lock);
	if (offset <=7) {
		ec_read(0xf2, &value);
		ret = (value >> offset) & 0x1;
	}
	else {
		ec_read(0xf3, &value);
		ret = (value >> (offset -8)) & 0x1;
	}
	mutex_unlock(&silicom_data_ptr->lock);

	return ret;
}

static struct ecfw_led *mcled_cdev_to_led(struct led_classdev_mc *mc_cdev)
{
	return container_of(mc_cdev, struct ecfw_led, mc_cdev);
}

static struct ecfw_led *cdev_to_ecfw_led(struct led_classdev *cdev)
{
	return container_of(cdev, struct ecfw_led, cdev);
}

static struct ecfw_led *dev_to_ecfw_led(struct device *dev)
{
	return cdev_to_ecfw_led(dev_get_drvdata(dev));
}

static struct ecfw_led *mcdev_to_ecfw_led(struct device *dev)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct led_classdev_mc *mcled_cdev = lcdev_to_mccdev(led_cdev);

	return mcled_cdev_to_led(mcled_cdev);
}

static ssize_t mc_host_owned_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ecfw_led *led = mcdev_to_ecfw_led(dev);

	return sprintf(buf, "%u\n", led->host_owned);
}

static ssize_t mc_host_owned_store(struct device *dev,
				   struct device_attribute *host_owned_attr,
				   const char *buf, size_t size)
{
	struct ecfw_led *led = mcdev_to_ecfw_led(dev);
	struct silicom_data *data = led->data;
	union acpi_object arg;
	struct acpi_object_list arg_list;
	acpi_handle handle = acpi_device_handle(data->acpi_dev);
	acpi_status status;
	unsigned long state;
	ssize_t ret;

	mutex_lock(&data->lock);
	ret = kstrtoul(buf, 10, &state);
	if (ret)
		goto out;

	ret = size;
	led->host_owned = state;

	arg.type = ACPI_TYPE_INTEGER;
	arg.integer.value = led->index;
	arg_list.count = 1;
	arg_list.pointer = &arg;
	status = acpi_evaluate_object(handle, "LDSO", &arg_list, NULL);

out:
	mutex_unlock(&data->lock);
	return ret;
}

struct device_attribute dev_attr_mc_host_owned = 
	__ATTR(host_owned, 0644, mc_host_owned_show, mc_host_owned_store);

static struct attribute *ecfw_mc_led_attrs[] = {
	&dev_attr_mc_host_owned.attr,
	NULL,
};
ATTRIBUTE_GROUPS(ecfw_mc_led);

static ssize_t host_owned_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ecfw_led *led = dev_to_ecfw_led(dev);

	return sprintf(buf, "%u\n", led->host_owned);
}

static ssize_t host_owned_store(struct device *dev,
				struct device_attribute *host_owned_attr,
				const char *buf, size_t size)
{

	struct ecfw_led *led = dev_to_ecfw_led(dev);
	struct silicom_data *data = led->data;
	union acpi_object arg;
	struct acpi_object_list arg_list;
	acpi_handle handle = acpi_device_handle(data->acpi_dev);
	acpi_status status;
	unsigned long state;
	ssize_t ret;

	mutex_lock(&data->lock);
	ret = kstrtoul(buf, 10, &state);
	if (ret)
		goto out;

	ret = size;
	led->host_owned = state;
	arg.type = ACPI_TYPE_INTEGER;
	arg.integer.value = led->index;
	arg_list.count = 1;
	arg_list.pointer = &arg;
	status = acpi_evaluate_object(handle, "LDSO", &arg_list, NULL);

out:
	mutex_unlock(&data->lock);
	return ret;
}

static DEVICE_ATTR_RW(host_owned);

static struct attribute *ecfw_led_attrs[] = {
	&dev_attr_host_owned.attr,
	NULL,
};
ATTRIBUTE_GROUPS(ecfw_led);

static int ecfw_set_brightness(struct led_classdev *cdev,
		enum led_brightness brightness)
{
	struct ecfw_led *led = cdev_to_ecfw_led(cdev);
	struct silicom_data *data = led->data;

	union acpi_object args[] = {
		{ .type = ACPI_TYPE_INTEGER, },
		{ .type = ACPI_TYPE_INTEGER, },
	};
	struct acpi_object_list arg_list = {
		.pointer = args,
		.count = ARRAY_SIZE(args),
	};

	acpi_handle handle = acpi_device_handle(data->acpi_dev);
	acpi_status status;

	mutex_lock(&data->lock);

	args[0].integer.value = led->index;
	args[1].integer.value = brightness;

	status = acpi_evaluate_object(handle, "LDBL", &arg_list, NULL);
	mutex_unlock(&data->lock);
	if (!ACPI_SUCCESS(status)) {
		return -1;
	} else {
		return 0;
	}
}

static int ecfw_set_mc_brightness(struct led_classdev *cdev,
		enum led_brightness brightness)
{
	struct led_classdev_mc *mc_cdev = lcdev_to_mccdev(cdev);
	struct ecfw_led *led = mcled_cdev_to_led(mc_cdev);
	struct silicom_data *data = led->data;

	union acpi_object args[] = {
		{ .type = ACPI_TYPE_INTEGER, },
		{ .type = ACPI_TYPE_INTEGER, },
	};
	struct acpi_object_list arg_list = {
		.pointer = args,
		.count = ARRAY_SIZE(args),
	};

	acpi_handle handle = acpi_device_handle(data->acpi_dev);
	acpi_status status;
	uint32_t rgb_color;

	mutex_lock(&data->lock);

	rgb_color = led->mc_cdev.subled_info[0].intensity << 16 |
		led->mc_cdev.subled_info[1].intensity << 8 |
		led->mc_cdev.subled_info[2].intensity;

	args[0].integer.value = led->index;
	args[1].integer.value = rgb_color;

	/* first, set the 'intensity' */
	status = acpi_evaluate_object(handle, "LDCS", &arg_list, NULL);
	if (!ACPI_SUCCESS(status)) {
		mutex_unlock(&data->lock);
		return -1;
	}

	args[1].integer.value = 0;
	args[1].integer.value = brightness;
	status = acpi_evaluate_object(handle, "LDBL", &arg_list, NULL);
	mutex_unlock(&data->lock);
	if (ACPI_SUCCESS(status)) {
		return 0;
	}
	else {
		return -1;
	}
}

static int ecfw_set_mc_blink(struct led_classdev *cdev,
			     unsigned long *delay_on,
			     unsigned long *delay_off)
{
	struct led_classdev_mc *mc_cdev = lcdev_to_mccdev(cdev);
	struct ecfw_led *led = mcled_cdev_to_led(mc_cdev);
	struct silicom_data *data = led->data;

	union acpi_object args[] = {
		{ .type = ACPI_TYPE_INTEGER, },
		{ .type = ACPI_TYPE_INTEGER, },
	};
	struct acpi_object_list arg_list = {
		.pointer = args,
		.count = ARRAY_SIZE(args),
	};

	acpi_handle handle = acpi_device_handle(data->acpi_dev);
	acpi_status status;
	uint32_t rgb_color;

	mutex_lock(&data->lock);

	rgb_color = led->mc_cdev.subled_info[0].intensity << 16 |
		led->mc_cdev.subled_info[1].intensity << 8 |
		led->mc_cdev.subled_info[2].intensity;

	args[0].integer.value = led->index;
	args[1].integer.value = rgb_color;

	/* first, set the 'intensity' */
	status = acpi_evaluate_object(handle, "LDCS", &arg_list, NULL);
	if (!ACPI_SUCCESS(status)) {
		mutex_unlock(&data->lock);
		return -1;
	}

	args[0].integer.value = led->index;
	args[1].integer.value = ((*delay_off & 0xFFFF) << 16) | (*delay_on & 0xFFFF);

	status = acpi_evaluate_object(handle, "LDBS", &arg_list, NULL);
	mutex_unlock(&data->lock);
	if (!ACPI_SUCCESS(status)) {
		return -1;
	} else {
		return 0;
	}
}
/*
 * RGB leds will end up being called rgb_x where x is numbers starting at 0
 * and counting up to as many as there are.  GPIO leds will end up similarly
 * being called gpio_x.  Which are which is dependent upon the order BIOS 
 * returns the leds.
 */
static int ecfw_init_led(struct device *dev, struct ecfw_led *led, int num_colors, int unit)
{
	struct mc_subled *mc_led_info;
	struct led_classdev *led_cdev;
	int ret;

	led->index = unit;
	if (num_colors > 1) {
		mc_led_info = devm_kcalloc(dev, num_colors, sizeof(*mc_led_info), GFP_KERNEL);

		if (!mc_led_info)
			return -ENOMEM;
		
		led_cdev = &led->mc_cdev.led_cdev;
		led_cdev->name = "rgb";
		led_cdev->brightness_set_blocking = ecfw_set_mc_brightness;
		led_cdev->blink_set = ecfw_set_mc_blink;
		led_cdev->max_brightness = 255;

		led->mc_cdev.num_colors = num_colors;
		mc_led_info[0].color_index = LED_COLOR_ID_RED;
		mc_led_info[1].color_index = LED_COLOR_ID_GREEN;
		mc_led_info[2].color_index = LED_COLOR_ID_BLUE;

		led->mc_cdev.subled_info = mc_led_info;

		ret = devm_led_classdev_multicolor_register(dev, &led->mc_cdev);

		ret = device_add_groups(led_cdev->dev, ecfw_mc_led_groups);
	} else {
		led_cdev = &led->cdev;
		led_cdev->name = "gpio";
		led_cdev->max_brightness = 100;
		led->cdev.brightness_set_blocking = ecfw_set_brightness;
		led_cdev->groups = ecfw_led_groups;

		ret = devm_led_classdev_register(dev, &led->cdev);
	}


	if (ret) {
		dev_err(dev, "led register err: %d\n", ret);
		return ret;
	}

	return 0;
}

static void ecfw_remove_led_files(struct silicom_data *data)
{
	struct ecfw_led *(*led)[4] = &data->mc_led_array;
	struct led_classdev *led_cdev;
	int i;

	for (i = 0; i < 3; i++) {
		led_cdev = &(*led)[i]->mc_cdev.led_cdev;
		device_remove_groups(led_cdev->dev, ecfw_mc_led_groups);
	}
}

static inline int ecfw_pwm_to_lmsensors(int value)
{
	return value * 255 / 100;
}

static inline int ecfw_lmsensors_to_pwm(int value)
{
	value = clamp_val(value, 0, 100);
//	value = clamp_val(value, 0, 255);
//	return value * 100 / 255;
	return value;
}

static ssize_t fan_input_show(struct device *dev, struct device_attribute *devattr,
			       char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);

	u8 high = 0;
	u8 low = 0;

	if (attr->index == 0) {
		ec_read(0x73, &low);
		ec_read(0x74, &high);
	} else {
		ec_read(0x75, &low);
		ec_read(0x76, &high);
	}

	return sprintf(buf, "%d\n", (high << 8 | low));
}

static ssize_t fan_status_show(struct device *dev, struct device_attribute *devattr,
				char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct silicom_data *data = dev_get_drvdata(dev);
	acpi_handle handle = acpi_device_handle(data->acpi_dev);
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	struct acpi_buffer format = { sizeof("NN"), "NN" };
	union acpi_object *obj;
	struct ecfw_fan_entry fan;
	struct acpi_buffer fanentry = { sizeof(fan),  &fan };
	acpi_status status;

	status = acpi_evaluate_object(handle, "FRTS", NULL, &buffer);
	if (!ACPI_FAILURE(status)) {
		obj = buffer.pointer;
		if (!obj || obj->type != ACPI_TYPE_PACKAGE || obj->package.count != 2) {
			dev_err(handle, "Invalid FRTS");
			goto out;
		}

		status = acpi_extract_package(&obj->package.elements[1], &format, &fanentry);

		if (ACPI_FAILURE(status)) {
			dev_err(handle, "Invalid FRTS Entry");
			goto out;
		} else {
			dev_err(handle, "Temperature: %d", (int)fan.temp);
			dev_err(handle, "RPM: %d", (int)fan.rpm);
		}
	}
out:
	return sprintf(buf, "%d\n", attr->index);
}

static SENSOR_DEVICE_ATTR_RO(fan1_input, fan_input, 0);
static SENSOR_DEVICE_ATTR_RO(fan2_input, fan_input, 1);
static SENSOR_DEVICE_ATTR_RO(fan1_status, fan_status, 0);
static SENSOR_DEVICE_ATTR_RO(fan2_status, fan_status, 1);

static struct attribute *ecfw_fan_attrs[] = {
	&sensor_dev_attr_fan1_input.dev_attr.attr,
	&sensor_dev_attr_fan1_status.dev_attr.attr,
	&sensor_dev_attr_fan2_input.dev_attr.attr,
	&sensor_dev_attr_fan2_status.dev_attr.attr,
	NULL
};

static const struct attribute_group ecfw_fan_group = {
	.attrs = ecfw_fan_attrs,
};

static ssize_t pwm_show(struct device *dev, struct device_attribute *devattr,
			char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	uint8_t value;

	switch(attr->index) {
	case 0:
		ec_read(0x3D, &value);
		break;
	case 1:
		ec_read(0x3E, &value);
		break;
	default:
		return 0;
	}
	return sprintf(buf, "%u\n", value);
}

static ssize_t pwm_store(struct device *dev, struct device_attribute *devattr,
			  const char *buf, size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct silicom_data *data = dev_get_drvdata(dev);
	long val;
	uint8_t enable;
	int err;

	err = kstrtol(buf, 10, &val);
	if (err)
		return err;

//	val = ecfw_lmsensors_to_pwm(val);

	/* TBD, may interfere with \_TZ operations */
	mutex_lock(&data->lock);
	ec_read(0x4d, &enable);
	ec_write(0x4d, enable|2);
	ec_write(0x41, attr->index);
	ec_write(0x43, (uint8_t)(val >> 8));
	ec_write(0x44, (uint8_t)val);
	ec_write(0x3A, 0x1A);		/* update PWM cmd */
	mutex_unlock(&data->lock);

	return count;
}

static SENSOR_DEVICE_ATTR_RW(pwm1, pwm, 0);
static SENSOR_DEVICE_ATTR_RW(pwm2, pwm, 1);

static struct attribute *ecfw_pwm_attrs[] = {
	&sensor_dev_attr_pwm1.dev_attr.attr,
	&sensor_dev_attr_pwm2.dev_attr.attr,
	NULL
};

static const struct attribute_group ecfw_pwm_group = {
	.attrs = ecfw_pwm_attrs,
};

static ssize_t temp_show(struct device *dev, struct device_attribute *devattr,
			 char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct silicom_data *data = dev_get_drvdata(dev);
	uint16_t value;
	uint8_t val;

	mutex_lock(&data->lock);
	switch(attr->index) {
	case 0:
		ec_read(0x93, &val);
		value = val;
		ec_read(0x94, &val);
		value |= val << 8;
		break;
	case 1:
		ec_read(0x95, &val);
		value = val;
		ec_read(0x96, &val);
		value |= val << 8;
		break;
	case 2:
		ec_read(0x97, &val);
		value = val;
		ec_read(0x98, &val);
		value |= val << 8;
		break;
	case 3:
		ec_read(0x99, &val);
		value = val;
		ec_read(0x9A, &val);
		value |= val << 8;
		break;
	default:
		pr_err("Unknown temp input index\n");
		mutex_unlock(&data->lock);
		return 0;
	}
	mutex_unlock(&data->lock);
	return sprintf(buf, "%u\n", value*1000);

}

static SENSOR_DEVICE_ATTR_RO(temp1, temp,  0);
static SENSOR_DEVICE_ATTR_RO(temp2, temp,  1);
static SENSOR_DEVICE_ATTR_RO(temp3, temp,  2);
static SENSOR_DEVICE_ATTR_RO(temp4, temp,  3);

static struct attribute *ecfw_temp_attrs[] = {
	&sensor_dev_attr_temp1.dev_attr.attr,
	&sensor_dev_attr_temp2.dev_attr.attr,
	&sensor_dev_attr_temp3.dev_attr.attr,
	&sensor_dev_attr_temp4.dev_attr.attr,
	NULL
};

static const struct attribute_group ecfw_temp_group = {
	.attrs = ecfw_temp_attrs,
};

static const struct attribute_group *ecfw_groups[] = {
	&ecfw_fan_group,
	&ecfw_pwm_group,
	&ecfw_temp_group,
	NULL
};

static int silicom_hwmon_init(struct silicom_data *data)
{
	struct device *dev = &data->platform_device->dev;
	struct device *hwmon;

	hwmon = devm_hwmon_device_register_with_groups(dev, "ecfw", data,
							ecfw_groups);
	if (IS_ERR(hwmon)) {
		pr_err("Couldn't register ecfw hwmon device\n");
		return PTR_ERR(hwmon);
	}
	return 0;
}

enum event_types {
	HOTKEY = 1
};

//static void acpi_notify(acpi_handle handle, u32 event, void *data)
static void acpi_notify(struct acpi_device *device,  u32 event)
{
	u8 ev_type = HOTKEY;

	switch(event) {
	case ACPI_BUTTON_NOTIFY_SWBTN_PRESSED:
		break;
	case ACPI_BUTTON_NOTIFY_SWBTN_RELEASE:
		break;
	default:
		pr_err("silicom_mec unknown notify event: 0x%x\n",event);
	}
	acpi_bus_generate_netlink_event(device->pnp.device_class,
			dev_name(&device->dev), ev_type, event);
}

static int silicom_gpio_chip_init(struct silicom_data *data)
{
	int err;
	struct device *dev = &data->platform_device->dev;


	err = devm_gpiochip_add_data(dev, &silicom_gpio_chip, &plat_gpio_channels);
	if (err) {
		dev_err(dev, "Failed to register gpiochip: %d\n", err);
		return err;
	}

	data->gpiochip = &silicom_gpio_chip;
	data->gpio_channels = plat_gpio_channels;
	data->ngpio = silicom_gpio_chip.ngpio;
    silicom_data_ptr = data;

	return 0;
}

static int ecfw_platform_init(struct silicom_data *data)
{
	int ret;

	data->platform_device = platform_device_alloc("ecfw", -1);
	if (!data->platform_device)
		return -ENOMEM;
	platform_set_drvdata(data->platform_device, data);

	ret = platform_device_add(data->platform_device);
	if (ret)
		goto fail_platform_device;

	ret = silicom_gpio_chip_init(data);
	if (ret)
		goto fail_platform_device;

	return 0;

fail_platform_device:
	platform_device_put(data->platform_device);
	return ret;
}

static void ecfw_platform_exit(struct silicom_data *data)
{
	platform_device_unregister(data->platform_device);
	gpiochip_remove(data->gpiochip);
}

static ssize_t rstbtn_soft_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct silicom_data *data = dev_get_drvdata(dev);
	acpi_handle handle = acpi_device_handle(data->acpi_dev);
	acpi_status status;
	unsigned long long value;

	if (acpi_has_method(handle, "RSTG")) {
		status = acpi_evaluate_integer(handle, "RSTG", NULL, &value);
		if (ACPI_FAILURE(status)) {
			value = -1;
		}
	} else {
		pr_err("BIOS does not support soft reset button, please update your BIOS");
		return -ENODEV;
	}

	return sprintf(buf, "%u\n", (int)value);
}

static ssize_t rstbtn_soft_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct silicom_data *data = dev_get_drvdata(dev);
	acpi_handle handle = acpi_device_handle(data->acpi_dev);
	int err,val;

	err = kstrtoint(buf, 0, &val);

	if (err)
		return err;

	if (val == 0 || val == 1) {
		if (acpi_has_method(handle, "RSTB")) {
			acpi_execute_simple_method(handle, "RSTB", val);
		} else {
			pr_err("BIOS does not support soft reset button, please update your BIOS");
			return -ENODEV;
		}
	}

	return count;
}

static ssize_t rstbtn_status_show(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	struct silicom_data *data = dev_get_drvdata(dev);
	acpi_handle handle = acpi_device_handle(data->acpi_dev);
	acpi_status status;
	unsigned long long value;

	if (acpi_has_method(handle, "RSTS")) {
		status = acpi_evaluate_integer(handle, "RSTS", NULL, &value);
		if (ACPI_FAILURE(status)) {
			value = -1;
		}
	} else {
	       pr_err("BIOS does not have reset button status method, please upgrade your BIOS");
	       value = -1;
	}

	return sprintf(buf, "%u\n", (int)value);
}

static DEVICE_ATTR_RW(rstbtn_soft);
static DEVICE_ATTR_RO(rstbtn_status);

static struct attribute *rstbtn_sysfs_entries[] = {
	&dev_attr_rstbtn_soft.attr,
	&dev_attr_rstbtn_status.attr,
	NULL,
};

static const struct attribute_group rstbtn_attr_group = {
	.name 	= NULL,
	.attrs 	= rstbtn_sysfs_entries,
};

static int acpi_add(struct acpi_device *acpi_dev)
{
	int ret;
	struct silicom_data *data;
	struct ecfw_led *led;
	struct ecfw_button *button;
	struct input_dev *input;
	struct device *dev = &acpi_dev->dev;
	acpi_handle handle;
	acpi_status status;
	int i;

	data = devm_kzalloc(&acpi_dev->dev, sizeof(*data), GFP_KERNEL);

	if (!data)
		return -ENOMEM;

	acpi_dev->driver_data = data;
	data->acpi_dev = acpi_dev;
	dev_set_drvdata(&acpi_dev->dev, data);

	ret = ecfw_platform_init(data);
	if (ret)
		goto fail_platform;

	ret = silicom_hwmon_init(data);
	if (ret)
		goto fail_hwmon;

	for (i = 0; i < 3; i++) {
		led = devm_kzalloc(dev, sizeof(*led), GFP_KERNEL);
		ret = ecfw_init_led(dev, led, 3, i);

		if (ret) 
			goto out;

		data->mc_led_array[i] = led;
		led->data = data;
	}

	for (i = 3; i < 6; i++) {
		led = devm_kzalloc(dev, sizeof(*led), GFP_KERNEL);
		ret = ecfw_init_led(dev, led, 1, i);

		if (ret)
			goto out;

		data->gpio_led_array[i-3] = led;
		led->data = data;
	}

	mutex_init(&data->lock);

	/*
	 * setup button as an input device
	 */
	button = devm_kzalloc(dev, sizeof(*button), GFP_KERNEL);
	if (!button)
		goto fail_input;

	button->input = input = devm_input_allocate_device(dev);

	if (!input) {
		ret = -ENOMEM;
		goto out;
	}

	data->button = button;

	snprintf(button->phys, sizeof(button->phys), "%s/button/input0", "ecfw");

	ret = sparse_keymap_setup(input, silicom_array_keymap, NULL);
	input->name = "ECFW Recessed Button";
	input->phys = button->phys;
	input->id.bustype = BUS_HOST;
	input->dev.parent = dev;

	if (ret)
		goto out;

	ret = input_register_device(input);
	if (ret)
		goto out;

	handle = acpi_device_handle(data->acpi_dev);
	if (acpi_has_method(handle, "RSTB")) {
		ret = sysfs_create_group(&acpi_dev->dev.kobj, &rstbtn_attr_group);

		if (ret)
			goto out;

		acpi_execute_simple_method(handle, "RSTB", 1);
	}

	return 0;

	input_free_device(input);
out:
fail_input:
	pr_err("Unable to allocate Silicom MEC172x input device\n");
fail_hwmon:
	pr_err("Unable to install Silicom MEC172x driver\n");
fail_platform:
	pr_err("Unable to allocate Silicom MEC172x platform device\n");
	return ret;
}

static int acpi_remove(struct acpi_device *acpi_dev)
{
	struct silicom_data *data;

	data = acpi_driver_data(acpi_dev);

	ecfw_remove_led_files(data);

	ecfw_platform_exit(data);

	mutex_destroy(&data->lock);

	return 0;
}

static const struct acpi_device_id device_ids[] = {
	{"SILC1001", 0},
	{"", 0},
};
MODULE_DEVICE_TABLE(acpi, device_ids);

static struct acpi_driver silicom_driver = {
	.name = "Silicom MEC17xx ECFW support",
	.class = "silicom-mec",
	.owner = THIS_MODULE,
	.ids = device_ids,
	.flags = ACPI_DRIVER_ALL_NOTIFY_EVENTS,
	.ops = {
		.add = acpi_add,
		.remove = acpi_remove,
		.notify = acpi_notify,
	},
};

module_acpi_driver(silicom_driver);
