/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define GREEN_LED_NODE DT_ALIAS(greenled)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec green_led = GPIO_DT_SPEC_GET(GREEN_LED_NODE, gpios);

const struct device *const lis3mdl = DEVICE_DT_GET_ONE(st_lis3mdl_magn);
const struct device *const bme680 = DEVICE_DT_GET_ONE(bosch_bme680);

int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&green_led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&green_led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	
	struct sensor_value temp, press, humidity, gas_res;

	if (!device_is_ready(bme680)) {
		printk("sensor: device not ready.\n");
		return 0;
	}

	printf("Device %p name is %s\n", bme680, bme680->name);

	struct sensor_value magn_xyz[3];

	if (!device_is_ready(lis3mdl)) {
		printk("%s: device not ready.\n", lis3mdl->name);
		return 0;
	}

	printf("Device %p name is %s\n", lis3mdl, lis3mdl->name);


	while (1) {
		ret = gpio_pin_toggle_dt(&green_led);
		if (ret < 0) {
			return 0;
		}

		sensor_sample_fetch(bme680);
		sensor_channel_get(bme680, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(bme680, SENSOR_CHAN_PRESS, &press);
		sensor_channel_get(bme680, SENSOR_CHAN_HUMIDITY, &humidity);
		sensor_channel_get(bme680, SENSOR_CHAN_GAS_RES, &gas_res);

		printf("T: %d.%06d; P: %d.%06d; H: %d.%06d; G: %d.%06d\n",
				temp.val1, temp.val2, press.val1, press.val2,
				humidity.val1, humidity.val2, gas_res.val1,
				gas_res.val2);

		if (sensor_sample_fetch(lis3mdl) < 0) {
			printf("LIS3MDL Sensor sample update error\n");
			return 0;
		}

		sensor_channel_get(lis3mdl, SENSOR_CHAN_MAGN_XYZ, magn_xyz);

		printf(
		 "LIS3MDL: Magnetic field (gauss): x: %.1f, y: %.1f, z: %.1f\n",
		 sensor_value_to_double(&magn_xyz[0]),
		 sensor_value_to_double(&magn_xyz[1]),
		 sensor_value_to_double(&magn_xyz[2]));

		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
