/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define TMP117_NODE DT_INST(0, zephyr_custom_tmp117)

#if !DT_NODE_EXISTS(TMP117_NODE)
#error "TMP117 device not found in device tree"
#endif

int main(void)
{
	const struct device *tmp117 = DEVICE_DT_GET(TMP117_NODE);
	struct sensor_value temp;
	int ret;
	int retry_count = 0;
	const int max_retries = 10;

	LOG_INF("Waiting for TMP117 device to be ready...");

	/* Wait for device to be ready with retries */
	while (!device_is_ready(tmp117) && retry_count < max_retries) {
		LOG_DBG("Device not ready, retrying... (%d/%d)", retry_count + 1, max_retries);
		k_msleep(100);
		retry_count++;
	}

	if (!device_is_ready(tmp117)) {
		LOG_ERR("TMP117 device is not ready after %d retries", max_retries);
		LOG_ERR("Check I2C connection and device tree configuration");
		return 0;
	}

	LOG_INF("TMP117 sensor initialized. Starting temperature readings...");

	while (1) {
		/* Fetch sensor data */
		ret = sensor_sample_fetch(tmp117);
		if (ret < 0) {
			LOG_ERR("Failed to fetch sensor data: %d", ret);
			k_msleep(500);
			continue;
		}

		/* Get temperature value */
		ret = sensor_channel_get(tmp117, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		if (ret < 0) {
			LOG_ERR("Failed to get temperature: %d", ret);
			k_msleep(500);
			continue;
		}

		/* Print temperature */
		LOG_INF("Temperature: %d.%06d °C", temp.val1, temp.val2);

		/* Sleep for 500ms */
		k_msleep(500);
	}

	return 0;
}

