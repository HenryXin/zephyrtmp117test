/*
* Copyright (c) 2019 Centaur Analytics, Inc
*
* SPDX-License-Identifier: Apache-2.0
*/

#define DT_DRV_COMPAT zephyr_custom_tmp117

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <sys/types.h>

#include "custom_tmp117.h"

#define EEPROM_SIZE_REG        sizeof(uint16_t)
#define EEPROM_TMP117_RESERVED (2 * sizeof(uint16_t))
#define EEPROM_TMP117_SIZE     (4 * sizeof(uint16_t))  /* 4 EEPROM registers: EEPROM1-EEPROM4 */
#define EEPROM_MIN_BUSY_MS     7
#define RESET_MIN_BUSY_MS      2

LOG_MODULE_REGISTER(TMP117, CONFIG_SENSOR_LOG_LEVEL);

int tmp117_reg_read(const struct device *dev, uint8_t reg, uint16_t *val)
{
   const struct tmp117_dev_config *cfg = dev->config;

   if (i2c_burst_read_dt(&cfg->bus, reg, (uint8_t *)val, 2) < 0) {
       return -EIO;
   }

   *val = sys_be16_to_cpu(*val);

   return 0;
}

int tmp117_reg_write(const struct device *dev, uint8_t reg, uint16_t val)
{
   const struct tmp117_dev_config *cfg = dev->config;
   uint8_t tx_buf[3] = {reg, val >> 8, val & 0xFF};

   return i2c_write_dt(&cfg->bus, tx_buf, sizeof(tx_buf));
}

int tmp117_write_config(const struct device *dev, uint16_t mask, uint16_t conf)
{
   uint16_t config = 0;
   int result;

   result = tmp117_reg_read(dev, TMP117_REG_CFGR, &config);

   if (result < 0) {
       return result;
   }

   config &= ~mask;
   config |= conf;

   return tmp117_reg_write(dev, TMP117_REG_CFGR, config);
}

static inline bool tmp117_is_offset_supported(const struct tmp117_data *drv_data)
{
   return drv_data->id == TMP117_DEVICE_ID || drv_data->id == TMP119_DEVICE_ID;
}

/**
* @brief Convert sensor_value temperature to TMP117 register format
*
* This function converts a temperature from sensor_value format (val1 in degrees C,
* val2 in micro-degrees C) to the TMP117 register format. It uses 64-bit arithmetic
* to prevent overflow and clamps the result to the valid int16_t range.
*
* @param val Pointer to sensor_value containing temperature
* @return Temperature value in TMP117 register format (int16_t)
*/
static inline int16_t tmp117_sensor_value_to_reg_format(const struct sensor_value *val)
{
   int64_t temp_micro = ((int64_t)val->val1 * 1000000) + val->val2;
   int64_t temp_scaled = (temp_micro * 10) / TMP117_RESOLUTION;

   /* Clamp to int16_t range */
   if (temp_scaled > INT16_MAX) {
       return INT16_MAX;
   } else if (temp_scaled < INT16_MIN) {
       return INT16_MIN;
   } else {
       return (int16_t)temp_scaled;
   }
}

static bool check_eeprom_bounds(const struct device *dev, off_t offset, size_t len)
{
   struct tmp117_data *drv_data = dev->data;

   if ((offset + len) > EEPROM_TMP117_SIZE || offset % EEPROM_SIZE_REG != 0 ||
       len % EEPROM_SIZE_REG != 0) {
       return false;
   }

   /* TMP117 and TMP119 uses EEPROM[2] as temperature offset register */
   if ((drv_data->id == TMP117_DEVICE_ID || drv_data->id == TMP119_DEVICE_ID) &&
       offset <= EEPROM_TMP117_RESERVED && (offset + len) > EEPROM_TMP117_RESERVED) {
       return false;
   }

   return true;
}

int tmp117_eeprom_await(const struct device *dev)
{
   int res;
   uint16_t val;

   k_sleep(K_MSEC(EEPROM_MIN_BUSY_MS));

   WAIT_FOR((res = tmp117_reg_read(dev, TMP117_REG_EEPROM_UL, &val)) != 0 ||
            val & TMP117_EEPROM_UL_BUSY,
        100, k_msleep(1));

   return res;
}

int tmp117_eeprom_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
   uint8_t reg;
   const uint16_t *src = data;
   int res;

   if (!check_eeprom_bounds(dev, offset, len)) {
       return -EINVAL;
   }

   res = tmp117_reg_write(dev, TMP117_REG_EEPROM_UL, TMP117_EEPROM_UL_UNLOCK);
   if (res) {
       return res;
   }

   for (reg = (offset / 2); reg < offset / 2 + len / 2; reg++) {
       uint16_t val = *src;

       res = tmp117_reg_write(dev, reg + TMP117_REG_EEPROM1, val);
       if (res != 0) {
           break;
       }

       res = tmp117_eeprom_await(dev);
       src++;

       if (res != 0) {
           break;
       }
   }

   res = tmp117_reg_write(dev, TMP117_REG_EEPROM_UL, 0);

   return res;
}

int tmp117_eeprom_read(const struct device *dev, off_t offset, void *data, size_t len)
{
   uint8_t reg;
   uint16_t *dst = data;
   int res = 0;

   if (!check_eeprom_bounds(dev, offset, len)) {
       return -EINVAL;
   }

   for (reg = (offset / 2); reg < offset / 2 + len / 2; reg++) {
       res = tmp117_reg_read(dev, reg + TMP117_REG_EEPROM1, dst);
       if (res != 0) {
           break;
       }
       dst++;
   }

   return res;
}

/**
* @brief Check the Device ID
*
* @param[in]   dev     Pointer to the device structure
* @param[in]	id	Pointer to the variable for storing the device id
*
* @retval 0 on success
* @retval -EIO Otherwise
*/
static inline int tmp117_device_id_check(const struct device *dev, uint16_t *id)
{
   if (tmp117_reg_read(dev, TMP117_REG_DEVICE_ID, id) != 0) {
       LOG_ERR("%s: Failed to get Device ID register!", dev->name);
       return -EIO;
   }

   if ((*id != TMP116_DEVICE_ID) && (*id != TMP117_DEVICE_ID) && (*id != TMP119_DEVICE_ID)) {
       LOG_ERR("%s: Failed to match the device IDs!", dev->name);
       return -EINVAL;
   }

   return 0;
}

static int tmp117_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
   struct tmp117_data *drv_data = dev->data;
   uint16_t value;
   uint16_t cfg_reg = 0;
   int rc;

   __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_AMBIENT_TEMP);

   /* clear sensor values */
   drv_data->sample = 0U;

   /* Make sure that a data is available */
   rc = tmp117_reg_read(dev, TMP117_REG_CFGR, &cfg_reg);
   if (rc < 0) {
       LOG_ERR("%s, Failed to read from CFGR register", dev->name);
       return rc;
   }

   if ((cfg_reg & TMP117_CFGR_DATA_READY) == 0) {
       LOG_DBG("%s: no data ready", dev->name);
       return -EBUSY;
   }

   /* Get the most recent temperature measurement */
   rc = tmp117_reg_read(dev, TMP117_REG_TEMP, &value);
   if (rc < 0) {
       LOG_ERR("%s: Failed to read from TEMP register!", dev->name);
       return rc;
   }

   /* store measurements to the driver */
   drv_data->sample = (int16_t)value;

   return 0;
}

/*
* See datasheet "Temperature Results and Limits" section for more
* details on processing sample data.
*/
static void tmp117_temperature_to_sensor_value(int16_t temperature, struct sensor_value *val)
{
   int32_t tmp;

   tmp = (temperature * (int32_t)TMP117_RESOLUTION) / 10;
   val->val1 = tmp / 1000000; /* uCelsius */
   val->val2 = tmp % 1000000;
}

static int tmp117_channel_get(const struct device *dev, enum sensor_channel chan,
                 struct sensor_value *val)
{
   struct tmp117_data *drv_data = dev->data;

   if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
       return -ENOTSUP;
   }

   tmp117_temperature_to_sensor_value(drv_data->sample, val);

   return 0;
}

static int16_t tmp117_conv_value(const struct sensor_value *val)
{
   uint32_t freq_micro = sensor_value_to_micro(val);

   switch (freq_micro) {
   case 64000000: /* 1 / 15.5 ms has been rounded down */
       return TMP117_DT_ODR_15_5_MS;
   case 8000000:
       return TMP117_DT_ODR_125_MS;
   case 4000000:
       return TMP117_DT_ODR_250_MS;
   case 2000000:
       return TMP117_DT_ODR_500_MS;
   case 1000000:
       return TMP117_DT_ODR_1000_MS;
   case 250000:
       return TMP117_DT_ODR_4000_MS;
   case 125000:
       return TMP117_DT_ODR_8000_MS;
   case 62500:
       return TMP117_DT_ODR_16000_MS;
   default:
       LOG_ERR("%" PRIu32 " uHz not supported", freq_micro);
       return -EINVAL;
   }
}

static bool tmp117_is_attr_store_supported(enum sensor_attribute attr)
{
   switch ((int)attr) {
   case SENSOR_ATTR_SAMPLING_FREQUENCY:
   case SENSOR_ATTR_LOWER_THRESH:
   case SENSOR_ATTR_UPPER_THRESH:
   case SENSOR_ATTR_OFFSET:
   case SENSOR_ATTR_OVERSAMPLING:
   case SENSOR_ATTR_TMP117_SHUTDOWN_MODE:
   case SENSOR_ATTR_TMP117_CONTINUOUS_CONVERSION_MODE:
   case SENSOR_ATTR_TMP117_ALERT_PIN_POLARITY:
   case SENSOR_ATTR_TMP117_ALERT_MODE:
       return true;
   case SENSOR_ATTR_TMP117_ONE_SHOT_MODE:
       return false;
   }

   return false;
}

static int tmp117_attr_store_reload(const struct device *dev)
{
   int await_res = tmp117_eeprom_await(dev);
   int reset_res = tmp117_reg_write(dev, TMP117_REG_CFGR, TMP117_CFGR_RESET);

   k_sleep(K_MSEC(RESET_MIN_BUSY_MS));

   return await_res != 0 ? await_res : reset_res;
}

static int tmp117_attr_set(const struct device *dev, enum sensor_channel chan,
              enum sensor_attribute attr, const struct sensor_value *val)
{
   const struct tmp117_dev_config *cfg = dev->config;
   struct tmp117_data *drv_data = dev->data;
   int16_t value;
   int res = 0;
   bool store;
   int store_res = 0;

   if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
       return -ENOTSUP;
   }

   store = cfg->store_attr_values && tmp117_is_attr_store_supported(attr);
   if (store) {
       store_res = tmp117_reg_write(dev, TMP117_REG_EEPROM_UL, TMP117_EEPROM_UL_UNLOCK);
       if (store_res != 0) {
           return store_res;
       }
   }

   switch ((int)attr) {
   case SENSOR_ATTR_SAMPLING_FREQUENCY:
       value = tmp117_conv_value(val);
       if (value < 0) {
           return value;
       }

       res = tmp117_write_config(dev, TMP117_CFGR_CONV, value);
       break;

   case SENSOR_ATTR_OFFSET:
       if (!tmp117_is_offset_supported(drv_data)) {
           LOG_ERR("%s: Offset is not supported", dev->name);
           return -EINVAL;
       }
       /*
        * The offset is encoded into the temperature register format.
        */
       value = tmp117_sensor_value_to_reg_format(val);

       res = tmp117_reg_write(dev, TMP117_REG_TEMP_OFFSET, value);
       break;

   case SENSOR_ATTR_OVERSAMPLING:
       /* sensor supports averaging 1, 8, 32 and 64 samples */
       switch (val->val1) {
       case 1:
           value = TMP117_AVG_1_SAMPLE;
           break;

       case 8:
           value = TMP117_AVG_8_SAMPLES;
           break;

       case 32:
           value = TMP117_AVG_32_SAMPLES;
           break;

       case 64:
           value = TMP117_AVG_64_SAMPLES;
           break;

       default:
           res = -EINVAL;
           break;
       }

       if (res == 0) {
           res = tmp117_write_config(dev, TMP117_CFGR_AVG, value);
       }

       break;

   case SENSOR_ATTR_TMP117_SHUTDOWN_MODE:
       res = tmp117_write_config(dev, TMP117_CFGR_MODE, TMP117_MODE_SHUTDOWN);
       break;

   case SENSOR_ATTR_TMP117_CONTINUOUS_CONVERSION_MODE:
       res = tmp117_write_config(dev, TMP117_CFGR_MODE, TMP117_MODE_CONTINUOUS);
       break;

   case SENSOR_ATTR_TMP117_ONE_SHOT_MODE:
       res = tmp117_write_config(dev, TMP117_CFGR_MODE, TMP117_MODE_ONE_SHOT);
       break;


   default:
       res = -ENOTSUP;
       break;
   }

   if (store) {
       store_res = tmp117_attr_store_reload(dev);
   }

   return res != 0 ? res : store_res;
}

static int tmp117_attr_get(const struct device *dev, enum sensor_channel chan,
              enum sensor_attribute attr, struct sensor_value *val)
{
   uint16_t data;
   int rc;

   if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
       return -ENOTSUP;
   }

   switch (attr) {
   case SENSOR_ATTR_CONFIGURATION:
       rc = tmp117_reg_read(dev, TMP117_REG_CFGR, &data);

       if (rc == 0) {
           val->val1 = data;
           val->val2 = 0;
       }

       return rc;

   case SENSOR_ATTR_OFFSET:
       if (!tmp117_is_offset_supported(dev->data)) {
           LOG_ERR("%s: Offset is not supported", dev->name);
           return -EINVAL;
       }

       rc = tmp117_reg_read(dev, TMP117_REG_TEMP_OFFSET, &data);
       if (rc == 0) {
           tmp117_temperature_to_sensor_value(data, val);
       }

       return rc;

   default:
       return -ENOTSUP;
   }
}

static DEVICE_API(sensor, tmp117_driver_api) = {
   .attr_set = tmp117_attr_set,
   .attr_get = tmp117_attr_get,
   .sample_fetch = tmp117_sample_fetch,
   .channel_get = tmp117_channel_get,
};

static int tmp117_init(const struct device *dev)
{
   struct tmp117_data *drv_data = dev->data;
   const struct tmp117_dev_config *cfg = dev->config;
   int rc;
   uint16_t id;

   if (!device_is_ready(cfg->bus.bus)) {
       LOG_ERR("I2C dev %s not ready", cfg->bus.bus->name);
       return -EINVAL;
   }

   /* Check the Device ID */
   rc = tmp117_device_id_check(dev, &id);
   if (rc < 0) {
       return rc;
   }
   LOG_DBG("Got device ID: %x", id);
   drv_data->id = id;

   rc = tmp117_write_config(dev, TMP117_CFGR_CONV, cfg->odr);
   if (rc < 0) {
       return rc;
   }

   rc = tmp117_write_config(dev, TMP117_CFGR_AVG, cfg->oversampling);
   if (rc < 0) {
       return rc;
   }

   int8_t value = cfg->alert_pin_polarity ? TMP117_CFGR_ALERT_PIN_POL : 0;

   rc = tmp117_write_config(dev, TMP117_CFGR_ALERT_PIN_POL, value);
   if (rc < 0) {
       return rc;
   }

   value = cfg->alert_mode ? TMP117_CFGR_ALERT_MODE : 0;
   rc = tmp117_write_config(dev, TMP117_CFGR_ALERT_MODE, value);
   if (rc < 0) {
       return rc;
   }

   value = cfg->alert_dr_sel ? TMP117_CFGR_ALERT_DR_SEL : 0;
   rc = tmp117_write_config(dev, TMP117_CFGR_ALERT_DR_SEL, value);
   if (rc < 0) {
       return rc;
   }

   return rc;
}

#ifdef CONFIG_PM_DEVICE
BUILD_ASSERT(!DT_INST_NODE_HAS_PROP(_num, power_domains), "Driver does not support power domain");
static int tmp117_pm_control(const struct device *dev, enum pm_device_action action)
{
   int ret = 0;

   switch (action) {
   case PM_DEVICE_ACTION_RESUME: {
       const struct tmp117_dev_config *cfg = dev->config;

       ret = tmp117_write_config(dev, TMP117_CFGR_CONV, cfg->odr);
       if (ret < 0) {
           LOG_ERR("Failed to resume TMP117");
       }
       break;
   }
   case PM_DEVICE_ACTION_SUSPEND: {
       ret = tmp117_write_config(dev, TMP117_CFGR_MODE, TMP117_MODE_SHUTDOWN);
       if (ret < 0) {
           LOG_ERR("Failed to suspend TMP117");
       }
       break;
   }
   default:
       ret = -ENOTSUP;
   }

   return ret;
}
#endif /* CONFIG_PM_DEVICE */


#define DEFINE_TMP117_TRIGGER(_num)


#define DEFINE_TMP117(_num)                                                                        \
   static struct tmp117_data tmp117_data_##_num;                                              \
   static const struct tmp117_dev_config tmp117_config_##_num = {                             \
       .bus = I2C_DT_SPEC_INST_GET(_num),                                                 \
       .odr = DT_INST_PROP_OR(_num, odr, 1000000),                                         \
       .oversampling = DT_INST_PROP_OR(_num, oversampling, 1),                             \
       .alert_pin_polarity = DT_INST_PROP_OR(_num, alert_polarity, false),                 \
       .alert_mode = DT_INST_PROP_OR(_num, alert_mode, false),                             \
       .alert_dr_sel = DT_INST_PROP_OR(_num, alert_dr_sel, false),                        \
       .store_attr_values = DT_INST_PROP_OR(_num, store_attr_values, false),             \
       DEFINE_TMP117_TRIGGER(_num)};                                                      \
                                                                                                  \
   PM_DEVICE_DT_INST_DEFINE(_num, tmp117_pm_control);                                         \
                                                                                                  \
   SENSOR_DEVICE_DT_INST_DEFINE(_num, tmp117_init, PM_DEVICE_DT_INST_GET(_num),               \
                    &tmp117_data_##_num, &tmp117_config_##_num, POST_KERNEL,      \
                    CONFIG_SENSOR_INIT_PRIORITY, &tmp117_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_TMP117)
