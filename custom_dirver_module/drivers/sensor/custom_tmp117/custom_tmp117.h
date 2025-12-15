/*
 * Copyright (c) 2019 Centaur Analytics, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #ifndef ZEPHYR_DRIVERS_SENSOR_TMP117_TMP117_H_
 #define ZEPHYR_DRIVERS_SENSOR_TMP117_TMP117_H_
 
#include <zephyr/sys/util_macro.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
 
 #define TMP117_REG_TEMP        0x0
 #define TMP117_REG_CFGR        0x1
 #define TMP117_REG_HIGH_LIM    0x2
 #define TMP117_REG_LOW_LIM     0x3
 #define TMP117_REG_EEPROM_UL   0x4
 #define TMP117_REG_EEPROM1     0x5
 #define TMP117_REG_EEPROM2     0x6
 #define TMP117_REG_EEPROM3     0x7
 #define TMP117_REG_TEMP_OFFSET 0x7
 #define TMP117_REG_EEPROM4     0x8
 #define TMP117_REG_DEVICE_ID   0xF
 
 #define TMP117_RESOLUTION     78125 /* in tens of uCelsius*/
 #define TMP117_RESOLUTION_DIV 10000000
 
 #define TMP116_DEVICE_ID 0x1116
 #define TMP117_DEVICE_ID 0x0117
 #define TMP119_DEVICE_ID 0x2117
 
 #define TMP117_CFGR_RESET       BIT(1)
 #define TMP117_CFGR_AVG         (BIT(5) | BIT(6))
 #define TMP117_CFGR_CONV        (BIT(7) | BIT(8) | BIT(9))
 #define TMP117_CFGR_MODE        (BIT(10) | BIT(11))
 #define TMP117_CFGR_DATA_READY  BIT(13)
 #define TMP117_EEPROM_UL_UNLOCK BIT(15)
 #define TMP117_EEPROM_UL_BUSY   BIT(14)
 
 /* Alert pin configuration bits */
 #define TMP117_CFGR_ALERT_DR_SEL  BIT(2) /* ALERT pin select (1=Data ready) (0=alert) */
 #define TMP117_CFGR_ALERT_PIN_POL BIT(3) /* Alert pin polarity */
 #define TMP117_CFGR_ALERT_MODE    BIT(4) /* Alert pin mode (1=therm, 0=alert) */
 
 #define TMP117_AVG_1_SAMPLE    0
 #define TMP117_AVG_8_SAMPLES   BIT(5)
 #define TMP117_AVG_32_SAMPLES  BIT(6)
 #define TMP117_AVG_64_SAMPLES  (BIT(5) | BIT(6))
#define TMP117_MODE_CONTINUOUS 0
#define TMP117_MODE_SHUTDOWN   BIT(10)
#define TMP117_MODE_ONE_SHOT   (BIT(10) | BIT(11))

/* Output data rate constants (for device tree) */
#define TMP117_DT_ODR_15_5_MS   0
#define TMP117_DT_ODR_125_MS    1
#define TMP117_DT_ODR_250_MS    2
#define TMP117_DT_ODR_500_MS    3
#define TMP117_DT_ODR_1000_MS   4
#define TMP117_DT_ODR_4000_MS   5
#define TMP117_DT_ODR_8000_MS   6
#define TMP117_DT_ODR_16000_MS  7

/* Sensor attribute constants */
#define SENSOR_ATTR_TMP117_SHUTDOWN_MODE            (SENSOR_ATTR_PRIV_START + 1)
#define SENSOR_ATTR_TMP117_CONTINUOUS_CONVERSION_MODE (SENSOR_ATTR_PRIV_START + 2)
#define SENSOR_ATTR_TMP117_ALERT_PIN_POLARITY        (SENSOR_ATTR_PRIV_START + 3)
#define SENSOR_ATTR_TMP117_ALERT_MODE                (SENSOR_ATTR_PRIV_START + 4)
#define SENSOR_ATTR_TMP117_ONE_SHOT_MODE             (SENSOR_ATTR_PRIV_START + 5)

struct tmp117_data {
     uint16_t sample;
     uint16_t id;
 };
 
struct tmp117_dev_config {
    struct i2c_dt_spec bus;
    uint32_t odr;  /* Output data rate in microhertz (uHz) */
    uint16_t oversampling;
    bool alert_pin_polarity;
    bool alert_mode;
    bool alert_dr_sel;
    bool store_attr_values;
};
 
 /* Function declarations */
 int tmp117_write_config(const struct device *dev, uint16_t mask, uint16_t conf);
 int tmp117_reg_read(const struct device *dev, uint8_t reg, uint16_t *val);
 
 #endif /*  ZEPHYR_DRIVERS_SENSOR_TMP117_TMP117_H_ */
 