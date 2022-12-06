#ifndef ZEPHYR_DRIVERS_ST_LSM303AGR_H_
#define ZEPHYR_DRIVERS_ST_LSM303AGR_H_

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#ifdef CONFIG_LOG
#define PRINT(...) LOG_PRINTK(__VA_ARGS__)
#else
#define PRINT(...) (void)0
#endif

struct lsm303agr_config
{
    const struct device *i2c;
    const struct i2c_dt_spec i2c_acc;
    const struct i2c_dt_spec i2c_mag;
    const struct gpio_dt_spec gpio_acc_int1;
    const struct gpio_dt_spec gpio_acc_int2;
    const struct gpio_dt_spec gpio_mag_int0;
};

typedef union lsm303agr_sample_type
{
    uint8_t raw_xyz[7];
    struct
    {
        uint8_t status;
        int16_t xyz[3];
    } __packed;
} lsm303agr_sample;

struct lsm303agr_data
{
    lsm303agr_sample acc_sample;
    lsm303agr_sample mag_sample;
    int16_t acc_conv_scale;
    bool mag_single_shot;
    uint8_t raw_temp[2];

    struct gpio_callback gpio_acc_int1_cb;
    struct gpio_callback gpio_acc_int2_cb;
    struct gpio_callback gpio_mag_int0_cb;
};

typedef struct lsm303agr_reg_type
{
    uint16_t addr;
    bool write;
} lsm303agr_reg;

int lsm303agr_attr_get(const struct device *dev,
                       enum sensor_channel chan,
                       enum sensor_attribute attr,
                       struct sensor_value *val);

int lsm303agr_attr_set(const struct device *dev,
                       enum sensor_channel chan,
                       enum sensor_attribute attr,
                       const struct sensor_value *val);

int lsm303agr_channel_get(const struct device *dev,
                          enum sensor_channel chan,
                          struct sensor_value *val);

int lsm303agr_sample_fetch(const struct device *dev,
                           enum sensor_channel chan);

int lsm303agr_init_gpios(const struct device *dev);

#endif