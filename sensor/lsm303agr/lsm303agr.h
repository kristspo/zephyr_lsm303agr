#ifndef ZEPHYR_DRIVERS_ST_LSM303AGR_H_
#define ZEPHYR_DRIVERS_ST_LSM303AGR_H_

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

struct lsm303agr_config
{
    const struct device *i2c;
    const struct i2c_dt_spec i2c_acc;
    const struct i2c_dt_spec i2c_mag;
};

struct lsm303agr_data
{
    union
    {
        uint8_t raw_xyz[7];
        struct
        {
            uint8_t status;
            int16_t xyz[3];
        } __packed;
    } acc_sample;
    int16_t acc_scale;
    uint8_t raw_temp[2];
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

#endif