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

typedef union lsm303agr_buffer_type
{
    uint8_t raw_xyz[32 * 3 * 2];
    int16_t xyz[32 * 3];
} lsm303agr_buffer;

typedef struct lsm303agr_status_type
{
    uint8_t acc_rate : 4;
    uint8_t acc_int1 : 1;
    uint8_t acc_int2 : 1;
    uint8_t mag_int0 : 1;
    uint8_t mag_single_shot : 1;
    uint8_t mag_rate : 2;
    bool fifo_ready : 1;
    bool timer_started : 1;
} lsm303agr_status;

typedef struct lsm303agr_trig_type
{
#ifdef CONFIG_LSM303AGR_INTERRUPT_POLLING
    uint8_t active; // bits of fired polling interrupts
#endif
    uint8_t enable; // bit(s) of enabled interrupts
    sensor_trigger_handler_t handler;
} lsm303agr_trig;

struct lsm303agr_data
{
    lsm303agr_sample acc_sample;
    lsm303agr_sample mag_sample;
#ifdef CONFIG_LSM303AGR_FIFO_BURST_READ
    lsm303agr_buffer fifo_buffer;
#endif
    uint8_t fifo_size;
    int16_t acc_conv_scale;
    int16_t acc_thrs_scale;
    uint8_t raw_temp[2];

    lsm303agr_status status;
    struct gpio_callback gpio_acc_int1_cb;
    struct gpio_callback gpio_acc_int2_cb;
    struct gpio_callback gpio_mag_int0_cb;
    lsm303agr_trig acc_int1;
    lsm303agr_trig acc_int2;
    lsm303agr_trig mag_int0;
    const struct device *dev;
    struct k_work work;
#ifdef CONFIG_LSM303AGR_INTERRUPT_POLLING
    lsm303agr_trig poll_int;
    struct k_timer poll_timer;
    struct k_work poll_work;
#endif
};

typedef struct lsm303agr_reg_type
{
    uint16_t addr;
    bool write;
} lsm303agr_reg;

enum lsm303agr_int
{
    INT_CLICK,
    INT_AOI_1,
    INT_AOI_2,
    INT_ACT,
    INT_MAG,
    INT_FIFO,
};

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

int lsm303agr_trigger_set(const struct device *dev,
                          const struct sensor_trigger *trig,
                          sensor_trigger_handler_t handler);

/* functions used between source files */

int lsm303agr_fifo_set(const struct i2c_dt_spec *ctx,
                       bool enable);

int lsm303agr_init_gpios(const struct device *dev);

int lsm303agr_gpio_int_set(const struct gpio_dt_spec *gpio,
                           gpio_flags_t flags);

int lsm303agr_polling_init(const struct device *dev,
                           uint8_t enable, sensor_trigger_handler_t handler);

#endif