
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(lsm303agr, CONFIG_SENSOR_LOG_LEVEL);
#include "lsm303agr.h"
#include "lsm303agr_attr.h"
#include "lsm303agr_reg.h"

void lsm303agr_acc_interrupt(const lsm303agr_trig *trigger,
                             const enum lsm303agr_int interrupt,
                             const struct device *dev)
{
    const struct lsm303agr_config *cfg = dev->config;
    struct sensor_trigger resp;
    resp.chan = SENSOR_CHAN_ACCEL_XYZ;
    resp.type = trigger->enable;

    // read interrupt source register
    uint8_t src_reg, src_bits;
    switch (interrupt)
    {
    case INT_CLICK:
        src_reg = LSM303AGR_CLICK_SRC_A;
        break;
    case INT_AOI_1:
        src_reg = LSM303AGR_INT1_SRC_A;
        break;
    case INT_AOI_2:
        src_reg = LSM303AGR_INT2_SRC_A;
        break;
    case INT_FIFO_WTM:
    case INT_FIFO_OVR:
        src_reg = LSM303AGR_FIFO_SRC_REG_A;
        break;
    default: // no interrupt source register for INT_ACT
        src_reg = 0x00;
    }
    if (src_reg)
    {
        int status = lsm303agr_read_reg(&cfg->i2c_acc, src_reg, &src_bits, 1);
        if (status == 0)
        {
            PRINT(" interrupt src register 0x%02x : 0x%02x \n", src_reg, src_bits);
            resp.type |= TRIGGER_BITS_SET(src_bits);
        }
    }

    if (trigger->handler)
        trigger->handler(dev, &resp);
}

void lsm303agr_mag_interrupt(const lsm303agr_trig *trigger,
                             const enum lsm303agr_int interrupt,
                             const struct device *dev)
{
    struct sensor_trigger resp;
    resp.chan = SENSOR_CHAN_MAGN_XYZ;
    resp.type = trigger->enable;
    if (trigger->handler)
    {
        trigger->handler(dev, &resp);
    }
}

void lsm303agr_acc_int1_callback(const struct device *port,
                                 struct gpio_callback *cb,
                                 uint32_t pins)
{
    struct lsm303agr_data *data = CONTAINER_OF(cb, struct lsm303agr_data, gpio_acc_int1_cb);
    data->status.acc_int1 = 1;
    k_work_submit(&data->work);
}

void lsm303agr_acc_int2_callback(const struct device *port,
                                 struct gpio_callback *cb,
                                 uint32_t pins)
{
    struct lsm303agr_data *data = CONTAINER_OF(cb, struct lsm303agr_data, gpio_acc_int2_cb);
    data->status.acc_int2 = 1;
    k_work_submit(&data->work);
}

void lsm303agr_mag_int0_callback(const struct device *port,
                                 struct gpio_callback *cb,
                                 uint32_t pins)
{
    struct lsm303agr_data *data = CONTAINER_OF(cb, struct lsm303agr_data, gpio_mag_int0_cb);
    data->status.mag_int0 = 1;
    k_work_submit(&data->work);
}

void lsm303agr_work_cb(struct k_work *work)
{
    struct lsm303agr_data *data = CONTAINER_OF(work, struct lsm303agr_data, work);

    if (data->status.acc_int1)
    {
        data->status.acc_int1 = 0;
        if (data->acc_int1.enable & BIT_ACC_INT_CLICK)
            lsm303agr_acc_interrupt(&data->acc_int1, INT_CLICK, data->dev);
        else if (data->acc_int1.enable & BIT_ACC_INT_AOI1)
            lsm303agr_acc_interrupt(&data->acc_int1, INT_AOI_1, data->dev);
        else if (data->acc_int1.enable & BIT_ACC_INT_AOI2)
            lsm303agr_acc_interrupt(&data->acc_int1, INT_AOI_2, data->dev);
        else if (data->acc_int1.enable & BIT_ACC_INT_FIFO_WTM)
            lsm303agr_acc_interrupt(&data->acc_int1, INT_FIFO_WTM, data->dev);
        else if (data->acc_int1.enable & BIT_ACC_INT_FIFO_OVR)
            lsm303agr_acc_interrupt(&data->acc_int1, INT_FIFO_OVR, data->dev);
    }

    if (data->status.acc_int2)
    {
        data->status.acc_int2 = 0;
        if (data->acc_int2.enable & BIT_ACC_INT_CLICK)
            lsm303agr_acc_interrupt(&data->acc_int2, INT_CLICK, data->dev);
        else if (data->acc_int2.enable & BIT_ACC_INT_AOI1)
            lsm303agr_acc_interrupt(&data->acc_int2, INT_AOI_1, data->dev);
        else if (data->acc_int2.enable & BIT_ACC_INT_AOI2)
            lsm303agr_acc_interrupt(&data->acc_int2, INT_AOI_2, data->dev);
        else if (data->acc_int2.enable & BIT_ACC_INT_ACT)
            lsm303agr_acc_interrupt(&data->acc_int2, INT_ACT, data->dev);
    }

    if (data->status.mag_int0)
    {
        data->status.mag_int0 = 0;
    }
}

int lsm303agr_gpio_config(const struct gpio_dt_spec *gpio,
                          struct gpio_callback *gpio_cb,
                          gpio_callback_handler_t cb_handler)
{
    int status;
    if (!device_is_ready(gpio->port))
    {
        PRINT("Device %s is not ready", gpio->port->name);
        return -ENODEV;
    }

    uint32_t gpio_flags = gpio->dt_flags & (GPIO_PULL_DOWN | GPIO_PULL_UP | GPIO_ACTIVE_LOW);
    status = gpio_pin_configure(gpio->port, gpio->pin, GPIO_INPUT | gpio_flags);
    if (status < 0)
    {
        PRINT("Failed to configure gpio %s %d \n", gpio->port->name, gpio->pin);
        return status;
    }

    gpio_init_callback(gpio_cb, cb_handler, BIT(gpio->pin));
    status = gpio_add_callback(gpio->port, gpio_cb);
    if (status < 0)
    {
        PRINT("Failed to configure gpio %s %d \n", gpio->port->name, gpio->pin);
        return status;
    }

    return 0;
}

int lsm303agr_gpio_int_set(const struct gpio_dt_spec *gpio,
                           gpio_flags_t flags)
{
    int status = gpio_pin_interrupt_configure_dt(gpio, flags);
    if (status < 0)
        PRINT("Failed to set interrupt %s %d \n", gpio->port->name, gpio->pin);

    return status;
}

int lsm303agr_init_gpios(const struct device *dev)
{
    const struct lsm303agr_config *cfg = dev->config;
    struct lsm303agr_data *data = dev->data;
    int status;

    data->dev = dev;
    data->work.handler = lsm303agr_work_cb;

    if (cfg->gpio_acc_int1.port)
    {
        status = lsm303agr_gpio_config(&cfg->gpio_acc_int1, &data->gpio_acc_int1_cb, lsm303agr_acc_int1_callback);
        if (status < 0)
            return status;
    }
    if (cfg->gpio_acc_int2.port)
    {
        status = lsm303agr_gpio_config(&cfg->gpio_acc_int2, &data->gpio_acc_int2_cb, lsm303agr_acc_int2_callback);
        if (status < 0)
            return status;
    }
    if (cfg->gpio_mag_int0.port)
    {
        status = lsm303agr_gpio_config(&cfg->gpio_mag_int0, &data->gpio_mag_int0_cb, lsm303agr_mag_int0_callback);
        if (status < 0)
            return status;
    }

    return 0;
}
