
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(lsm303agr, CONFIG_SENSOR_LOG_LEVEL);
#include "lsm303agr.h"
#include "lsm303agr_attr.h"
#include "lsm303agr_reg.h"

void lsm303agr_acc_interrupt(lsm303agr_trig *trigger,
                             const enum lsm303agr_int interrupt,
                             const struct device *dev)
{
    const struct lsm303agr_config *cfg = dev->config;
    struct lsm303agr_data *data = dev->data;
    struct sensor_trigger resp;
    resp.chan = SENSOR_CHAN_ACCEL_XYZ;
    resp.type = trigger->enable;

    // read interrupt source register
    uint8_t src_reg, src_bits;
    uint8_t poll_bits = BIT(6); // polled interrupt active bit(s) in SRC register
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
    case INT_FIFO:
        src_reg = LSM303AGR_FIFO_SRC_REG_A;
        poll_bits = ((trigger->enable & BIT_POLL_INT_FIFO_WTM) ? BIT(7) : 0) | ((trigger->enable & BIT_POLL_INT_FIFO_OVR) ? BIT(6) : 0);
        break;
    default: // no interrupt source register for INT_ACT
        src_reg = 0x00;
    }

#ifdef CONFIG_LSM303AGR_INTERRUPT_POLLING
    bool call_handler = false;
    if (src_reg)
    {
        int status = lsm303agr_read_reg(&cfg->i2c_acc, src_reg, &src_bits, 1);
        if (status == 0)
        {
            // PRINT("interrupt src register 0x%02x : 0x%02x \n", src_reg, src_bits);
            if (trigger->enable & 0x01) // poll interrupt source register
            {
                const uint8_t trig_bits = poll_bits & src_bits; // bits of polled and set interrupt SRC
                const uint8_t mark_bits = ((interrupt == INT_FIFO) ? trig_bits : BIT(interrupt));
                if (trig_bits)
                {
                    if (!(trigger->active & mark_bits))
                    {
                        trigger->active |= mark_bits;
                        resp.type &= ALLOW_BITS_ALL;
                        call_handler = true;
                    }
                }
                else
                {
                    trigger->active &= ((interrupt == INT_FIFO) ? ~poll_bits : ~mark_bits);
                    return;
                }
            }
            else
                call_handler = true;

            resp.type |= TRIGGER_BITS_SET(src_bits);
            if (src_reg == LSM303AGR_FIFO_SRC_REG_A)
            {
                data->fifo_size = (src_bits & 0x1F);
                data->status.fifo_ready = true;
            }
        }
    }

    if (trigger->handler && call_handler)
        trigger->handler(dev, &resp);
#else
    if (src_reg)
    {
        int status = lsm303agr_read_reg(&cfg->i2c_acc, src_reg, &src_bits, 1);
        if (status == 0)
        {
            // PRINT("interrupt src register 0x%02x : 0x%02x \n", src_reg, src_bits);
            resp.type |= TRIGGER_BITS_SET(src_bits);
            if (src_reg == LSM303AGR_FIFO_SRC_REG_A)
            {
                data->fifo_size = (src_bits & 0x1F);
                data->status.fifo_ready = true;
            }
        }
    }

    if (trigger->handler)
        trigger->handler(dev, &resp);
#endif
}

void lsm303agr_mag_interrupt(lsm303agr_trig *trigger,
                             const struct device *dev)
{
    const struct lsm303agr_config *cfg = dev->config;
    struct sensor_trigger resp;
    resp.chan = SENSOR_CHAN_MAGN_XYZ;
    resp.type = trigger->enable;

    // read interrupt source register
    uint8_t src_bits;
    int status = lsm303agr_read_reg(&cfg->i2c_mag, LSM303AGR_INT_SOURCE_REG_M, &src_bits, 1);
    if (status == 0)
    {
        // PRINT("interrupt src register 0x%02x : 0x%02x \n", LSM303AGR_INT_SOURCE_REG_M, src_bits);
#ifdef CONFIG_LSM303AGR_INTERRUPT_POLLING
        if (trigger->enable & 0x01) // poll interrupt source register
        {
            const uint8_t poll_bit = BIT(0); // interrupt active bit in SOURCE register
            const uint8_t mark_bit = BIT(INT_MAG);
            if (poll_bit & src_bits)
            {
                if (trigger->active & mark_bit)
                {
                    return;
                }
                else
                {
                    trigger->active |= mark_bit;
                    resp.type &= ALLOW_BITS_ALL;
                }
            }
            else
            {
                trigger->active &= ~mark_bit;
                return;
            }
        }
#endif
        resp.type |= TRIGGER_BITS_SET(src_bits);
    }

    if (trigger->handler)
        trigger->handler(dev, &resp);
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

#ifdef CONFIG_LSM303AGR_INTERRUPT_POLLING
void lsm303agr_timer_callback(struct k_timer *timer)
{
    struct lsm303agr_data *data = timer->user_data;
    k_work_submit(&data->poll_work);
}

void lsm303agr_poll_work_cb(struct k_work *poll_work)
{
    struct lsm303agr_data *data = CONTAINER_OF(poll_work, struct lsm303agr_data, poll_work);

    if (data->poll_int.enable & BIT_POLL_INT_CLICK)
        lsm303agr_acc_interrupt(&data->poll_int, INT_CLICK, data->dev);
    if (data->poll_int.enable & BIT_POLL_INT_AOI1)
        lsm303agr_acc_interrupt(&data->poll_int, INT_AOI_1, data->dev);
    if (data->poll_int.enable & BIT_POLL_INT_AOI2)
        lsm303agr_acc_interrupt(&data->poll_int, INT_AOI_2, data->dev);
    if (data->poll_int.enable & BIT_POLL_INT_MAG)
        lsm303agr_mag_interrupt(&data->poll_int, data->dev);
    if (data->poll_int.enable & (BIT_POLL_INT_FIFO_WTM | BIT_POLL_INT_FIFO_OVR))
        lsm303agr_acc_interrupt(&data->poll_int, INT_FIFO, data->dev);
}
#endif

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
        else if (data->acc_int1.enable & (BIT_ACC_INT_FIFO_WTM | BIT_ACC_INT_FIFO_OVR))
            lsm303agr_acc_interrupt(&data->acc_int1, INT_FIFO, data->dev);
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
        lsm303agr_mag_interrupt(&data->mag_int0, data->dev);
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

#ifdef CONFIG_LSM303AGR_INTERRUPT_POLLING
    k_timer_init(&data->poll_timer, lsm303agr_timer_callback, NULL);
    k_timer_user_data_set(&data->poll_timer, data);
    data->poll_work.handler = lsm303agr_poll_work_cb;
#endif

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

#ifdef CONFIG_LSM303AGR_INTERRUPT_POLLING
#if !defined(CONFIG_LSM303AGR_INTERRUPT_POLL_INTERVAL) || !(CONFIG_LSM303AGR_INTERRUPT_POLL_INTERVAL > 0)
#error "Unexpected poll interval Kconfig value"
#endif

int lsm303agr_polling_init(const struct device *dev,
                           uint8_t enable,
                           sensor_trigger_handler_t handler)
{
    const struct lsm303agr_config *cfg = dev->config;
    struct lsm303agr_data *data = dev->data;
    lsm303agr_int_crtl_reg_m_t ctrl_reg;
    int status;

    if ((enable & (BIT_POLL_INT_FIFO_WTM | BIT_POLL_INT_FIFO_OVR)) && !(data->poll_int.enable & (BIT_POLL_INT_FIFO_WTM | BIT_POLL_INT_FIFO_OVR)))
    {
        data->status.fifo_ready = false;
        status = lsm303agr_fifo_set(&cfg->i2c_acc, true);
        if (status < 0)
            return status;
    }
    if (!(enable & (BIT_POLL_INT_FIFO_WTM | BIT_POLL_INT_FIFO_OVR)) && (data->poll_int.enable & (BIT_POLL_INT_FIFO_WTM | BIT_POLL_INT_FIFO_OVR)))
    {
        data->status.fifo_ready = false;
        status = lsm303agr_fifo_set(&cfg->i2c_acc, false);
        if (status < 0)
            return status;
    }

    if ((enable & BIT_POLL_INT_MAG) && !(data->poll_int.enable & BIT_POLL_INT_MAG))
    {
        // update magnetometer CGF_REG_B_M INT_ON_DATAOFF bit
        status = lsm303agr_mag_offset_int_conf_set(&cfg->i2c_mag, (enable & BIT_POLL_MAG_THRS_OFFSET) ? 1 : 0);
        if (status < 0)
            return status;
        // update magnetometer INT_CTRL_REG_M IEA, IEL, IEN bits
        status = lsm303agr_mag_int_gen_conf_get(&cfg->i2c_mag, &ctrl_reg);
        if (status < 0)
            return status;
        ctrl_reg.ien = 1;
        ctrl_reg.iel = 0;
        ctrl_reg.iea = 1;
        status = lsm303agr_mag_int_gen_conf_set(&cfg->i2c_mag, &ctrl_reg);
        if (status < 0)
            return status;
    }
    if (!(enable & BIT_POLL_INT_MAG) && (data->poll_int.enable & BIT_POLL_INT_MAG))
    {
        // clear magnetometer INT_CTRL_REG_M IEN bit
        status = lsm303agr_mag_int_gen_conf_get(&cfg->i2c_mag, &ctrl_reg);
        if (status < 0)
            return status;
        ctrl_reg.ien = 0;
        status = lsm303agr_mag_int_gen_conf_set(&cfg->i2c_mag, &ctrl_reg);
        if (status < 0)
            return status;
    }

    data->poll_int.active = 0x00;
    data->poll_int.enable = enable | 0x01; // mark this as polling interrupt
    data->poll_int.handler = handler;

    if (enable && !data->status.timer_started)
    {
        k_timeout_t period = K_MSEC(CONFIG_LSM303AGR_INTERRUPT_POLL_INTERVAL);
        k_timer_start(&data->poll_timer, period, period);
        data->status.timer_started = true;
    }
    if (!enable && data->status.timer_started)
    {
        k_timer_stop(&data->poll_timer);
    }

    return 0;
}
#endif