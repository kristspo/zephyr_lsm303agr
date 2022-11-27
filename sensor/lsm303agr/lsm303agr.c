
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lsm303agr, CONFIG_SENSOR_LOG_LEVEL);
#include "lsm303agr.h"
#include "lsm303agr_attr.h"
#include "lsm303agr_reg.h"

#if CONFIG_LOG
#define PRINT(...) LOG_PRINTK(__VA_ARGS__)
#else
#define PRINT(...) (void)0
#endif

#define DT_DRV_COMPAT st_lsm303agr

static const struct sensor_driver_api lsm303agr_driver_api = {
    .attr_set = lsm303agr_attr_set,
    .attr_get = lsm303agr_attr_get,
    .sample_fetch = lsm303agr_sample_fetch,
    .channel_get = lsm303agr_channel_get,
};

/* get lsm303agr register from sensor_attr value */
lsm303agr_reg lsm303agr_reg_get(enum lsm303agr_attribute attr)
{
    switch (attr)
    {
    /* accelerometer registers */
    case STATUS_REG_AUX_A:
        return (lsm303agr_reg){LSM303AGR_STATUS_REG_AUX_A, false};
    case OUT_TEMP_L_A:
        return (lsm303agr_reg){LSM303AGR_OUT_TEMP_L_A, false};
    case OUT_TEMP_H_A:
        return (lsm303agr_reg){LSM303AGR_OUT_TEMP_H_A, false};
    case INT_COUNTER_REG_A:
        return (lsm303agr_reg){LSM303AGR_INT_COUNTER_REG_A, false};
    case WHO_AM_I_A:
        return (lsm303agr_reg){LSM303AGR_WHO_AM_I_A, false};
    case TEMP_CFG_REG_A:
        return (lsm303agr_reg){LSM303AGR_TEMP_CFG_REG_A, true};
    case CTRL_REG1_A:
        return (lsm303agr_reg){LSM303AGR_CTRL_REG1_A, true};
    case CTRL_REG2_A:
        return (lsm303agr_reg){LSM303AGR_CTRL_REG2_A, true};
    case CTRL_REG3_A:
        return (lsm303agr_reg){LSM303AGR_CTRL_REG3_A, true};
    case CTRL_REG4_A:
        return (lsm303agr_reg){LSM303AGR_CTRL_REG4_A, true};
    case CTRL_REG5_A:
        return (lsm303agr_reg){LSM303AGR_CTRL_REG5_A, true};
    case CTRL_REG6_A:
        return (lsm303agr_reg){LSM303AGR_CTRL_REG6_A, true};
    case REFERENCE_A:
        return (lsm303agr_reg){LSM303AGR_REFERENCE_A, true};
    case STATUS_REG_A:
        return (lsm303agr_reg){LSM303AGR_STATUS_REG_A, false};
    case OUT_X_L_A:
        return (lsm303agr_reg){LSM303AGR_OUT_X_L_A, false};
    case OUT_X_H_A:
        return (lsm303agr_reg){LSM303AGR_OUT_X_H_A, false};
    case OUT_Y_L_A:
        return (lsm303agr_reg){LSM303AGR_OUT_Y_L_A, false};
    case OUT_Y_H_A:
        return (lsm303agr_reg){LSM303AGR_OUT_Y_H_A, false};
    case OUT_Z_L_A:
        return (lsm303agr_reg){LSM303AGR_OUT_Z_L_A, false};
    case OUT_Z_H_A:
        return (lsm303agr_reg){LSM303AGR_OUT_Z_H_A, false};
    case FIFO_CTRL_REG_A:
        return (lsm303agr_reg){LSM303AGR_FIFO_CTRL_REG_A, true};
    case FIFO_SRC_REG_A:
        return (lsm303agr_reg){LSM303AGR_FIFO_SRC_REG_A, false};
    case INT1_CFG_A:
        return (lsm303agr_reg){LSM303AGR_INT1_CFG_A, true};
    case INT1_SRC_A:
        return (lsm303agr_reg){LSM303AGR_INT1_SRC_A, false};
    case INT1_THS_A:
        return (lsm303agr_reg){LSM303AGR_INT1_THS_A, true};
    case INT1_DURATION_A:
        return (lsm303agr_reg){LSM303AGR_INT1_DURATION_A, true};
    case INT2_CFG_A:
        return (lsm303agr_reg){LSM303AGR_INT2_CFG_A, true};
    case INT2_SRC_A:
        return (lsm303agr_reg){LSM303AGR_INT2_SRC_A, false};
    case INT2_THS_A:
        return (lsm303agr_reg){LSM303AGR_INT2_THS_A, true};
    case INT2_DURATION_A:
        return (lsm303agr_reg){LSM303AGR_INT2_DURATION_A, true};
    case CLICK_CFG_A:
        return (lsm303agr_reg){LSM303AGR_CLICK_CFG_A, true};
    case CLICK_SRC_A:
        return (lsm303agr_reg){LSM303AGR_CLICK_SRC_A, false};
    case CLICK_THS_A:
        return (lsm303agr_reg){LSM303AGR_CLICK_THS_A, true};
    case TIME_LIMIT_A:
        return (lsm303agr_reg){LSM303AGR_TIME_LIMIT_A, true};
    case TIME_LATENCY_A:
        return (lsm303agr_reg){LSM303AGR_TIME_LATENCY_A, true};
    case TIME_WINDOW_A:
        return (lsm303agr_reg){LSM303AGR_TIME_WINDOW_A, true};
    case ACT_THS_A:
        return (lsm303agr_reg){LSM303AGR_ACT_THS_A, true};
    case ACT_DUR_A:
        return (lsm303agr_reg){LSM303AGR_ACT_DUR_A, true};

    /* magnetometer registers */
    case OFFSET_X_REG_L_M:
        return (lsm303agr_reg){LSM303AGR_OFFSET_X_REG_L_M, true};
    case OFFSET_X_REG_H_M:
        return (lsm303agr_reg){LSM303AGR_OFFSET_X_REG_H_M, true};
    case OFFSET_Y_REG_L_M:
        return (lsm303agr_reg){LSM303AGR_OFFSET_Y_REG_L_M, true};
    case OFFSET_Y_REG_H_M:
        return (lsm303agr_reg){LSM303AGR_OFFSET_Y_REG_H_M, true};
    case OFFSET_Z_REG_L_M:
        return (lsm303agr_reg){LSM303AGR_OFFSET_Z_REG_L_M, true};
    case OFFSET_Z_REG_H_M:
        return (lsm303agr_reg){LSM303AGR_OFFSET_Z_REG_H_M, true};
    case WHO_AM_I_M:
        return (lsm303agr_reg){LSM303AGR_WHO_AM_I_M, false};
    case CFG_REG_A_M:
        return (lsm303agr_reg){LSM303AGR_CFG_REG_A_M, true};
    case CFG_REG_B_M:
        return (lsm303agr_reg){LSM303AGR_CFG_REG_B_M, true};
    case CFG_REG_C_M:
        return (lsm303agr_reg){LSM303AGR_CFG_REG_C_M, true};
    case INT_CRTL_REG_M:
        return (lsm303agr_reg){LSM303AGR_INT_CRTL_REG_M, true};
    case INT_SOURCE_REG_M:
        return (lsm303agr_reg){LSM303AGR_INT_SOURCE_REG_M, false};
    case INT_THS_L_REG_M:
        return (lsm303agr_reg){LSM303AGR_INT_THS_L_REG_M, true};
    case INT_THS_H_REG_M:
        return (lsm303agr_reg){LSM303AGR_INT_THS_H_REG_M, true};
    case STATUS_REG_M:
        return (lsm303agr_reg){LSM303AGR_STATUS_REG_M, false};
    case OUTX_L_REG_M:
        return (lsm303agr_reg){LSM303AGR_OUTX_L_REG_M, false};
    case OUTX_H_REG_M:
        return (lsm303agr_reg){LSM303AGR_OUTX_H_REG_M, false};
    case OUTY_L_REG_M:
        return (lsm303agr_reg){LSM303AGR_OUTY_L_REG_M, false};
    case OUTY_H_REG_M:
        return (lsm303agr_reg){LSM303AGR_OUTY_H_REG_M, false};
    case OUTZ_L_REG_M:
        return (lsm303agr_reg){LSM303AGR_OUTZ_L_REG_M, false};
    case OUTZ_H_REG_M:
        return (lsm303agr_reg){LSM303AGR_OUTZ_H_REG_M, false};

    default:
        return (lsm303agr_reg){0, 0};
    };
}

int lsm303agr_attr_get(const struct device *dev,
                       enum sensor_channel chan,
                       enum sensor_attribute attr,
                       struct sensor_value *val)
{
    const struct lsm303agr_config *cfg = dev->config;
    lsm303agr_reg reg;
    uint8_t reg_value;
    int status;

    switch (chan)
    {
    case SENSOR_CHAN_ACCEL_X:
    case SENSOR_CHAN_ACCEL_Y:
    case SENSOR_CHAN_ACCEL_Z:
    case SENSOR_CHAN_ACCEL_XYZ:
        reg = lsm303agr_reg_get(attr);
        if (reg.addr == 0)
        {
            return -ENOTSUP;
        }
        else
        {
            status = lsm303agr_read_reg(&cfg->i2c_acc, reg.addr, &reg_value, 1);
            if (status < 0)
                return status;
            else
            {
                val->val1 = reg_value;
                val->val2 = reg.addr;
            }
        }
        break;

    case SENSOR_CHAN_MAGN_X:
    case SENSOR_CHAN_MAGN_Y:
    case SENSOR_CHAN_MAGN_Z:
    case SENSOR_CHAN_MAGN_XYZ:
        reg = lsm303agr_reg_get(attr);
        if (reg.addr == 0)
        {
            return -ENOTSUP;
        }
        else
        {
            status = lsm303agr_read_reg(&cfg->i2c_mag, reg.addr, &reg_value, 1);
            if (status < 0)
                return status;
            else
            {
                val->val1 = reg_value;
                val->val2 = reg.addr;
            }
        }
        break;

    default:
        return -ENOTSUP;
    }

    return 0;
}

int lsm303agr_attr_set(const struct device *dev,
                       enum sensor_channel chan,
                       enum sensor_attribute attr,
                       const struct sensor_value *val)
{
    const struct lsm303agr_config *cfg = dev->config;
    lsm303agr_reg reg;

    switch (chan)
    {
    case SENSOR_CHAN_ACCEL_X:
    case SENSOR_CHAN_ACCEL_Y:
    case SENSOR_CHAN_ACCEL_Z:
    case SENSOR_CHAN_ACCEL_XYZ:
        reg = lsm303agr_reg_get(attr);
        if (reg.addr == 0 || !reg.write)
            return -ENOTSUP;
        else
            return lsm303agr_write_reg(&cfg->i2c_acc, reg.addr, (uint8_t *)&val->val1, 1);

    case SENSOR_CHAN_MAGN_X:
    case SENSOR_CHAN_MAGN_Y:
    case SENSOR_CHAN_MAGN_Z:
    case SENSOR_CHAN_MAGN_XYZ:
        reg = lsm303agr_reg_get(attr);
        if (reg.addr == 0 || !reg.write)
            return -ENOTSUP;
        else
            return lsm303agr_write_reg(&cfg->i2c_mag, reg.addr, (uint8_t *)&val->val1, 1);

    default:
        return -ENOTSUP;
    }

    return 0;
}

int lsm303agr_channel_get(const struct device *dev,
                          enum sensor_channel chan,
                          struct sensor_value *val)
{
    return 0;
}

int lsm303agr_sample_fetch(const struct device *dev,
                           enum sensor_channel chan)
{
    return 0;
}

static int lsm303agr_init(const struct device *dev)
{
    const struct lsm303agr_config *cfg = dev->config;
    int status;
    uint8_t id;

    PRINT("\nSetup lsm303agr on %s\n", cfg->i2c->name);

    if (!device_is_ready(cfg->i2c))
        return -ENODEV;

    status = lsm303agr_xl_device_id_get(&cfg->i2c_acc, &id);
    if (status < 0)
        return status;

    if (id != LSM303AGR_ID_XL)
    {
        LOG_ERR("Unexpected chip ID: 0x%02x", id);
        return -EINVAL;
    }

    status = lsm303agr_mag_device_id_get(&cfg->i2c_mag, &id);
    if (status < 0)
        return status;

    if (id != LSM303AGR_ID_MG)
    {
        LOG_ERR("Unexpected chip ID: 0x%02x", id);
        return -EINVAL;
    }

    return 0;
}

#define LSM303AGR_DEFINE(inst)                                      \
    static const struct lsm303agr_config lsm303agr_cfg_##inst = {   \
        DEVICE_DT_GET(DT_BUS(DT_DRV_INST(inst))),                   \
        {                                                           \
            /* I2C definition for accelerometer */                  \
            DEVICE_DT_GET(DT_BUS(DT_DRV_INST(inst))),               \
            DT_REG_ADDR_BY_IDX(DT_DRV_INST(inst), 0),               \
        },                                                          \
        {                                                           \
            /* I2C definition for magnetometer */                   \
            DEVICE_DT_GET(DT_BUS(DT_DRV_INST(inst))),               \
            DT_REG_ADDR_BY_IDX(DT_DRV_INST(inst), 1),               \
        }};                                                         \
    DEVICE_DT_INST_DEFINE(inst, lsm303agr_init,                     \
                          NULL, NULL, &lsm303agr_cfg_##inst,        \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
                          &lsm303agr_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LSM303AGR_DEFINE)