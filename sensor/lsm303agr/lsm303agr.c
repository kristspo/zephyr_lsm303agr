
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

/* initial configuration register values according to configuration options */
const lsm303agr_reg_t ctrl_temp_cfg = {
#ifdef CONFIG_LSM303AGR_MEASURE_TEMPERATURE
    .temp_cfg_reg_a.temp_en = LSM303AGR_TEMP_ENABLE,
#endif
};

const lsm303agr_reg_t ctrl_reg_1 = {
    .ctrl_reg1_a.xen = 1,
    .ctrl_reg1_a.yen = 1,
    .ctrl_reg1_a.zen = 1,
#ifdef CONFIG_LSM303AGR_ACC_OP_MODE_LOW_POWER
    .ctrl_reg1_a.lpen = 1,
#endif
#ifdef CONFIG_LSM303AGR_ACC_ODR_1
    .ctrl_reg1_a.odr = LSM303AGR_XL_ODR_1Hz,
#endif
#ifdef CONFIG_LSM303AGR_ACC_ODR_2
    .ctrl_reg1_a.odr = LSM303AGR_XL_ODR_10Hz,
#endif
#ifdef CONFIG_LSM303AGR_ACC_ODR_3
    .ctrl_reg1_a.odr = LSM303AGR_XL_ODR_25Hz,
#endif
#ifdef CONFIG_LSM303AGR_ACC_ODR_4
    .ctrl_reg1_a.odr = LSM303AGR_XL_ODR_50Hz,
#endif
#ifdef CONFIG_LSM303AGR_ACC_ODR_5
    .ctrl_reg1_a.odr = LSM303AGR_XL_ODR_100Hz,
#endif
#ifdef CONFIG_LSM303AGR_ACC_ODR_6
    .ctrl_reg1_a.odr = LSM303AGR_XL_ODR_200Hz,
#endif
#ifdef CONFIG_LSM303AGR_ACC_ODR_7
    .ctrl_reg1_a.odr = LSM303AGR_XL_ODR_400Hz,
#endif
#ifdef CONFIG_LSM303AGR_ACC_ODR_8
    .ctrl_reg1_a.odr = LSM303AGR_XL_ODR_1kHz620_LP,
#endif
#ifdef CONFIG_LSM303AGR_ACC_ODR_9
    .ctrl_reg1_a.odr = LSM303AGR_XL_ODR_1kHz344_NM_HP_5kHz376_LP,
#endif
};

const lsm303agr_reg_t ctrl_reg_4 = {
#ifdef CONFIG_LSM303AGR_ACC_OP_MODE_HIGH_RES
    .ctrl_reg4_a.hr = 1,
#endif
#ifdef CONFIG_LSM303AGR_ACC_RANGE_2G
    .ctrl_reg4_a.fs = LSM303AGR_2g,
#endif
#ifdef CONFIG_LSM303AGR_ACC_RANGE_4G
    .ctrl_reg4_a.fs = LSM303AGR_4g,
#endif
#ifdef CONFIG_LSM303AGR_ACC_RANGE_8G
    .ctrl_reg4_a.fs = LSM303AGR_8g,
#endif
#ifdef CONFIG_LSM303AGR_ACC_RANGE_16G
    .ctrl_reg4_a.fs = LSM303AGR_16g,
#endif
    .ctrl_reg4_a.bdu = 1,
};

/* get lsm303agr accelerometer register from sensor_attr value */
lsm303agr_reg lsm303agr_acc_reg_get(enum lsm303agr_attribute attr)
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

    default:
        return (lsm303agr_reg){0, 0};
    };
}

/* get lsm303agr magnetometer register from sensor_attr value */
lsm303agr_reg lsm303agr_mag_reg_get(enum lsm303agr_attribute attr)
{
    switch (attr)
    {
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

/* set accelerometer full scale multiplier value */
const int16_t lsm303agr_acc_reg_to_scale[] = {
    // multiplier is expressed in fixed point 12.4 format ;)
    1563,  /* 97.6875  */
    3126,  /* 195.375  */
    6252,  /* 390.75   */
    18756, /* 1172.25  */
};

/* convert accelerometer 12 bit raw reading to sensor value in 1/1000 g */
void lsm303agr_acc_convert(int16_t raw_val, int16_t scale, int32_t *val)
{
    int32_t converted_val = ((raw_val * scale) >> 4) / 100;
    *val = converted_val;
}

/* convert temperature raw reading to sensor_value data */
void lsm303agr_temp_convert(uint8_t *raw_temp, struct sensor_value *val)
{
    // most significiant byte has signed integer relative temperature value in degrees
    val->val1 = (int8_t)raw_temp[1];
    // least significiant byte appears to have additional two bits of fractional part
    // put this as decimal part with three digits in sensor_value.val2
    val->val2 = (raw_temp[0] >> 6) * 250;
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
        reg = lsm303agr_acc_reg_get(attr);
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
        reg = lsm303agr_mag_reg_get(attr);
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
        reg = lsm303agr_acc_reg_get(attr);
        if (reg.addr == 0 || !reg.write)
            return -ENOTSUP;
        else
            return lsm303agr_write_reg(&cfg->i2c_acc, reg.addr, (uint8_t *)&val->val1, 1);

    case SENSOR_CHAN_MAGN_X:
    case SENSOR_CHAN_MAGN_Y:
    case SENSOR_CHAN_MAGN_Z:
    case SENSOR_CHAN_MAGN_XYZ:
        reg = lsm303agr_mag_reg_get(attr);
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
    struct lsm303agr_data *data = dev->data;
    int read_start, read_end;

    switch (chan)
    {
    case SENSOR_CHAN_DIE_TEMP:
        lsm303agr_temp_convert(data->raw_temp, val);
        return 0;

    case SENSOR_CHAN_ACCEL_X:
        read_start = read_end = 0;
        break;

    case SENSOR_CHAN_ACCEL_Y:
        read_start = read_end = 1;
        break;

    case SENSOR_CHAN_ACCEL_Z:
        read_start = read_end = 2;
        break;

    case SENSOR_CHAN_ACCEL_XYZ:
    case SENSOR_CHAN_ALL:
        read_start = 0;
        read_end = 2;
        break;

    default:
        return -ENOTSUP;
    }

    for (int z = read_start; z <= read_end; z++, val++)
    {
        val->val2 = 0;
        lsm303agr_acc_convert(data->acc_sample.xyz[z] >> 4, data->acc_scale, &val->val1);
    }
    return 0;
}

int lsm303agr_sample_fetch(const struct device *dev,
                           enum sensor_channel chan)
{
    const struct lsm303agr_config *cfg = dev->config;
    struct lsm303agr_data *data = dev->data;
    int status;

    switch (chan)
    {
    case SENSOR_CHAN_ALL:
    case SENSOR_CHAN_ACCEL_XYZ:
    case SENSOR_CHAN_ACCEL_X:
    case SENSOR_CHAN_ACCEL_Y:
    case SENSOR_CHAN_ACCEL_Z:
        status = lsm303agr_read_reg(&cfg->i2c_acc, LSM303AGR_STATUS_REG_A | LSM303AGR_AUTOINCR_ADDR, data->acc_sample.raw_xyz, 7);
        if (status < 0)
            return status;
        // check data available bit in status register
        switch (chan)
        {
        case SENSOR_CHAN_ACCEL_X:
            return (data->acc_sample.status & BIT(0)) ? 0 : -ENODATA;
        case SENSOR_CHAN_ACCEL_Y:
            return (data->acc_sample.status & BIT(1)) ? 0 : -ENODATA;
        case SENSOR_CHAN_ACCEL_Z:
            return (data->acc_sample.status & BIT(2)) ? 0 : -ENODATA;
        case SENSOR_CHAN_ALL:
        case SENSOR_CHAN_ACCEL_XYZ:
            return (data->acc_sample.status & BIT(3)) ? 0 : -ENODATA;
        default:
            return -ENOTSUP;
        }

    case SENSOR_CHAN_DIE_TEMP:
        // read internal temperature only if requested
        status = lsm303agr_read_reg(&cfg->i2c_acc, LSM303AGR_OUT_TEMP_L_A | LSM303AGR_AUTOINCR_ADDR, data->raw_temp, 2);
        return status;

    default:
        return -ENOTSUP;
    }
}

static int lsm303agr_init(const struct device *dev)
{
    const struct lsm303agr_config *cfg = dev->config;
    struct lsm303agr_data *data = dev->data;
    int status;
    uint8_t id;
    uint8_t raw[7];

    PRINT("\nSetup lsm303agr on %s\n", cfg->i2c->name);

    /** Verify i2c state and read sensor chip id **/

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

    /** Accelerometer initialization **/

    // Reset accelerometer control registers to default values
    // Accelerometer block does not have reset pin or SW reset register
    memset(raw, 0, sizeof(raw));
    raw[0] = ctrl_temp_cfg.byte; // TEMP_CFG_REG_A
    raw[1] = ctrl_reg_1.byte;    // CTRL_REG1_A
    raw[4] = ctrl_reg_4.byte;    // CTRL_REG4_A

    status = lsm303agr_write_reg(&cfg->i2c_acc, LSM303AGR_TEMP_CFG_REG_A | LSM303AGR_AUTOINCR_ADDR, raw, sizeof(raw));
    if (status < 0)
        return status;

    // Read measurement output and status registers once to clear possible block read update state
    // if previous reading was not completed before reset. This also clears random device data values.
    status = lsm303agr_read_reg(&cfg->i2c_acc, LSM303AGR_STATUS_REG_A | LSM303AGR_AUTOINCR_ADDR, data->acc_sample.raw_xyz, 7);
    if (status < 0)
        return status;
#ifdef CONFIG_LSM303AGR_MEASURE_TEMPERATURE
    status = lsm303agr_read_reg(&cfg->i2c_acc, LSM303AGR_OUT_TEMP_L_A | LSM303AGR_AUTOINCR_ADDR, data->raw_temp, 2);
    if (status < 0)
        return status;
#endif
    data->acc_scale = lsm303agr_acc_reg_to_scale[ctrl_reg_4.ctrl_reg4_a.fs];

    return 0;
}

#define LSM303AGR_DEFINE(inst)                                      \
    static struct lsm303agr_data lsm303agr_data_##inst;             \
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
                          NULL, &lsm303agr_data_##inst,             \
                          &lsm303agr_cfg_##inst,                    \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
                          &lsm303agr_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LSM303AGR_DEFINE)