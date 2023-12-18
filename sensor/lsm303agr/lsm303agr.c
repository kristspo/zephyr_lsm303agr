
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lsm303agr, CONFIG_SENSOR_LOG_LEVEL);
#include "lsm303agr.h"
#include "lsm303agr_attr.h"
#include "lsm303agr_reg.h"

#define DT_DRV_COMPAT st_lsm303agr

static const struct sensor_driver_api lsm303agr_driver_api = {
    .attr_set = lsm303agr_attr_set,
    .attr_get = lsm303agr_attr_get,
    .trigger_set = lsm303agr_trigger_set,
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

const lsm303agr_reg_t cfg_reg_a = {
#ifdef CONFIG_LSM303AGR_MAG_ODR_SINGLESHOT
    .cfg_reg_a_m.md = LSM303AGR_IDLE_DEFAULT,
#else
    .cfg_reg_a_m.md = LSM303AGR_CONTINUOUS_MODE,
#endif
#ifdef CONFIG_LSM303AGR_MAG_ODR_20HZ
    .cfg_reg_a_m.odr = LSM303AGR_MG_ODR_20Hz,
#endif
#ifdef CONFIG_LSM303AGR_MAG_ODR_50HZ
    .cfg_reg_a_m.odr = LSM303AGR_MG_ODR_50Hz,
#endif
#ifdef CONFIG_LSM303AGR_MAG_ODR_100HZ
    .cfg_reg_a_m.odr = LSM303AGR_MG_ODR_100Hz,
#endif
#ifdef CONFIG_LSM303AGR_MAG_OP_MODE_LOW_POWER
    .cfg_reg_a_m.lp = 1,
#endif
    .cfg_reg_a_m.comp_temp_en = 1,
};

const lsm303agr_reg_t cfg_reg_b = {
    .cfg_reg_b_m.set_rst = 1,
    .cfg_reg_b_m.off_canc_one_shot = 1,
};

const lsm303agr_reg_t cfg_reg_c = {
    .cfg_reg_c_m.bdu = 1,
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

/* set accelerometer interrupt threshold register LSB value */
const int16_t lsm303agr_acc_reg_to_thrs[] = {
    16,
    32,
    62,
    186,
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

int lsm303agr_acc_range_set(const struct device *dev, int32_t range)
{
    const struct lsm303agr_config *cfg = dev->config;
    struct lsm303agr_data *data = dev->data;
    lsm303agr_fs_a_t fs_bits;
    int status;

    switch (range) // range in g
    {
    case 2:
        fs_bits = LSM303AGR_2g;
        break;
    case 4:
        fs_bits = LSM303AGR_4g;
        break;
    case 8:
        fs_bits = LSM303AGR_8g;
        break;
    case 16:
        fs_bits = LSM303AGR_16g;
        break;
    default:
        return -EINVAL;
    }

    status = lsm303agr_xl_full_scale_set(&cfg->i2c_acc, fs_bits);
    if (status == 0)
    {
        data->acc_conv_scale = lsm303agr_acc_reg_to_scale[fs_bits];
        data->acc_thrs_scale = lsm303agr_acc_reg_to_thrs[fs_bits];
    }

    return status;
}

int lsm303agr_acc_odr_set(const struct device *dev, int32_t freq)
{
    const struct lsm303agr_config *cfg = dev->config;
    struct lsm303agr_data *data = dev->data;
    lsm303agr_ctrl_reg1_a_t ctrl_reg;
    lsm303agr_odr_a_t odr_bits;
    int status;

    switch (freq) // output data date in Hz
    {
    case 0:
        odr_bits = LSM303AGR_XL_POWER_DOWN;
        break;
    case 1:
        odr_bits = LSM303AGR_XL_ODR_1Hz;
        break;
    case 10:
        odr_bits = LSM303AGR_XL_ODR_10Hz;
        break;
    case 25:
        odr_bits = LSM303AGR_XL_ODR_25Hz;
        break;
    case 50:
        odr_bits = LSM303AGR_XL_ODR_50Hz;
        break;
    case 100:
        odr_bits = LSM303AGR_XL_ODR_100Hz;
        break;
    case 200:
        odr_bits = LSM303AGR_XL_ODR_200Hz;
        break;
    case 400:
        odr_bits = LSM303AGR_XL_ODR_400Hz;
        break;
    case 1620:
        odr_bits = LSM303AGR_XL_ODR_1kHz620_LP;
        break;
    case 1344:
    case 5376:
        odr_bits = LSM303AGR_XL_ODR_1kHz344_NM_HP_5kHz376_LP;
        break;
    default:
        return -EINVAL;
    }

    if (odr_bits == LSM303AGR_XL_ODR_1kHz620_LP || odr_bits == LSM303AGR_XL_ODR_1kHz344_NM_HP_5kHz376_LP)
    {
        status = lsm303agr_read_reg(&cfg->i2c_acc, LSM303AGR_CTRL_REG1_A, (uint8_t *)&ctrl_reg, 1);
        if (status < 0)
            return status;

        if (freq == 1344 && ctrl_reg.lpen)
        {
            // clear low power mode to use 1.344 kHz data rate
            ctrl_reg.lpen = 0;
            status = lsm303agr_write_reg(&cfg->i2c_acc, LSM303AGR_CTRL_REG1_A, (uint8_t *)&ctrl_reg, 1);
            if (status < 0)
                return status;
        }
        if (freq > 1344 && !ctrl_reg.lpen)
        {
            // set low power mode to use 1.620 or 5.376 kHz data rate
            status = lsm303agr_xl_operating_mode_set(&cfg->i2c_acc, LSM303AGR_LP_8bit);
            if (status < 0)
                return status;
        }
    }

    status = lsm303agr_xl_data_rate_set(&cfg->i2c_acc, odr_bits);
    if (status < 0)
        return status;

    if (status == 0)
        data->status.acc_rate = odr_bits;

    return status;
}

int lsm303agr_mag_odr_set(const struct device *dev, int32_t freq)
{
    const struct lsm303agr_config *cfg = dev->config;
    struct lsm303agr_data *data = dev->data;
    lsm303agr_mg_odr_m_t odr_bits;
    lsm303agr_md_m_t op_bits = LSM303AGR_CONTINUOUS_MODE;
    int status;

    switch (freq) // output data date in Hz
    {
    case 0:
        op_bits = LSM303AGR_IDLE_DEFAULT;
    case 10:
        odr_bits = LSM303AGR_MG_ODR_10Hz;
        break;
    case 20:
        odr_bits = LSM303AGR_MG_ODR_20Hz;
        break;
    case 50:
        odr_bits = LSM303AGR_MG_ODR_50Hz;
        break;
    case 100:
        odr_bits = LSM303AGR_MG_ODR_100Hz;
        break;
    default:
        return -EINVAL;
    }

    status = lsm303agr_mag_data_rate_set(&cfg->i2c_mag, odr_bits);
    if (status < 0)
        return status;

    status = lsm303agr_mag_operating_mode_set(&cfg->i2c_mag, op_bits);
    if (status < 0)
        return status;

    if (status == 0)
    {
        data->status.mag_single_shot = (bool)op_bits;
        data->status.mag_rate = odr_bits;
    }

    return status;
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
        if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY)
            return lsm303agr_acc_odr_set(dev, val->val1);
        else if (attr == SENSOR_ATTR_FULL_SCALE)
            return lsm303agr_acc_range_set(dev, val->val1);
        else
        {
            reg = lsm303agr_acc_reg_get(attr);
            if (reg.addr == 0 || !reg.write)
                return -ENOTSUP;
            else
                return lsm303agr_write_reg(&cfg->i2c_acc, reg.addr, (uint8_t *)&val->val1, 1);
        }

    case SENSOR_CHAN_MAGN_X:
    case SENSOR_CHAN_MAGN_Y:
    case SENSOR_CHAN_MAGN_Z:
    case SENSOR_CHAN_MAGN_XYZ:
        if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY)
            return lsm303agr_mag_odr_set(dev, val->val1);
        else
        {
            reg = lsm303agr_mag_reg_get(attr);
            if (reg.addr == 0 || !reg.write)
                return -ENOTSUP;
            else
                return lsm303agr_write_reg(&cfg->i2c_mag, reg.addr, (uint8_t *)&val->val1, 1);
        }

    default:
        return -ENOTSUP;
    }

    return 0;
}

int lsm303agr_channel_get(const struct device *dev,
                          enum sensor_channel chan,
                          struct sensor_value *val)
{
    const struct lsm303agr_config *cfg = dev->config;
    struct lsm303agr_data *data = dev->data;
    int read_start_a, read_end_a;
    int read_start_m, read_end_m;
    bool read_acc = false;
    bool read_mag = false;

    switch (chan)
    {
    case SENSOR_CHAN_DIE_TEMP:
        lsm303agr_temp_convert(data->raw_temp, val);
        return 0;

    case SENSOR_CHAN_ACCEL_XYZ_FIFO:
        if (data->status.fifo_ready && data->fifo_size)
        {
#ifdef CONFIG_LSM303AGR_FIFO_BURST_READ
            uint16_t read_count = data->fifo_size * 3; // samples count
            // refer to FIFO multiple read (burst) in datasheet
            int status = lsm303agr_read_reg(&cfg->i2c_acc, LSM303AGR_OUT_X_L_A | LSM303AGR_AUTOINCR_ADDR, data->fifo_buffer.raw_xyz, read_count * 2);
            if (status < 0)
                return status;
            for (uint16_t z = 0; z < read_count; z++, val++)
            {
                val->val2 = 0;
                lsm303agr_acc_convert(data->fifo_buffer.xyz[z] >> 4, data->acc_conv_scale, &val->val1);
            }
#else
            uint16_t read_count = data->fifo_size; // xyz samples count
            while (read_count--)
            {
                int16_t xyz[3];
                int status = lsm303agr_read_reg(&cfg->i2c_acc, LSM303AGR_OUT_X_L_A | LSM303AGR_AUTOINCR_ADDR, (uint8_t *)xyz, sizeof(xyz));
                if (status < 0)
                    return status;
                for (uint16_t y = 0; y < 3; y++, val++)
                {
                    val->val2 = 0;
                    lsm303agr_acc_convert(xyz[y] >> 4, data->acc_conv_scale, &val->val1);
                }
            }
#endif
            data->status.fifo_ready = false;
            return 0;
        }
        else
            return -ENODATA;

    case SENSOR_CHAN_ACCEL_X:
        read_acc = true;
        read_start_a = read_end_a = 0;
        break;

    case SENSOR_CHAN_ACCEL_Y:
        read_acc = true;
        read_start_a = read_end_a = 1;
        break;

    case SENSOR_CHAN_ACCEL_Z:
        read_acc = true;
        read_start_a = read_end_a = 2;
        break;

    case SENSOR_CHAN_ACCEL_XYZ:
        read_acc = true;
        read_start_a = 0;
        read_end_a = 2;
        break;

    case SENSOR_CHAN_MAGN_X:
        read_mag = true;
        read_start_m = read_end_m = 0;
        break;

    case SENSOR_CHAN_MAGN_Y:
        read_mag = true;
        read_start_m = read_end_m = 1;
        break;

    case SENSOR_CHAN_MAGN_Z:
        read_mag = true;
        read_start_m = read_end_m = 2;
        break;

    case SENSOR_CHAN_MAGN_XYZ:
        read_mag = true;
        read_start_m = 0;
        read_end_m = 2;
        break;

    case SENSOR_CHAN_ALL:
        read_acc = read_mag = true;
        read_start_a = read_start_m = 0;
        read_end_a = read_end_m = 2;
        break;

    default:
        return -ENOTSUP;
    }

    if (read_acc)
    {
        for (int z = read_start_a; z <= read_end_a; z++, val++)
        {
            val->val2 = 0;
            lsm303agr_acc_convert(data->acc_sample.xyz[z] >> 4, data->acc_conv_scale, &val->val1);
        }
    }
    if (read_mag)
    {
        for (int z = read_start_m; z <= read_end_m; z++, val++)
        {
            val->val2 = 0;
            val->val1 = (data->mag_sample.xyz[z] * 3) >> 1; // 1 LSB is 1.5 milli gauss
        }
    }

    return 0;
}

int lsm303agr_sample_fetch(const struct device *dev,
                           enum sensor_channel chan)
{
    const struct lsm303agr_config *cfg = dev->config;
    struct lsm303agr_data *data = dev->data;
    lsm303agr_sample raw_mag;
    bool newsample = true;
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
            newsample = newsample && (data->acc_sample.status & BIT(0));
        case SENSOR_CHAN_ACCEL_Y:
            newsample = newsample && (data->acc_sample.status & BIT(1));
        case SENSOR_CHAN_ACCEL_Z:
            newsample = newsample && (data->acc_sample.status & BIT(2));
        default:
            newsample = newsample && (data->acc_sample.status & BIT(3));
        }
        if (chan != SENSOR_CHAN_ALL)
            return newsample ? 0 : -ENODATA;

    case SENSOR_CHAN_MAGN_XYZ:
    case SENSOR_CHAN_MAGN_X:
    case SENSOR_CHAN_MAGN_Y:
    case SENSOR_CHAN_MAGN_Z:
        if (data->status.mag_single_shot)
        {
            status = lsm303agr_mag_operating_mode_set(&cfg->i2c_mag, LSM303AGR_SINGLE_TRIGGER);
            if (status < 0)
                return status;
            k_msleep(15);
            status = lsm303agr_read_reg(&cfg->i2c_mag, LSM303AGR_STATUS_REG_M | LSM303AGR_AUTOINCR_ADDR, raw_mag.raw_xyz, 7);
            if (status < 0)
                return status;
            status = lsm303agr_mag_operating_mode_set(&cfg->i2c_mag, LSM303AGR_SINGLE_TRIGGER);
            if (status < 0)
                return status;
            k_msleep(15);
        }
        status = lsm303agr_read_reg(&cfg->i2c_mag, LSM303AGR_STATUS_REG_M | LSM303AGR_AUTOINCR_ADDR, data->mag_sample.raw_xyz, 7);
        if (status < 0)
            return status;
        if (data->status.mag_single_shot)
        {
            // refer to AN5069 section Magnetometer offset cancellation
            data->mag_sample.xyz[0] += raw_mag.xyz[0];
            data->mag_sample.xyz[1] += raw_mag.xyz[1];
            data->mag_sample.xyz[2] += raw_mag.xyz[2];
            data->mag_sample.xyz[0] >>= 1;
            data->mag_sample.xyz[1] >>= 1;
            data->mag_sample.xyz[2] >>= 1;
        }

        // check data available bit in status register
        switch (chan)
        {
        case SENSOR_CHAN_MAGN_X:
            newsample = newsample && (data->mag_sample.status & BIT(0));
        case SENSOR_CHAN_MAGN_Y:
            newsample = newsample && (data->mag_sample.status & BIT(1));
        case SENSOR_CHAN_MAGN_Z:
            newsample = newsample && (data->mag_sample.status & BIT(2));
        default:
            newsample = newsample && (data->mag_sample.status & BIT(3));
        }
        return newsample ? 0 : -ENODATA;

    case SENSOR_CHAN_DIE_TEMP:
        // read internal temperature only if requested
        status = lsm303agr_read_reg(&cfg->i2c_acc, LSM303AGR_OUT_TEMP_L_A | LSM303AGR_AUTOINCR_ADDR, data->raw_temp, 2);
        return status;

    default:
        return -ENOTSUP;
    }
}

int lsm303agr_fifo_set(const struct i2c_dt_spec *ctx, bool enable)
{
    int status;
    if (enable)
    {
        // verify FIFO configuration
        lsm303agr_fm_a_t fifo_mode;
        status = lsm303agr_xl_fifo_mode_get(ctx, &fifo_mode);
        if (status < 0)
            return status;
        if (fifo_mode == LSM303AGR_BYPASS_MODE)
            return -ENODATA;
        // enable FIFO
        status = lsm303agr_xl_fifo_set(ctx, 1);
        if (status < 0)
            return status;
    }
    else
    {
        // disable FIFO
        status = lsm303agr_xl_fifo_set(ctx, 0);
        if (status < 0)
            return status;
        status = lsm303agr_xl_fifo_mode_set(ctx, LSM303AGR_BYPASS_MODE);
        if (status < 0)
            return status;
    }
    return 0;
}

int lsm303agr_trigger_set(const struct device *dev,
                          const struct sensor_trigger *trig,
                          sensor_trigger_handler_t handler)
{
    const struct lsm303agr_config *cfg = dev->config;
    struct lsm303agr_data *data = dev->data;
    int status;

    uint16_t trig_type = (trig->type & 0x00FF);
    uint8_t trig_bits = (trig->type >> 8);

    switch (trig->chan)
    {
    case SENSOR_CHAN_ACCEL_X:
    case SENSOR_CHAN_ACCEL_Y:
    case SENSOR_CHAN_ACCEL_Z:
    case SENSOR_CHAN_ACCEL_XYZ:
        switch (trig_type)
        {
        case TRIG_ACC_INT1:
            if (cfg->gpio_acc_int1.port)
            {
                if ((trig_bits == 0x00) || (trig_bits & ALLOW_BITS_ACC_INT1))
                {
                    if ((trig_bits == 0x00) && (data->acc_int1.enable & (BIT_ACC_INT_FIFO_WTM | BIT_ACC_INT_FIFO_OVR)))
                    {
                        data->status.fifo_ready = false;
                        status = lsm303agr_fifo_set(&cfg->i2c_acc, false);
                        if (status < 0)
                            return status;
                    }
                    else if (trig_bits & (BIT_ACC_INT_FIFO_WTM | BIT_ACC_INT_FIFO_OVR))
                    {
                        data->status.fifo_ready = false;
                        status = lsm303agr_fifo_set(&cfg->i2c_acc, true);
                        if (status < 0)
                            return status;
                    }

                    // set INT1 configuration register
                    trig_bits &= ALLOW_BITS_ACC_INT1;
                    status = lsm303agr_write_reg(&cfg->i2c_acc, LSM303AGR_CTRL_REG3_A, &trig_bits, 1);
                    if (status < 0)
                        return status;

                    data->acc_int1.enable = trig_bits;
                    data->acc_int1.handler = handler;
                    // set gpio pin interrupt
                    status = lsm303agr_gpio_int_set(&cfg->gpio_acc_int1, (trig_bits && handler) ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE);
                    if (status < 0)
                        return status;

                    return 0;
                }
                else
                    return -ENOTSUP;
            }
            else
            {
                LOG_ERR("INT1 pin not defined in device tree");
                return -ENODEV;
            }

        case TRIG_ACC_INT2:
            if (cfg->gpio_acc_int2.port)
            {
                if ((trig_bits == 0x00) || (trig_bits & ALLOW_BITS_ACC_INT2))
                {
                    // set INT2 configuration register
                    trig_bits &= ALLOW_BITS_ACC_INT2;
                    status = lsm303agr_write_reg(&cfg->i2c_acc, LSM303AGR_CTRL_REG6_A, &trig_bits, 1);
                    if (status < 0)
                        return status;

                    data->acc_int2.enable = trig_bits;
                    data->acc_int2.handler = handler;
                    // set gpio pin interrupt
                    status = lsm303agr_gpio_int_set(&cfg->gpio_acc_int2, (trig_bits && handler) ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE);
                    if (status < 0)
                        return status;

                    return 0;
                }
                else
                    return -ENOTSUP;
            }
            else
            {
                LOG_ERR("INT2 pin not defined in device tree");
                return -ENODEV;
            }

        case TRIG_POLLING:
#ifdef CONFIG_LSM303AGR_INTERRUPT_POLLING
            return lsm303agr_polling_init(dev, trig_bits, handler);
#else
            LOG_ERR("Interrupt polling not enabled");
#endif
        default:
            return -ENOTSUP;
        }

    case SENSOR_CHAN_MAGN_X:
    case SENSOR_CHAN_MAGN_Y:
    case SENSOR_CHAN_MAGN_Z:
    case SENSOR_CHAN_MAGN_XYZ:
        switch (trig_type)
        {
        case TRIG_MAG_INT:
            if (cfg->gpio_mag_int0.port)
            {
                lsm303agr_int_crtl_reg_m_t ctrl_reg;
                gpio_flags_t int_flags = GPIO_INT_DISABLE;

                // read INT_CTRL_REG_M
                status = lsm303agr_mag_int_gen_conf_get(&cfg->i2c_mag, &ctrl_reg);
                if (status < 0)
                    return status;

                ctrl_reg.ien = 1;
                ctrl_reg.iel = 0;
                ctrl_reg.iea = 1;

                switch (trig_bits & (MAG_INT_BIT_THRS_OFFSET - 1))
                {
                case MAG_INT_OFF:
                    ctrl_reg.ien = 0;
                    // clear gpio pin interrupt
                    status = lsm303agr_gpio_int_set(&cfg->gpio_mag_int0, GPIO_INT_DISABLE);
                    if (status < 0)
                        return status;
                    break;
                case MAG_INT_THRS_DEFAULT:
                    int_flags = GPIO_INT_EDGE_TO_ACTIVE;
                    break;
                case MAG_INT_THRS_LESS:
                    int_flags = GPIO_INT_EDGE_TO_INACTIVE;
                    break;
                case MAG_INT_THRS_BOTH:
                    int_flags = GPIO_INT_EDGE_BOTH;
                    break;
                default:
                    return -ENOTSUP;
                }

                if (ctrl_reg.ien)
                {
                    // update CGF_REG_B_M INT_ON_DATAOFF bit
                    status = lsm303agr_mag_offset_int_conf_set(&cfg->i2c_mag, (trig_bits & MAG_INT_BIT_THRS_OFFSET) ? 1 : 0);
                    if (status < 0)
                        return status;
                }

                // update INT_CTRL_REG_M IEA, IEL, IEN bits
                status = lsm303agr_mag_int_gen_conf_set(&cfg->i2c_mag, &ctrl_reg);
                if (status < 0)
                    return status;

                // update CGF_REG_C_M INT_MAG_PIN bit
                status = lsm303agr_mag_int_on_pin_set(&cfg->i2c_mag, ctrl_reg.ien);
                if (status < 0)
                    return status;

                data->mag_int0.enable = trig_bits;
                data->mag_int0.handler = handler;

                if (ctrl_reg.ien)
                {
                    k_msleep(10);
                    // set gpio pin interrupt
                    status = lsm303agr_gpio_int_set(&cfg->gpio_mag_int0, int_flags);
                    if (status < 0)
                        return status;
                }

                return 0;
            }
            else
            {
                LOG_ERR("INT_MAG pin not defined in device tree");
                return -ENODEV;
            }

        case TRIG_POLLING:
#ifdef CONFIG_LSM303AGR_INTERRUPT_POLLING
            return lsm303agr_polling_init(dev, trig_bits, handler);
#else
            LOG_ERR("Interrupt polling not enabled");
#endif
        default:
            return -ENOTSUP;
        }

    default:
        return -ENOTSUP;
    }
}

int lsm303agr_init(const struct device *dev)
{
    const struct lsm303agr_config *cfg = dev->config;
    struct lsm303agr_data *data = dev->data;
    int status;
    uint8_t id;
    uint8_t raw[7];

    PRINT("\nSetup lsm303agr on %s\n", cfg->i2c->name);
    if (cfg->gpio_acc_int1.port)
        PRINT("Interrupt pin %s %d \n", cfg->gpio_acc_int1.port->name, cfg->gpio_acc_int1.pin);
    if (cfg->gpio_acc_int2.port)
        PRINT("Interrupt pin %s %d \n", cfg->gpio_acc_int2.port->name, cfg->gpio_acc_int2.pin);
    if (cfg->gpio_mag_int0.port)
        PRINT("Interrupt pin %s %d \n", cfg->gpio_mag_int0.port->name, cfg->gpio_mag_int0.pin);

    /** Verify i2c state and read sensor chip id **/

    if (!device_is_ready(cfg->i2c))
        return -ENODEV;

#ifdef CONFIG_LSM303AGR_RESET_I2C
    /* Recover I2C bus */
    status = i2c_recover_bus(cfg->i2c_acc.bus);
    if (status < 0)
        return status;
#endif

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

    status = lsm303agr_write_reg(&cfg->i2c_acc, LSM303AGR_FIFO_CTRL_REG_A, &raw[6], 1);
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
    data->acc_conv_scale = lsm303agr_acc_reg_to_scale[ctrl_reg_4.ctrl_reg4_a.fs];
    data->acc_thrs_scale = lsm303agr_acc_reg_to_thrs[ctrl_reg_4.ctrl_reg4_a.fs];
    data->status.acc_rate = ctrl_reg_1.ctrl_reg1_a.odr;

    /** Magnetometer initialization **/

    // Set soft reset bit
    status = lsm303agr_mag_reset_set(&cfg->i2c_mag, true);
    if (status < 0)
        return status;

    k_busy_wait(5);

    // Set default configuration
    raw[0] = cfg_reg_a.byte; // CFG_REG_A_M
    raw[1] = cfg_reg_b.byte; // CFG_REG_B_M
    raw[2] = cfg_reg_c.byte; // CFG_REG_C_M

    status = lsm303agr_write_reg(&cfg->i2c_mag, LSM303AGR_CFG_REG_A_M | LSM303AGR_AUTOINCR_ADDR, raw, 3);
    if (status < 0)
        return status;

    data->status.mag_single_shot = (bool)cfg_reg_a.cfg_reg_a_m.md;
    data->status.mag_rate = cfg_reg_a.cfg_reg_a_m.odr;

#ifdef CONFIG_LSM303AGR_INTERRUPT_POLLING
    if (true)
#else
    if (cfg->gpio_acc_int1.port || cfg->gpio_acc_int2.port || cfg->gpio_mag_int0.port)
#endif
    {
        status = lsm303agr_init_gpios(dev);
        if (status < 0)
            return status;
    }

    return 0;
}

#define GPIO_DT_SPEC_INST_GET_BY_IDX_COND(id, prop, idx)       \
    COND_CODE_1(DT_INST_PROP_HAS_IDX(id, prop, idx),           \
                (GPIO_DT_SPEC_INST_GET_BY_IDX(id, prop, idx)), \
                ({.port = NULL, .pin = 0, .dt_flags = 0}))

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
        },                                                          \
        GPIO_DT_SPEC_INST_GET_BY_IDX_COND(inst, irq_acc_gpios, 0),  \
        GPIO_DT_SPEC_INST_GET_BY_IDX_COND(inst, irq_acc_gpios, 1),  \
        GPIO_DT_SPEC_INST_GET_BY_IDX_COND(inst, irq_mag_gpios, 0),  \
    };                                                              \
    DEVICE_DT_INST_DEFINE(inst, lsm303agr_init,                     \
                          NULL, &lsm303agr_data_##inst,             \
                          &lsm303agr_cfg_##inst,                    \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
                          &lsm303agr_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LSM303AGR_DEFINE)