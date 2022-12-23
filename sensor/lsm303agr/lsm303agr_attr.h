#ifndef LSM303AGR_ENUMS_H
#define LSM303AGR_ENUMS_H

#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // register names as listed in datasheet
    enum lsm303agr_attribute
    {
        ACC_ATTR_START = SENSOR_ATTR_PRIV_START,

        STATUS_REG_AUX_A = ACC_ATTR_START,
        OUT_TEMP_L_A,
        OUT_TEMP_H_A,
        INT_COUNTER_REG_A,
        WHO_AM_I_A,
        TEMP_CFG_REG_A,
        CTRL_REG1_A,
        CTRL_REG2_A,
        CTRL_REG3_A,
        CTRL_REG4_A,
        CTRL_REG5_A,
        CTRL_REG6_A,
        REFERENCE_A,
        STATUS_REG_A,
        OUT_X_L_A,
        OUT_X_H_A,
        OUT_Y_L_A,
        OUT_Y_H_A,
        OUT_Z_L_A,
        OUT_Z_H_A,
        FIFO_CTRL_REG_A,
        FIFO_SRC_REG_A,
        INT1_CFG_A,
        INT1_SRC_A,
        INT1_THS_A,
        INT1_DURATION_A,
        INT2_CFG_A,
        INT2_SRC_A,
        INT2_THS_A,
        INT2_DURATION_A,
        CLICK_CFG_A,
        CLICK_SRC_A,
        CLICK_THS_A,
        TIME_LIMIT_A,
        TIME_LATENCY_A,
        TIME_WINDOW_A,
        ACT_THS_A,
        ACT_DUR_A,
        ACC_ATTR_END,

        MAG_ATTR_START = ACC_ATTR_END,

        OFFSET_X_REG_L_M = MAG_ATTR_START,
        OFFSET_X_REG_H_M,
        OFFSET_Y_REG_L_M,
        OFFSET_Y_REG_H_M,
        OFFSET_Z_REG_L_M,
        OFFSET_Z_REG_H_M,
        WHO_AM_I_M,
        CFG_REG_A_M,
        CFG_REG_B_M,
        CFG_REG_C_M,
        INT_CRTL_REG_M,
        INT_SOURCE_REG_M,
        INT_THS_L_REG_M,
        INT_THS_H_REG_M,
        STATUS_REG_M,
        OUTX_L_REG_M,
        OUTX_H_REG_M,
        OUTY_L_REG_M,
        OUTY_H_REG_M,
        OUTZ_L_REG_M,
        OUTZ_H_REG_M,
        MAG_ATTR_END,
    };

    enum lsm303agr_channel
    {
        SENSOR_CHAN_ACCEL_XYZ_FIFO = SENSOR_CHAN_PRIV_START,
    };

    // LSM303AGR specific trigger types
    enum lsm303agr_trigger
    {
        TRIG_ACC_INT1 = SENSOR_TRIG_PRIV_START,
        TRIG_ACC_INT2,
        TRIG_MAG_INT,
    };

    // LSM303AGR full scale and data rate values
    enum lsm303agr_acc_range
    {
        ACC_RANGE_2G = 2,
        ACC_RANGE_4G = 4,
        ACC_RANGE_8G = 8,
        ACC_RANGE_16G = 16,
    };
    enum lsm303agr_acc_odr
    {
        ACC_ODR_POWERDOWN = 0,
        ACC_ODR_1Hz = 1,
        ACC_ODR_10Hz = 10,
        ACC_ODR_25Hz = 25,
        ACC_ODR_50Hz = 50,
        ACC_ODR_100Hz = 100,
        ACC_ODR_200Hz = 200,
        ACC_ODR_400Hz = 400,
        ACC_ODR_1344Hz = 1344,    // setting this data rate will clear low power mode if active
        ACC_ODR_1620Hz_LP = 1620, // setting this data rate will set low power operating mode
        ACC_ODR_5376Hz_LP = 5376, // setting this data rate will set low power operating mode
    };
    enum lsm303agr_mag_odr
    {
        MAG_ODR_SINGLESHOT = 0,
        MAG_ODR_10Hz = 10,
        MAG_ODR_20Hz = 20,
        MAG_ODR_50Hz = 50,
        MAG_ODR_100Hz = 100,
    };

#define BIT_ACC_INT_CLICK BIT(7)
#define BIT_ACC_INT_AOI1 BIT(6)
#define BIT_ACC_INT_AOI2 BIT(5)
#define BIT_ACC_INT_ACT BIT(3)
#define BIT_ACC_INT_FIFO_WTM BIT(2)
#define BIT_ACC_INT_FIFO_OVR BIT(1)

#define ALLOW_BITS_ACC_INT1 (BIT_ACC_INT_CLICK | BIT_ACC_INT_AOI1 | BIT_ACC_INT_AOI2 | BIT_ACC_INT_FIFO_WTM | BIT_ACC_INT_FIFO_OVR)
#define ALLOW_BITS_ACC_INT2 (BIT_ACC_INT_CLICK | BIT_ACC_INT_AOI1 | BIT_ACC_INT_AOI2 | BIT_ACC_INT_ACT)

#define TRIGGER_BITS_SET(x) (x << 8)
#define TRIGGER_BITS_GET(x) (x & 0x00FF)
#define TRIGGER_SRC_GET(x) (x >> 8)

// accelerometer INT1 and INT2 configuration bits
#define BIT_ACC_CFG_AND_EVT BIT(7)
#define BIT_ACC_CFG_6D_EVT BIT(6)
#define BIT_ACC_CFG_Z_HI_UP BIT(5)
#define BIT_ACC_CFG_Z_LO_DOWN BIT(4)
#define BIT_ACC_CFG_Y_HI_UP BIT(3)
#define BIT_ACC_CFG_Y_LO_DOWN BIT(2)
#define BIT_ACC_CFG_X_HI_UP BIT(1)
#define BIT_ACC_CFG_X_LO_DOWN BIT(0)

// accelerometer CLICK_CFG configuration bits
#define BIT_ACC_CFG_Z_D_CLICK BIT(5)
#define BIT_ACC_CFG_Z_S_CLICK BIT(4)
#define BIT_ACC_CFG_Y_D_CLICK BIT(3)
#define BIT_ACC_CFG_Y_S_CLICK BIT(2)
#define BIT_ACC_CFG_X_D_CLICK BIT(1)
#define BIT_ACC_CFG_X_S_CLICK BIT(0)

// accelerometer FIFO mode configuration
#define ACC_FIFO_OFF 0x00
#define ACC_FIFO_FIFO 0x40
#define ACC_FIFO_STREAM 0x80
#define ACC_FIFO_ON_INT1 0xC0
#define ACC_FIFO_ON_INT2 0xE0

    // magnetometer interrupt configuration
    enum lsm303agr_mag_int
    {
        MAG_INT_OFF,
        MAG_INT_THRS_DEFAULT,
        MAG_INT_THRS_LESS,
        MAG_INT_THRS_BOTH,
        MAG_INT_BIT_THRS_OFFSET = 0x80,
    };

// magnetometer INT_CTRL_REG configuration bits
#define BIT_MAG_CFG_XIEN BIT(7)
#define BIT_MAG_CFG_YIEN BIT(6)
#define BIT_MAG_CFG_ZIEN BIT(5)

#ifdef __cplusplus
}
#endif

#endif