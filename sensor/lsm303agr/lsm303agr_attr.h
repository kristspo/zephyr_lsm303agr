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

#ifdef __cplusplus
}
#endif

#endif