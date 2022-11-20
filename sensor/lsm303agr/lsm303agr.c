
#include <zephyr/drivers/sensor.h>

#define DT_DRV_COMPAT st_lsm303agr

static const struct sensor_driver_api lsm303agr_driver_api = {
    // no function calls defined yet
};

static int lsm303agr_init(const struct device *dev)
{
    return 0;
}

#define LSM303AGR_DEFINE(inst)                                      \
    DEVICE_DT_INST_DEFINE(inst, lsm303agr_init,                     \
                          NULL, NULL, NULL,                         \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
                          &lsm303agr_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LSM303AGR_DEFINE)