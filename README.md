
### 30 Nov 2022

Add `sensor_sample_fetch` and `sensor_channel_get` functions to read LSM303AGR accelerometer block. Channels `SENSOR_CHAN_ACCEL_X`, `SENSOR_CHAN_ACCEL_Y`, `SENSOR_CHAN_ACCEL_Z`, `SENSOR_CHAN_ACCEL_XYZ` are supported.

**Kconfig options implemented to set initial state of accelerometer:**

Accelerometer operating mode - power usage vs resolution selection: \
`CONFIG_LSM303AGR_ACC_OP_MODE_NORMAL` (default value if not set with 10 bit output) \
`CONFIG_LSM303AGR_ACC_OP_MODE_LOW_POWER` (8 bit output) \
`CONFIG_LSM303AGR_ACC_OP_MODE_HIGH_RES` (12 bit output)

Accelerometer measurement range: \
`CONFIG_LSM303AGR_ACC_RANGE_2G` (default value if not set) \
`CONFIG_LSM303AGR_ACC_RANGE_4G` \
`CONFIG_LSM303AGR_ACC_RANGE_8G` \
`CONFIG_LSM303AGR_ACC_RANGE_16G`

Accelerometer output data rate selection: \
`CONFIG_LSM303AGR_ACC_ODR_0` (default value if not set - power down mode) \
`CONFIG_LSM303AGR_ACC_ODR_1` (1Hz) \
`CONFIG_LSM303AGR_ACC_ODR_2` (10Hz) \
`CONFIG_LSM303AGR_ACC_ODR_3` (25Hz) \
`CONFIG_LSM303AGR_ACC_ODR_4` (50Hz) \
`CONFIG_LSM303AGR_ACC_ODR_5` (100Hz) \
`CONFIG_LSM303AGR_ACC_ODR_6` (200Hz) \
`CONFIG_LSM303AGR_ACC_ODR_7` (400Hz) \
`CONFIG_LSM303AGR_ACC_ODR_8` (1.620KHz available in low power mode only) \
`CONFIG_LSM303AGR_ACC_ODR_9` (1.344KHz or 5.376KHz depending on operating mode)

Enable or disable internal temperature sensor: \
`CONFIG_LSM303AGR_MEASURE_TEMPERATURE` (if not specified default state is temperature sensor off)

Temperature sensor is part of accelerometer block. As temperature sensor does not seem to be very useful temperature measurements are not processed by sensor_sample_fetch but can be requested using `sensor_sample_fetch_chan` using `SENSOR_CHAN_DIE_TEMP` as channel value.