
### 13 Dec 2022

Add `sensor_attr_set` parameters to change full scale range and output data rate for accelerometer block and output data rate for magnetometer block.

Attributes `SENSOR_ATTR_FULL_SCALE` and `SENSOR_ATTR_SAMPLING_FREQUENCY` can be used with channel `SENSOR_CHAN_ACCEL_XYZ`. Attribute `SENSOR_ATTR_SAMPLING_FREQUENCY` can be used with channel `SENSOR_CHAN_MAGN_XYZ`. Value to set is specified in `sensor_value` member `val1`.

Following value presets are added to set `SENSOR_CHAN_ACCEL_XYZ` attribute `SENSOR_ATTR_FULL_SCALE`: \
`ACC_RANGE_2G` \
`ACC_RANGE_4G` \
`ACC_RANGE_8G` \
`ACC_RANGE_16G`

Following value presets are added to set `SENSOR_CHAN_ACCEL_XYZ` attribute `SENSOR_ATTR_SAMPLING_FREQUENCY`: \
`ACC_ODR_POWERDOWN` \
`ACC_ODR_1Hz` \
`ACC_ODR_10Hz` \
`ACC_ODR_25Hz` \
`ACC_ODR_50Hz` \
`ACC_ODR_100Hz` \
`ACC_ODR_200Hz` \
`ACC_ODR_400Hz` \
`ACC_ODR_1344Hz` (setting this data rate will clear low power mode if it is currently active) \
`ACC_ODR_1620Hz_LP` (setting this data rate will set low power mode if it is currently not active) \
`ACC_ODR_5376Hz_LP` (setting this data rate will set low power mode if it is currently not active)

Following value presets are added to set `SENSOR_CHAN_MAGN_XYZ` attribute `SENSOR_ATTR_FULL_SCALE`: \
`MAG_ODR_SINGLESHOT` \
`MAG_ODR_10Hz` \
`MAG_ODR_20Hz` \
`MAG_ODR_50Hz` \
`MAG_ODR_100Hz`

**Kconfig options added to set initial output data rate of magnetometer:**

`CONFIG_LSM303AGR_MAG_ODR_SINGLESHOT` (default value if not set - measurement is read at each `sensor_sample_fetch` call) \
`CONFIG_LSM303AGR_MAG_ODR_10HZ` \
`CONFIG_LSM303AGR_MAG_ODR_20HZ` \
`CONFIG_LSM303AGR_MAG_ODR_50HZ` \
`CONFIG_LSM303AGR_MAG_ODR_100HZ`

### 10 Dec 2022

Add `sensor_trigger_set` to enable accelerometer INT1 and INT2 interrupts. INT1 and INT2 interrupt gpio pins are configured according to device tree property `irq-acc-gpios`. Just one (INT1) or both (INT1 and INT2) gpios can be defined. Gpio pins should be set as `GPIO_ACTIVE_HIGH` or `(GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN)`.

To use interrupt corresponding interrupt `CFG`, `THS` and `DURATION` registers need to be set using `sensor_attr_set`. For click interrupts three `TIME_xxx` registers are used to set duration parameters.

Following bit defines are added that can be used to set `INT1_CFG_A` and `INT2_CFG_A` register: \
`BIT_ACC_CFG_AND_EVT` \
`BIT_ACC_CFG_6D_EVT` \
`BIT_ACC_CFG_Z_HI_UP` \
`BIT_ACC_CFG_Z_LO_DOWN` \
`BIT_ACC_CFG_Y_HI_UP` \
`BIT_ACC_CFG_Y_LO_DOWN` \
`BIT_ACC_CFG_X_HI_UP` \
`BIT_ACC_CFG_X_LO_DOWN`

Following bit defines are added that can be used to set `CLICK_CFG_A` register: \
`BIT_ACC_CFG_Z_D_CLICK` \
`BIT_ACC_CFG_Z_S_CLICK` \
`BIT_ACC_CFG_Y_D_CLICK` \
`BIT_ACC_CFG_Y_S_CLICK` \
`BIT_ACC_CFG_X_D_CLICK` \
`BIT_ACC_CFG_X_S_CLICK`

Interrupt is configured to INT1 or INT2 pin according to `sensor_trigger` parameter passed to `sensor_trigger_set` call. Member `chan` should be set to `SENSOR_CHAN_ACCEL_XYZ` (also `SENSOR_CHAN_ACCEL_X`, `SENSOR_CHAN_ACCEL_Y`, `SENSOR_CHAN_ACCEL_Z` can be used with same result).

Member `type` encodes interrupt configuration where lowest 8 bits of 16 bit value defines INT pin. Driver expects `enum lsm303agr_trigger` values `TRIG_ACC_INT1`, `TRIG_ACC_INT2`, `TRIG_MAG_INT`. Highest 8 bits defines interrupt type that will be configured for interrupt pin.

Following bit defines are added to set interrupt enable bits: \
`BIT_ACC_INT_CLICK` \
`BIT_ACC_INT_AOI1` \
`BIT_ACC_INT_AOI2` \
`BIT_ACC_INT_ACT` \
`BIT_ACC_INT_FIFO_WTM` \
`BIT_ACC_INT_FIFO_OVR`

Bits (or possibly bitmask) can be put in correct location in `sensor_trigger` member `type` using macro `TRIGGER_BITS_SET()`. As for example: `TRIG_ACC_INT1 | TRIGGER_BITS_SET(BIT_ACC_INT_AOI2)` would configure INT2 interrupt block to act on INT1 interrupt pin.

Each of `TRIG_ACC_INT1`, `TRIG_ACC_INT2`, `TRIG_MAG_INT` interrupts can use separate trigger handler callback function. If highest bits of `sensor_trigger` member `type` bits are all cleared or trigger handler function is passed as `NULL` interrupt line is disabled.

### 09 Dec 2022

Add `CONFIG_LSM303AGR_RESET_I2C` configuration option to use I2C recover function at driver startup.

### 02 Dec 2022

Add `sensor_sample_fetch` and `sensor_channel_get` functions to read LSM303AGR magnetometer block. Channels `SENSOR_CHAN_MAGN_X`, `SENSOR_CHAN_MAGN_Y`, `SENSOR_CHAN_MAGN_Z`, `SENSOR_CHAN_MAGN_XYZ` are supported. Using `sensor_channel_get` with `SENSOR_CHAN_ALL` will return accelerometer and magnetometer readings by setting six `sensor_value` variables.

**Kconfig options implemented to set initial state of magnetometer:**

Magnetometer operating mode: \
`LSM303AGR_MAG_OP_MODE_HIGH_RES` (default value if not set) \
`LSM303AGR_MAG_OP_MODE_LOW_POWER`

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
