
if(CONFIG_LSM303AGR)
    message(NOTICE "Add module ${ZEPHYR_LSM303AGR_MODULE_DIR}")
    add_subdirectory(lsm303agr)
endif()
