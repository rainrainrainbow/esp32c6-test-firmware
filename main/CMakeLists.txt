idf_component_register(
  SRCS "test_firmware.c"
  INCLUDE_DIRS "."
  REQUIRES driver nvs_flash sdmmc esp_timer esp_driver_gpio esp_driver_ledc
           esp_driver_i2c esp_driver_i2s esp_driver_spi esp_driver_sdspi
)
