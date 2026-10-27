idf_component_register(
  SRCS "test_firmware.c"
  INCLUDE_DIRS "."
  REQUIRES driver nvs_flash sdmmc esp_timer esp_driver_sdmmc esp_driver_i2s
)
