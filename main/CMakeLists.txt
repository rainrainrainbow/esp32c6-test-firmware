idf_component_register(
  SRCS "test_firmware.c"
  INCLUDE_DIRS "."
  REQUIRES driver nvs_flash sdmmc esp_timer
)
