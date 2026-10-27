// ESP32-C6 board test firmware (via GitHub Actions CI)
// Board: ESP32-C6-MINI-1-N4 based, peripherals from netlist:
//   I2C  SDA=IO6 SCL=IO7  -> AHT20, DS3231, MAX17048, MPU-6500, MMC5603
//   Buttons: BOOT=IO9 DOWN=IO3 GPIO2=IO2 (active-low, pull-up)
//   Buzzer=IO18 (LEDC)
//   EPD: CS=IO14 RST=IO0 BUSY=IO1 (+IO4 IO5)
//   SDMMC: CLK=IO21 CMD=IO20 D0=IO22 CD=IO8
//   I2S mic ICS-43434: SCK=IO16 WS=IO17 SD=IO23
//   USB: D-=IO12 D+=IO13
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_sd.h"

static const char *TAG = "TEST";

#define I2C_SDA 6
#define I2C_SCL 7
#define BTN_BOOT 9
#define BTN_DOWN 3
#define BTN_GPIO2 2
#define BUZZ 18
#define EPD_CS 14
#define EPD_RST 0
#define EPD_BUSY 1
#define EPD_IO4 4
#define EPD_IO5 5
#define SD_CLK 21
#define SD_CMD 20
#define SD_D0 22
#define SD_CD 8
#define MIC_SCK 16
#define MIC_WS 17
#define MIC_SD 23

static i2c_master_bus_handle_t bus;

static void i2c_scan_and_read(void) {
    printf("\n--- I2C scan (SDA=%d SCL=%d) ---\n", I2C_SDA, I2C_SCL);
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_device_config_t dev = { .dev_addr_length = I2C_ADDR_BIT_7, .device_address = addr,
                                    .scl_speed_hz = 100000 };
        i2c_master_dev_handle_t h;
        if (i2c_master_bus_add_device(bus, &dev, &h) == ESP_OK) {
            uint8_t b[2] = {0};
            esp_err_t r = i2c_master_transmit_receive(h, b, 0, b, 1, 100);
            if (r == ESP_OK) {
                printf("  [0x%02X] ACK, reg0=0x%02X", addr, b[0]);
                const char *name = NULL;
                switch (addr) {
                    case 0x38: name = "AHT20"; break;
                    case 0x68: name = "DS3231/MPU6500"; break;
                    case 0x69: name = "MPU6500(alt)"; break;
                    case 0x36: name = "MAX17048"; break;
                    case 0x30: name = "MMC5603"; break;
                    default: break;
                }
                if (name) printf("  <- %s", name);
                printf("\n");
                found++;
            }
            i2c_master_bus_rm_device(h);
        }
    }
    printf("  I2C devices found: %d\n", found);
}

static void buttons_test(void) {
    printf("\n--- Buttons (active-low, pull-up) ---\n");
    const int pins[] = { BTN_BOOT, BTN_DOWN, BTN_GPIO2 };
    const char *names[] = { "BOOT(IO9)", "DOWN(IO3)", "IO2" };
    for (int i = 0; i < 3; i++) {
        printf("  %s = %s\n", names[i], gpio_get_level(pins[i]) ? "RELEASED(HIGH)" : "PRESSED(LOW)");
    }
}

static void buzzer_beep(void) {
    printf("\n--- Buzzer (IO%d) ---\n", BUZZ);
    ledc_timer_config_t t = { .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT,
                              .timer_num = LEDC_TIMER_0, .freq_hz = 2000, .clk_cfg = LEDC_AUTO_CLK };
    ledc_timer_config(&t);
    ledc_channel_config_t c = { .gpio_num = BUZZ, .speed_mode = LEDC_LOW_SPEED_MODE,
                                .channel = LEDC_CHANNEL_0, .intr_type = LEDC_INTR_DISABLE,
                                .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 };
    ledc_channel_config(&c);
    for (int i = 0; i < 3; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(150));
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    printf("  beeped 3x\n");
}

static void epd_probe(void) {
    printf("\n--- E-paper probe ---\n");
    printf("  CS(IO14)=%d RST(IO0)=%d BUSY(IO1)=%d IO4=%d IO5=%d\n",
           gpio_get_level(EPD_CS), gpio_get_level(EPD_RST), gpio_get_level(EPD_BUSY),
           gpio_get_level(EPD_IO4), gpio_get_level(EPD_IO5));
    // pulse reset low then high
    gpio_set_level(EPD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(EPD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    printf("  after RST pulse: BUSY=%d\n", gpio_get_level(EPD_BUSY));
}

static void sd_test(void) {
    printf("\n--- SD card (SDMMC 1-bit) ---\n");
    printf("  CD(IO8)=%s\n", gpio_get_level(SD_CD) ? "card present(LOW=insert)" : "card inserted(LOW)");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;
    host.slot = SDMMC_HOST_SLOT_1;
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.clk = SD_CLK;
    slot.cmd = SD_CMD;
    slot.d0 = SD_D0;
    slot.width = 1;
    slot.flags = SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    sdmmc_card_t *card = NULL;
    esp_err_t r = sdmmc_card_init(&host, &slot, &card);
    if (r != ESP_OK) { printf("  SD init FAILED: %s\n", esp_err_to_name(r)); return; }
    printf("  SD init OK, size=%lluMB, type=%d\n", (unsigned long long)((uint64_t)card->csd.capacity * card->csd.sector_size / (1024 * 1024)), card->type);
    sdmmc_card_deinit();
}

static void i2s_mic_test(void) {
    printf("\n--- I2S mic (ICS-43434) ---\n");
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_chan_handle_t rx = NULL;
    if (i2s_new_channel(&chan_cfg, NULL, &rx) != ESP_OK) { printf("  channel create FAILED\n"); return; }
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = MIC_SCK,
            .ws = MIC_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = MIC_SD,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false, .dout_inv = false, .din_inv = false },
        },
    };
    if (i2s_channel_init_std_mode(rx, &std_cfg) != ESP_OK) { printf("  std init FAILED\n"); return; }
    if (i2s_channel_enable(rx) != ESP_OK) { printf("  enable FAILED\n"); return; }
    int32_t buf[512];
    size_t n = 0;
    esp_err_t r = i2s_channel_read(rx, buf, sizeof(buf), &n, 2000);
    if (r == ESP_OK) {
        int samples = n / sizeof(int32_t);
        long long sum = 0;
        for (int i = 0; i < samples; i++) sum += (long long)buf[i] * buf[i];
        double rms = samples ? sqrt((double)sum / samples) : 0;
        printf("  samples=%d rms=%.1f\n", samples, rms);
    } else {
        printf("  read FAILED: %s\n", esp_err_to_name(r));
    }
    i2s_channel_disable(rx);
    i2s_del_channel(rx);
}

static void run_tests(int loop) {
    do {
        printf("\n==== ESP32-C6 board test @%lld ms ====\n", esp_timer_get_time() / 1000);
        i2c_scan_and_read();
        buttons_test();
        buzzer_beep();
        epd_probe();
        sd_test();
        i2s_mic_test();
        printf("\n==== test cycle done ====\n");
        if (!loop) break;
        vTaskDelay(pdMS_TO_TICKS(5000));
    } while (loop);
}

void app_main(void) {
    esp_chip_info_t info;
    esp_chip_info(&info);
    printf("\n############################################\n");
    printf("  ESP32-C6 board test firmware\n");
    printf("  chip=%s rev=%d cores=%d flash=%luMB\n", info.model == CHIP_ESP32C6 ? "ESP32-C6" : "other",
           (int)info.revision, (int)info.cores, (unsigned long)(info.size / 1024 / 1024));
    printf("############################################\n");

    nvs_flash_init();

    // I2C bus
    i2c_master_bus_config_t bus_cfg = { .clk_source = I2C_CLK_SRC_DEFAULT,
                                        .i2c_port = I2C_NUM_0,
                                        .scl_io_num = I2C_SCL,
                                        .sda_io_num = I2C_SDA,
                                        .glitch_ignore_cnt = 7,
                                        .flags = { .enable_internal_pullup = true } };
    esp_err_t r = i2c_new_master_bus(&bus_cfg, &bus);
    printf("I2C bus init: %s\n", esp_err_to_name(r));

    // inputs
    gpio_set_direction(BTN_BOOT, GPIO_MODE_INPUT); gpio_set_pull_mode(BTN_BOOT, GPIO_PULLUP_ONLY);
    gpio_set_direction(BTN_DOWN, GPIO_MODE_INPUT); gpio_set_pull_mode(BTN_DOWN, GPIO_PULLUP_ONLY);
    gpio_set_direction(BTN_GPIO2, GPIO_MODE_INPUT); gpio_set_pull_mode(BTN_GPIO2, GPIO_PULLUP_ONLY);
    gpio_set_direction(EPD_BUSY, GPIO_MODE_INPUT); gpio_set_pull_mode(EPD_BUSY, GPIO_PULLUP_ONLY);
    gpio_set_direction(SD_CD, GPIO_MODE_INPUT); gpio_set_pull_mode(SD_CD, GPIO_PULLUP_ONLY);
    // outputs
    gpio_set_direction(EPD_RST, GPIO_MODE_OUTPUT); gpio_set_level(EPD_RST, 1);
    gpio_set_direction(EPD_CS, GPIO_MODE_OUTPUT); gpio_set_level(EPD_CS, 1);
    gpio_set_direction(EPD_IO4, GPIO_MODE_OUTPUT); gpio_set_level(EPD_IO4, 1);
    gpio_set_direction(EPD_IO5, GPIO_MODE_OUTPUT); gpio_set_level(EPD_IO5, 1);

    // single shot by default; pass "--loop" via menuconfig or just run once
    run_tests(0);
    printf("done. hold BOOT to re-run.\n");
}
