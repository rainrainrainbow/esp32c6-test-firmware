// ESP32-C6 board test firmware (GitHub Actions CI / ESP-IDF 5.3.1)
// Pin map source: Sheet_20261125.xlsx
//   I2C  SDA=IO6 SCL=IO7 -> AHT20(0x38), DS3231(0x68), MAX17048(0x36),
//                            MPU-6500(0x68/0x69), MMC5603NJ(0x30)
//   Buttons BOOT=IO9 DOWN=IO3 KEY1=IO2 (active-low, pull-up)
//   Buzzer IO18 (LEDC)
//   EPD: RST=IO0 BUSY=IO1 DCX=IO4 CS=IO5 CS2=IO14
//   SD (SPI mode, since C6 has no SDMMC host):
//        SCK=IO21(CLK) MOSI=IO20(CMD) MISO=IO22(D0) CS=IO8(CD/DAT3)
//   I2S mic ICS-43434: SCK=IO16 WS=IO17 SD=IO23
//   USB: D-=IO12 D+=IO13
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/spi_master.h"
#include "driver/sdspi_host.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "sdmmc_cmd.h"

static const char *TAG = "TEST";

#define I2C_SDA 6
#define I2C_SCL 7
#define BTN_BOOT 9
#define BTN_DOWN 3
#define BTN_GPIO2 2
#define BUZZ 18
#define EPD_RST 0
#define EPD_BUSY 1
#define EPD_IO4 4
#define EPD_IO5 5
#define EPD_CS 14
#define SD_SCK 21   // SD CLK
#define SD_MOSI 20  // SD CMD
#define SD_MISO 22  // SD DAT0
#define SD_CS 8     // SD CD/DAT3 -> CS in SPI mode
#define MIC_SCK 16
#define MIC_WS 17
#define MIC_SD 23

static i2c_master_bus_handle_t bus;

/* ---------- I2C helpers (new driver) ---------- */
static esp_err_t reg_read(uint8_t addr, uint8_t reg, uint8_t *out) {
    i2c_device_config_t dev = { .dev_addr_length = I2C_ADDR_BIT_7,
                                .device_address = addr, .scl_speed_hz = 100000 };
    i2c_master_dev_handle_t h;
    if (i2c_master_bus_add_device(bus, &dev, &h) != ESP_OK) return ESP_FAIL;
    uint8_t r = reg;
    esp_err_t e = i2c_master_transmit_receive(h, &r, 1, out, 1, 200);
    i2c_master_bus_rm_device(h);
    return e;
}
static esp_err_t reg_write(uint8_t addr, uint8_t reg, uint8_t val) {
    i2c_device_config_t dev = { .dev_addr_length = I2C_ADDR_BIT_7,
                                .device_address = addr, .scl_speed_hz = 100000 };
    i2c_master_dev_handle_t h;
    if (i2c_master_bus_add_device(bus, &dev, &h) != ESP_OK) return ESP_FAIL;
    uint8_t b[2] = { reg, val };
    esp_err_t e = i2c_master_transmit(h, b, 2, 200);
    i2c_master_bus_rm_device(h);
    return e;
}

/* ---------- I2C scan + sensor identification ---------- */
static void i2c_scan_and_read(void) {
    printf("\n--- I2C scan (SDA=%d SCL=%d) ---\n", I2C_SDA, I2C_SCL);
    static const struct { uint8_t addr; const char *name; } known[] = {
        {0x38, "AHT20"}, {0x36, "MAX17048"}, {0x30, "MMC5603NJ"},
        {0x68, "DS3231/MPU6500"}, {0x69, "MPU6500(AD0=1)"}
    };
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        esp_err_t r = i2c_master_probe(bus, addr, 100);
        if (r == ESP_OK) {
            printf("  [0x%02X] ACK", addr);
            for (unsigned i = 0; i < sizeof(known)/sizeof(known[0]); i++)
                if (known[i].addr == addr) printf(" <- %s", known[i].name);
            printf("\n");
            found++;
        }
    }
    printf("  I2C devices found: %d\n", found);

    // AHT20: trigger measurement, read T/RH
    uint8_t v;
    if (reg_read(0x38, 0x00, &v) == ESP_OK) {
        i2c_device_config_t dev = { .dev_addr_length = I2C_ADDR_BIT_7,
                                    .device_address = 0x38, .scl_speed_hz = 100000 };
        i2c_master_dev_handle_t h;
        if (i2c_master_bus_add_device(bus, &dev, &h) == ESP_OK) {
            uint8_t cmd[3] = {0xAC, 0x33, 0x00}, d[6] = {0};
            i2c_master_transmit(h, cmd, 3, 100);
            vTaskDelay(pdMS_TO_TICKS(90));
            if (i2c_master_receive(h, d, 6, 200) == ESP_OK && !(d[0] & 0x80)) {
                uint32_t rh = ((uint32_t)d[1]<<12)|((uint32_t)d[2]<<4)|(d[3]>>4);
                uint32_t tp = (((uint32_t)d[3]&0x0F)<<16)|((uint32_t)d[4]<<8)|d[5];
                printf("  AHT20: T=%.2fC RH=%.2f%% (status=0x%02X)\n",
                       (double)tp*200.0/1048576.0-50.0, (double)rh*100.0/1048576.0, d[0]);
            } else printf("  AHT20: read busy/failed\n");
            i2c_master_bus_rm_device(h);
        }
    } else printf("  AHT20: absent\n");

    // DS3231 RTC: read seconds/minutes/hours (BCD)
    if (reg_read(0x68, 0x00, &v) == ESP_OK) {
        uint8_t sec, min, hr;
        reg_read(0x68, 0x00, &sec); reg_read(0x68, 0x01, &min); reg_read(0x68, 0x02, &hr);
        printf("  DS3231: %02X:%02X:%02X (BCD) raw sec=0x%02X\n", hr, min, sec, sec);
    } else printf("  DS3231: absent\n");

    // MAX17048 fuel gauge: version(0x0F), VCELL(0x02/03), SOC(0x04)
    uint8_t ver, soc, vh, vl;
    if (reg_read(0x36, 0x0F, &ver) == ESP_OK) {
        reg_read(0x36, 0x02, &vh); reg_read(0x36, 0x03, &vl); reg_read(0x36, 0x04, &soc);
        uint16_t cell = ((uint16_t)vh<<4)|(vl>>4);
        printf("  MAX17048: ver=0x%02X V=%.3fV SOC=%d%%\n", ver, (double)cell*1.25/1000.0, soc);
    } else printf("  MAX17048: absent\n");

    // MPU-6500 WHO_AM_I at 0x75 (0x70/0x71)
    bool mpu = false;
    for (uint8_t a = 0x68; a <= 0x69; a++) {
        if (reg_read(a, 0x75, &v) == ESP_OK && (v == 0x70 || v == 0x71)) {
            printf("  MPU-6500 @0x%02X: WHO_AM_I=0x%02X OK\n", a, v); mpu = true;
        }
    }
    if (!mpu) printf("  MPU-6500: WHO_AM_I mismatch/absent\n");

    // MMC5603NJ WHO_AM_I at 0x39 -> 0x10
    if (reg_read(0x30, 0x39, &v) == ESP_OK)
        printf("  MMC5603NJ: WHO_AM_I=0x%02X (expect 0x10)\n", v);
    else printf("  MMC5603NJ: absent\n");
}

/* ---------- buttons ---------- */
static void buttons_test(void) {
    printf("\n--- Buttons (active-low, pull-up) ---\n");
    const int pins[] = { BTN_BOOT, BTN_DOWN, BTN_GPIO2 };
    const char *names[] = { "BOOT(IO9)", "DOWN(IO3)", "KEY1(IO2)" };
    for (int i = 0; i < 3; i++)
        printf("  %-12s = %s\n", names[i],
               gpio_get_level(pins[i]) ? "RELEASED(HIGH)" : "PRESSED(LOW)");
}

/* ---------- buzzer ---------- */
static void buzzer_beep(void) {
    printf("\n--- Buzzer (IO%d) ---\n", BUZZ);
    ledc_timer_config_t t = { .speed_mode = LEDC_LOW_SPEED_MODE,
                              .duty_resolution = LEDC_TIMER_10_BIT,
                              .timer_num = LEDC_TIMER_0, .freq_hz = 2000,
                              .clk_cfg = LEDC_AUTO_CLK };
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    ledc_channel_config_t c = { .gpio_num = BUZZ, .speed_mode = LEDC_LOW_SPEED_MODE,
                                .channel = LEDC_CHANNEL_0, .intr_type = LEDC_INTR_DISABLE,
                                .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 };
    ESP_ERROR_CHECK(ledc_channel_config(&c));
    for (int i = 0; i < 3; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(150));
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    printf("  beeped 3x @2kHz\n");
}

/* ---------- EPD real refresh: WFT0154CZ17 / GxEPD2_154_T8 (IL0373 152x152 B/W) ----------
 * FPC-24P: RST=IO0 BUSY=IO1 DCX=IO4 CS=IO14; SCLK/MOSI shared with SD on SPI2 (IO21/IO20).
 * Panel setting 0xbf = LUT from register (needs 5 LUTs), BUSY LOW = busy.
 * Exact sequence from GxEPD2_154_T8.cpp (ZinggJM), panel GDEW0154T8.
 */
#define EPD_SCLK 21
#define EPD_MOSI 20
#define EPD_DC   4
#define EPD_W    152
#define EPD_H    152
#define EPD_BPL  19   // 152/8 = 19 bytes per line

static spi_device_handle_t epd_dev_h;

static const uint8_t lut_vcom[44] = {
    0x00,0x08,0x00,0x00,0x00,0x02, 0x60,0x28,0x28,0x00,0x00,0x01,
    0x00,0x14,0x00,0x00,0x00,0x01, 0x00,0x12,0x12,0x00,0x00,0x01,
    0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00};
static const uint8_t lut_ww[42] = {
    0x40,0x08,0x00,0x00,0x00,0x02, 0x90,0x28,0x28,0x00,0x00,0x01,
    0x40,0x14,0x00,0x00,0x00,0x01, 0xA0,0x12,0x12,0x00,0x00,0x01,
    0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00};
static const uint8_t lut_bw[42] = {
    0x40,0x08,0x00,0x00,0x00,0x02, 0x90,0x28,0x28,0x00,0x00,0x01,
    0x40,0x14,0x00,0x00,0x00,0x01, 0xA0,0x12,0x12,0x00,0x00,0x01,
    0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00};
static const uint8_t lut_wb[42] = {
    0x80,0x08,0x00,0x00,0x00,0x02, 0x90,0x28,0x28,0x00,0x00,0x01,
    0x80,0x14,0x00,0x00,0x00,0x01, 0x50,0x12,0x12,0x00,0x00,0x01,
    0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00};
static const uint8_t lut_bb[42] = {
    0x80,0x08,0x00,0x00,0x00,0x02, 0x90,0x28,0x28,0x00,0x00,0x01,
    0x80,0x14,0x00,0x00,0x00,0x01, 0x50,0x12,0x12,0x00,0x00,0x01,
    0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00};

static void epd_cmd(uint8_t c) {
    gpio_set_level(EPD_DC, 0); gpio_set_level(EPD_CS, 0);
    spi_transaction_t t = { .length = 8, .tx_buffer = &c };
    spi_device_polling_transmit(epd_dev_h, &t);
    gpio_set_level(EPD_CS, 1);
}
static void epd_data(const uint8_t *d, size_t len) {
    gpio_set_level(EPD_DC, 1); gpio_set_level(EPD_CS, 0);
    size_t off = 0;
    while (off < len) {
        size_t chunk = len - off; if (chunk > 1024) chunk = 1024;
        spi_transaction_t t = { .length = chunk * 8, .tx_buffer = d + off };
        esp_err_t e = spi_device_polling_transmit(epd_dev_h, &t);
        if (e != ESP_OK) { printf("  EPD spi err: %s\n", esp_err_to_name(e)); break; }
        off += chunk;
    }
    gpio_set_level(EPD_CS, 1);
}
static void epd_d1(uint8_t v) { epd_data(&v, 1); }
/* IL0373 BUSY LOW = busy. returns ms; *saw_low = BUSY ever asserted */
static int epd_wait_busy(int max_ms, bool *saw_low) {
    int t = 0;
    while (gpio_get_level(EPD_BUSY) == 0) { if (saw_low) *saw_low = true; if (t >= max_ms) break; vTaskDelay(pdMS_TO_TICKS(10)); t += 10; }
    return t;
}
static void epd_full_test(void) {
    printf("\n--- E-paper IL0373 152x152 (SCLK=%d MOSI=%d DCX=%d CS=%d) ---\n",
           EPD_SCLK, EPD_MOSI, EPD_DC, EPD_CS);
    printf("  before reset: BUSY=%d RST=%d IO5=%d\n",
           gpio_get_level(EPD_BUSY), gpio_get_level(EPD_RST), gpio_get_level(EPD_IO5));

    // hardware reset (GxEPD2 _reset: low 2ms, high 20ms), repeated twice
    for (int i = 0; i < 2; i++) {
        gpio_set_level(EPD_RST, 0); vTaskDelay(pdMS_TO_TICKS(2));
        gpio_set_level(EPD_RST, 1); vTaskDelay(pdMS_TO_TICKS(20));
    }
    printf("  after reset: BUSY=%d\n", gpio_get_level(EPD_BUSY));

    spi_bus_config_t sb = { .mosi_io_num = EPD_MOSI, .miso_io_num = -1, .sclk_io_num = EPD_SCLK,
                            .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 2048 };
    esp_err_t e = spi_bus_initialize(SPI2_HOST, &sb, SDSPI_DEFAULT_DMA);
    if (e != ESP_OK) { printf("  SPI init FAILED: %s\n", esp_err_to_name(e)); return; }
    spi_device_interface_config_t dc = { .command_bits = 0, .address_bits = 0, .mode = 0,
                                         .clock_speed_hz = 4000000, .spics_io_num = -1, .queue_size = 2 };
    e = spi_bus_add_device(SPI2_HOST, &dc, &epd_dev_h);
    if (e != ESP_OK) { printf("  dev add FAILED: %s\n", esp_err_to_name(e)); spi_bus_free(SPI2_HOST); return; }

    // ---- _InitDisplay ----
    epd_cmd(0x01); epd_d1(0x03); epd_d1(0x00); epd_d1(0x2b); epd_d1(0x2b); epd_d1(0x03); // power setting
    epd_cmd(0x06); epd_d1(0x17); epd_d1(0x17); epd_d1(0x17);                              // boost soft start
    epd_cmd(0x00); epd_d1(0xbf); epd_d1(0x0d);                                            // panel: LUT from reg
    epd_cmd(0x30); epd_d1(0x3a);                                                          // PLL 100Hz
    epd_cmd(0x61); epd_d1(EPD_W); epd_d1(0x00); epd_d1(EPD_H);                            // 152x152
    // ---- _Init_Full: VCOM + LUTs ----
    epd_cmd(0x82); epd_d1(0x08);
    epd_cmd(0x50); epd_d1(0x97);
    epd_cmd(0x20); epd_data(lut_vcom, sizeof(lut_vcom));
    epd_cmd(0x21); epd_data(lut_ww,   sizeof(lut_ww));
    epd_cmd(0x22); epd_data(lut_bw,   sizeof(lut_bw));
    epd_cmd(0x23); epd_data(lut_wb,   sizeof(lut_wb));
    epd_cmd(0x24); epd_data(lut_bb,   sizeof(lut_bb));
    printf("  init+LUT sent, BUSY=%d\n", gpio_get_level(EPD_BUSY));

    // ---- RAM: 152x152/8 = 2888 bytes per plane ----
    static uint8_t frame[EPD_BPL * EPD_H];  // 2888
    memset(frame, 0xFF, sizeof(frame));
    epd_cmd(0x10); epd_data(frame, sizeof(frame));   // previous = white
    for (int y = 0; y < EPD_H; y++)
        for (int x = 0; x < EPD_BPL; x++)
            frame[y * EPD_BPL + x] = (((x / 2 + y / 10) & 1) ? 0x00 : 0xFF); // stripes
    epd_cmd(0x13); epd_data(frame, sizeof(frame));   // current = pattern
    printf("  2x2888 bytes RAM written, BUSY=%d\n", gpio_get_level(EPD_BUSY));

    bool low1=false, low2=false, low3=false;
    epd_cmd(0x04); int t1 = epd_wait_busy(3000, &low1);  // power on
    printf("  power-on : wait=%dms BUSY=%d saw_low=%d\n", t1, gpio_get_level(EPD_BUSY), low1);
    epd_cmd(0x12); vTaskDelay(pdMS_TO_TICKS(100));
    int t2 = epd_wait_busy(6000, &low2);                // full refresh (~1.6s)
    printf("  refresh  : wait=%dms BUSY=%d saw_low=%d\n", t2, gpio_get_level(EPD_BUSY), low2);
    epd_cmd(0x02); int t3 = epd_wait_busy(3000, &low3);  // power off
    printf("  power-off: wait=%dms BUSY=%d saw_low=%d\n", t3, gpio_get_level(EPD_BUSY), low3);
    epd_cmd(0x07); epd_d1(0xA5);                        // deep sleep

    spi_bus_remove_device(epd_dev_h);
    spi_bus_free(SPI2_HOST);
    printf("  EPD result: %s\n", (low1||low2||low3) ? "RESPONSIVE (BUSY asserted LOW)" : "NO RESPONSE (BUSY never asserted)");
}

/* ---------- SD card via SDSPI (C6 has no SDMMC host) ---------- */
static void sd_test(void) {
    printf("\n--- SD card (SPI: SCK=%d MOSI=%d MISO=%d CS=%d) ---\n",
           SD_SCK, SD_MOSI, SD_MISO, SD_CS);
    printf("  CD/DAT3(IO8)=%s\n", gpio_get_level(SD_CS) ? "HIGH" : "LOW");

    spi_bus_config_t sb = {
        .mosi_io_num = SD_MOSI, .miso_io_num = SD_MISO, .sclk_io_num = SD_SCK,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 4096,
    };
    esp_err_t e = spi_bus_initialize(SPI2_HOST, &sb, SDSPI_DEFAULT_DMA);
    if (e != ESP_OK) { printf("  SPI bus init FAILED: %s\n", esp_err_to_name(e)); return; }

    sdspi_device_config_t dc = SDSPI_DEVICE_CONFIG_DEFAULT();
    dc.host_id = SPI2_HOST;
    dc.gpio_cs = SD_CS;          // CD/DAT3 doubles as CS in SPI mode
    dc.gpio_cd = GPIO_NUM_NC;    // avoid conflict with CS
    sdspi_dev_handle_t dh;
    e = sdspi_host_init_device(&dc, &dh);
    if (e != ESP_OK) { printf("  SDSPI device init FAILED: %s\n", esp_err_to_name(e)); return; }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = dh;
    host.max_freq_khz = SDMMC_FREQ_PROBING;
    sdmmc_card_t card;
    memset(&card, 0, sizeof(card));
    e = sdmmc_card_init(&host, &card);
    if (e == ESP_OK) {
        uint64_t mb = (uint64_t)card.csd.capacity * card.csd.sector_size / (1024*1024);
        printf("  SD OK: name=%.8s type=%s%s capacity=%llu MB\n",
               card.cid.name, card.is_sdio ? "SDIO" : (card.is_mmc ? "MMC" : "SD"),
               "", (unsigned long long)mb);
    } else {
        printf("  SD init FAILED: %s (0x%x)\n", esp_err_to_name(e), e);
    }
    sdspi_host_remove_device(dh);
    spi_bus_free(SPI2_HOST);
}

/* ---------- I2S mic ---------- */
static void i2s_mic_test(void) {
    printf("\n--- I2S mic (ICS-43434: SCK=%d WS=%d SD=%d) ---\n", MIC_SCK, MIC_WS, MIC_SD);
    i2s_chan_config_t cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_chan_handle_t rx = NULL;
    esp_err_t e = i2s_new_channel(&cfg, NULL, &rx);
    if (e != ESP_OK) { printf("  channel create FAILED: %s\n", esp_err_to_name(e)); return; }

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED, .bclk = MIC_SCK, .ws = MIC_WS,
            .dout = I2S_GPIO_UNUSED, .din = MIC_SD,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    e = i2s_channel_init_std_mode(rx, &std_cfg);
    if (e != ESP_OK) { printf("  std init FAILED: %s\n", esp_err_to_name(e)); goto done; }
    e = i2s_channel_enable(rx);
    if (e != ESP_OK) { printf("  enable FAILED: %s\n", esp_err_to_name(e)); goto done; }

    int32_t buf[512]; size_t n = 0;
    e = i2s_channel_read(rx, buf, sizeof(buf), &n, 2000);
    if (e == ESP_OK) {
        int samples = n / sizeof(int32_t);
        long long sum = 0;
        for (int i = 0; i < samples; i++) sum += (long long)buf[i] * buf[i];
        double rms = samples ? sqrt((double)sum / samples) : 0;
        printf("  samples=%d rms=%.1f %s\n", samples, rms,
               samples > 0 ? "(I2S clock/data active)" : "(no data)");
    } else printf("  read FAILED: %s\n", esp_err_to_name(e));
    i2s_channel_disable(rx);
done:
    i2s_del_channel(rx);
}

/* ---------- orchestration ---------- */
static void run_tests(void) {
    printf("\n==== ESP32-C6 board test @%lld ms ====\n",
           (long long)(esp_timer_get_time()/1000));
    i2c_scan_and_read();
    buttons_test();
    buzzer_beep();
    epd_full_test();
    sd_test();
    i2s_mic_test();
    printf("\n==== test cycle done ====\n");
}

void app_main(void) {
    esp_chip_info_t info;
    esp_chip_info(&info);
    printf("\n############################################\n");
    printf("  ESP32-C6 board test firmware (CI build)\n");
    printf("  build: %s %s\n", __DATE__, __TIME__);
    printf("  chip=%s rev=%d cores=%d features=0x%" PRIx32 "\n",
           info.model == CHIP_ESP32C6 ? "ESP32-C6" : "other",
           (int)info.revision, (int)info.cores, info.features);
    printf("############################################\n");

    esp_err_t nvs = nvs_flash_init();
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); nvs_flash_init();
    }

    i2c_master_bus_config_t bc = { .clk_source = I2C_CLK_SRC_DEFAULT,
                                   .i2c_port = I2C_NUM_0,
                                   .scl_io_num = I2C_SCL, .sda_io_num = I2C_SDA,
                                   .glitch_ignore_cnt = 7,
                                   .flags = { .enable_internal_pullup = true } };
    esp_err_t r = i2c_new_master_bus(&bc, &bus);
    printf("I2C bus init: %s\n", esp_err_to_name(r));

    const int inputs[] = { BTN_BOOT, BTN_DOWN, BTN_GPIO2, EPD_BUSY };
    for (unsigned i = 0; i < sizeof(inputs)/sizeof(inputs[0]); i++) {
        gpio_set_direction(inputs[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(inputs[i], GPIO_PULLUP_ONLY);
    }
    const int outputs[] = { EPD_RST, EPD_IO4, EPD_IO5, EPD_CS };
    for (unsigned i = 0; i < sizeof(outputs)/sizeof(outputs[0]); i++) {
        gpio_set_direction(outputs[i], GPIO_MODE_OUTPUT);
        gpio_set_level(outputs[i], 1);
    }

    run_tests();
    printf("\ndone. Reset to re-run.\n");
}
