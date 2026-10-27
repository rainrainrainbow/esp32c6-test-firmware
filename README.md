# ESP32-C6 多传感器板 测试固件（CI 构建）

借助 GitHub Actions 编译的 ESP-IDF 测试固件，用于验证这块 ESP32-C6 板子上的外设。

## 板载外设（从 netlist 提取）

| 外设 | 接口/引脚 | 说明 |
|---|---|---|
| AHT20 | I2C 0x38, SDA=IO6 SCL=IO7 | 温湿度 |
| DS3231MZ | I2C 0x68 | RTC（INT=IO15） |
| MAX17048 | I2C 0x36 | 电池电量计 |
| MPU-6500 | I2C 0x68/0x69 | IMU（INT=IO19） |
| MMC5603NJ | I2C 0x30 | 磁力计 |
| ICS-43434 | I2S, SCK=IO16 WS=IO17 SD=IO23 | 数字麦克风 |
| 墨水屏 | CS=IO14 RST=IO0 BUSY=IO1 | 24P FPC，含升压 |
| TF 卡 | SDMMC 1bit, CLK=IO21 CMD=IO20 D0=IO22 CD=IO8 | 自弹卡座 |
| 按键 | BOOT=IO9 DOWN=IO3 GPIO2 | 低有效，上拉 |
| 蜂鸣器 | IO18 | LEDC |
| USB | D-=IO12 D+=IO13 | Type-C |

## 构建（本机无需装环境，全走 CI）

1. 推送 `main` 分支 → `.github/workflows/build.yml` 自动在 espressif/idf:v5.3.1 容器里编译
2. 下载 artifact `esp32c6-test-firmware`
3. 烧录（esptool）：

```bash
esptool.py --chip esp32c6 -p /dev/ttyUSB0 --baud 921600 write_flash \
  0x0 build/merged_binary.bin
```

或按 `build/flasher_args.json` 里的地址分别烧 bootloader/partition/app。

## 运行

上电后串口（默认 115200，IO16? 见 sdkconfig 控制台配置）输出逐项测试结果：
I2C 扫描、按键、蜂鸣器、墨水屏引脚、SD 卡、I2S 麦克风 RMS。单次运行后停止。
