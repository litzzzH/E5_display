# E5_display

基于 MSPM0G3507 的 IPS200 显示与循迹小车控制工程，包含电机、五路灰度传感器、IMU660RB、激光和屏幕调参模块。

## 开发环境

- MCU：MSPM0G3507
- IDE：Keil MDK 5.37
- 显示屏：IPS200（SPI）

## 目录结构

- `E5_04_ips200_display_demo/`：应用源码与 Keil 工程
- `libraries/`：TI SDK、CMSIS 和逐飞科技设备驱动

使用 Keil 打开 `E5_04_ips200_display_demo/keil/SeekFree_MSPM0G3507_Device_Library.uvprojx` 即可构建。各功能模块可在 `E5_04_ips200_display_demo/code/module_config.h` 中启用或关闭。

## 许可证

源码中的逐飞科技开源库声明采用 GPL 3.0 或更高版本；第三方组件遵循其各自附带的许可声明。
