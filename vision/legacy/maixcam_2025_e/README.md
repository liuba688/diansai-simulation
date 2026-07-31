# MaixCAM 2025 E 视觉工程

这是 2025 E“简易自行瞄准装置”的模块化视觉工程。当前阶段只识别 A4 黑胶带靶框、
计算透视靶心并显示偏差；激光和电机保持关闭。

## 当前参数

- 分辨率：`240×180`
- 完整检测：每 2 帧一次；中间帧复用结果
- 计时：标准库 `time.monotonic/time.time` 兼容写法
- UART：默认关闭；启用后为 UART1，A19/A18，115200

## 状态

- `TARGET` / 1：完整可信目标，可供控制端使用
- `HOLD` / 2：短时丢失，仅显示，电机必须停止
- `SEARCH` / 3：目标越出一边，只供搜索观察，禁止精确锁定
- `LOST` / 0：目标丢失

## UART 数据包

固定 17 字节：`AA 55 version type length seq status target_x target_y laser_x laser_y confidence crc8`。
四个坐标为小端 `int16`；CRC-8/ATM 使用多项式 `0x07`，覆盖 version 到 confidence。

单文件快速刷新版本仍位于 `../maixcam_color_blob/main.py`，两者算法参数保持一致。
