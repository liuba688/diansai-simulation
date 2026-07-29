# 当前接线

## MaixCAM 与 MSPM0G3507

| MaixCAM | MSPM0G3507 | 用途 |
|---|---|---|
| A19 / UART1_TX | PA24 / UART2_RX | 视觉目标数据 |
| A18 / UART1_RX | PA23 / UART2_TX | 小车状态回传 |
| GND | GND | 共地 |

MaixCAM 使用 Type-C 供电时不要连接小车 UART2 的 +5 V。双方均为 3.3 V 逻辑。

## MPU6050/MPU6500

| 模块 | MSPM0G3507 |
|---|---|
| SDA | PA0 |
| SCL | PA1 |
| VCC | 按模块额定电压连接 |
| GND | GND |

## OLED 与按键

- OLED 软件 I²C：SCL=PA31，SDA=PA28，地址 `0x3C`。
- 上、下、确认、返回：PB8、PB9、PB10、PB11，低电平有效。
- 返回键运行中触发安全停车并重新打开菜单。

## 其余接口

- 调试串口 UART1：PB6/PB7，115200。
- 心跳 LED：PB16。
- 蜂鸣器/使能提示：PA7。
- 灰度传感器选择/输出：PB25、PB18、PB21、PB22。
- 不初始化 UART3 PB2/PB3，不连接蓝牙模块。
