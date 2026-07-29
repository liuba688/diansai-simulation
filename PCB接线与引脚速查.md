# PCB 接线与引脚速查

当前融合版本使用：

| 功能 | 引脚 |
|---|---|
| MaixCAM UART2 | PA23 TX、PA24 RX |
| MPU6050/6500 软件 I²C | PA1 SCL、PA0 SDA |
| OLED 软件 I²C | PA31 SCL、PA28 SDA |
| 菜单上/下/确认/返回 | PB8/PB9/PB10/PB11 |
| 调试 UART1 | PB6 TX、PB7 RX |
| 心跳 LED | PB16 |
| 灰度选择/输出 | PB25/PB18/PB21/PB22 |

UART3 PB2/PB3 不再初始化。MaixCAM 与小车必须共地，UART 信号均按 3.3 V 使用。
