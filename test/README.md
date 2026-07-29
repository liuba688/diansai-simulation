# 天猛星 + MPU6050 + HC-05 独立测试

本目录只用于验证以下链路，不接电机、编码器、灰度和 OLED：

`MSPM0G3507 天猛星 -> MPU6050 -> HC-05 -> 电脑`

## 工程位置

Keil 工程：

`MSPM0G3507_MPU6050_HC05/project/mdk/SeekFree_MSPM0G3507_Device_Library.uvprojx`

主程序：

`MSPM0G3507_MPU6050_HC05/project/user/src/main.c`

## 接线

### MPU6050

| MPU6050 | 天猛星 |
|---|---|
| VCC | 按模块标注接 3.3 V 或 5 V；GY-521 通常可接 5 V |
| GND | GND |
| SCL | PA1 |
| SDA | PA0 |
| AD0 | GND |
| INT | 不接 |

### HC-05

| HC-05 | 天猛星 |
|---|---|
| VCC | 5 V |
| GND | GND |
| RXD | PB2 / UART3 TX |
| TXD | PB3 / UART3 RX |

> 必须共地。首次测试只连接控制板、MPU6050 和 HC-05，不连接电机电源。

## 电脑端设置

1. 给 HC-05 上电，并在电脑蓝牙设置中完成配对。
2. 在设备管理器中找到 HC-05 的“传出 COM 端口”。
3. 用串口助手打开该端口：`9600 baud, 8 data bits, no parity, 1 stop bit`。
4. 复位天猛星。

正常时先看到：

```text
MSPM0G3507 MPU6050 HC05 TEST
WHO_AM_I=0x68
ax,ay,az,temp,gx,gy,gz
```

随后每 100 ms 收到一行 MPU6050 原始数据。静止平放时，某一加速度轴的绝对值应接近 16384；转动模块时陀螺仪三轴数据应明显变化。

`WHO_AM_I=0x68` 是标准 MPU6050；现有驱动也接受兼容模块返回的 `0x70`。

## 首次上电检查

1. 逐针核对 VCC、GND、SCL、SDA，确认模块没有反插。
2. 上电后先观察 30 秒；如果 MPU6050 明显发烫，立即断电。
3. 若显示 `ERROR: MPU6050 NOT FOUND`，记录 `WHO_AM_I` 数值，再检查供电、共地、PA1/PA0 和 AD0。
4. 若电脑收到乱码，先确认串口端为 9600 8N1，并单独检查 PB2 到 HC-05 RXD 的连线。
