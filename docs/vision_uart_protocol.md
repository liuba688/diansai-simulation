# MaixCAM–MSPM0G3507 UART 协议

## 1. 物理层

```text
UART：115200 baud，8 data bits，no parity，1 stop bit
MaixCAM：UART1，A19=TX，A18=RX，设备 /dev/ttyS1
MSPM0G3507：UART2，PA23=TX，PA24=RX
```

TX 与 RX 交叉连接并共地。电气电平必须为 3.3 V TTL。

## 2. 通用帧

| 偏移 | 字段 | 长度 | 说明 |
|---:|---|---:|---|
| 0 | Header 0 | 1 | `0xAA` |
| 1 | Header 1 | 1 | `0x55` |
| 2 | Version | 1 | 当前 `0x01` |
| 3 | Type | 1 | 数据类型 |
| 4 | Length | 1 | Payload 字节数 |
| 5 | Sequence | 1 | 0～255 循环 |
| 6 | Payload | N | 小端序多字节字段 |
| 6+N | CRC8 | 1 | 从 Version 到 Payload 末尾的 CRC |

CRC 参数：

```text
多项式：0x07
初值：0x00
不反射
结果异或：0x00
```

## 3. 钢珠目标帧

方向：MaixCAM → MSPM0G3507
类型：`0x10`
Payload：16 字节
总帧长：23 字节

| Payload 偏移 | 字段 | 类型 | 说明 |
|---:|---|---|---|
| 0 | flags | uint8 | 目标标志 |
| 1 | center_x | uint16 LE | 目标中心 X，左上角为原点 |
| 3 | center_y | uint16 LE | 目标中心 Y |
| 5 | width | uint16 LE | 检测框宽 |
| 7 | height | uint16 LE | 检测框高 |
| 9 | frame_width | uint16 LE | 模型输入图宽，当前 320 |
| 11 | frame_height | uint16 LE | 模型输入图高，当前 320 |
| 13 | confidence | uint8 | 0～100 |
| 14 | candidate_count | uint8 | 当前可控候选数 |
| 15 | reserved | uint8 | 保留，当前为 0 |

`flags`：

| Bit | 名称 | 含义 |
|---:|---|---|
| 0 | `VALID` | 当前存在可发送目标 |
| 1 | `CONFIRMED` | 目标已通过多帧确认 |
| 2 | `CLOSE` | 框高已达到接近阈值 |
| 3 | `MULTIPLE` | 当前有多个可控候选 |

MaixCAM 最多约 30 Hz 发送目标帧。无目标时仍发送 `flags=0`，使主控及时清除旧坐标。

## 4. 小车状态帧

方向：MSPM0G3507 → MaixCAM
类型：`0x20`
Payload：8 字节
总帧长：15 字节

| Payload 偏移 | 字段 | 类型 | 说明 |
|---:|---|---|---|
| 0 | state | uint8 | 状态机编号 |
| 1 | flags | uint8 | 运行/电磁铁/持球/故障 |
| 2 | fault | uint8 | 故障编号 |
| 3 | line_mask | uint8 | 八路灰度位图 |
| 4 | left_rpm_x10 | int16 LE | 左轮目标 RPM × 10 |
| 6 | right_rpm_x10 | int16 LE | 右轮目标 RPM × 10 |

状态编号：

```text
0 IDLE
1 LINE_FOLLOW
2 TARGET_CONFIRM
3 STOP_LOCK
4 APPROACH
5 FINAL_CREEP
6 PICKUP_SETTLE
7 BACKTRACK
8 REACQUIRE_LINE
9 LINE_FOLLOW_CARRY
10 FAULT
```

状态 `flags`：

```text
bit0 ENABLED
bit1 MAGNET_ON
bit2 PAYLOAD_HELD
bit3 FAULT
```

主控约 5 Hz 回传状态。该回传用于 MaixCAM 屏幕显示和诊断，不参与 MSPM0 的安全停车判断。

## 5. 实现位置

- MSPM0：`project/MSPM0G3507_Car/project/code/vision_protocol.c`
- MaixCAM：`maixcam_ball_car/vision_protocol.py`
- 协议自动测试：`tests/test_vision_protocol.py`

修改协议时必须同时更新 C、Python、测试和本文件，不得只改一端。
