# UART 协议与调参说明

## 1. 通用帧

```text
AA 55 | version | type | payload_length | sequence | payload | CRC8
```

- version=`0x02`；最大 payload 20 字节。
- CRC8 覆盖 version 到 payload，初值 0，多项式 0x07，不包含 AA55 和 CRC 字节。
- 多字节任务字段使用小端；X42S 厂商帧的多字节字段按手册使用大端，两种协议不可混用。

## 2. 消息类型

| 类型 | 值 | 方向 |
|---|---:|---|
| BALL_STATE | 0x10 | CAM→MCU |
| MODE_SELECT | 0x30 | MCU→CAM |
| START | 0x31 | MCU→CAM |
| STOP | 0x32 | MCU→CAM |
| MCU_HEARTBEAT | 0x33 | MCU→CAM |
| MODE_READY | 0x40 | CAM→MCU |
| STARTED | 0x41 | CAM→MCU |
| CAM_HEARTBEAT | 0x42 | CAM→MCU |
| TASK_COMPLETE | 0x43 | CAM→MCU，预留 |
| FAULT | 0x44 | CAM→MCU |

MCU 命令 payload 固定 10 字节：

```text
task_id:u8 | run_id:u8 | target_x10_mm:i16 | speed_tier:u8 |
flags:u8 | timestamp_ms:u32
```

BALL_STATE payload 固定 14 字节：

```text
task_id:u8 | run_id:u8 | flags:u8 | position_x10_mm:i16 |
velocity_mm_s:i16 | confidence:u8 | measurement_age_ms:u16 |
camera_timestamp_ms:u32
```

事件 payload 为 task_id、run_id、result、state。所有改变状态的相机回包必须同时匹配
当前 task_id/run_id；迟到的上一轮包只计数，不触发动作。

## 3. X42S 帧

标准位置帧共 13 字节：

```text
Addr FD Dir Speed_H Speed_L Acc Pulse_31..0 Mode Sync 6B
```

本工程 Addr=1、Speed=30 RPM、Acc=100、Sync=0。正目标映射为 CCW(1)，负目标为
CW(0)；Mode=0 表示相对上一输入目标，准备时 Mode=2/Pulse=0 锚定当前轴位置。

驱动只接受 4 字节 ACK `Addr Function Status 6B`：02 成功，E2 参数/条件错误，EE 格式
错误，9F 到位通知。120 ms 无 ACK 计错，连续 3 次进入故障。立即停止帧固定为
`01 FE 98 00 6B`。

## 4. 参数位置

| 参数 | 文件 |
|---|---|
| 四档速度、循迹 Kp、速度 PID | `mspm0/project/code/car_menu.c` |
| 红外滤波、入弯阈值、候选区和圆弧渐变 | `mspm0/project/code/line_follow.h` |
| A-B 110/140/150 cm 减速点、RACE 550/600 cm 减速点和单向软停 | `mspm0/project/code/car_app.c` |
| Task 3/4/5 控制参数 | `mspm0/project/code/ball_balance.c` |
| X42S 速度、加速度、周期、限幅 | `mspm0/project/code/zdt_emm_v5.c` |
| 相机三点标定、滤波、误检阈值 | `maixcam/ball_estimator.py` |
| 相机置信度、分辨率、UART周期 | `maixcam/main.py` |

所有程序仍可独立修改参数和算法；统一 main 只负责调用，模块不是打包后不可修改的黑盒。

## 5. 调参顺序

1. 固定相机和摆杆，校准 +50/O/-50 三点，确认符号。
2. 脱开连杆核对 X42S 正负、ACK、目标斜率和 BACK。
3. 静止底盘复验 Task 3 三轮，记录完成时间、两端峰值、串口错误。
4. 静止底盘复验 Task 4 正负扰动，确认 ±10 mm 内水平锁存不会延迟滑脱。
5. Task 4 选 CONSERVATIVE 跑 A-B；出界时关联球 x/v、底盘档位和循迹阶段。
6. 优先降低底盘加速度/弯道速度，再调整滚球 Kd；不要同时改多个方向和增益。
7. 每个候选参数至少连续三轮，保留原始视频、UART日志、版本号和失败原因。

Task 3 的 `-0.20°` 完成保持偏置来自原台架零位，机械拆装后可能需要单独重测；这是最应
优先检查的机构相关参数。Task 4/5 的 v4.2.5 参数只证明静止台架抗扰，整车结果应以新日志为准。
