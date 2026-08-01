# 统一工程结构、任务绑定与算法

## 1. 固定控制链

```text
MaixCAM
  钢球检测、像素三点标定、位置/速度估计、显示/图传画面
  -> UART1 A19/A18, 115200 8N1
MSPM0G3507
  唯一菜单、任务状态机、计时、安全仲裁、滚球算法、底盘控制
  -> UART1 PB6/PB7, 115200 8N1, Emm_V5.0
ZDT X42S V2.0
  标准位置模式执行摆杆目标

MSPM0G3507 -> PWM+DIR -> TB6612 -> 双底盘电机
八路红外 -> MSPM0 循迹；双编码器 -> 速度闭环/里程/停车补偿
```

MaixCAM 不识别赛道；无线显示/录像端不参与控制闭环；X42S 不使用 CAN、STEP/DIR 或
未确认固件版本的快速位置模式。

## 2. 目录职责

| 路径 | 内容 |
|---|---|
| `mspm0/project/user/src/main.c` | 薄入口，只初始化时钟并调用 `car_app` |
| `mspm0/project/code/car_app.*` | 10 ms 主循环和成熟底盘逻辑的接入层 |
| `mspm0/project/code/car_menu.*` | 四键、八项菜单、四档速度和准备/启动按键事件 |
| `mspm0/project/code/h_mission.*` | MENU/CONFIGURING/READY/STARTING/RUNNING/FINISHED/FAULT |
| `mspm0/project/code/vision_protocol.*` | AA55+CRC8 二进制协议 |
| `mspm0/project/code/vision_uart.*` | UART2 接收环形缓冲、相机事件和球状态快照 |
| `mspm0/project/code/zdt_emm_v5.*` | UART1 Emm 标准位置、ACK、限幅、超时和急停 |
| `mspm0/project/code/ball_balance.*` | Task 3、Task 4/5 滚球控制器 |
| `mspm0/project/code/line_*` | 复用八路红外连续 PD 与丢线保护 |
| `mspm0/project/code/speed_pid.*` | 20 ms 双轮速度闭环 |
| `maixcam/main.py` | 单常驻相机、YOLO 检测、任务协议和画面叠加 |
| `maixcam/ball_estimator.py` | 三点毫米换算、速度滤波、单帧误检拒绝 |

## 3. 八项菜单和固定绑定

| OLED | 任务 | 相机 | 底盘距离 | 摆杆 |
|---|---|---|---|---|
| TASK 1 VIDEO | 第1问 | 检测/画面/图传检查 | 禁止运动 | 不发运动目标 |
| TASK 2 RACE | 第2问 | 画面和心跳 | RACE LINE | 不发运动目标 |
| TASK 3 BALL | 第3问 | 球位置/速度 | 停车 | O→+50→-50 mm |
| TASK 4 A-B | 第4问 | 球位置/速度 | A-B 150 cm | 保持 0 mm |
| TASK 5 LAP | 第5问 | 球位置/速度 | RACE LINE | 保持 0 mm |
| TASK 6 RESERVED | 第6问 | 预留 | 设计绑定为 RACE LINE | 算法空出，当前禁止启动 |
| CALIBRATE | 标定 | 本版不切换任务 | 停车 | 不运动 |
| DIAGNOSTIC | 诊断 | 状态链路 | 停车 | 不运动 |

距离没有单独菜单，避免现场选错：Task 4 固定 150 cm，其余需要小车的一圈任务固定
RACE LINE。Task 6 虽保留了路由定义，但因指定位置算法按要求暂空，当前菜单会显示
`TASK 6 RESERVED / ALGORITHM EMPTY` 并返回，不会只开小车而让钢球失控。

## 4. 四档底盘参数

| 档位 | 直线 RPM | 弯道 RPM | 线 Kp | 速度 Kp/Ki/Kd |
|---|---:|---:|---:|---:|
| CONSERVATIVE | 110 | 52-72 | 0.24 | 12/1/0 |
| NORMAL | 140 | 82-100 | 0.20 | 12/1/0 |
| FAST | 160 | 92-115 | 0.20 | 12/1/0 |
| SPRINT | 175 | 90-130 | 0.20 | 14/1.2/0 |

默认改为 FAST；这是现有工程的满载稳定基线。Task 2 追求 20 s 时可选 SPRINT；
Task 4/5 首次整车联合测试应从 CONSERVATIVE 开始，再逐档提高。

## 5. 两阶段启动

题目选择/速度确认后只执行准备：

1. MSPM0 生成新 `run_id`，底盘保持 PWM=0。
2. 向 MaixCAM 发送 MODE_SELECT。
3. 平衡任务只使能 X42S 并用“相对当前实时位置、0 脉冲”锚定软件零点，不产生位移。
4. 相机清空旧滤波并取得连续 3 帧有效球坐标，回复 MODE_READY。
5. X42S 与相机均准备完成后 OLED 才显示 `READY / OK TO START`。
6. 再按 OK 的瞬间锁存 `start_tick`，发送 START；收到 STARTED 后才放行底盘。

MODE_READY 重发最多 3 次；旧 task/run 回包丢弃。MSPM0 每 200 ms 发送心跳，MaixCAM
以 1.2 s 为 MCU 掉线门限。STARTED 300 ms 超时或 X42S 连续 3 次错误都会禁止底盘并停止摆杆。

## 6. Task 3 算法移植

来源：`maix-h-task3-round-trip-bench-v3.4.5`，当前配置名
`task3_completion_priority_v7`。关键动作与完成优先修订如下：

- O 点连续稳定 0.35 s；中心保持 0.25 s。
- 正向用 +3.00°破静摩擦，球开始朝正端移动即切 -2.20°制动。
- 进入 +50 mm 的 ±10 mm 评分带后立即折返。
- 负向主要驱动 -1.60°；反向速度尚大时使用 -3.00°。
- 负端使用位置-速度 PD，进入 -50 mm 的 ±9 mm 且速度 ≤12 mm/s 0.20 s 后完成。
- 取消 5 s 软件故障退出；超过评分时间或暂时离开评分误差带时继续闭环，直到完成。
- 正负端 settle 控制器允许静摩擦补偿再次触发，因此制动过早可二段或多段起滚。
- 单帧视觉无效不再中止，先水平等待；连续 1 s 无有效视觉才安全停止。
- 完成后不撤销保持：快速负向时 +0.30°提前卸载，安静时使用台架实测 -0.20°偏置。
- 软件角度限 ±3°，电机目标自然不超过 ±160 pulse。

原台架 Python 版连续三次通过时间 3.82/4.31/4.38 s。本工程是等参数 C 移植，编译和
静态协议测试通过，但换成 MaixCAM→MSPM0→X42S 后的通信延迟必须重新做至少三次实机验收。

## 7. Task 4/5 算法移植

来源：`maix-h_task4_center_hold_bench-v4.2.5-in-spec-latch`，配置名
`task4_in_spec_latch_v11`。Task 4 和 Task 5 共用：

- 近区 Kp/Kd=0.040/0.035，远区=0.045/0.032，积分关闭。
- `|x|≥9 mm` 或 `|v|≥30 mm/s` 进入恢复；`|x|≤8 mm` 且 `|v|≤12 mm/s`
  连续 0.40 s 返回保持。
- `|x|≤8 mm` 且 `|v|≤5 mm/s` 进入水平锁存；到 9 mm 或 12 mm/s 才解除。
- 静摩擦触发 0.20 s，朝 O 速度达到 4 mm/s 立即释放起滚补偿。
- 同向电机目标步长 24 pulse/40 ms，减小倾角或反向制动允许 48 pulse/40 ms。
- HOLD 限 ±3.2°，恢复限 ±3.9°，总软件限 ±200 pulse。

静止台架 v4.2.5 已验证两侧约 43 s 不持续振荡。Task 5 只是延长底盘路线，滚球参数相同；
若整车振动导致出界，应先降低底盘档位/加速度，不应立即放大滚球 Kp。

## 8. 复用底盘行为

- 八路数字红外，黑线高电平，误差 -350..+350。
- 丢线前 50 ms 低速确认，随后正向差速搜索，再短时含反转搜索，仍失败即停车。
- 目标速度每 10 ms 加速最多 4 RPM、减速最多 10 RPM。
- A-B 使用编码器累计 150 cm 后软停。
- RACE LINE 第一次宽黑线清里程，至少 550 cm 后第二次宽线触发停车补偿。
- 摄像头数据从不用于赛道识别或底盘转向。
