# 寻球小车软件交付完成审计

审计日期：2026-07-27

本文件按用户目标逐项记录可复核证据。审计范围是仓库替换、MaixCAM 与 MSPM0 软件实现、构建、自动测试和新增接线说明；整车机械安装后的距离与时间参数仍需按实物标定。

## 1. 最新仓库基线

| 要求 | 证据 | 结论 |
|---|---|---|
| 使用用户后来确认的最新仓库 | `origin=https://github.com/JUYU-fsw/IT-preparation.git` | 通过 |
| 本地基线与远端一致 | `HEAD=origin/main=3773bb9864de353aa64500cf07ac5ef362e22581`，`HEAD...origin/main=0/0` | 通过 |
| 旧本地代码不丢失 | 备份在 `D:\diansai\following_new_legacy_backup_20260727_0cc1165` | 通过 |
| 不上传 GitHub | 功能改动保持为本地未提交工作区；远端提交未改变 | 通过 |

## 2. 抖音示例功能

目标流程：

```text
循迹
→ 发现并确认钢珠
→ 停车锁定
→ 脱线视觉接近
→ 电磁铁吸取
→ 倒放离线路径
→ 灰度重新捕线
→ 带球继续循迹
```

实现证据：

- `project/MSPM0G3507_Car/project/code/ball_car.c/.h`
- `project/MSPM0G3507_Car/project/code/car_app.c/.h`
- `project/MSPM0G3507_Car/project/code/magnet_driver.c/.h`
- `tests/test_ball_car.c`

主机测试已真实执行，编译参数包含：

```text
GCC 13
-Wall -Wextra -Werror
-fsanitize=address,undefined
```

已覆盖并通过：

- 低置信度或未确认目标不能让小车离线；
- 完整循迹、确认、接近、吸取、倒退、捕线、带球循迹循环；
- 视觉持续丢失后关闭电磁铁并倒退；
- 接近超时后倒退；
- 重新捕线超时进入停车故障；
- 已持球时停车保持吸合，人工释放后关闭电磁铁；
- C 端目标帧解析、CRC 错误识别和状态帧构造。

## 3. MaixCAM 视觉与通信

实现证据：

- `maixcam_ball_car/main.py`
- `maixcam_ball_car/ball_tracker.py`
- `maixcam_ball_car/vision_uart.py`
- `maixcam_ball_car/vision_protocol.py`
- `maixcam_ball_car/config.py`
- `maixcam_ball_car/steel_ball_320_v2.mud`
- `maixcam_ball_car/steel_ball_320_v2.cvimodel`
- `maixcam_ball_car/app.yaml`

验证结果：

- Python 协议和目标跟踪共 6 项测试通过；
- 全部 Python 文件通过语法编译；
- 应用清单列出的所有文件和配套模型均存在；
- 显示阈值 50%，控制阈值 70%，控制前连续确认 5 帧；
- 目标帧最高约 30 Hz，小车状态约 5 Hz，MSPM0 端 200 ms 链路超时。

## 4. MSPM0G3507 构建

验证环境：

```text
CCS：D:\ccs
TI ArmClang：5.1.1.LTS
MSPM0 SDK：2.11.00.07
```

验证结果：

- CCS ProjectSpec 可导入；
- CCS 完整构建：0 errors，5 个上游逐飞库警告；
- `build.ps1 -Rebuild` 成功；
- 最终 Map 中存在 `car_app_run`、`ball_car_update`、`vision_uart_process`、`vision_protocol_feed`、`magnet_driver_set`、`line_follow_update` 和 `speed_pid_update`；
- Flash 使用 25,264 bytes，SRAM 使用 7,007 bytes；
- `.out`、`.hex` 和 `.map` 已生成。

## 5. 新增接线

已写入：

- `docs/ball_car_wiring.md`
- `PCB接线与引脚速查.md`

新增信号：

```text
MaixCAM A19 TX → PCB UART2 RX / PA24
MaixCAM A18 RX ← PCB UART2 TX / PA23
MaixCAM GND ↔ 小车 GND
PB10 → 外置 3.3 V 兼容 MOSFET IN
```

MaixCAM 用 Type-C 供电时，PCB UART2 的 `+5V` 留空。电磁铁线圈必须使用外置 MOSFET、合适的功率电源和续流保护，禁止直接连接 PB10。

## 6. 实车阶段边界

软件交付已完成并通过静态、主机和目标编译验证。以下参数依赖实际镜头高度、电磁铁位置、轮胎打滑和线圈规格，不应在没有整车的情况下伪造“最终值”：

- `CLOSE_BOX_HEIGHT_RATIO`
- `BALL_CAR_VISION_STEERING_SIGN`
- `BALL_CAR_FINAL_CREEP_TICKS`
- `BALL_CAR_PICKUP_SETTLE_TICKS`

首次联合测试必须按 `docs/deployment_and_test.md` 的架空、单向 UART、视觉转向、接近距离和电磁铁五阶段顺序执行。
