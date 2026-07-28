# MSPM0G3507 + MaixCAM 循迹寻球小车

本仓库的当前正式主线是在已经通过实车验证的 70 RPM 循迹小车上，加入 MaixCAM 钢珠识别、脱线接近、电磁铁吸取、原路退回和重新循迹功能。

## 当前完成状态

- 保留原有 YB-MVX05 八路灰度循迹、双编码器速度 PI、TB6612 驱动和 HC-05 启停逻辑。
- MaixCAM 使用 `steel_ball_320_v2` YOLOv8 模型，经过多帧跟踪后只选择一个稳定主目标。
- MaixCAM 与 MSPM0G3507 通过 115200 baud 双向 UART 和 CRC8 二进制协议通信。
- MSPM0 已实现完整寻球取球状态机：红框预警时保持循迹并降至原输出的三分之一，绿框时立即停车锁定并开启电磁铁，随后视觉接近、丢失后低速盲走 2 s、15 s 吸合保持、指令轨迹倒放、搜线和带球循迹。
- PB10 只控制外置 MOSFET 模块的逻辑输入，电磁铁线圈不直接连接 MCU。
- MSPM0 工程已在本机 CCS/TI ArmClang 环境完整编译通过；Python 协议测试、MaixCAM 代码语法检查和纯 C 状态机主机测试均通过。

截至 2026-07-27，软件链路已完成，但 MaixCAM、整车和电磁铁三者的联合实机测试仍需在安全架空状态下进行。视觉转向符号、2 s 盲走距离、全白右转方向和电磁铁温升属于必须按实车标定的项目。

## 目录

| 路径 | 用途 |
|---|---|
| `project/MSPM0G3507_Car/` | MSPM0G3507 正式小车工程 |
| `project/MSPM0G3507_Car/project/code/` | 循迹、速度环、寻球状态机、UART 协议与电磁铁驱动 |
| `project/MSPM0G3507_Car/project/ccs/` | CCS ProjectSpec、TI ArmClang Makefile、链接脚本和构建脚本 |
| `maixcam_ball_car/` | 可直接由 MaixVision 打开、运行和打包的视觉应用 |
| `docs/` | 架构、协议、接线、部署与首次测试文档 |
| `PCB/` | 本地 PCB 原理图和生产资料 |
| `tests/` | UART 协议和状态机测试 |

## 快速编译 MSPM0 固件

在 PowerShell 7 中运行：

```powershell
Set-Location D:\diansai\following_new\project\MSPM0G3507_Car\project\ccs
.\build.ps1 -Rebuild
```

输出文件：

```text
build/mspm0g3507_ball_car.out
build/mspm0g3507_ball_car.hex
build/mspm0g3507_ball_car.map
```

也可在 CCS 中导入：

```text
D:\diansai\following_new\project\MSPM0G3507_Car\project\ccs\mspm0g3507_ball_car.projectspec
```

## MaixCAM 应用

在 MaixVision 中打开整个 `maixcam_ball_car` 文件夹，不要只复制 `main.py`。临时测试可直接运行；制作开机应用时按 `app.yaml` 打包并安装。模型 `.mud` 与 `.cvimodel` 已放在应用目录内。

## 上车前必读

- [系统架构与状态机](docs/ball_car_architecture.md)
- [MaixCAM–MSPM0 UART 协议](docs/vision_uart_protocol.md)
- [接线与供电](docs/ball_car_wiring.md)
- [编译、部署和首次测试](docs/deployment_and_test.md)
- [软件交付完成审计](docs/completion_audit.md)
- [PCB 接线与引脚速查](PCB接线与引脚速查.md)

最重要的安全规则：所有 UART 设备必须共地；MaixCAM IO 只能接 3.3 V 逻辑；MaixCAM 用 Type-C 供电时不要再接 PCB UART2 的 `+5V`；电磁铁必须由合适的外置 MOSFET 和独立功率回路驱动。
