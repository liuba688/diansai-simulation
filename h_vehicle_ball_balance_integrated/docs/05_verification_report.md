# 构建与验证报告

日期：2026-08-01

## 已完成的软件验证

- TI ArmClang 5.1.1.LTS 完整重建成功，生成 `.out/.hex/.map`。
- Flash 使用 `0xB290 / 0x20000`（45,712 / 131,072 字节，约 34.9%）。
- SRAM 使用 `0x4D75 / 0x8000`（19,829 / 32,768 字节，约 60.5%），剩余 12,939 字节。
- Python 8 项测试通过：CRC、分块解析、有符号目标、BALL_STATE 布局、CRC拒绝、三点标定、
  单帧误检拒绝、源代码链路约束。
- `main.py / mission_protocol.py / ball_estimator.py` Python 语法编译通过。
- MaixCAM ZIP 内容和 app.yaml 文件清单经过打包后检查。
- X42S 代码只出现 Emm 标准位置 `FD`、使能 `F3` 和停止 `FE`；未使用快速位置 `FC`。

编译告警来自复用的 `line_follow` 与逐飞底层库（未使用函数、符号比较、自赋值），新增加的
协议、任务、滚球和 X42S 模块没有编译告警。

## 需求对应

| 需求 | 当前证据 |
|---|---|
| 新目录独立交付 | 本目录完整源码、docs、dist |
| 相机与单片机通信 | v2 协议双端实现和 Python 协议测试 |
| 单片机与 X42S 通信 | 本地手册核对、UART1驱动、固件构建 |
| 六题菜单 | Task 1-6 + Calibrate + Diagnostic |
| 两次 OK | CONFIGURING/READY/STARTING 状态机 |
| 四档速度 | 复用四组已测参数，默认 FAST |
| 距离固定绑定 | Task 4=150 cm；Task 2/5=RACE LINE |
| Task 3 | v3.4.5 参数/状态机 C 移植 |
| Task 4/5 | v4.2.5 参数/滞回/斜率 C 移植 |
| Task 6 空出 | 菜单明确保留且禁止运动 |
| 固件和相机包 | dist 中 HEX/OUT/Maix ZIP |

## 尚需实物验证

本报告不把“编译成功”冒充“整车验收”。以下必须在硬件上完成：

1. 烧录后四键和 OLED 页面回归。
2. 逻辑分析仪确认 UART2 MODE_SELECT/READY/STARTED 与 task/run 一致。
3. 脱开连杆确认 UART1 `F3/FD/FE` 回包、TTL 电平和 BACK 独立急停。
4. 新链路下 Task 3 至少连续三轮 ≤5 s，正负端均在 ±10 mm 误差范围。
5. Task 4 在小车 A-B 全过程保持 ±10 mm 且 ≤8 s。
6. Task 5 全圈保持 ±10 mm 且 ≤30 s。
7. 环线外设备对 Task 1-5 每次测试完整录像并能回放。

因此当前交付是“可烧录、可安装、软件构建/协议测试通过的整合首版”，不是未经实车复验就
宣称各题已最终达标的比赛定版。建议严格按 `03_installation_and_operation.md` 顺序联调。
