# 2026 H题车载平衡滚球统一工程

本目录是独立交付工程，不修改 `D:\diansai\following_new` 基线。它整合了：

- MSPM0G3507 四键选题、两阶段启动、计时与安全仲裁；
- MaixCAM 单常驻应用、钢球三点标定、毫米位置/速度 UART 输出和画面显示；
- MSPM0 到 ZDT X42S V2.0（Emm_V5.0）的 UART 标准位置模式驱动；
- 已验证底盘八路红外循迹、双编码器速度 PID、A-B 150 cm 与 RACE LINE 一圈停车；
- Task 3 `v3.4.5` 往返算法与 Task 4 `v4.2.5` 零点抗振算法的 C 版移植；
- Task 5 复用 Task 4 中心保持控制器；Task 6 按要求保留为不可启动的明确空实现。

## 交付入口

- `dist/h_vehicle_ball_balance_mspm0g3507_v1.0.2.hex`：MSPM0 最新烧录文件；Task 3 完成优先、无 5 s 故障退出。
- `dist/maix-h_vehicle_ball_balance-v1.0.0.zip`：MaixVision 安装包。
- `docs/01_architecture_and_tasks.md`：结构、任务绑定与算法说明。
- `docs/02_wiring_and_safety.md`：接线和首次上电安全流程。
- `docs/03_installation_and_operation.md`：烧录、安装、按键操作和联调步骤。
- `docs/04_protocol_and_tuning.md`：UART 帧、参数位置和调参方法。
- `docs/05_verification_report.md`：构建、测试和未完成的硬件验证边界。

## 关键操作

车载按钮为 PB13=UP、PB23=DOWN、PB26=OK、PB27=BACK，均内部上拉、按下为低。

1. UP/DOWN 选择题目。
2. 需要小车的 Task 2/4/5/6 会先进入速度页；UP/DOWN 选择四档，OK 确认并进入相机准备。
3. OLED 显示 `READY / OK TO START` 后，再按 OK；此刻立即开始计时并发送 START。
4. 任意时刻按 BACK，底盘先停、MaixCAM 收到 STOP、MSPM0 直接向 X42S 发送 `01 FE 98 00 6B`。

首次整机上电前必须阅读安全文档。带连杆、带球测试不得跳过脱开连杆验证。
