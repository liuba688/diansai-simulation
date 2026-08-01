# 安装、烧录和操作说明

## 1. MSPM0 烧录

最新交付文件：`dist/h_vehicle_ball_balance_mspm0g3507_v1.0.17.hex`。

可用 UniFlash/CCS/XDS 烧录 Intel HEX。烧录后先不接 X42S 连杆，复位并确认 OLED 出现
八项菜单。需要调试符号时使用 `dist/h_vehicle_ball_balance_mspm0g3507_v1.0.17.out`。

烧录后的首次动作测试必须拆球并脱开连杆。每次整车与 X42S 共同上电前先人工将水管调平；
程序只在本次上电初始化时建立一次软件水平零点，Task 3 完成或按 BACK 后返回该零点。

源码重建：

```powershell
cd D:\diansai\h_vehicle_ball_balance_integrated\mspm0\project\ccs
.\build.ps1 -Rebuild
```

已验证环境：CCS `D:\ccs`、TI ArmClang 5.1.1.LTS、MSPM0 SDK 2.11.00.07。

## 2. MaixCAM 安装

交付文件：`dist/maix-h_vehicle_ball_balance-v1.0.0.zip`。

在 MaixVision 中连接 MaixCAM，使用“安装应用/从本地 ZIP 安装”，选择该文件。应用安装后
启动 `H题车载平衡滚球统一程序`。程序会初始化 640×128/60 fps 相机、YOLO 模型、
显示和 UART1；不再直接打开 X42S。

也可用 SFTP 解压到：

```text
/maixapp/apps/h_vehicle_ball_balance
```

并运行：

```sh
cd /maixapp/apps/h_vehicle_ball_balance
python3 main.py
```

正式测试前在环线外接收设备提前开始完整录像。MaixCAM 屏幕画面包含 task/run/state、
球位置、速度、测量年龄、帧率和 UART 错误计数，便于录像后定位问题。

## 3. 菜单操作

### Task 1/3

1. UP/DOWN 选择题号。
2. 第一次 OK：发送 MODE_SELECT，执行机构保持静止。
3. 相机连续检测有效后 OLED 显示 `READY / OK TO START`。
4. 放球条件满足后按第二次 OK；该按键时刻立即开始计时。
5. BACK 停止并返回。

### Task 2/4/5

1. UP/DOWN 选择题号，按 OK 进入速度页。
2. UP/DOWN 选择 CONSERVATIVE/NORMAL/FAST/SPRINT。
3. OK 确认速度并开始 MODE_SELECT 准备；此时不计时、不运动。
4. OLED 显示 READY 后再按 OK，立即计时并启动。
5. Task 4 从 110 cm 开始分段减速并在 150 cm 软停；Task 2/5 在 550-600 cm 预减速，第一次重新经过基准线后只向前软停。

Task 6 当前显示保留提示并返回菜单，绝不会启动底盘。

## 4. 各题建议档位

- Task 2：先 FAST 验证停车，再用 SPRINT 冲击 20 s。
- Task 4：首次 CONSERVATIVE；稳定后 NORMAL。8 s 目标需结合真实 A-B 里程再提高。
- Task 5：首次 CONSERVATIVE；确认全圈钢球均在 ±10 mm 后依次 NORMAL、FAST。
- Task 1/3 不使用底盘速度，菜单不额外询问档位。

## 5. 标定与方向

当前标定：+50 mm=212 px，O=324 px，-50 mm=435 px，图像左侧为物理正方向。
相机、镜头、摆杆或门架拆装后，修改 `maixcam/ball_estimator.py` 三个像素常量并重新打包。

当前摆杆映射：0°=0 pulse、1°=80 pulse、3°=160 pulse，3-4°继续按 40 pulse/°外推。
如果实际执行器端正方向相反，只能在脱开连杆时确认后统一修改驱动方向映射；不可同时改
相机坐标符号和电机方向来“试到能动”。

## 6. 现场故障提示

| OLED/现象 | 含义 | 处理 |
|---|---|---|
| CAM PREPARING 长时间不变 | 相机没回 MODE_READY或球未连续识别 | 查 UART、task/run、模型框 |
| WAIT CAM READY | 配置阶段误按启动 | 等 READY，不会运动 |
| MISSION FAULT CODE 10 | MODE_READY 重试失败 | 查相机程序和 TX/RX |
| CODE 11 | STARTED 300 ms 超时 | 查相机是否仍 READY |
| CODE 21 | Task 3 连续 1 s 无有效视觉 | 停车，查遮挡/帧率/UART |
| CODE 22 | 球接近端部 | 取下球并查方向/参数 |
| CODE 30 | X42S 连续错误 | 查 TTL版本、地址、0x6B、供电和共地 |
| CODE 31 | 相机心跳超时 | 查相机进程/UART |

故障后用 BACK 返回。不要在带球、带连杆状态下反复按 OK 试错。
