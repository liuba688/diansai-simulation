# 编译、部署与首次联合测试

## 1. 已验证的软件环境

```text
CCS：D:\ccs
TI ArmClang：ti-cgt-armllvm_5.1.1.LTS
MSPM0 SDK：D:\ccs\mspm0_sdk_2_11_00_07
目标 MCU：MSPM0G3507
PowerShell：7
```

2026-07-27 本机验证结果：

```text
CCS ProjectSpec 导入：成功
CCS 完整构建：0 errors，5 warnings
Makefile 构建：成功
Python 协议与跟踪测试：6 tests passed
MaixCAM Python 语法检查：通过
C 状态机主机测试：通过（GCC 13，ASan/UBSan）
```

5 个警告来自上游逐飞底层库的未使用函数、符号比较和自赋值，不来自新增寻球代码。

## 2. 编译 MSPM0G3507

推荐直接运行仓库内构建脚本：

```powershell
Set-Location D:\diansai\following_new\project\MSPM0G3507_Car\project\ccs
.\build.ps1 -Rebuild
```

生成：

```text
build/mspm0g3507_ball_car.out
build/mspm0g3507_ball_car.hex
build/mspm0g3507_ball_car.map
```

当前链接结果约使用 25,264 bytes Flash 和 7,007 bytes SRAM（SRAM 数字包含 4 KiB 栈），低于 MSPM0G3507 的 128 KiB Flash / 32 KiB SRAM。

### CCS ProjectSpec

在 CCS 的 ProjectSpec 导入界面选择：

```text
D:\diansai\following_new\project\MSPM0G3507_Car\project\ccs\mspm0g3507_ball_car.projectspec
```

导入工程名为 `mspm0g3507_ball_car`。若修改了仓库中的头文件，重新导入 ProjectSpec 以刷新 CCS 工作区内的头文件副本；命令行 `build.ps1` 始终直接使用仓库源码。

也可复现本机无界面构建：

```powershell
$workspace = 'D:\diansai\following_new\tmp\ccs_workspace_ball_car'
$spec = 'D:\diansai\following_new\project\MSPM0G3507_Car\project\ccs\mspm0g3507_ball_car.projectspec'
$cli = 'D:\ccs\ccs\eclipse\ccs-server-cli.bat'

& $cli -workspace $workspace -application projectImport '-ccs.location' $spec '-ccs.overwrite'
& $cli -workspace $workspace -application projectBuild '-ccs.projects' mspm0g3507_ball_car '-ccs.buildType' full '-ccs.configuration' Debug '-ccs.listProblems'
```

PowerShell 中带点号的 CCS 参数必须像上例一样加引号。

## 3. 下载到 MSPM0G3507

1. 连接调试器和小车控制板，先只给控制板供电，不接线圈功率。
2. 在 CCS 中选择实际使用的调试探针和 `MSPM0G3507`。
3. 连接 Target。
4. 加载 `build/mspm0g3507_ball_car.out`。
5. 复位并运行。

ProjectSpec 默认生成 XDS110 目标配置；如果实物使用 CMSIS-DAP 或其他探针，应在 CCS 中选择匹配的连接配置后再下载。编译成功不等于调试器已经连接或程序已烧入实物。

启动后：

- OLED 显示 `TI BALL CAR READY`。
- UART1 PB6/PB7 以 115200 输出调试日志。
- HC-05 UART3 PB2/PB3 保持 9600。
- PB16 为心跳 LED。

## 4. 部署 MaixCAM 应用

1. 在 MaixVision 中打开整个目录：

   ```text
   D:\diansai\following_new\maixcam_ball_car
   ```

2. 临时测试时直接运行项目。
3. 确认画面出现 `UART:ON/OFF`、`CAR:<state>`、钢珠框和 FPS。
4. 需要上电自启动时，使用目录内 `app.yaml` 打包。
5. 将生成的应用包安装到 MaixCAM，并在应用/启动设置中选择“寻球小车视觉”为开机自启动应用。

不能遗漏：

```text
steel_ball_320_v2.mud
steel_ball_320_v2.cvimodel
```

两者必须来自同一轮转换模型。

## 5. 首次联合测试顺序

### 阶段 A：只验证原循迹

1. 断开电磁铁线圈，架空车轮。
2. 不放钢珠，HC-05 发送 `1`。
3. 确认等待 3 s 后仍按原有灰度逻辑运行。
4. 发送 `0`，确认两轮立即停车。

### 阶段 B：验证 UART

1. 按接线文档连接 A19/A18/GND。
2. 启动 MaixCAM 应用。
3. MSPM0 UART1 日志中应看到 `vision=1`、持续增加的 `packets`，且 `crc`/`ovf` 不持续增加。
4. MaixCAM 画面应从 `CAR:--` 变成 `CAR:IDLE` 或 `CAR:LINE`。

若只有 MaixCAM → MSPM0 单向链路，识别和控制仍能工作，但 MaixCAM 会一直显示 `CAR:--`。

### 阶段 C：验证视觉转向

1. 仍然架空车轮，电磁铁线圈保持断开。
2. 让钢珠先达到红框，确认小车仍按灰度误差转向，但左右轮目标速度均约为原输出的三分之一；电磁铁保持关闭。
3. 等待红框变为绿框，确认该周期两轮目标立即变为 0，电磁铁开启并进入 `LOCK`。
4. 进入 `APPROACH` 后，将钢珠分别放在画面左侧和右侧，确认两轮目标转速方向会使车头朝钢珠转。
5. 若方向完全相反，只将 `ball_car.h` 中 `BALL_CAR_VISION_STEERING_SIGN` 从 `1.0f` 改为 `-1.0f`，重新编译；不要同时交换电机和视觉坐标。

### 阶段 D：验证丢失目标后的盲走

1. 先架空车轮，准备立即发送 `0`。
2. 合格目标第一次被锁定时，确认 PB10 已立即开启电磁铁。
3. 进入 `APPROACH` 后移走钢珠，确认偶发丢失不会立刻切换状态。
4. 连续约 100 ms 看不到目标后，确认小车以 11 RPM 直行 2 s。
5. 2 s 结束后确认程序按“已吸住”处理，立即开始倒放；按实际吸取距离只调整 `BALL_CAR_FINAL_CREEP_RPM` 或 `BALL_CAR_FINAL_CREEP_TICKS`。

### 阶段 E：接入电磁铁

1. 先用万用表/LED 验证 PB10 高电平能控制 MOSFET。
2. 接入额定电压正确的线圈和续流保护。
3. 降低整车速度并在短时间内测试一次吸取，检查线圈、MOSFET、导线和接头温升。
4. 确认电磁铁从首次锁定目标开始开启；2 s 盲走结束后开始 15 s 保持计时。
5. 确认倒放、重新捕线和恢复巡线期间电磁铁保持开启，15 s 到时自动关闭。
6. 验证全白区域时持续大幅右转，任一灰度通道重新检测到黑线后恢复巡线。首次必须架空车轮测试。

## 6. 命令和紧急处理

| HC-05 字符 | 功能 |
|---|---|
| `1` | 请求启动，3 s 后运动 |
| `0` | 立即停止车轮 |
| `2` | 停车状态下释放持球状态并关闭电磁铁 |

紧急情况首先切断电机/线圈功率。若只是软件停车，发送 `0`；若电磁铁仍因“已持球”保持吸合，再发送 `2`。不要把蓝牙命令当作唯一硬件急停手段。

## 7. 首轮验收记录

每次测试至少记录：

- 环境与背景、镜头焦距、光照；
- MaixCAM FPS、目标置信度、误检情况；
- 进入 `APPROACH` 时的左右轮方向；
- 目标进入盲区时的实际距离；
- 连续丢失确认时间、2 s 盲走距离和是否可靠吸取；
- 电磁铁从判定吸取开始的 15 s 保持时间；
- 倒放后离原线路距离；
- 搜线耗时、全白右转方向以及是否进入 `FAULT`；
- 电磁铁、MOSFET、TB6612、电机和接头温升。

实测结论应回填到 `开发进度.md` 和本文件，避免下一轮重复猜参数。
