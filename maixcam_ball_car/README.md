# MaixCAM 寻球小车视觉应用

本应用把现有 `steel_ball_320_v2` YOLOv8 模型与 MSPM0G3507 小车控制协议整合在一起。
它仍会显示所有达到显示阈值的钢珠。红框目标会作为预警目标发送给小车，使当前灰度巡线左右轮
目标速度同时降低为原输出的三分之一，但不会开启电磁铁或进入寻球；达到更高控制阈值并完成多帧
确认后变为绿框，绿框出现的当前控制周期立即停车、开启电磁铁并进入寻球锁定流程。

## 当前阈值

```text
模型原始阈值：45%
画框阈值：50%
小车控制阈值：60%
显示确认：连续 3 帧
控制确认：连续 5 帧
```

这些参数集中在 `config.py`。调低 `CONTROL_CONFIDENCE_THRESHOLD` 会增加小车追错目标的概率，
因此不要只为提高画框数量而降低控制阈值。

## UART

```text
MaixCAM UART1
TX = A19
RX = A18
设备 = /dev/ttyS1
波特率 = 115200
```

程序以最高约 30 Hz 发送 23 字节二进制目标帧，无目标时也会持续发送 `valid=0` 的帧，
使 MSPM0 能及时清除旧目标。MSPM0 以约 5 Hz 回传小车状态；接好双向串口后，画面左上角会显示
`CAR:LINE`、`CAR:APPROACH` 等状态。

目标帧标志含义：

- 红框：`VALID=1, CONFIRMED=0`，TI 保持灰度巡线但将实时左右轮输出同时乘以 `1/3`。
- 绿框：`VALID=1, CONFIRMED=1`，TI 立即停车并进入寻球锁定。
- 无框：`VALID=0`，TI 使用原巡线速度。

## 打包与临时运行

在 MaixVision 中打开整个 `maixcam_ball_car` 文件夹，不要只打开 `main.py`。临时测试时运行项目；
需要上电自启动时按 `app.yaml` 打包安装，再在 MaixCAM 设置中选择该应用为开机自启动。

模型的 `.mud` 与 `.cvimodel` 必须同时保留在应用目录，不能与其他训练轮次混用。

MSPM0 固件、完整接线和联合测试步骤见仓库根目录：

- `README.md`
- `docs/ball_car_wiring.md`
- `docs/deployment_and_test.md`

## 接近与盲走

当前 MSPM0 控制逻辑不再根据 `CLOSE_BOX_HEIGHT_RATIO` 切换取球状态。该字段仍保留在 UART 帧和画面显示中，
用于兼容旧固件与调试观察；小车会在目标连续丢失约 100 ms 后，以 11 RPM 继续直行 2 s，再按“已吸住”
处理并倒放。盲走距离应在 MSPM0 的 `ball_car.h` 中通过 `BALL_CAR_FINAL_CREEP_RPM` 和
`BALL_CAR_FINAL_CREEP_TICKS` 标定，修改相机端的 `CLOSE_BOX_HEIGHT_RATIO` 不会改变当前小车动作。
