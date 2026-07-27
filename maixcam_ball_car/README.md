# MaixCAM 寻球小车视觉应用

本应用把现有 `steel_ball_320_v2` YOLOv8 模型与 MSPM0G3507 小车控制协议整合在一起。
它仍会显示所有达到显示阈值的钢珠，但只从达到更高控制阈值、连续出现更多帧的目标中选择一个
稳定主目标发送给小车，避免低置信度误检直接触发离线追球。

## 当前阈值

```text
模型原始阈值：45%
画框阈值：50%
小车控制阈值：70%
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

## 打包与临时运行

在 MaixVision 中打开整个 `maixcam_ball_car` 文件夹，不要只打开 `main.py`。临时测试时运行项目；
需要上电自启动时按 `app.yaml` 打包安装，再在 MaixCAM 设置中选择该应用为开机自启动。

模型的 `.mud` 与 `.cvimodel` 必须同时保留在应用目录，不能与其他训练轮次混用。

MSPM0 固件、完整接线和联合测试步骤见仓库根目录：

- `README.md`
- `docs/ball_car_wiring.md`
- `docs/deployment_and_test.md`

## 接近阈值标定

`CLOSE_BOX_HEIGHT_RATIO = 0.30` 是首轮估计。架空车轮进行测试，记录钢珠刚进入电磁铁可靠吸取范围时
检测框高度占 320 像素画面的比例，再修改该值。数值过小会提前盲走，数值过大可能让钢珠先进入镜头
盲区而无法触发最终吸取。
