# MaixCAM 钢珠识别

本项目使用 MaixCAM 和训练后的 YOLOv8 模型实时检测直径约 10 mm 的钢珠。
当前主线已经从早期的 HSV、霍夫圆和红色背景传统视觉方案切换为神经网络目标检测，
不再强制依赖红色背景。

程序会：

- 使用红色矩形框标出确认后的钢珠；
- 显示 `BALL1 68%` 形式的编号和模型置信度；
- 在左上角显示确认钢珠数量和完整循环帧率；
- 在终端输出原始检测数、候选数、确认数、坐标、尺寸和置信度；
- 对检测框做形状过滤、连续帧确认和位置平滑，降低瞬时误检。

## 当前技术配置

```text
视觉设备：MaixCAM
模型类型：YOLOv8
模型输入：320 × 320 RGB
模型类别：steel_ball（单类别）
模型加载：nn.YOLOv8
双缓冲：启用
模型文件：steel_ball_320_v2.mud + steel_ball_320_v2.cvimodel
应用 ID：maixcam_steel_ball
应用版本：0.3.0
安装路径：/maixapp/apps/maixcam_steel_ball/
```

当前主要参数位于 `main.py` 顶部：

```python
DETECTOR_CONFIDENCE_THRESHOLD = 0.45
CANDIDATE_CONFIDENCE_THRESHOLD = 0.45
DISPLAY_CONFIDENCE_THRESHOLD = 0.50
IOU_THRESHOLD = 0.35
MIN_ASPECT_RATIO = 0.70
MIN_BOX_SIDE = 8
TRACK_CONFIRM_FRAMES = 3
TRACK_MAX_MISSES = 2
```

## 文件位置

主程序：

```text
D:\diansai\vision\maixcam_steel_ball\
```

训练、导出和转换结果：

```text
D:\diansai\vision\maixcam_steel_ball_dataset\models\steel_ball_320_v2\
```

MaixCAM 部署文件：

```text
D:\diansai\vision\maixcam_steel_ball_dataset\models\steel_ball_320_v2\maixcam_deploy\
```

安装成应用后的模型：

```text
/maixapp/apps/maixcam_steel_ball/steel_ball_320_v2.mud
/maixapp/apps/maixcam_steel_ball/steel_ball_320_v2.cvimodel
```

`.mud` 文件记录模型类型、输入格式、归一化参数、标签和 `.cvimodel` 文件名。
两个文件必须属于同一次转换，并放在同一目录。

程序优先加载与 `main.py` 位于同一目录的模型，因此应用安装后不再依赖
`/tmp/maixpy_run/` 或 `/root/models/`。仅当同目录模型不存在时，才回退到：

```text
/root/models/steel_ball_320_v2/steel_ball_320_v2.mud
```

## 检测流程

```text
相机采集 320 × 320 RGB 图像
  -> YOLOv8 NPU 推理
  -> 置信度与 NMS
  -> 最小尺寸和长宽比过滤
  -> 多帧位置匹配
  -> 连续 3 帧确认
  -> 平滑坐标与置信度
  -> 画框、显示和终端日志
```

当前长宽比过滤范围约为：

```text
0.70 <= width / height <= 1.43
```

它用于排除明显细长的误检框，但不能代替模型判断。钢珠被遮挡、贴近画面边缘或高速运动时，
检测框可能不再接近正方形，过滤条件需要结合比赛场景调整。

## 三层置信度阈值

### 原始检测阈值

```python
DETECTOR_CONFIDENCE_THRESHOLD
```

传给 YOLOv8 推理接口。低于该值的结果不会进入 Python 后处理。
降低它可以提高召回率，但会增加 NPU 后处理结果和误检候选。

### 候选阈值

```python
CANDIDATE_CONFIDENCE_THRESHOLD
```

原始结果必须达到该值，并通过尺寸和长宽比过滤，才能进入连续帧跟踪。

### 最终显示阈值

```python
DISPLAY_CONFIDENCE_THRESHOLD
```

跟踪目标的平滑置信度达到该值，并连续出现至少 `TRACK_CONFIRM_FRAMES` 帧后才会画框。

如果只想提高最终画框要求，同时保留低置信度诊断信息，推荐：

```python
DETECTOR_CONFIDENCE_THRESHOLD = 0.40
CANDIDATE_CONFIDENCE_THRESHOLD = 0.40
DISPLAY_CONFIDENCE_THRESHOLD = 0.70
```

如果三个阈值全部设为 `0.70`，低于 70% 的结果会在模型输出阶段直接丢弃，
终端也无法再观察这些低置信度候选。

`IOU_THRESHOLD` 不是置信度。它用于 NMS 重叠框抑制；数值越低，对重复框的抑制通常越强，
但相互接触的多个钢珠也可能被误合并。

## MaixVision 手动运行

1. 使用 Type-C 将 MaixCAM 连接电脑。
2. 打开 MaixVision 并连接设备。
3. 使用“打开文件夹/项目”打开整个 `D:\diansai\vision\maixcam_steel_ball`。
4. 确认项目内同时存在 `main.py`、`.mud` 和 `.cvimodel`。
5. 按 `Ctrl+S` 保存参数修改。
6. 点击“运行项目”，不要只运行当前文件。
7. 点击方形“停止”按钮结束程序。

相机不需要单独启动。程序执行以下代码时会初始化相机和屏幕：

```python
cam = camera.Camera(
    detector.input_width(),
    detector.input_height(),
    detector.input_format(),
)
disp = display.Display()
```

重新点击运行前，应先停止上一份程序并等待约 2 秒。MaixCAM 的相机、ISP 和显示资源不能被
两份 Python 程序同时占用。

## 打包、安装和上电自启动

本目录已经按自包含 MaixPy 应用整理完成，`app.yaml` 会打包：

```text
main.py
README.md
steel_ball_320_v2.mud
steel_ball_320_v2.cvimodel
```

操作步骤：

1. 在 MaixVision 中打开整个 `D:\diansai\vision\maixcam_steel_ball` 文件夹。
2. 连接 MaixCAM。
3. 点击左下角“安装应用”。
4. 点击“打包应用”。
5. 点击“安装应用”。
6. 断开 MaixVision 后，在 MaixCAM 应用菜单中选择“钢珠识别”运行。

需要上电直接运行时，在 MaixCAM 中进入：

```text
设置 -> 开机自启动 / Auto-Start -> 钢珠识别
```

取消自启动也在同一设置项完成。不要通过修改 `/etc/rc.local` 强制启动相机程序，
否则可能与 MaixVision 争用相机和屏幕资源。

官方文档：

- `https://wiki.sipeed.com/maixpy/doc/en/basic/maixvision.html`
- `https://wiki.sipeed.com/maixpy/doc/en/basic/auto_start.html`

## 手动切换模型

开发时建议每套模型使用独立目录，不要混放同名的 `.mud`、`.cvimodel` 和 `main.py`。

当前主模型：

```python
MODEL_FILE_NAME = "steel_ball_320_v2.mud"
detector = nn.YOLOv8(model=MODEL_PATH, dual_buff=True)
```

本地另有两个 MaixHub YOLOv5 测试包：

```text
D:\diansai\model-296890.maixcam.zip
D:\diansai\model-297541.maixcam.zip
```

解压后分别使用：

```python
detector = nn.YOLOv5(model="model_296890.mud")
detector = nn.YOLOv5(model="model_297541.mud")
```

YOLOv5 和 YOLOv8 的加载类不能混用，`.mud` 中的 `model_type` 也必须与代码一致。
这两套外部模型只用于对比测试，不是当前项目主线。

## 终端日志

程序每隔约 0.5 秒输出：

```text
balls=2 candidates=3 raw=4 fps=58.9
x=82 y=138 w=43 h=41 conf=64
x=159 y=138 w=40 h=38 conf=65
```

字段含义：

- `raw`：YOLOv8 原始检测数量；
- `candidates`：通过候选阈值、尺寸和长宽比过滤后的数量；
- `balls`：通过连续帧确认和最终阈值后的数量；
- `fps`：相机读取、推理、后处理、画框和显示组成的完整循环帧率；
- `x/y/w/h`：模型输入图像坐标系中的检测框；
- `conf`：经过跟踪平滑后的模型置信度百分比。

常见判断：

- 真钢珠出现但 `raw=0`：原始阈值过高、模型泛化不足或钢珠成像过小；
- `raw>0` 且 `candidates=0`：被候选阈值、尺寸或长宽比过滤；
- `candidates>0` 且 `balls=0`：未连续出现 3 帧或平滑置信度不足；
- 无钢珠却持续 `balls>0`：需要提高阈值并补充困难负样本。

## 帧率说明

本程序显示的是完整视觉循环帧率，不是单独的 NPU 推理帧率。
以下操作都计入实际帧间隔：

```text
cam.read
detector.detect
候选过滤和多目标匹配
绘制矩形与文字
disp.show
MaixVision 视频编码和电脑端预览
```

在本项目的干净直连测试中，当前 `320 × 320 YOLOv8 + dual_buff=True` 曾达到约
`59～60 FPS`。通过 MaixVision 运行并开启电脑端画面预览时，视频编码、USB 传输和
MaixVision 通信会额外消耗 CPU 与内存，完整循环下降到约 `25～30 FPS` 是可能的。
这不等于 NPU 本身只能运行 30 FPS。

2026-07-26 的一次低帧率现场检查结果：

```text
Python 主程序 CPU：约 66%
MaixVision 服务 CPU：约 13%
系统 CPU 空闲：约 7%
设备空闲内存：约 5.5 MB
设备温度：约 52.8°C
Python 检测进程：1 个
```

因此该次低于 30 FPS 不是重复启动模型或过热造成，主要瓶颈是完整显示链路和较高的系统负载。

### 帧率排查顺序

1. 确认只运行一份 `python3` 检测程序。
2. 确认仍使用 `320 × 320` 主模型，而不是 `448 × 448` 测试模型。
3. 保持 `dual_buff=True`。
4. 比较“MaixVision 电脑端预览”和“只看 MaixCAM 屏幕”两种条件。
5. 暂时减少终端日志频率和画面文字数量，观察帧率变化。
6. 如果低阈值产生大量 `raw` 候选，提高原始阈值，降低 Python 后处理负担。
7. 停止程序并重新运行；若多媒体资源未正常释放，再重启 MaixCAM。
8. 长时间运行后检查 CPU、空闲内存和设备温度。

如果比赛控制只需要坐标，不需要电脑端实时画面，正式版本应减少或关闭 MaixVision 预览，
并降低不必要的绘制和日志频率。

## 数据集和重新训练

当前模型仍会受到钢珠表面反射、光照、背景、距离和相机曝光的影响。改善泛化能力时应继续采集：

- 暗光、强光、侧光、背光和不同色温；
- 不同背景、距离、角度和钢珠成像尺寸；
- 清晰对焦为主，并加入少量轻微前偏焦和后偏焦样本；
- 单球、多球、接触、遮挡和运动模糊；
- 没有钢珠的纯负样本；
- 容易误识别的圆形、金属和高光物体困难负样本。

训练时应合并新旧数据，避免只用新图片导致遗忘。训练集、验证集和测试集必须按拍摄场景划分，
不能把同一段连续视频的相邻帧随机分到不同集合。

更新流程：

```text
合并并检查标注
  -> 按场景划分数据
  -> 使用现有 best.pt 作为初始权重重新训练
  -> 在独立场景测试集评估误检和漏检
  -> 导出 ONNX
  -> 转换为 CVIModel + MUD
  -> 上传 MaixCAM
  -> 重新标定阈值
```

每次重新转换后，必须同时替换 `.mud` 和 `.cvimodel`，不能混用不同训练轮次的文件。
更换模型后还必须同步提高 `app.yaml` 的版本号，再重新打包和安装应用。

## 已知限制

- 镜面钢珠会反射整个环境，同一颗钢珠换光线和背景后外观可能明显变化；
- 模型报告中的验证准确率不能代替全新场景测试；
- 多个钢珠紧密接触时，NMS 可能合并检测框；
- 低阈值会提高召回率，但会增加圆形、高光物体的误检；
- 连续帧确认可以抑制瞬时误检，不能消除持续性的错误分类；
- 当前模型仍需通过更多光照场景和困难负样本继续训练。
