# MaixCAM 钢珠数据集自动采集与标注

该工具用于建立不依赖固定红色背景的钢珠检测数据集。背景可以是红色、白色、木纹、
黑色桌面或其他比赛现场表面，但同一次 `scene` 采集中必须保持相机和背景不动。

用户不需要手动画框。流程由两部分组成：

```text
MaixCAM main.py
  -> 自动拍摄空背景参考帧
  -> 自动录制含钢珠的样本

PC tools/build_dataset.py
  -> 空背景差分
  -> 圆形、边缘和金属明暗特征复核
  -> 自动排除手、相机晃动和不确定帧
  -> 导出 YOLO 标注、预览图和报告
```

## 1. 在 MaixCAM 上采集

在 MaixVision 中运行本目录的 `main.py`。每次运行会自动建立新的
`/root/steel_ball_captures/scene_XXXX/`，不会覆盖旧数据。

一次采集分三个阶段：

1. `EMPTY BACKGROUND`，持续 3 秒：画面中不要放钢珠。
2. `PLACE / MOVE BALLS`，持续 6 秒：放入钢珠。
3. `RECORDING`，持续 90 秒：改变钢珠数量和位置，可以让钢珠滚动。

程序以 `640x480`、约 5 张/秒保存图片，一次约生成 450 张样本。画面上出现
`CAPTURE COMPLETE` 后本场景完成。

同一场景的录制阶段不要移动相机或背景。手进入画面的帧会在 PC 端因变化区域过大
而自动进入 `review/`，不会直接加入训练集。

建议至少采集这些场景：

- 哑光红底；
- 白纸或浅色桌面；
- 木纹桌面；
- 黑色或深灰色桌面；
- 带少量纹理、阴影的比赛模拟场地；
- 加入螺母、垫片、金属碎片和圆形污点等干扰物；
- 每个背景分别包含 0～8 个球、相接球、边缘球和运动球。

为了按场景隔离训练集与验证集，至少需要 3 个场景；正式训练建议 10 个以上不同
背景/光照场景。

## 2. 把采集目录复制到电脑

使用 MaixVision 文件管理器、`scp` 或 WinSCP，把设备上的：

```text
/root/steel_ball_captures/
```

复制到：

```text
D:\diansai\vision\maixcam_steel_ball_dataset\captures\
```

目录结构应为：

```text
captures/
  scene_0001/
    meta.json
    background/
    samples/
```

## 3. 自动标注并导出 YOLO 数据集

PC 环境依赖见 `requirements.txt`。运行：

```powershell
python tools/build_dataset.py
```

每次执行会建立新的目录，不覆盖之前结果：

```text
datasets/dataset_run_0001/
  data.yaml
  report.json
  annotations.csv
  images/train/
  images/val/
  labels/train/
  labels/val/
  preview/train/
  preview/val/
  review/
```

其中：

- `images/` 和 `labels/` 是可直接训练的 YOLO 数据；
- `preview/` 是已经接受的标注框预览；
- `review/` 保存不确定、变化过大或未能可靠解释的画面，这些画面默认不会进入训练；
- `report.json` 记录接受、空背景、排除帧和标注框数量；
- `annotations.csv` 保存每个自动框的圆心、半径、得分及特征，便于追溯。

默认半径范围是 `20～34 px`，对应 `640x480` 下当前相机距离附近的 10 mm 钢珠。
如果相机高度明显变化，可使用：

```powershell
python tools/build_dataset.py --min-radius 12 --max-radius 48
```

## 自动标注边界

自动标注不等于把所有候选强行当作真值。本工具采取保守策略：

- 高置信候选才写入 YOLO 标签；
- 有变化却找不到可靠圆形时，整张图进入 `review/`；
- 手或相机晃动造成大面积变化时，整张图进入 `review/`；
- 空背景参考帧会作为确定的无球负样本加入数据集；
- 少于 3 个场景时只能按图片拆分训练/验证集，报告会给出警告。

生成数据后由 Codex 查看 `preview/` 与 `review/`，修正算法参数或少量异常标签，
不要求用户逐张标注。

## 4. 相机发生移动时的保守补救

空背景差分仍是首选方案。若同一场景中相机发生移动，不能通过简单放宽变化面积
阈值强行生成标签，否则容易出现手部遮挡和画面边缘钢珠漏标。

对于已有但机位发生移动的采集，可使用本地钢珠 YOLO 权重作为教师模型，并由
圆形、尺寸和金属明暗候选做一对一复核：

```powershell
python tools/build_teacher_dataset.py --circle-score 0.58
```

额外依赖：

```powershell
pip install -r requirements_teacher.txt
```

输出目录为：

```text
datasets/teacher_dataset_run_XXXX/
```

只有教师模型框与传统视觉强候选完全一一对应的帧才进入训练集。教师模型漏检、
传统视觉不一致、手部遮挡或人工复核排除的帧全部进入 `review/`，不会生成训练
标签。人工复核排除清单保存在：

```text
quality_overrides/teacher_exclusions.txt
```

该补救方案适合从不完美采集中提取少量高质量种子数据，不能替代固定相机、固定
背景的规范采集，也不能仅凭单一场景的验证集指标判断模型泛化能力。

### 固定机位小目标与 ROI

不同相机距离下的钢珠像素半径不同，应按场景分别构建。画面含有实验室杂物时，
可指定可信桌面区域；输出训练图片会同步裁剪到 ROI，避免区域外未标注物体进入
训练集。

`scene_0007` 的已验证参数：

```powershell
python tools/build_teacher_dataset.py `
  --scene scene_0007 `
  --roi 280,0,360,480 `
  --min-radius 6 `
  --max-radius 16 `
  --teacher-conf 0.15 `
  --teacher-probe-conf 0.10 `
  --circle-score 0.78 `
  --imgsz 320 `
  --min-explained-change 0.75 `
  --max-changed-fraction 0.12
```

其中：

- `--min-explained-change 0.75`：至少 75% 的背景变化必须被钢珠框覆盖，防止
  教师模型和圆检测共同漏掉暗球。
- `--max-changed-fraction 0.12`：ROI 内变化超过 12% 时拒绝该帧，主要用于
  排除手部进入画面的样本。
- `--roi` 只适用于该场景已经确认的可信区域；更换机位后必须重新检查坐标。
