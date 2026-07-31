# 钢珠检测训练与部署报告

更新日期：2026-07-26

## 1. 地毯场景 `scene_0009`

- 原始采集：14 张空背景、443 张样本，分辨率 `640x480`。
- 相机与地毯保持固定，包含单球、分散多球、相邻球和成团球。
- 画面左侧柜体存在反光，因此只使用可信地毯区域：

  ```text
  ROI = x:280, y:100, width:360, height:380
  ```

- 最终自动标注参数：

  ```powershell
  python tools/build_dataset.py `
    --scene scene_0009 `
    --roi 280,100,360,380 `
    --min-radius 6 `
    --max-radius 16 `
    --max-changed-fraction 0.06 `
    --min-explained-change 0.70
  ```

- 最终输出：

  ```text
  datasets/dataset_run_0003/
  ```

- 结果：268 张含球图、8 张采样空背景、3 张标准空背景、1144 个钢珠框；
  167 张不确定或含手画面进入 `review/`，未参加训练。

## 2. 三场景合并

合并数据集：

```text
datasets/combined_dataset_run_0001/
```

采用按场景隔离的划分，禁止同一场景同时出现在训练集和验证集：

- 训练：`scene_0004`、`scene_0009`
  - 316 张图
  - 1244 个钢珠框
  - 14 张无球负样本
- 验证：`scene_0007`
  - 174 张图
  - 438 个钢珠框
  - 3 张无球负样本

合并命令：

```powershell
python tools/combine_scene_datasets.py `
  --train datasets/teacher_dataset_run_0003 `
  --train datasets/dataset_run_0003 `
  --val datasets/teacher_dataset_run_0004
```

## 3. 首轮训练

- 输入尺寸：`320x320`
- 初始化权重：本地开源钢珠 `best.pt`
- GPU：NVIDIA GeForce RTX 4070 Laptop GPU
- PyTorch：`2.7.1+cu128`
- Windows 数据加载进程：`workers=0`
- 训练目录：

  ```text
  training_runs/steel_ball_320_v1_retry/
  ```

训练在第 36 轮完成后因 Windows 虚拟内存不足停止，但每轮均已保存检查点。
验证指标在第 10 轮达到最佳，后续出现跨场景泛化下降，因此无需采用最后一轮。

最佳权重独立复验结果：

| 指标 | 数值 |
|---|---:|
| Precision | 0.980 |
| Recall | 0.904 |
| mAP50 | 0.968 |
| mAP50-95 | 0.684 |

独立复验输出：

```text
training_runs/steel_ball_320_v1_best_val/
```

## 4. 固化模型

首版 PyTorch 权重：

```text
models/steel_ball_320_v1/steel_ball_320_v1_best.pt
```

SHA256：

```text
4E97BAC20B86C6C1AD346FFCF83EBFCD29055F7A96F0FAF421BDA6EA51F7C9C6
```

用于 MaixCAM INT8 转换的 100 张校准图：

```text
models/steel_ball_320_v1/calibration_images_100.zip
```

校准包 SHA256：

```text
F5C06F5335F282FE56C7DA07A93C62A49A06BD10F76DF04BF88CDEC91790421F
```

已导出并通过 ONNX Runtime 复验的固定输入模型：

```text
models/steel_ball_320_v1/steel_ball_320_v1_best.onnx
```

- 输入：`images [1,3,320,320]`
- 输出：`output0 [1,5,2100]`
- ONNX 验证：Precision 0.966、Recall 0.905、mAP50 0.950、mAP50-95 0.682
- SHA256：`F5BC6250CD7E9B8604AFD8687CBA6CD0DDD273B3116E8131156BC712D9B3312B`

按 MaixCAM 官方推荐节点裁剪后的 TPU-MLIR 输入：

```text
models/steel_ball_320_v1/steel_ball_320_v1_export.onnx
```

- 输出 1：`/model.22/dfl/conv/Conv_output_0 [1,1,4,2100]`
- 输出 2：`/model.22/Sigmoid_output_0 [1,1,2100]`
- SHA256：`5BAA83F6E23AF8467C231F50821F67FFF6E889CEB70F118DB7DD8904974799F9`

已经准备好的 Linux 转换工作区：

```text
models/steel_ball_320_v1/maixcam_convert/
```

其中包含 `export.onnx`、100 张校准图、测试图、`convert_sg2002.sh` 和
`steel_ball_320_v1.mud`。脚本目标处理器固定为 `cv181x`、量化方式为 INT8。

2026-07-26 已使用 TPU-MLIR `1.28.1-20260429` 完成面向
MaixCAM/SG2002（`cv181x`）的 INT8 转换。ONNX、TPU MLIR 和最终
CVIModel 数值比较全部通过。

部署文件位于：

```text
models/steel_ball_320_v1/maixcam_deploy/
├── steel_ball_320_v1.mud
└── steel_ball_320_v1.cvimodel
```

`steel_ball_320_v1.cvimodel` 大小为 3,228,752 字节，SHA256 为：

```text
CC3FA60FDBF8841E36E8663620A4A19BAA4C0857DA3DF8B894E03866C08AE982
```

## 5. 第二轮新增场景

第二轮加入了用户实拍的蓝底、反光白底和哑光红底场景：

| 场景 | 背景 | 进入训练的含球图 | 标准空背景 | 钢珠框 |
|---|---|---:|---:|---:|
| `scene_0011` | 蓝色 | 118 | 3 | 1037 |
| `scene_0013` | 反光白色 | 8 | 3 | 13 |
| `scene_0015` | 哑光红色 | 168 | 3 | 1254 |

对应数据集目录：

```text
datasets/dataset_run_0004/
datasets/teacher_dataset_run_0009/
datasets/dataset_run_0008/
```

蓝底和红底使用背景差分、圆形约束及人工可视化复核生成标注。反光白底
无法由背景差分稳定标注，因此只保留了旧模型检测结果中同时满足严格圆形
约束的少量样本；这部分数据不能视为独立验证集。

## 6. 第二轮合并与训练

第二轮训练数据集：

```text
datasets/combined_dataset_run_0002/
```

- 训练集：619 张图、3548 个钢珠框、23 张无球负样本。
- 验证集：174 张图、438 个钢珠框、3 张无球负样本。
- 验证场景仍固定为旧的 `scene_0007`，避免同一采集序列同时进入训练和验证。

训练配置：

- 输入尺寸：`320x320`
- 初始化权重：`steel_ball_320_v1_best.pt`
- 最大轮数：100
- 实际记录：75 轮，早停
- 最佳轮次：第 45 轮
- 训练目录：`training_runs/steel_ball_320_v2_train/`

第 45 轮训练记录：

| 指标 | 数值 |
|---|---:|
| Precision | 0.975 |
| Recall | 0.950 |
| mAP50 | 0.992 |
| mAP50-95 | 0.621 |

## 7. 新场景对比

为观察加入新背景前后的差异，另建：

```text
datasets/combined_dataset_run_0003/
```

该集合包含第二轮的 303 张新场景图、2304 个框和 9 张空背景。相同评估
设置下：

| 模型 | Precision | Recall | mAP50 | mAP50-95 |
|---|---:|---:|---:|---:|
| v1 | 0.135 | 0.166 | 0.030 | 0.010 |
| v2 | 0.828 | 0.830 | 0.837 | 0.461 |

此结果说明 v2 已明显学会新增背景，但蓝底和红底图参与过 v2 训练，因此
这些数值不是严格的跨场景泛化成绩；反光白底标注还受到 v1 教师模型筛选
偏差影响。最终效果必须用重新拍摄、从未参加训练的完整新场景复测。

## 8. 第二版固化模型

第二版 PyTorch 权重：

```text
models/steel_ball_320_v2/steel_ball_320_v2_best.pt
```

SHA256：

```text
862CC398BC3C15EC0417228CCA26036C4A9B36448E40135642367208F9170DD0
```

标准 YOLOv8 ONNX：

```text
models/steel_ball_320_v2/steel_ball_320_v2_best.onnx
```

SHA256：

```text
92D7A27F7748BADB90E70965D54274060DBB73A7672676E9F119BD8984087AB3
```

按 MaixCAM 推荐节点裁剪后的 TPU-MLIR 输入：

```text
models/steel_ball_320_v2/steel_ball_320_v2_export.onnx
```

- 输出 1：`/model.22/dfl/conv/Conv_output_0`
- 输出 2：`/model.22/Sigmoid_output_0`
- SHA256：`D45F6907790D17D1CF33FB3E126766ECFB76B64096AD25B2EF5B5BE7C31A2454`

100 张校准图按场景均衡抽取：

```text
models/steel_ball_320_v2/calibration_images_100.zip
```

SHA256：

```text
29C60F282441675DA64B4AF5F8A10F737BFC2AEC41DFB449DDDA62D21099E4F2
```

## 9. MaixCAM 部署包

已使用 TPU-MLIR `1.28.1`，按 `cv181x` 处理器和 INT8 量化完成转换。
ONNX/MLIR 到 INT8 TPU、INT8 TPU 到最终 CVIModel 的数值比较均通过。

部署目录：

```text
models/steel_ball_320_v2/maixcam_deploy/
├── steel_ball_320_v2.mud
└── steel_ball_320_v2.cvimodel
```

`steel_ball_320_v2.cvimodel`：

- 大小：3,228,032 字节
- SHA256：`D323BC22D0DDEE5AE6F5BE139A6BDD319727A18F38013F8428AFDBFEA9752350`

可直接复制到 MaixCAM 的压缩包：

```text
models/steel_ball_320_v2/steel_ball_320_v2_maixcam.zip
```

- 大小：2,748,199 字节
- SHA256：`1776B5D89FD00A075EA7DA083CE82E030869F246F8CD3F10273A505350B5F75F`

压缩包内只有同名的 `.mud` 与 `.cvimodel` 两个部署文件。下一步是在
MaixCAM 上临时切换到 v2，使用全新的背景、距离、角度和光照组合复测
误检、漏检、置信度阈值和帧率；测试通过前不覆盖当前可回退模型。
