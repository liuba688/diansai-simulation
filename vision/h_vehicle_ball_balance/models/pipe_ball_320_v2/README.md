# pipe_ball_320_v2

这是当前 2026 H 题四分 PPR 水管场景的正式钢球特化模型。它在相同验证集上优于第一版，第一版权重已按要求删除。

## 数据

- 原始素材：4 个场景，共 1728 张 `640×480` 图像。
- 裁剪范围：纵向 `y=[176,304)`，横向完整保留，得到 `640×128` 图像。
- 数据集：`vision/dataset/generated/dataset_run_0004/`。
- 有球标注：1666 张；无球负样本：49 张；排除自动标注未检出图像：13 张。
- 训练场景：`scene_0001`、`scene_0003`。
- 验证场景：`scene_0002`、`scene_0017`。
- 训练与验证按场景隔离。

## 正常训练配置

- 初始权重：`steel_ball_320_v2_best.pt`
- 训练轮数：60
- 最佳轮次：35
- 输入尺寸：320
- batch：16
- AMP：开启
- Mosaic：开启
- rectangular batches：关闭
- 随机种子：2027
- Ultralytics：8.4.110
- PyTorch：2.7.1+cu128

## 同条件复测结果

验证集固定为 839 张图像，其中包含 816 个钢球实例和 23 张负样本。两版均采用 `imgsz=320`、`batch=16` 和同一 GPU 复测。

| 模型 | Precision | Recall | mAP50 | mAP50-95 |
|---|---:|---:|---:|---:|
| 第一版（已删除） | 0.95678 | 0.94949 | 0.93854 | 0.52079 |
| 第二版（当前） | **0.96073** | **0.95952** | **0.94524** | **0.53804** |

第二版在四项指标上均更优。相对第一版，Recall 提高约 1.00 个百分点，mAP50-95 提高约 1.72 个百分点。

## 正式权重

```text
best.pt
SHA256: 51030EC5F07F37038EA3E6A50CB7E6C148EDCE3D94436F05DBF4B7C3E7663EF1
```

训练记录保存在：

```text
vision/models/training_runs/pipe_ball_320_v2_normal/
```

该目录仅保留训练配置、指标、曲线和最佳权重；非最佳 `last.pt` 不作为正式模型保留。

## 后续验证

已使用本地 WSL2 Docker 环境中的 `TPU-MLIR 1.28.1-20260429` 完成转换：

```text
ONNX 320×320
  -> YOLOv8 DFL/Sigmoid 输出节点裁剪
  -> 100 张四场景均衡图片进行 INT8 校准
  -> cv181x INT8 cvimodel
  -> MUD 模型描述
```

转换过程中，ONNX 到 MLIR、INT8 TPU 输出以及最终 cvimodel 模拟器输出比较均通过。
可部署文件位于：

```text
maixcam_deploy/pipe_ball_320_v2.mud
maixcam_deploy/pipe_ball_320_v2.cvimodel
```

下一步应在 MaixCAM 真机测量帧率、漏检、误检以及钢球中心坐标的静态和运动抖动。当前验证集来自四段已有素材，仍需补充一次独立拍摄的测试集。
