# 模型版本

每个正式模型使用独立英文目录，例如：

```text
pipe_ball_v1/
  dataset_manifest.json
  training_config.yaml
  metrics.json
  best.pt
  model.onnx
  model.cvimodel
  model.mud
  evaluation.md
```

不同训练轮次的 `.mud` 和 `.cvimodel` 不得混用。

## 当前正式版本

- `pipe_ball_320_v2/`：正常内存配置训练的水管钢球特化模型。
- PyTorch 权重：`pipe_ball_320_v2/best.pt`
- 训练与验证记录：`pipe_ball_320_v2/README.md`
- 第一版在同验证集上的各项指标均低于第二版，权重已删除。

## 真机候选版本

- `pipe_ball_480x96_v1/`：由当前最佳权重导出的 `480×96` INT8候选模型。
- 它不替换当前 `320×320` 正式版本。
- 对应独立MaixVision项目：`vision/runtime_480x96/`。
- 必须完成同机位A/B测试后，才能决定是否升级为正式模型。
