# MaixCAM 水管钢球全幅采集

本应用保存 `640 × 480` 原图，水管沿画面横向尽量填满宽度。黄色横线只是纵向 ROI
预览，绿色纵线是 `x=320` 中轴线，用于对准水管零刻度 `O`。保存的 JPEG 中没有
参考线、文字或裁剪。

## 使用

1. 相机固定在水管正上方，光轴尽量垂直。
2. 水管左右端接近画面边缘，但保留端点标定余量。
3. 在 MaixVision 中打开本目录并运行整个项目。
4. `EMPTY PIPE` 阶段保持水管内没有钢球。
5. `PLACE STEEL BALL` 阶段放入钢球。
6. `RECORDING` 阶段让钢球覆盖中点、两端、指定点和运动状态。
7. 完成后将设备目录复制到电脑。

设备原图目录：

```text
/root/h_ball_pipe_captures/
```

电脑目标目录：

```text
projects/2026_h_vehicle_ball_balance/vision/dataset/captures_raw/
```

每换一次机位、曝光、灯光、摆杆角度或车体状态，应重新运行程序生成独立 `scene`。
验证集和测试集按 `scene` 隔离。
