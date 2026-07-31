# MaixCAM color blob smoke test

这是 MaixCAM 视觉链路的第一个最小可运行程序，用于验证：

- 相机取流；
- LAB 色块识别；
- 最大目标筛选；
- 屏幕画框和中心坐标；
- `FOUND` / `LOST` 调试输出。

## 运行

使用 MaixVision 打开本目录，然后选择“运行项目”。项目入口必须保持为 `main.py`。

## 首次调参

把目标放到真实背景前，先调整 `main.py` 顶部的参数：

```python
COLOR_THRESHOLDS = [[0, 80, 40, 80, 10, 80]]
MIN_PIXELS = 500
MIN_AREA = 500
```

阈值格式为：

```text
[L_MIN, L_MAX, A_MIN, A_MAX, B_MIN, B_MAX]
```

验收时应测试目标在不同位置、距离和光照下的连续识别，并确认无目标时不会持续误报。
