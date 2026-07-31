# MaixCAM 第四问零点抗扰台架版

当前工作区入口已切换为：

```text
version=4.0.5-bench
profile=task4_ramped_breakaway_v6
target=0 mm
```

程序只执行：

```text
SEARCH_BALL -> RECOVER_CENTER -> HOLD_CENTER <-> RECOVER_CENTER
```

钢球可从摆杆内任意可识别位置启动回零，不要求先放在 O 点。视觉丢球时摆杆回水平并
持续搜索，重新识别后自动回中。不会自动运行 `O -> +5 cm -> -5 cm`。台架接线、首次
安全验证、日志字段和调参顺序见：

```text
../../docs/2026-07-31_task4_center_hold_bench.md
```

MaixVision 应直接打开当前 `task4_center_hold_bench/` 文件夹并运行 `main.py`。
当前目录只包含第四问运行所需文件，不包含第一问控制器、测试代码或 `versions/`。

以下内容是原第一问基线说明，仅作历史参考。正式第一问候选版保存在原开发目录中，
没有被本次整理覆盖。

---

# MaixCAM 固定动作表完整第一问（历史）

> 当前工作目录保留后续实验代码，不作为现场推荐入口。当前保留候选版是
> `versions/timed_full_task_v3_1_4_cal435_r3/`，第 34 次在 `4.82 s` 严格通过。
> 现场复测必须从该版本目录或对应 ZIP 启动，避免误运行工作区中的其他实验参数。

保留候选版 `v3.1.4-cal435-r3` 执行完整轨迹：

```text
WAIT_CENTER
  -> HOLD_CENTER
  -> POS_PUSH
  -> POS_BRAKE
  -> POS_SETTLE
  -> NEG_PUSH
  -> NEG_BRAKE
  -> NEG_SETTLE
  -> COMPLETE
```

不是只运行到 `+50 mm`。只有视觉确认球已到正端附近且速度足够低，程序才进入负向段；
最后在 `-50 mm` 持续闭环并满足稳定判据后进入 `COMPLETE`。

## 固定动作表

| 阶段 | 球杆角 | 最长时间 | 视觉提前切换条件 |
|---|---:|---:|---|
| `POS_PUSH` | `+2.20°` | `0.50 s` | `x>=+5 mm` 或 `v>=+25 mm/s` |
| `POS_BRAKE` | `-2.20°` | `0.40 s` | 进入正端 PD 接管区 |
| `NEG_PUSH` | `-1.40°` | `0.90 s` | `x<=+25 mm` 或 `v<=-90 mm/s` |
| `NEG_BRAKE` | `+1.00°` | `1.00 s` | `x<=-42 mm` 或速度回升到 `-40 mm/s` |

各阶段都有最短执行时间，避免单帧速度噪声立即触发换相。正端 PD 阶段只有实际进入
`+40～+60 mm` 合格带并满足速度门槛后才能折返，禁止仅因阶段超时提前折返。

正负端收敛使用同一简单 PD：

```text
angle = 0.045 * position_error - 0.018 * velocity
```

- 收敛角限制：`±2.20°`；
- 低速且误差超过 `5 mm` 时，最低起滚角 `1.10°`；
- 正端到点：`|x-50|<=10 mm` 且 `|v|<=40 mm/s` 后折返；
- 负端完成：`|x+50|<=8 mm`、`|v|<=12 mm/s`，持续 `0.12 s`；
- 总时间限制：`5.00 s`。
- `COMPLETE` 后立即输出 `0°/0 pulse`，不再继续执行负端静摩擦补偿。

所有首轮调参量集中在 `task1_timed_control.py` 文件顶部，后续按实机日志逐段修改。

## 安全边界

- 球达到 `±105 mm` 立即 `ball edge`；
- 视觉超时、电机应答异常立即回水平；
- X42S 相对上电水平位保持 `±160 pulse`；
- 最大控制角仍为已标定的 `±3°`；
- 机械硬限位、挡片和总电源急停必须保留；
- 未确认具体固件版本前不使用快速位置模式。

## 接线与标定

```text
MaixCAM A19 / UART1_TX -> X42S TTL_RX
MaixCAM A18 / UART1_RX <- X42S TTL_TX
MaixCAM GND            -> X42S GND
UART 115200 8N1，地址 1，校验字节 0x6B
```

视觉标定：

```text
+50 mm -> raw_x 212
  0 mm -> raw_x 324
-50 mm -> raw_x 435
```

连杆标定：

```text
0° -> 0 pulse
1° -> 80 pulse
3° -> 160 pulse
```

## 日志与调参

日志字段：

```text
state t raw_x x v ref rv ra angle cmd lim boost sent ack err rej fault fps
```

每次只调一个阶段：

1. `POS_PUSH` 后速度过高：减小 `+2.20°` 或缩短 `0.50 s`；
2. 正端过冲：更早进入 `POS_BRAKE`，或增大 `-2.20°`；
3. 正端停在 `+42 mm` 之前：延长正向推进或减小正向制动；
4. 负向推进不足：增大 `-1.40°` 或延长 `0.90 s`；
5. 负向过早反转：减小 `+1.00°` 或延后负端 PD 交接；
6. 越过负端仍高速：增大负向制动角或提前开始制动；
7. 最终来回振荡：优先增大速度阻尼，不增加位置比例。

`rej` 是被拒绝的单帧跳变累计数。单帧测量相对上次有效球位跳变超过
`max(12 mm, 300 mm/s × 帧间隔)` 时不进入控制；只有连续两帧落在相近新位置才重新
捕获，避免 `raw_x≈73` 一类瞬时误检触发端部故障。

下一轮必须返回从 `POS_PUSH` 到 `COMPLETE/FAULT` 的完整日志，不能只截取最后一行。

## 安装与验证

安装包：

```text
dist/maix-h_pipe_ball-timed-full-task-v3.1.4-cal435-r3.zip
```

保留源码、安装包副本、SHA-256、第 34 次日志和结果摘要位于：

```text
versions/timed_full_task_v3_1_4_cal435_r3/
```

完整开发原理和 r1～r3 演进见：

```text
../../docs/2026-07-30_ball_beam_development_progress.md
```
