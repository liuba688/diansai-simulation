# OLED 四键预设菜单

## 当前功能

- OLED 一屏显示 4 项，菜单共有 8 个任务位，可上下滚动。
- 菜单分为比赛任务(5项) + 诊断工具(2项) + 设置(1项)。
- 第 1 项 `CALIBRATE`：IMU 偏航校准（上电后必须先校准再比赛）。
- 第 2 项 `RACE LINE`：基础循线一圈 + 行驶计时 + A 点停车（16 分项）。
- 第 3 项 `STATIC BALL`：静态滚球 O↔±5cm（13 分项，待实现）。
- 第 4 项 `A-B BALANCE`：A 到 B 动态平衡 1.5m（20 分项，待实现）。
- 第 5 项 `LAP BALANCE`：整圈中心/任意位置平衡（20 分项，待实现）。
- 第 6 项 `ANGLE HOLD`：航向保持诊断。
- 第 7 项 `ODOMETER`：查询累计里程。
- 第 8 项 `SPEED TIER`：速度档位切换（保守/正常/快速），OK 键循环切换。
- BACK 键短按/长按都会立即停车并返回菜单。
- 任务运行时 OLED 显示行驶计时（RACE LINE）、速度、航向和灰度状态；停车后计时冻结。

## 速度档位

| 档位 | 直线 RPM | 弯道 RPM | 巡线 Kp | 速度 Kp/Ki/Kd |
|---|---|---|---|---|
| CONSERVATIVE | 120 | 75-90 | 0.18 | 12/1/0 |
| NORMAL | 140 | 82-100 | 0.20 | 12/1/0 |
| FAST (默认) | 160 | 92-115 | 0.20 | 12/1/0 |

选中 `SPEED TIER` 后按 OK 循环切换，OLED 显示当前档位名和关键参数 1.2 秒后返回菜单。

## 比赛计时规则

- 计时起点：0.5s 启动延迟结束、电机开始转动的时刻（`motion_enabled=true`）。
- 计时终点：终点横线确认、电机刹车的时刻。
- 不计入：启动延迟、终点倒车寻线对齐阶段。
- 停车后 OLED 冻结最终行驶时间，BACK 返回菜单后清除。

## 按键映射

| 按键 | 引脚 | 功能 |
|---|---|---|
| UP | PB13 | 上一项 |
| DOWN | PB23 | 下一项 |
| OK | PB26 | 短按：执行当前项；长按：查询里程 |
| BACK | PB27 | 停车并返回菜单 |

按键按下有效电平为低电平，GPIO 使用内部上拉。

## 赛题公布后增加任务

1. 在 `car_menu.h` 的 `car_task_t` 中确定任务编号。
2. 在 `car_menu.c` 的 `car_menu_items[]` 中修改对应英文短名称。
3. 在 `car_app.c` 的 `car_apply_menu_task()` 中添加任务启动逻辑。
4. 如需蓝牙快捷键，在 `car_bluetooth_query_command()` 中返回相同的 `car_task_t`。

菜单文字建议不超过 19 个 ASCII 字符。当前字库没有中文字形，因此菜单使用英文。

## 编译验证

2026-07-31 改造后待编译验证：

```text
Program Size: Code=xxxxx RO-data=xxxx RW-data=xxx ZI-data=xxxx
0 Error(s), 0 Warning(s)
```
