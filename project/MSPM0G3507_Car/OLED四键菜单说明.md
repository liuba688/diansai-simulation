# OLED 四键预设菜单

## 当前功能

- OLED 一屏显示 4 项，菜单共有 8 个任务位，可上下滚动。
- 第 1 项 `LINE FOLLOW`：与蓝牙命令 `1` 共用巡线启动入口。
- 第 2 项 `ANGLE HOLD`：与蓝牙命令 `h` 共用当前航向保持入口。
- 第 3～8 项为 `EMPTY TASK`，确认后不驱动电机，只通过串口报告空任务并返回菜单。
- 蓝牙 `0` 或 BACK 键短按/长按都会立即停车并返回菜单。
- 任务运行时 OLED 显示原有速度、航向和灰度状态；停车后恢复菜单。
- 蓝牙诊断命令 `r`、`d`、`?` 保持原功能。

## 暂定按键映射

| 按键 | 暂定引脚 | 功能 |
|---|---|---|
| UP | PB8 | 上一项 |
| DOWN | PB9 | 下一项 |
| OK | PB10 | 执行当前项 |
| BACK | PB11 | 停车并返回菜单 |

按键按下有效电平暂按低电平处理，GPIO 使用内部上拉。第一次烧录前必须用万用表通断档或 GPIO
测试程序确认四个轻触开关确实连接到上述引脚，并确认按下时对地导通。

如果实测顺序不同，只修改：

`project/code/car_menu.h`

中的以下四个宏：

```c
#define CAR_MENU_KEY_UP_PIN       (B8)
#define CAR_MENU_KEY_DOWN_PIN     (B9)
#define CAR_MENU_KEY_OK_PIN       (B10)
#define CAR_MENU_KEY_BACK_PIN     (B11)
```

不要使用逐飞 `zf_device_key` 的默认 `PA30/PA31/PB0/PB1` 配置；其中 PA31 已由当前 OLED
软件 I2C 占用。

## 赛题公布后增加任务

1. 在 `car_menu.h` 的 `car_task_t` 中确定任务编号。
2. 在 `car_menu.c` 的 `car_menu_items[]` 中修改对应英文短名称。
3. 在 `main.c` 的统一任务分发位置增加该任务的启动状态机。
4. 如需蓝牙快捷键，在 `car_bluetooth_query_command()` 中返回相同的 `car_task_t`。

菜单文字建议不超过 19 个 ASCII 字符。当前字库没有中文字形，因此菜单先使用英文。

## 编译验证

2026-07-28 使用 Keil µVision 5.43.1、ArmClang 6.24 完整编译通过：

```text
Program Size: Code=36008 RO-data=4188 RW-data=220 ZI-data=5652
0 Error(s), 0 Warning(s)
```

