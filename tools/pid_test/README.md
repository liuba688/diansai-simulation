# 速度闭环 PID 串口测试工具

本工具当前对应天猛星核心板板载 Type-C/CH340：MSPM0G3507 UART0
（PA10 TX、PA11 RX），使用 115200 baud，不需要外接 USB-TTL。HC-05 蓝牙调试已放弃。
控制环和串口遥测均为 50 Hz。
工具直接启动一次有上限的速度阶跃测试、采集遥测、保存原始日志
和 CSV，并生成 JSON 指标。

## 安全与协议

- PC 发出 `@PIDTEST,START,<rpm>,<duration_ms>`；结束、异常或 Ctrl+C 时重复发送
  `@PIDTEST,STOP`。
- 启动前必须先收到主控对 `@PIDTEST,PING` 的 `READY` 应答；没有握手时不会发送
  START，避免单向串口故障造成误启动。
- 开发板自身仍有独立的 120 秒最大限时、PID 测试 1 秒启动延时、本地菜单按键接管和原有
  `tb6612_stop_all()` 停止路径。PC 断开不影响板端超时停机。
- 遥测帧固定为
  `@PID,3,t_ms,target_l,target_r,actual_l,actual_r,out_l,out_r,error_l,error_r,enabled,line_mask,line_error,line_state,straight_ms,curve_ms,sharp_ms,lost_ms`。
  `line_state` 中 `0=直线`、`1=普通弯`、`2=急弯（左右合并）`、
  `3=丢线（搜索和停车合并）`。四个时间字段是本次任务内各状态的累计时间；
  每次从 OLED 菜单启动新任务时清零，停车时另发 `@LINESTAT,1` 耗时与进入次数摘要。
  工具同时兼容旧版 `@PID,1/2` 帧。

## 使用

先将包含本次主控改动的固件编译并烧录。查看串口：

### 下载上一圈巡线日志

巡线时固件不再发送 50 Hz 实时遥测，而是在 RAM 中以 10 Hz 记录最多 60 秒。
停车时会自动发送一次；如果当时电脑没有监听，日志仍保留到下一次巡线任务开始，
可重新连接板载 Type-C 对应的 COM 口后执行（将 `COM17` 替换为设备管理器中的实际端口）：

```powershell
.\tools\pid_test\.venv\Scripts\python.exe .\tools\pid_test\download_run_log.py --port COM17
```

工具发送 `@PIDTEST,DUMP`，把上一圈数据保存到 `output/run_logs/*.csv`。
新的巡线任务开始时旧日志才会清除。紧急停车路径会先停止电机，再进行串口导出。
v4 CSV除偏航角速度外，还包含 `odometer_cm`、终点解锁、严格/降级宽线判定、
软刹状态、计时冻结和终点确认计数，用于定位终点触发与停车距离。

```powershell
.\tools\pid_test\.venv\Scripts\python.exe .\tools\pid_test\pid_test.py --list
```

开始 20 秒、25 rpm 测试（只有一个串口时可省略 `--port`）：

```powershell
.\tools\pid_test\.venv\Scripts\python.exe .\tools\pid_test\pid_test.py --port COM5 --baud 115200 --duration 20 --target 25
```

产物写入 `tools/pid_test/logs/`：`.log` 是所有可解码串口文本，`.csv` 是结构化
有效帧，`.json` 是机器可读结果。终端也会输出单行 `RESULT_JSON=...`。

分析包括左右轮稳态误差、超调量、5%（至少 1 rpm）带宽调节时间、末段速度标准差、
左右轮平均速度差、输出接近 ±8000 的饱和比例、名义/有效采样率和 0–100 综合分。
综合分只适合在相同架空/负载条件下比较多次测试，不代表绝对控制品质。

常见错误会给出明确结果：无串口、多串口未指定、串口占用/断开、长时间无数据、
字段错误、有效样本不足与缺少依赖。串口选择使用 `--non-interactive` 时不会等待输入。

编码器诊断不会驱动电机。运行后分别手转两只轮子，程序会显示 MOTOR1/MOTOR2 的
A/B 原始边沿计数：

```powershell
.\tools\pid_test\.venv\Scripts\python.exe .\tools\pid_test\pid_test.py --port COM9 --duration 10 --encoder-test
```

## 串口调参与有界自动搜索

运行时参数只写入 RAM，复位后恢复 `speed_pid.h` 默认值。单轮指定参数：

```powershell
.\tools\pid_test\.venv\Scripts\python.exe .\tools\pid_test\pid_test.py --port COM9 --duration 15 --target 25 --kp 12 --ki 0.8 --kd 0
```

自动搜索最多执行 6 个受限候选组合；每轮均握手、限时运行并停止，最后把最高分参数
应用到 RAM，但不会写入 Flash：

```powershell
.\tools\pid_test\.venv\Scripts\python.exe .\tools\pid_test\pid_test.py --port COM9 --duration 12 --target 25 --auto-tune --max-runs 6
```

自动调参时必须架空车轮、保持人员在电源开关旁。串口异常、用户中断、板端超时或
编码器在高输出下持续无反馈都会触发停止。最终参数应再用 10/25/40 rpm 和落地负载验证。
