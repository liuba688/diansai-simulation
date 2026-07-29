# 速度闭环 PID 串口测试工具

本工具对应 TI MSPM0G3507 主控工程的调试串口 `UART_1`（B6 TX、B7 RX，
115200 baud）。它直接启动一次有上限的速度阶跃测试、采集遥测、保存原始日志
和 CSV，并生成 JSON 指标。

## 安全与协议

- PC 发出 `@PIDTEST,START,<rpm>,<duration_ms>`；结束、异常或 Ctrl+C 时重复发送
  `@PIDTEST,STOP`。
- 开发板自身仍有独立的 120 秒最大限时、1 秒启动延时、本地菜单按键接管和原有
  `tb6612_stop_all()` 停止路径。PC 断开不影响板端超时停机。
- 遥测帧固定为
  `@PID,1,t_ms,target_l,target_r,actual_l,actual_r,out_l,out_r,error_l,error_r,enabled`。
  仅 `@PID,1,` 开头的 12 字段行会进入 CSV；启动日志、调试文本和复位乱码会被忽略。

## 使用

先将包含本次主控改动的固件编译并烧录。查看串口：

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
