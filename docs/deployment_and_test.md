# 部署与测试

1. 在 Keil 打开 `project/mdk/SeekFree_MSPM0G3507_Device_Library.uvprojx`，或用
   CCS 导入 `project/ccs/mspm0g3507_ball_car.projectspec`。
2. 编译并烧录 MSPM0G3507。
3. 将 `maixcam_ball_car` 中清单列出的文件和模型部署到 MaixCAM。
4. 先架空车轮，确认 OLED 菜单四键顺序和返回键停车。
5. 执行 `CALIBRATE YAW` 时保持小车完全静止。
6. 先测试 `LINE ONLY`，确认全白搜索约 200 ms 后停车。
7. 再测试 `ANGLE HOLD`，确认偏差阈值和电机方向。
8. 最后测试 `BALL VISION`：红框减速、绿框停车、接近、盲走、倒退、重新捕线。

视觉联调前确认 UART2 TX/RX 交叉、双方共地且 MaixCAM 没有接入 5 V 信号。
任何方向相反时只修改对应符号宏，不要同时改 PID 参数。
