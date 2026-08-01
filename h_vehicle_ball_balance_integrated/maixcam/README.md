# H题统一 MaixCAM 应用

本应用常驻运行，摄像头、模型和 UART 只初始化一次。MSPM0 通过 UART1
发送 `MODE_SELECT / START / STOP` 切换六项任务。相机仅输出钢球毫米位置、
速度、置信度、时间戳和画面；X42S 不再直连 MaixCAM。

接线：A19(TX) -> MSPM0 PA24(UART2 RX)，A18(RX) <- PA23(UART2 TX)，
双方共地，115200 8N1。不得连接载板上的 5 V/12 V 串口电源脚。

当前三点标定：`+50 mm=212 px, O=324 px, -50 mm=435 px`。改变相机位置、
焦距或摆杆后必须重新标定 `ball_estimator.py`。
