# MaixCAM–MSPM0 UART2 协议

UART2 使用 115200、8N1。帧格式：

```text
AA 55 | Version | Type | Length | Sequence | Payload | CRC8
```

CRC8 多项式为 `0x07`，覆盖 Version 到 Payload。

## 目标帧 `0x10`

16 字节载荷依次为：flags、center_x、center_y、width、height、frame_width、
frame_height、confidence、candidate_count、reserved。多字节字段为小端序。

目标 flags：

- bit0：有效；
- bit1：连续确认；
- bit2：接近；
- bit3：多候选。

## 状态帧 `0x20`

8 字节载荷依次为：state、flags、fault、line_mask、left_rpm_x10、right_rpm_x10。

状态 flags 仅使用：

- bit0：运动已启用；
- bit3：故障。

状态编号：0 IDLE、1 LINE、2 CONFIRM、3 LOCK、4 APPROACH、5 CREEP、
6 RETURN_PREP、7 BACKTRACK、8 REACQUIRE、9 COMPLETE、10 FAULT。
