# PID 调参指南

理论部分请先参考必修视频：[从不懂到会用！PID从理论到实践~](https://www.bilibili.com/video/BV1B54y1V7hp)

本文聚焦实操，帮助你在看过理论后快速上手调参。

## 1. 参数概览

串级 PID 通常有 **11 个参数**，包含内环/速度环和外环/位置环各自的 P、I、D、output limit、I output limit，以及低通滤波系数。

```cpp
#define YAW_OUTER_KP
#define YAW_OUTER_KI
#define YAW_OUTER_KD
#define YAW_OUTER_OUT_LIMIT
#define YAW_OUTER_IOUT_LIMIT
#define YAW_INNER_KP
#define YAW_INNER_KI
#define YAW_INNER_KD
#define YAW_INNER_OUT_LIMIT
#define YAW_INNER_IOUT_LIMIT
#define YAW_INNER_LOWPASS_FILTER_PARA
```

| 参数 | 作用 |
|------|------|
| P | 响应的幅度，越大越"用力" |
| I | 消除稳态误差，一般给 0 或极小值 |
| D | 抑制震荡，一般给较小值 |
| output limit | 输出上限，防止过冲 |
| 低通滤波 | 1 表示无滤波，0.8 已是较大滤波 |

**经验规律**：

- 实际调车中很少加 I，D 通常较小，P 可以较大
- 重负载比轻负载**好调很多**，新手建议从重负载场景开始
- PID 数值可以用**二分法**逼近：假设 P=5 不动、P=6 疯狂抖动，则下一步试 P=5.5
- 不同电机的参数尺度不同：3508 的 P 上限约 16000（给几千属正常），4310 使用弧度制（P 通常在十位甚至个位数）

## 2. 粗调：手动感受法

适用于能直接接触电机转子的场景（如底盘 yaw/pitch 轴、拨弹轮）。先调内环再调外环。

### 2.1 内环

1. **限制外环**：将外环 P 给极小，output limit 也极小，避免外环干扰判断。
2. **清零内环**：只保留内环 P，I/D/output limit/I limit 全部清零。
3. **逐步加大 P**：握住转子延伸结构（如枪管、拨盘），感受电机状态，直到轻轻拨动时能感到**将要抖动但尚未剧烈抖动**。根据需求可加少量 D 增加稳定性。

### 2.2 外环

1. 放开 output limit。
2. 逐步加大 P，直到用手拨不动且抖动在可接受范围。
3. 加 D 至抖动基本消除。

## 3. 精调：Vofa 数据辅助

Vofa 使用方法详见 [Vofa 调试](./Vofa.md)。

### 3.1 可手动粗调的场景

1. 将 **output limit 调小**至波形图中常见极大值附近，防止过冲。
2. 微调 P 和 D，观察波形。

理想效果参考：

![vofa优秀调参](vofa优秀调参.png)

### 3.2 无法手动接触的场景（如摩擦轮）

建议先复用前辈的 PID 参数作为起点，再通过 Vofa 观察速度环和位置环的实际值与目标值偏差，定位是内环还是外环的问题，针对性调整。

> PID 调参是经验活，方法虽可传授，手感需亲自积累。如有疑问请多请教有调参经验的队员。

> **作者**: [Qing](https://github.com/ZhangChuqing) | **修改日期**: 2026-07-18
