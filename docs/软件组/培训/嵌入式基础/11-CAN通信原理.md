# CAN 通信原理与实践

CAN（Controller Area Network，控制器局域网）是 RoboMaster 机器人里最核心的通信协议，所有[电机](./07-舵机与电机.md)、电调、超级电容都挂在 CAN 总线上和主控通信。它要解决的问题很具体：在电机全速运转、电磁干扰极强的环境里，用尽量少的线把一堆设备可靠地连起来。理解它的工作原理，是排查电机通信故障的前提。

> 上一篇：[I2C 通信原理与实践](./10-I2C通信原理.md)

---

## 1. 为什么需要 CAN

先看传统协议在机器人场景下会遇到什么麻烦。UART 是点对点的，连 N 个电机就要 N 个 UART 外设；SPI 虽然能一主多从，但每多一个从设备就要多一根片选线，而且是单端信号，抗干扰差。两者还有个共同的致命短板：没有硬件重传，数据一旦被干扰破坏就是永久丢失，只能靠软件层自己校验、自己重发。

```
UART 方案:
  MCU ──TX──→ 电机1
   │  ←─RX──
   │
   └──TX──→ 电机2
      ←─RX──
  需要 N 个 UART 外设，只支持点对点

SPI 方案:
  MCU ──SCLK/MOSI/MISO──┬── 电机1 (CS1)
                         ├── 电机2 (CS2)
                         ├── 电机3 (CS3)
                         └── 电机4 (CS4)
  需要 N 根 CS 线，高速但抗干扰差，且只能一主多从
```

CAN 换了一套思路。它用一对差分线（CAN_H 和 CAN_L）就把所有设备并在同一条总线上，任何节点都能主动发送（多主架构），多个节点抢总线时靠 ID 仲裁——ID 小的帧优先。最关键的是它把错误处理下沉到了硬件：CRC 校验、位填充检查、帧格式检查都由 CAN 控制器自动完成，一旦某帧因电机产生的电磁干扰（EMI）而损坏，控制器会自动重发，全程不需要软件介入。8 个大功率电机同时运转产生的 EMI 足以轻易破坏单端信号，正是差分传输加硬件重传这一组合，让 CAN 在这种环境下依然可靠——这也是机器人和车辆领域普遍选用 CAN 的根本原因。

### 1.1 物理拓扑与差分信号

```
┌─────────┐      ┌─────────┐      ┌─────────┐      ┌─────────┐
│  MCU    │      │ 电机 1  │      │ 电机 2  │      │ 电机 3  │
│ (主控)  │      │(GM6020) │      │(M3508)  │      │(DM4310) │
└────┬────┘      └────┬────┘      └────┬────┘      └────┬────┘
     │                │                │                │
     ├────────────────┼────────────────┼────────────────┤── CAN_H
     │                │                │                │
     ├────────────────┼────────────────┼────────────────┤── CAN_L
     │                │                │                │
    [120Ω]                               [120Ω]       ← 终端电阻
     │                                                │
    GND                                              GND
```

物理层上有两个要素决定通信是否可靠：差分信号和终端电阻。

差分信号是 CAN 抗干扰的来路。电机运转时的电磁噪声会同时耦合到 CAN_H 和 CAN_L 两根线上，这叫共模干扰。接收端并不看单根线的绝对电平，只看 CAN_H 减去 CAN_L 的差值——两根线上的共模噪声幅度一致，做减法时恰好抵消。因此即便每根线上叠加了数伏特的噪声，只要两根线受到的干扰相同，差模信号就纹丝不动。单端信号没有这层保护，线上的噪声就是实打实的噪声。

终端电阻则和信号反射有关。从传输线理论看，电信号到达导线末端会反射回来，就像水波撞墙反弹。CAN 跑 1Mbps 时，1 位宽约 1μs，而一条 2 米长的总线上反射返回约需 10ns，恰好落在同一位的时间窗内，叠加到原始信号上造成畸变。总线两端各并联的 120Ω 电阻相当于"吸收器"，把到达末端的能量吸收掉而非反射回去。两端都要装，是因为总线的两个端点都会反射，只装一端另一端照样出问题。少了终端电阻，信号完整性会彻底崩溃——所以排查 CAN 故障时，第一件事往往就是用万用表量 CAN_H 与 CAN_L 之间的电阻，两个 120Ω 并联应当读到 60Ω。

```
差分信号示意:

CAN_H: ───────────┐         ┌───────────
                  │         │
                  └─────────┘
CAN_L: ───────────┐         ┌───────────
                 ┌┘         └┐
                 └───────────┘
差值:  ────────────┐       ┌─────────────
                   │       │
                   └───────┘
          显性(0)   隐性(1)  显性(0)
          
显性电平 (Dominant): CAN_H - CAN_L ≈ 2V  → 逻辑 0
隐性电平 (Recessive): CAN_H - CAN_L ≈ 0V  → 逻辑 1
重要: 显性覆盖隐性 → ID 小的帧能"赢"仲裁
```

---

## 2. CAN 数据帧格式

标准 CAN 2.0A 用 11-bit ID，GSRL 主要用这种格式。一帧数据由若干字段依次拼成：

```
CAN 标准数据帧 (CAN 2.0A):

┌─────┬─────┬──────┬────┬────┬─────┬──────┬────┬────┬──────┬─────┬──────┬───┐
│ SOF │ ID  │ RTR  │IDE │ r0 │ DLC │ Data │ CRC│ACK │ EOF  │ IFS │ 间隔 │...│
│  1  │ 11  │  1   │ 1  │ 1  │  4  │0~64  │ 15 │ 2  │  7   │  3  │      │   │
│ bit │ bits│ bit  │bit │bit │ bits│ bits │bits│bits│ bits │ bits│      │   │
└─────┴─────┴──────┴────┴────┴─────┴──────┴────┴────┴──────┴─────┴──────┴───┘
```

各字段的含义如下：

| 字段 | 含义 | 说明 |
|------|------|------|
| **SOF** | Start of Frame | 1 bit 显性位，表示帧开始 |
| **ID** | Identifier | 11-bit（标准帧）或 29-bit（扩展帧），决定优先级和数据含义 |
| **RTR** | Remote Transmission Request | 0=数据帧，1=远程帧（请求对方发送数据） |
| **IDE** | Identifier Extension | 0=标准帧(11-bit ID)，1=扩展帧(29-bit ID) |
| **DLC** | Data Length Code | 数据长度，0~8 字节 |
| **Data** | Data Field | 实际数据，0~8 字节 |
| **CRC** | Cyclic Redundancy Check | 15-bit CRC 校验 |
| **ACK** | Acknowledgment | 接收方应答，至少一个节点收到就确认 |
| **EOF** | End of Frame | 7 bit 隐性位 |

### 2.1 ID 的双重职能

CAN 的 ID 有两层身份：它既是这一帧的优先级，也是数据内容的标识符。接收方靠 ID 判断这帧是否该收下（过滤），而多个节点同时发送时，ID 还决定谁能占用总线（仲裁）。

仲裁是 CAN 最巧妙的设计，专为实时系统而生，且完全在硬件层零延迟完成。节点在发送 ID 的同时监听总线电平，由于显性位（逻辑 0）会覆盖隐性位（逻辑 1），ID 数值越小的帧就自动赢得仲裁，输的一方一检测到自己发出的隐性位被拉成显性，立刻停发。这意味着紧急消息（比如电机急停，ID=0x001）可以在任何时刻抢占总线，低优先级消息（比如温度上报，ID=0x500）自动退让，全程不需要任何中央调度器。I2C 这类基于地址的协议做不到这一点——两个设备同时想说话时，得靠一套复杂的软件仲裁去协调。

```
仲裁示例（两个节点同时发送）:

节点A ID: 0 0 1 1 0 0 1 0 1 0 0  (0x194)
节点B ID: 0 0 1 1 0 0 1 0 1 0 1  (0x195)
总线:     0 0 1 1 0 0 1 0 1 0 0 (节点A获胜！)

前10位完全相同，第11位：
- 节点A发送 0 (显性) → 覆盖节点B的 1 (隐性)
- 节点B检测到自己发的1被拉成了0 → 知道自己输了 → 停止发送
```

---

## 3. RoboMaster 中的 CAN 通信规范

### 3.1 大疆电机 CAN 协议

大疆电机使用标准 CAN 帧（11-bit ID），[波特率](./02-时钟树与总线架构.md) 1Mbps。以 GM6020 为例，控制方向（MCU → 电机，ID = 0x1FF + 控制组）的一帧可以同时携带最多 4 个电机的电压指令，这就是所谓的"一控多"：

| 数据字节 | 含义 |
|---------|------|
| Data[0:1] | 电机1 控制电压 (-30000 ~ +30000) |
| Data[2:3] | 电机2 控制电压 |
| Data[4:5] | 电机3 控制电压 |
| Data[6:7] | 电机4 控制电压 |

反馈方向（电机 → MCU，ID = 0x204 + 电机序号）则回传角度、转速、电流和温度：

| 数据字段 | 含义 | 范围 |
|---------|------|------|
| Data[0:1] | 机械角度（编码器值） | 0 ~ 8191 (13-bit) |
| Data[2:3] | 转速 (RPM) | -10000 ~ +10000 |
| Data[4:5] | 实际转矩电流 | -16384 ~ +16384 |
| Data[6] | 温度 | 0 ~ 127 ℃ |

### 3.2 达妙电机 MIT 协议

达妙 DM4310 走的是 MIT 力矩控制模式，帧格式和大疆不同。控制帧（ID = masterID << 8 | motorID）把期望位置、速度、前馈力矩和 PD 参数打包进 8 个字节：

```
Data[0:1] = 期望位置 (float → uint16, PMAX 归一化)
Data[2:3] = 期望速度 (float → uint16, VMAX 归一化)
Data[4:5] = 前馈力矩 (float → uint16, TMAX 归一化)
Data[6:7] = KP + KD
```

反馈帧（ID = 0x300 + motorID）回传当前状态，还包含 MOSFET 和线圈两处温度以及错误标志：

```
Data[0:1] = 当前位置
Data[2:3] = 当前速度
Data[4:5] = 当前力矩
Data[6]   = 温度 (MOSFET)
Data[7]   = 温度 (线圈) + 错误状态
```

---

## 4. HAL 库的 CAN 操作

在 CubeMX 里配置 CAN，最需要留意的是波特率相关的时序参数。RoboMaster 统一用 1Mbps，对应的分频和位时间分段取值如下：

| 参数 | 含义 | RoboMaster 典型值 |
|------|------|-------------------|
| Bit Rate | 波特率 | 1 Mbps |
| Prescaler | 时钟分频 | 3 (APB1=42MHz → 42/(3×(1+11+2))=1Mbps) |
| Time Quanta | Bit 时间分段 | BS1=11, BS2=2, SJW=1 |
| Mode | 工作模式 | Normal |

收发的代码模式很固定：发送时填好帧头（ID、帧类型、数据长度），连同数据交给 `HAL_CAN_AddTxMessage`；接收则在 HAL 的回调里用 `HAL_CAN_GetRxMessage` 取出帧头和数据，再按 ID 判断来自哪个电机、解析对应内容。

```c
// 发送 CAN 消息
CAN_TxHeaderTypeDef txHeader;
txHeader.StdId = 0x1FF;       // 标准帧 ID
txHeader.ExtId = 0;           // 不用扩展帧
txHeader.IDE = CAN_ID_STD;    // 标准帧
txHeader.RTR = CAN_RTR_DATA;  // 数据帧
txHeader.DLC = 8;             // 数据长度 8 字节

uint8_t txData[8] = {0};
uint32_t txMailbox;
HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);

// 接收 CAN 消息（在 HAL 回调中）
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);
    // rxHeader.StdId → 判断是哪个电机的反馈
    // rxData → 解析电机状态
}
```

---

## 5. GSRL 的 CAN 驱动设计

### 5.1 全通过滤器加软件过滤

CAN 控制器自带硬件滤波器，但数量有限，而 RoboMaster 比赛里电机、裁判系统、超级电容各有不同的 ID，硬件滤波器不够灵活。GSRL 的做法是把滤波器配成"全通"——接收所有 ID 的帧，真正的过滤交给软件层去做，想怎么分流都行。

```c
// drv_can.c: 全通过滤器初始化
static void can_all_pass_filter_init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef can_filter_st = {0};
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    // Mask 全为 0 → 不关心 ID → 接收所有帧
    can_filter_st.FilterFIFOAssignment = CAN_FilterFIFO0;
    can_filter_st.FilterActivation = CAN_FILTER_ENABLE;
    HAL_CAN_ConfigFilter(hcan, &can_filter_st);
}
```

### 5.2 双路径数据分发

收到帧后，驱动同时走两条路：一条把消息塞进 FreeRTOS 队列，交给任务线程稍后从容处理；另一条直接在中断里调用用户注册的回调，用于对实时性要求高的场合。两条路径并存，兼顾了灵活性和响应速度。

```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    can_rx_message_t s_rx_msg;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &s_rx_msg.header, s_rx_msg.data);

    // 路径 1: FreeRTOS 队列（任务线程延迟处理）
    xQueueSendToBackFromISR(canRxQueueHandle, &s_rx_msg, &woken);

    // 路径 2: 用户注册的回调（ISR 中直接处理）
    CAN_Manage_Object_t *can_obj = CAN_Get_Object(hcan);
    if (can_obj->rxCallbackFunction != NULL) {
        can_obj->rxCallbackFunction(&s_rx_msg);
    }

    portYIELD_FROM_ISR(woken);
}
```

### 5.3 离线检测

电机是否掉线，GSRL 靠反馈序列号来判断：每收到一帧反馈序列号就更新一次，如果连续多帧都没有新数据进来，就认定这个电机离线了。

```cpp
// 每次收到反馈帧，序列号递增
// 如果连续 N 次未收到新数据，判定电机离线
bool decodeCanRxMessageFromISR(const can_rx_message_t *rxMessage) {
    m_motorLastFeedbackSequence = m_motorFeedbackSequence;
    m_motorFeedbackSequence = rxMessage->header.StdId & 0x07;  // 或其他序列号字段
    // 判断: 连续多帧未更新 → 离线
}
```

---

## 6. 与其他协议的对比

把 CAN 和 [UART](./08-UART通信原理.md)、[SPI](./09-SPI通信原理.md)、[I2C](./10-I2C通信原理.md) 摆在一起，CAN 在多主、硬件错误检测、自动重发和抗干扰上的优势就很清楚，代价则是速率不算高、单帧最多 8 字节：

| 特性 | CAN | [UART](./08-UART通信原理.md) | [SPI](./09-SPI通信原理.md) | [I2C](./10-I2C通信原理.md) |
|------|-----|------|-----|-----|
| 信号类型 | 差分 | 单端 | 单端 | 单端 |
| 最大节点数 | 理论无限（实际受驱动能力限制） | 点对点 | 一主多从 | 127 |
| 多主 | 支持 | 不支持 | 不支持 | 支持 |
| 硬件错误检测 | CRC、位填充、帧检查 | 可选校验位 | 无 | 无 |
| 自动重发 | 有（硬件） | 无 | 无 | 无 |
| 典型速率 | 1Mbps | 115200bps | 10Mbps+ | 400kbps |
| 数据帧长 | 0~8 字节 | 无固定帧 | 无固定帧 | 无固定帧 |
| 抗干扰 | 强 | 弱 | 弱 | 弱 |

---

## 7. 常见问题与调试

CAN 出问题时，绝大多数线索都落在物理层，尤其是终端电阻和接线上。下面几种是最常见的：

| 问题 | 原因 | 排查方法 |
|------|------|---------|
| 电机无反馈 | 终端电阻缺失 | 用万用表测 CAN_H-CAN_L 电阻，应为 60Ω（两个 120Ω 并联） |
| 数据乱码 | 波特率不匹配 | 确认所有节点波特率一致（RoboMaster 为 1Mbps） |
| 间歇通信失败 | 信号反射 | 检查终端电阻，缩短分支线长度 |
| 电机 ID 冲突 | 两个电机同 ID | 每个电机 ID 必须唯一，通过电调软件修改 |
| CAN [中断](./04-中断与NVIC.md)风暴 | 滤波器未配置 | 检查滤波器设置，限制接收范围 |
| HAL_CAN 错误回调频繁触发 | 总线连接不良 | 检查接线，确保 CAN_H/CAN_L 未接反 |

---

## 8. 小结

CAN 靠差分信号抗住电机的强干扰，靠显性覆盖隐性的电气特性实现非破坏性仲裁——ID 越小越优先，一帧最多 8 字节，大疆的"一控多"模式一帧就能指挥 4 个电机。总线两端各一个 120Ω 终端电阻消除反射，这几点构成了它可靠工作的基础。

CAN 是 GSRL 中最重要的通信协议。`drv_can.c` 封装了 HAL CAN 的初始化、过滤、收发和中断处理，用的正是前面讲的全通过滤、双路径分发和离线检测这套设计；所有 `Motor` 派生类（GM6020、M3508、DM4310 等）都通过 CAN 收发反馈和控制指令。工程里 CAN1 通常挂底盘电机，CAN2 挂云台电机，两个外设独立工作，避免单条总线负载过高。

> 下一篇：[IMU 姿态传感器](./12-IMU姿态传感器.md)

> **作者**: [Qing](https://github.com/ZhangChuqing) | **修改日期**: 2026-07-26
