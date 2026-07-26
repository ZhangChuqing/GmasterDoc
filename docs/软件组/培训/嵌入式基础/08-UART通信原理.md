# UART 通信原理与实践

UART（Universal Asynchronous Receiver/Transmitter，通用异步收发器）是嵌入式系统里最简单、也最常见的串行通信方式。从调试串口到遥控器接收，处处都能见到它。本章先说清它靠什么在没有时钟线的情况下完成通信，再讲它的物理层和 STM32 上的实现，最后落到 GSRL 的接收方案。

> 上一篇：[舵机与电机控制](./07-舵机与电机.md)

---

## 1. 异步串行通信

通信协议最基础的一个分野，是收发双方要不要一根独立的[时钟](./02-时钟树与总线架构.md)线来对齐节拍。SPI 那样带时钟线的叫同步通信，主设备用时钟明确告诉从设备何时采样；UART 则不带时钟线，只有数据线，收发双方各自独立计时，靠事先约定好的传输速率（波特率）来保持一致，这就是"异步"的含义。

```
同步通信（如 SPI）:
  SCLK ──┐┌──┐┌──┐┌──┐┌──    ← 独立的时钟线，收发双方用同一时钟
  MOSI ──┘└──┘└──┘└──┘└──    ← 数据线
        上升沿采样

异步通信（如 UART）:
  TX ──┐     ┌───┐   ┌─      ← 只有数据线，没有时钟线
       │     │   │   │
       └─────┘   └───┘
       收发双方各自独立计时，必须约定相同的波特率
```

UART 要解决的事情，是把并行的字节数据拆成一位一位的串行比特流，通过一根 TX（发送）线和一根 RX（接收）线送出去，对方再拼回字节。省掉时钟线是它最大的特点，代价则是双方必须精确约定波特率。

```
发送端:                            接收端:
   ┌───┐                            ┌───┐
   │ H │ 并行数据                    │ H │
   │ e │ ────┐               ┌────  │ e │
   │ l │     │   串行比特流   │      │ l │
   │ l │     └───────────────┘      │ l │
   │ o │  TX ───────────────── RX   │ o │
   └───┘                            └───┘
```

---

## 2. 物理层

### 2.1 数据帧格式

既然没有时钟线，接收方就得从数据流本身判断一帧数据从哪开始、到哪结束，UART 的帧格式正是为此设计的。空闲时线路保持高电平；一帧以一个低电平的起始位开头，随后是 5~9 个数据位（通常 8 个），可选一个校验位，最后是 1~2 个高电平的停止位。

```
空闲    起始位    数据位 (5~9 bit)    [校验位]  停止位   空闲
────┐   ┌───┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌───┐ ┌───┐
    │   │   │ │ │ │ │ │ │ │ │ │ │ │   │ │   │
    └───┘   └─┘ └─┘ └─┘ └─┘ └─┘ └───┘ └───┘
        └ D0  D1  D2  D3  D4  D5  D6  D7 ┘
        先发 LSB（最低位），最后发 MSB（最高位）
```

| 组成部分 | 长度 | 说明 |
|---------|------|------|
| **空闲** | 任意 | 空闲时保持高电平 |
| **起始位** | 1 bit | 低电平，告知接收方"数据来了" |
| **数据位** | 5~9 bit（通常 8） | 实际数据，LSB 先发 |
| **校验位** | 0~1 bit（通常无） | 奇偶校验，可选 |
| **停止位** | 1~2 bit（通常 1） | 高电平，表示一帧结束 |

起始位和停止位都不是多余的。空闲状态是高电平，起始位的下降沿给了接收方一个明确的"数据开始"信号——检测到这个下降沿后，接收方就以它为起点，按约定的波特率推算出后续每个数据位的最佳采样时刻，本质上是用它做了一次同步。停止位则保证相邻两帧之间有一个最小间隔：如果没有它，前一帧的末位会和后一帧的起始位无缝衔接，接收方就分不清帧的边界；同时停止位的高电平还让接收方有机会检验自己的时钟是否仍然同步，若在停止位时刻采到的却是低电平，说明双方波特率偏差已经过大。至于数据位为什么先发 LSB（低位），最初是硬件使然：移位寄存器自然向右移位，最低位最先被移出，这个惯例沿袭至今，成了所有 UART 设备的默认。

### 2.2 波特率

波特率就是每秒传输的比特数，包含起始、停止和校验位在内，收发双方必须设成相同的值。常用的有 9600、19200、38400、57600、115200、921600 等。以 115200 bps 为例，可以推算出实际的吞吐能力：

```
每 bit 时间 = 1 / 115200 ≈ 8.68 μs
传输 1 字节（8N1 = 起始 1 + 数据 8 + 停止 1 = 10 bit）
耗时 = 10 × 8.68 μs ≈ 86.8 μs
最大吞吐 = 115200 / 10 = 11520 字节/秒 ≈ 11.5 KB/s
```

### 2.3 电平标准

同样的逻辑高低，在不同电平标准下对应的实际电压并不相同，接线时必须留意。

| 标准 | 逻辑 1 (高) | 逻辑 0 (低) | 用途 |
|------|------------|------------|------|
| **TTL/CMOS** (MCU 直连) | 3.3V / 5V | 0V | 板级短距离通信 |
| **RS-232** | -3V ~ -15V | +3V ~ +15V | PC 串口（DB9 接口） |
| **RS-485** | 差分信号 (A > B) | 差分信号 (A < B) | 工业长距离（千米级） |

STM32 引脚输出的是 TTL 电平，和 PC 的 RS-232 电平不兼容，直接对接会损坏芯片。要连 PC 串口需经 MAX232 之类的电平转换芯片，或直接用 USB-TTL 转换器（如 CP2102、CH340）。

---

## 3. STM32 的 USART 外设

STM32 上负责收发的外设叫 USART（Universal Synchronous/Asynchronous Receiver/Transmitter），名字里的 Synchronous 说明它也支持带时钟线的同步模式，但 GSRL 中只用异步模式，也就是 UART。它内部由发送/接收移位寄存器完成串并转换，由波特率发生器（BRR）从总线时钟分频出采样时钟。

```
┌─────────────────────────────────────────────┐
│               USART 外设内部                  │
│                                              │
│  TX 引脚 ←── 发送移位寄存器 ←── 发送数据寄存器  │
│              (并→串转换)       (TDR)          │
│                                              │
│  RX 引脚 ──→ 接收移位寄存器 ──→ 接收数据寄存器  │
│              (串→并转换)       (RDR)          │
│                                              │
│          波特率发生器 (BRR)                    │
│          ┌─────────────────┐                 │
│          │ 总线时钟 ÷ BRR  │ → 采样时钟       │
│          └─────────────────┘                 │
│                                              │
│          过采样 (16× 或 8×)                   │
│          每个 bit 采样 16 次，取中间 3 次判多数  │
└─────────────────────────────────────────────┘
```

由于接收方并不知道发送方时钟的精确相位，它采用过采样来找准每一位的采样点：在每个比特周期内采样 16 次，检测到起始位下降沿后，在其后第 8、9、10 次采样处取多数来确认电平。这样即便双方时钟略有偏差，也能采在比特中央最稳定的位置。

```
发送信号:  ──┐     ┌───────────┐     ┌──
            │     │           │     │
            └─────┘           └─────┘
           起始位             数据位

16× 采样:  0123456789ABCDEF  0123456789...
                 ↑                ↑
           检测到下降沿      在第 8,9,10 次
           之后第 8,9,10     采样中取多数
           次采样确认起始位
```

---

## 4. HAL 库的 UART 操作

HAL 提供三种收发方式，区别在于谁来搬运数据、CPU 要不要一直等着。阻塞模式最直接但会卡住 CPU，中断模式每个字节触发一次中断，DMA 则由硬件自动搬运、CPU 零开销。

| 模式 | API | 特点 |
|------|-----|------|
| **阻塞** (Blocking) | `HAL_UART_Transmit()` / `HAL_UART_Receive()` | 等传输完才返回，卡住 CPU |
| [**中断**](./04-中断与NVIC.md) (Interrupt) | `HAL_UART_Transmit_IT()` / `HAL_UART_Receive_IT()` | 每字节产生中断 |
| [**DMA**](./04-中断与NVIC.md) | `HAL_UART_Transmit_DMA()` / `HAL_UART_Receive_DMA()` | 硬件自动搬运，CPU 零开销 |

GSRL 的接收采用 DMA 配合空闲中断（IDLE）的组合，这是权衡下来的最优方案。逐字节中断的开销不容忽视：以 115200 bps 计，每字节 10 bit，每秒就有 11520 次中断，而一次中断保存/恢复寄存器加上执行回调通常要 5~8μs，光是接收 UART 就能吃掉 5%~8% 的 CPU，对跑着 1kHz 控制循环的机器人来说无法接受。DMA（Direct Memory Access，直接存储器访问）在传输期间完全不占用 CPU，硬件自动把数据从 UART 寄存器搬到内存缓冲；再配上空闲中断，一整帧数据只在结束时触发一次中断，而不是每字节一次。

GSRL 的接收流程分四步：启动一次 DMA 接收，指定缓冲区和最大长度；一帧数据发完后，RX 线上超过一个字节时间没有新数据，触发空闲中断；在回调里处理缓冲区中的有效数据；处理完重启 DMA，准备接收下一帧。

```
① 启动 DMA 接收：
   HAL_UARTEx_ReceiveToIdle_DMA(&huart, buffer, maxLen);
   配置 DMA 将收到的数据从 UART 数据寄存器搬运到 buffer
   直到收到 maxLen 个字节或 UART 空闲中断发生

② 一帧数据结束 → UART 空闲中断触发：
   当 RX 线上超过 1 个字节时间没有新数据 → IDLE 中断

③ 回调处理数据：
   HAL_UARTEx_RxEventCallback(huart, actualLen)
   → 用户回调处理 buffer 中的有效数据

④ 重启 DMA 接收：
   HAL_UARTEx_ReceiveToIdle_DMA(...)
   准备接收下一帧
```

```
时间轴：
  RX:   [字节1][字节2][字节3] ...... [字节N]  ────空闲 > 1字节时间────
                                                    │
                                              空闲中断触发
                                             Size = N, 调用回调
```

这套流程在 `drv_uart.c` 中的实现如下。`UART_Init` 保存用户回调和长度上限并启动接收；`HAL_UARTEx_RxEventCallback` 在一帧结束时被调用，转交给用户回调后重启接收；`HAL_UART_ErrorCallback` 则在出错时重启 DMA，保证接收不会因一次错误而永久停摆。

```c
void UART_Init(UART_HandleTypeDef *huart, UART_Call_Back rxCallbackFunction, 
               uint16_t rxDataLimit)
{
    UART_Manage_Object_t *uart_obj = UART_Get_Object(huart);
    uart_obj->rxCallbackFunction = rxCallbackFunction;
    uart_obj->rxDataLimit = rxDataLimit;
    // 启动 DMA + 空闲中断接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_obj->rxBuffer, rxDataLimit);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    UART_Manage_Object_t *uart_obj = UART_Get_Object(huart);
    if (uart_obj->rxCallbackFunction != NULL) {
        uart_obj->rxCallbackFunction(uart_obj->rxBuffer, Size);
    }
    // 重启接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_obj->rxBuffer, uart_obj->rxDataLimit);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // 错误恢复：重启 DMA 接收
    UART_Manage_Object_t *uart_obj = UART_Get_Object(huart);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_obj->rxBuffer, uart_obj->rxDataLimit);
}
```

---

## 5. UART 在 GSRL 中的应用

UART 在 GSRL 中主要承担三类数据。一是遥控器接收：DR16 接收机以 100Hz 通过 UART 输出 18 字节的遥控数据帧，用户任务里用 `UART_Init` 注册回调，中断中把数据交给 `dr16` 设备解析。

```cpp
// 用户任务中初始化
UART_Init(&huart3, dr16ITCallback, 36);

// ISR 中回调
void dr16ITCallback(uint8_t *Buffer, uint16_t Length) {
    dr16.receiveRxDataFromISR(Buffer);
}
```

二是裁判系统通信，RoboMaster 裁判系统通过 UART 下发比赛状态数据，由 `dvc_referee` 设备封装。三是调试串口，把 `printf` 重定向到 UART，经 USB 虚拟串口或物理引脚输出调试信息。

---

## 6. 常见问题与对策

UART 使用中的问题大多能从帧格式和电平的角度找到原因，下面把常见故障归拢在一起，方便对照排查。

| 问题 | 原因 | 对策 |
|------|------|------|
| 收到乱码 | 波特率不匹配 | 检查收发双方波特率设置 |
| 数据粘包/断帧 | 空闲中断配置不当 | 使用 IDLE 中断自动分帧 |
| DMA 接收中断停止 | 错误中断后未重启 | 在 `ErrorCallback` 中重启 |
| 收发同时进行时丢数据 | 全双工但驱动未处理并发 | 使用独立 TX/RX DMA 通道 |
| 长距离通信不稳定 | TTL 电平抗干扰差 | 改用 RS-485（差分信号） |

回到 GSRL：`drv_uart.c` 封装了 DMA 加空闲中断的接收模式，`dvc_remotecontrol`、`dvc_referee` 等设备类都建立在这个驱动之上，遥控器和裁判系统的数据都经由 UART 进入系统。

> 下一篇：[SPI 通信原理与实践](./09-SPI通信原理.md)

> **作者**: [Qing](https://github.com/ZhangChuqing) | **修改日期**: 2026-07-26
