# 中断与 NVIC

中断要解决的问题很单纯：MCU 只有一个 CPU，却要应付许多随时可能发生的外部事件，比如 CAN 收到一帧数据、串口收完一帧、传感器数据就绪。如果让 CPU 挨个去问"你来了没有"，大部分时间都浪费在无谓的询问上。中断换了个思路——事件发生时由硬件主动通知 CPU，CPU 平时专心做自己的事，被通知了才去处理。本章从这个机制讲起，再深入到 NVIC（Nested Vectored Interrupt Controller，嵌套向量中断控制器）如何调度这些中断。

> 上一篇：[GPIO 与高低电平](./03-GPIO与高低电平.md)

---

## 1. 轮询与中断

理解中断，最好先看它的替代品——轮询。设想等一个快递：轮询是每隔五分钟去门口看一次，大部分时间浪费在"看"这个动作上，而且快递可能刚好在两次查看之间到达，延迟最长达五分钟；中断则是门铃响了再去开门，平时可以做别的事，快递到了立刻响应。

```
轮询模式:
┌──────┐    ┌──────┐    ┌──────┐    ┌──────┐
│ 做正事│ →  │查标志│ →  │ 做正事│ →  │查标志│ → ...
└──────┘    └──────┘    └──────┘    └──────┘
   ↑ 浪费时间查标志，事件响应有延迟

中断模式:
┌────────────────────────────────────────┐
│            做正事 (主程序)              │
│    ← 被中断打断 ┐                       │
│                │                       │
│          ┌─────┴─────┐                 │
│          │ 中断服务   │ → 处理完毕返回   │
│          │ 函数 ISR  │                 │
│          └───────────┘                 │
└────────────────────────────────────────┘
   ↑ 无需轮询，事件发生时自动响应
```

中断高效的根源在于 CPU 不再空转。轮询的本质是反复检查一个几乎不会变化的标志位，在机器人系统里，CAN 消息可能每秒到达 100 次，而轮询线程却可能每秒检查 1000 次，九成的检查都是徒劳。轮询还有一个绕不开的问题：检查间隔决定了最坏响应延迟，每 1ms 检查一次，事件就可能刚好在检查之后发生，白白多等近 1ms。中断把这两种浪费都消除了——CPU 只在事件真正发生时被唤醒，其余时间全部用于有意义的计算。

从 CPU 的角度看，中断是硬件层面的"插队"机制。外设（比如 [CAN](./11-CAN通信原理.md) 控制器）收到数据后产生一个中断请求信号，CPU 执行完当前这条指令，检查中断优先级决定是否响应；一旦决定响应，就保存当前上下文，跳转到中断向量表里对应的 ISR（Interrupt Service Routine，中断服务函数）执行，执行完再恢复上下文，回到原来被打断的地方继续。

---

## 2. NVIC——中断的总调度

ARM Cortex-M 处理器的中断分两类，一类是 CPU 内部产生的系统异常，一类是片上外设产生的外设中断。

| 类型 | 来源 | 举例 |
|------|------|------|
| **系统异常** | CPU 内部 | Reset、NMI、HardFault、SysTick |
| **外设中断** | 片上外设 | CAN_RX、[USART](./08-UART通信原理.md)、TIM、EXTI |

NVIC 是 Cortex-M 内核的一部分，负责管理所有外设中断——接收各路中断请求，做优先级仲裁，处理嵌套，最终决定什么时候打断 CPU。

```
┌─────────────────────────────────────────────────┐
│                  Cortex-M4 内核                  │
│  ┌───────────────────────────────────────────┐  │
│  │              NVIC                         │  │
│  │  ┌────┐ ┌────┐ ┌────┐      ┌────┐       │  │
│  │  │IRQ0│ │IRQ1│ │IRQ2│ ...  │IRQn│       │  │
│  │  └──┬─┘ └──┬─┘ └──┬─┘      └──┬─┘       │  │
│  │     │      │      │           │          │  │
│  │     └──────┴──────┴───────────┘          │  │
│  │                  │                        │  │
│  │          优先级仲裁 + 嵌套管理              │  │
│  └──────────────────┬────────────────────────┘  │
│                     │                            │
│                     ▼                            │
│                  CPU 核心                         │
└─────────────────────────────────────────────────┘
        ▲          ▲          ▲
        │          │          │
   ┌────┴────┐┌───┴───┐┌────┴────┐
   │ CAN RX  ││ UART  ││ TIM IRQ│   ← 外设中断信号
   └─────────┘└───────┘└─────────┘
```

### 2.1 优先级与嵌套

NVIC 支持中断嵌套，也就是高优先级的中断可以打断正在执行的低优先级中断。

```
时间轴 ──────────────────────────────────────────→

低优先级 ISR:  ┌──────────────┐         ┌────────┐
               │  正在执行...  │         │ 继续...│
               └──────┬───────┘         └────────┘
                      │
高优先级 ISR:         ┌──────┐
                      │ 抢断 │
                      └──────┘
```

嵌套的价值在实时系统里很关键。假设 MCU 正在处理优先级为 7 的 UART 接收中断，此时电机驱动板检测到过流故障，触发了优先级为 3 的 CAN 中断。如果不支持嵌套，CAN 中断必须等 UART 的 ISR 全部执行完才能响应，这可能要几百微秒——而对过流故障来说，几百微秒的延迟足以烧毁 MOS 管。嵌套保证了高优先级事件总能立即抢占 CPU，无论当前正在处理什么。

STM32（Cortex-M4）用 4 bit 表示优先级，这 4 bit 可以在抢占优先级和子优先级之间分配。抢占优先级决定谁能打断谁，子优先级只在抢占优先级相同时决定响应先后。STM32F4 默认把 4 bit 全部分给抢占优先级（取值 0~15），子优先级占 0 bit。无论哪种，都遵循同一条规则：数值越小，优先级越高。

在 CubeMX 的 NVIC 配置页面可以调整每个中断的优先级，生成的代码反映在 `stm32f4xx_hal_msp.c` 中：

```c
HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);   // 抢占优先级 5
HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);     // 抢占优先级 6
HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);           // 使能中断
```

---

## 3. 中断向量表

中断向量表是位于 Flash 起始位置的一张跳转表，每个表项存着一个函数的入口地址。当某个中断发生时，CPU 硬件会自动从表中取出对应地址并跳转过去。它相当于中断号到 ISR 地址的映射，是硬件找到处理函数的唯一途径。

```
内存地址              内容
─────────────────────────────────
0x0000_0000         初始栈指针 (SP)
0x0000_0004         Reset_Handler      ← 上电/复位入口
0x0000_0008         NMI_Handler        ← 不可屏蔽中断
0x0000_000C         HardFault_Handler  ← 硬件错误
0x0000_0010         MemManage_Handler
0x0000_0014         BusFault_Handler
0x0000_0018         UsageFault_Handler
...                 ...
0x0000_0058         SysTick_Handler    ← 系统滴答[定时器](./06-定时器与PWM.md)
...                 ...
0x0000_00C4         CAN1_RX0_IRQHandler  ← CAN1 接收中断
...                 ...
```

这张表在启动文件 `startup_stm32f407xx.s`（汇编）中定义：

```asm
g_pfnVectors:
    .word  _estack              /* 栈顶 */
    .word  Reset_Handler        /* 复位 */
    .word  NMI_Handler
    .word  HardFault_Handler
    /* ... */
    .word  CAN1_RX0_IRQHandler  /* CAN1 接收中断 */
    /* ... */

/* 默认弱定义：如果用户没有提供 ISR，使用死循环 */
.weak  CAN1_RX0_IRQHandler
.thumb_set CAN1_RX0_IRQHandler, Default_Handler
```

这里的 `.weak` 是弱定义：如果在别处实现了同名函数，就用那个版本，否则退回到默认版本（一个死循环）。这样即便某个中断没有被用户处理，程序也不会因为跳到无效地址而崩溃。

---

## 4. 中断的完整执行路径

把前面的概念串起来，以一帧 CAN 数据到达为例，追踪中断从触发到返回的全过程：

```
第1步：硬件触发
    CAN 控制器收到一帧数据
    → CAN1 的 RX FIFO 0 非空
    → 产生 CAN1_RX0 中断请求

第2步：NVIC 裁决
    NVIC 比较 CAN1_RX0 的优先级（5）和当前运行的中断优先级
    如果优先级更高 → 触发 CPU 中断

第3步：CPU 响应
    ① 硬件自动压栈：保存 R0-R3, R12, LR, PC, xPSR
    ② 从中断向量表取出 CAN1_RX0_IRQHandler 的地址
    ③ 跳转到 CAN1_RX0_IRQHandler

第4步：ISR 执行（stm32f4xx_it.c）
    void CAN1_RX0_IRQHandler(void)
    {
        HAL_CAN_IRQHandler(&hcan1);  // HAL 库的中断分发函数
    }

第5步：HAL 中断分发（stm32f4xx_hal_can.c）
    void HAL_CAN_IRQHandler(CAN_HandleTypeDef *hcan)
    {
        // 检查中断标志位，确认是哪个事件
        if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_RF0M)) {
            // 清除中断标志
            __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_RF0M);
            // 读取数据
            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data);
            // 调用用户回调
            HAL_CAN_RxFifo0MsgPendingCallback(hcan);
        }
    }

第6步：用户[回调](./05-回调机制与HAL中断处理.md)（drv_can.c / 用户代码）
    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
    {
        // 用户自定义的数据处理逻辑
        // 放入 FreeRTOS 队列 / 直接处理
    }

第7步：返回
    ISR 执行完毕 → 硬件自动出栈恢复上下文 → 继续执行被中断的代码
```

### 4.1 ISR 要短

中断服务函数里不能做耗时操作，这是嵌入式编程里一条很硬的规矩。原因在于 ISR 执行期间，所有同优先级和更低优先级的中断都被阻塞，FreeRTOS 的任务调度器也被冻结，整个系统的任务线程全部停摆。算一笔账就清楚了：如果 CAN 接收 ISR 耗时 100μs，而 CAN 消息以 1kHz 频率到达，仅这一个中断每秒就占用 100μs × 1000 = 100ms，即 10% 的 CPU 时间；万一某个 ISR 写成了 1ms 又以 1kHz 触发，CPU 就被彻底占满，连 FreeRTOS 空闲任务和看门狗喂狗都跑不起来。

所以 ISR 只应该做必要的事——读数据、清标志、发信号，把解析协议、PID 计算这类重活通过 [FreeRTOS](./01-嵌入式系统概述与HAL库.md) 队列或信号量交给任务线程。

```c
// ❌ 错误：在 ISR 中做耗时计算
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 解析协议、PID 计算、发送应答... 全部在 ISR 中！
}

// ✅ 正确：ISR 只入队，任务线程做重活
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    can_rx_message_t msg;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &msg.header, msg.data);
    xQueueSendToBackFromISR(canRxQueueHandle, &msg, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);  // 必要时触发任务切换
}
```

---

## 5. 常见中断类型及用途

下面这些中断在 GSRL 中都有实际用途：

| 中断源 | 在 GSRL 中的用途 | 触发条件 |
|--------|-----------------|----------|
| **CAN_RX** | 接收电机反馈、裁判系统数据 | CAN 总线收到匹配 ID 的帧 |
| **USART_IDLE** | 接收遥控器、裁判系统串口数据 | 串口空闲（一帧数据接收完毕） |
| **EXTI** (外部中断) | IMU 数据就绪通知 | BMI088 的 INT1 引脚上升/下降沿 |
| **TIM** (定时器中断) | 系统时间基准（SysTick 替代）、PWM 控制 | 定时器计数值达到预设值 |
| **DMA** | [SPI](./09-SPI通信原理.md)/UART 传输完成通知 | DMA 传输完成（DMA 回调处理见[回调机制](./05-回调机制与HAL中断处理.md)） |

它们的 ISR 实现都能在 `stm32f4xx_it.c` 中找到，形式都很一致——ISR 只是把控制权转交给对应的 HAL 分发函数：

```c
void CAN1_RX0_IRQHandler(void)  { HAL_CAN_IRQHandler(&hcan1);  }
void USART1_IRQHandler(void)    { HAL_UART_IRQHandler(&huart1); }
void DMA2_Stream2_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_spi1_rx); }
void EXTI4_IRQHandler(void)     { HAL_GPIO_EXTI_IRQHandler(INT1_ACCEL_Pin); }
```

---

## 6. 中断优先级配置建议

在机器人控制系统里，优先级的分配是在权衡各路事件的实时性要求。电机反馈和 IMU 姿态数据时效性最强，配得高一些；遥控器和 DMA 完成通知能承受轻微延迟，配中等即可。

| 中断 | 建议优先级 | 原因 |
|------|-----------|------|
| CAN 接收 | 较高（3~5） | 电机反馈数据不能丢失 |
| IMU 外部中断 | 较高（3~5） | 姿态数据有严格的时效性 |
| USART 空闲中断 | 中等（5~7） | 遥控器数据可以承受轻微延迟 |
| DMA 完成中断 | 中等（5~7） | 传输完成通知，及时处理即可 |
| 定时器 | 根据用途 | 系统时钟（SysTick）通常用最低优先级 |

有两条限制需要特别留意。

第一条来自 FreeRTOS：中断优先级不能高于 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`（通常设为 5），否则 ISR 中不能调用 FreeRTOS 的 API（如 `xQueueSendFromISR`）。FreeRTOS 通过临界区保护内核数据结构（任务就绪列表、队列等），临界区的实现方式是暂时屏蔽优先级低于某个阈值的中断。如果某个 ISR 的优先级高于这个阈值，它就能在临界区内"插队"执行，此时若调用 FreeRTOS API 去修改内核数据，就会造成竞态，轻则数据错乱，重则系统崩溃。因此凡是需要调用 FreeRTOS API 的 ISR，其优先级数值必须 ≥ 5，确保被临界区正确屏蔽。

第二条是 SysTick 必须用最低优先级。SysTick 是 FreeRTOS 的心跳，每次中断都会触发调度器检查是否有更高优先级任务就绪。如果 SysTick 优先级高于某个硬件中断（比如 CAN 接收），它就可能在 CAN 的 ISR 执行期间抢进来尝试切换任务，而在 ISR 上下文中做任务切换是危险的。把 SysTick 设为最低优先级，它就永远不会打断任何硬件中断，只在所有 ISR 处理完毕后才执行任务调度。

---

## 7. 小结

| 要点 | 说明 |
|------|------|
| 中断 | 硬件事件触发 CPU 暂停当前任务，执行 ISR 的机制 |
| NVIC | Cortex-M 的中断控制器，管理优先级和嵌套 |
| 中断向量表 | 中断号 → ISR 函数地址的映射表 |
| ISR 执行路径 | 硬件触发 → NVIC 裁决 → 查向量表 → ISR → HAL 分发 → 回调 |
| ISR 原则 | 快速执行，复杂逻辑通过队列/信号量交给任务 |
| 优先级 | 数值越小优先级越高，FreeRTOS API 有最高优先级限制 |

> 下一篇：[回调机制与 HAL 中断处理](./05-回调机制与HAL中断处理.md)

> **作者**: [Qing](https://github.com/ZhangChuqing) | **修改日期**: 2026-07-26
