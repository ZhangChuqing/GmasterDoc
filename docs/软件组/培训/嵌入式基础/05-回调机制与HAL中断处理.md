# 回调机制与 HAL 中断处理

上一章讲了硬件[中断](./04-中断与NVIC.md)是怎么触发和执行的。这一章接着回答一个工程问题：中断的 ISR 入口是固定的，业务逻辑却是千变万化的，两者怎么衔接。HAL 库给出的答案是回调机制——把"清理硬件"和"处理数据"这两件事拆开。理解这条从硬件中断到用户回调的处理链，是写出健壮嵌入式代码的关键。本章剖析这条链路，并说明 GSRL 如何在它之上构建 Driver 层。

> 上一篇：[中断与 NVIC](./04-中断与NVIC.md)

---

## 1. 回调解决了什么问题

每个外设中断都需要一个 ISR，而 ISR 的名字和数量由启动文件里的中断向量表固定死了。比如 [CAN](./11-CAN通信原理.md)1 接收中断的 ISR 必须叫 `CAN1_RX0_IRQHandler`，改不了。如果直接在这个函数里写数据处理逻辑，麻烦立刻就来了：

```c
// ❌ 问题1：直接耦合
void CAN1_RX0_IRQHandler(void)
{
    // CAN1 收电机反馈 → 解析 → 更新电机状态
    // CAN1 收裁判系统 → 解析 → 更新裁判数据
    // 两种完全不同的逻辑混在一个 ISR 里
}

// ❌ 问题2：不可复用
// 另一个工程也用 CAN1，但外设完全不同，ISR 代码得重写
```

HAL 库的做法是在固定入口和用户逻辑之间加一层分发：

```
CAN1_RX0_IRQHandler          (固定入口，CubeMX 生成)
    │
    ▼
HAL_CAN_IRQHandler           (HAL 库统一分发，判中断源)
    │
    ▼
HAL_CAN_RxFifo0MsgPendingCallback (用户实现的回调，可自由定义)
```

ISR 只做通用的中断清理，也就是清标志位、读寄存器，具体的业务逻辑交给回调函数。用户只需要实现回调，不用操心 ISR 入口和中断标志。

这种分层带来两个实实在在的好处。一是容错隔离。ISR 里的寄存器清理是硬件正常运行的前提，标志位不清就会导致中断反复触发，这部分逻辑绝不能出错；如果让用户直接在 ISR 里写业务代码，一旦访问野指针或除零触发 HardFault，整个中断处理链就断了。分层之后，HAL 的 ISR 先把硬件伺候好，再调用用户回调，即便回调崩溃，中断系统本身还能运转。二是多路复用，同一个 CAN 中断可能对应多个用户（电机模块、裁判系统模块都要订阅），中间层可以维护一个回调列表，一个中断触发就通知所有订阅者。

说到底，回调就是"事件发生时，由框架来调用你的函数"。你把一个函数指针注册进去，框架在合适的时机替你调用它。

```c
// 伪代码：回调的本质
typedef void (*CallbackFunc)(void *data);   // 回调函数类型

struct Driver {
    CallbackFunc userCallback;              // 存储用户提供的函数指针
};

// 用户注册回调
driver.userCallback = myHandler;

// 框架在事件发生时调用
void onEvent(struct Driver *drv) {
    if (drv->userCallback != NULL) {
        drv->userCallback(eventData);       // 回调用户函数
    }
}
```

---

## 2. HAL 的中断处理链路

以 CAN 接收中断为例，从硬件产生信号到用户代码拿到数据，完整链路是这样的：

```
┌─────────────────────────────────────────────────────┐
│                   硬件层                              │
│  CAN 控制器 RX FIFO 非空 → 产生中断信号               │
└────────────────────────┬────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────┐
│          中断向量表 → CAN1_RX0_IRQHandler            │
│          (startup_stm32f407xx.s / stm32f4xx_it.c)    │
│                                                     │
│  void CAN1_RX0_IRQHandler(void)                     │
│  {                                                  │
│      HAL_CAN_IRQHandler(&hcan1);  ← 传入句柄         │
│  }                                                  │
└────────────────────────┬────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────┐
│             HAL_CAN_IRQHandler (HAL 库)              │
│                                                     │
│  ① 检查中断状态寄存器，确认是哪个事件                  │
│  ② 读取数据：HAL_CAN_GetRxMessage()                  │
│  ③ 清除中断标志                                      │
│  ④ 调用回调函数                                      │
│                                                     │
│  if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_RF0M))       │
│  {                                                  │
│      __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_RF0M);     │
│      HAL_CAN_GetRxMessage(hcan, ..., &header, data);│
│  #if USE_HAL_CAN_REGISTER_CALLBACKS                  │
│      hcan->RxFifo0MsgPendingCallback(hcan);         │
│  #else                                               │
│      HAL_CAN_RxFifo0MsgPendingCallback(hcan);       │
│  #endif                                              │
│  }                                                  │
└────────────────────────┬────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────┐
│     HAL_CAN_RxFifo0MsgPendingCallback (GSRL 实现)    │
│     (文件: GSRL/Driver/src/drv_can.c)               │
│                                                     │
│  void HAL_CAN_RxFifo0MsgPendingCallback(...)         │
│  {                                                  │
│      ① 从 HAL 句柄读取数据                           │
│      ② 入 FreeRTOS 队列                              │
│      ③ 调用用户注册的回调函数（如果有）                │
│  }                                                  │
└────────────────────────┬────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────┐
│        用户回调 (如 tsk_test.cpp 中的 can1RxCallback)│
│                                                     │
│  void can1RxCallback(can_rx_message_t *pRxMsg)      │
│  {                                                  │
│      motor.decodeCanRxMessageFromISR(pRxMsg);       │
│      // 解析电机反馈数据                              │
│  }                                                  │
└─────────────────────────────────────────────────────┘
```

HAL 为每个外设都定义了一套回调函数，覆盖常见的事件：

| 外设 | 常见回调 | 触发时机 |
|------|---------|----------|
| **CAN** | `HAL_CAN_RxFifo0MsgPendingCallback` | CAN 收到数据 |
| **[UART](./08-UART通信原理.md)** | `HAL_UARTEx_RxEventCallback` | UART 空闲中断（一帧收完） |
| **UART** | `HAL_UART_ErrorCallback` | UART 通信错误 |
| **[SPI](./09-SPI通信原理.md)** | `HAL_SPI_TxRxCpltCallback` | SPI 收发完成 |
| **SPI** | `HAL_SPI_ErrorCallback` | SPI 通信错误 |
| **TIM** | `HAL_TIM_PeriodElapsedCallback` | 定时器溢出 |
| **GPIO** | `HAL_GPIO_EXTI_Callback` | 外部中断触发 |

这些回调都带 `__weak` 标记，是弱定义。HAL 库给了一个什么都不做的默认空实现，保证不重写时也能正常编译链接；用户想处理某个事件，只要在自己的文件里定义一个同名函数，链接器就会用这个强定义覆盖掉默认的弱定义。

```c
// HAL 库中的弱定义（什么都不做）
__weak void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    /* NOTE: This function should not be modified, and can be implemented
             in the user file */
}

// 用户在 drv_can.c 中重写（强定义覆盖弱定义）
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 实际的数据处理逻辑
}
```

弱定义这套机制之所以重要，是因为它实现了生成代码与用户代码的彻底分离。HAL 库源码和 CubeMX 生成的代码都不需要用户去改，回调默认指向空实现。用户的回调写在独立的文件里，于是 HAL 库升级时只换库文件，用户代码毫发无伤；CubeMX 重新生成代码时，也不会覆盖用户的回调。固件工程师由此能在不动第三方代码的前提下定制行为，这是 STM32 开发生态里很典型的一种设计。

---

## 3. GSRL Driver 层的回调管理

同一条 CAN 总线上可能挂着电机、裁判系统、超级电容等多种设备，而 HAL 层的回调只有一个入口，怎么区分数据该送去哪里？GSRL 的办法是在 Driver 层实现 HAL 回调，内部维护用户注册的回调函数指针。

以 CAN 驱动为例（`drv_can.c`），管理对象里存着 HAL 句柄和用户回调，初始化时把用户回调保存下来，HAL 回调触发时先入队列再调用用户回调：

```c
// CAN 管理对象
typedef struct {
    CAN_HandleTypeDef *hcan;                        // HAL 句柄
    CAN_Call_Back rxCallbackFunction;               // 用户注册的回调
} CAN_Manage_Object_t;

// 初始化时注册回调
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back rxCallbackFunction)
{
    CAN_Manage_Object_t *can_obj = CAN_Get_Object(hcan);
    can_obj->hcan = hcan;
    can_obj->rxCallbackFunction = rxCallbackFunction;  // 保存用户回调
    // ...
}

// HAL 回调中：先入队列，再调用用户回调
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    can_rx_message_t s_rx_msg;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &s_rx_msg.header, s_rx_msg.data);

    // 1. 放入 FreeRTOS 队列（给任务线程使用）
    xQueueSendToBackFromISR(canRxQueueHandle, &s_rx_msg, &woken);

    // 2. 调用用户注册的回调（给 ISR 直接处理）
    CAN_Manage_Object_t *can_obj = CAN_Get_Object(hcan);
    if (can_obj->rxCallbackFunction != NULL) {
        can_obj->rxCallbackFunction(&s_rx_msg);
    }
}
```

UART 驱动（`drv_uart.c`）用的是同一套模式，只是接收方式换成了 DMA + 空闲中断：

```c
typedef struct {
    UART_HandleTypeDef *huart;
    UART_Call_Back rxCallbackFunction;   // 用户回调
    uint8_t rxBuffer[UART_BUFFER_SIZE];  // DMA 接收缓冲区
    uint16_t rxDataLimit;               // 最大接收长度
} UART_Manage_Object_t;

// 初始化：启动 DMA 接收，注册回调
void UART_Init(UART_HandleTypeDef *huart, UART_Call_Back rxCallbackFunction,
               uint16_t rxDataLimit)
{
    UART_Manage_Object_t *uart_obj = UART_Get_Object(huart);
    uart_obj->rxCallbackFunction = rxCallbackFunction;
    HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_obj->rxBuffer, rxDataLimit);
}

// HAL 空闲中断回调
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    UART_Manage_Object_t *uart_obj = UART_Get_Object(huart);
    if (uart_obj->rxCallbackFunction != NULL) {
        uart_obj->rxCallbackFunction(uart_obj->rxBuffer, Size);
    }
    // 重启 DMA 接收，准备下一帧
    HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_obj->rxBuffer, uart_obj->rxDataLimit);
}
```

UART 接收之所以要用 DMA 配合空闲中断，是因为 UART 数据帧长度往往不固定（遥控器数据 18 字节，裁判系统数据可能 128 字节），接收方需要一个办法知道一帧什么时候结束。这里有两个问题要解决。一个是谁来搬数据：如果每收到一个字节就触发一次中断，在 115200 bps 下每秒约 11520 个字节就是 11520 次中断，每次中断至少几十个 CPU 周期，合计约 8.6% 的 CPU 时间全耗在搬字节上；DMA 让硬件自动把每个字节写进内存，CPU 完全不参与。另一个是怎么知道一帧结束：当总线空闲超过一个字节的时间，硬件自动产生空闲（IDLE）中断，此时 DMA 已经把这一帧的字节全搬完，回调直接取走完整一帧即可。DMA 加空闲中断的组合，让 CPU 从"逐字节接客"变成了"收完一帧再叫我"。

在任务里，分别初始化不同外设的驱动，注册各自的回调即可：

```cpp
// tsk_test.cpp
extern "C" void test_task(void *argument)
{
    CAN_Init(&hcan1, can1RxCallback);        // CAN1: 处理电机反馈
    UART_Init(&huart3, dr16ITCallback, 36);  // UART3: 处理遥控器数据
    // ...
}

// CAN 接收回调
extern "C" void can1RxCallback(can_rx_message_t *pRxMsg)
{
    motor.decodeCanRxMessageFromISR(pRxMsg);
}

// UART 接收回调
extern "C" void dr16ITCallback(uint8_t *Buffer, uint16_t Length)
{
    dr16.receiveRxDataFromISR(Buffer);
}
```

---

## 4. 中断的上半部与下半部

中断处理在实践中通常拆成两半。上半部在 ISR 里执行，只做读寄存器、清标志、把数据丢进缓冲区或队列这类微秒级的轻活，不允许阻塞；下半部在任务里执行，做协议解析、PID 计算、状态机更新这类毫秒级的重活，可以被更高优先级的任务打断。

```
┌──────────────────────────────────────────────┐
│          上半部 (Top Half / ISR 中)           │
│  - 读取硬件寄存器数据                          │
│  - 清除中断标志                                │
│  - 将数据放入缓冲区/队列                        │
│  - 执行时间：微秒级                            │
│  - 不允许阻塞/耗时操作                         │
├──────────────────────────────────────────────┤
│          下半部 (Bottom Half / 任务中)         │
│  - 解析协议                                    │
│  - PID 计算                                   │
│  - 状态机更新                                  │
│  - 发送响应数据                                │
│  - 执行时间：毫秒级                            │
│  - 可以被高优先级任务打断                       │
└──────────────────────────────────────────────┘
```

GSRL 的 CAN 驱动同时提供了两条路径：ISR 回调直接处理，或者经 FreeRTOS 队列延迟到任务里处理。

```
                     CAN 中断触发
                          │
            ┌─────────────┴─────────────┐
            ▼                           ▼
    ┌───────────────┐          ┌───────────────────┐
    │ 用户回调 (ISR) │          │ FreeRTOS 队列      │
    │ 直接解析数据   │          │ 延迟到任务中解析    │
    │ (轻量操作)     │          │ (重量操作)         │
    └───────────────┘          └──────────┬────────┘
            │                             │
            │                             ▼
            │                    ┌───────────────────┐
            │                    │ 任务线程中调用     │
            │                    │ decodeCanRxMessage │
            │                    │ FromQueue()       │
            └──────────┬────────┘───────────────────┘
                       │
                       ▼
              ┌─────────────────┐
              │ Motor 对象      │
              │ 更新电机状态     │
              └─────────────────┘
```

选回调还是选队列，取决于操作的耗时和复杂度。ISR 回调适合 1~2 微秒级的极轻量操作，比如把 CAN 帧里某几个字节提取出来赋给一个标志位；队列适合涉及状态机、协议解析或多级计算的复杂操作。以电机位置反馈为例：电机通过 CAN 发回 8 字节原始数据，ISR 回调里只做"把这 8 字节 raw data 塞进一个结构体"（约 2μs），然后通过队列发给 Motor 任务；Motor 任务收到后再执行完整的协议解析、零点偏移补偿、单位换算，最终更新电机对象状态（约 50μs）。要是把这 50μs 的活放进 ISR 回调，既阻塞所有低优先级中断，又会累积出可观的 CPU 占用。

对应到 Motor 类里，就是两个方法，分别服务上半部和下半部：

```cpp
// ISR 中直接调用（上半部）
bool decodeCanRxMessageFromISR(const can_rx_message_t *rxMessage);

// 任务中从队列取出后调用（下半部）
bool decodeCanRxMessageFromQueue(const can_rx_message_t *rxMessage, uint8_t Size);
```

---

## 5. 常见陷阱

| 陷阱 | 后果 | 正确做法 |
|------|------|---------|
| ISR 中调用阻塞函数 | 系统死锁或优先级反转 | ISR 只能用 `FromISR` 后缀的 [FreeRTOS](./01-嵌入式系统概述与HAL库.md) API |
| ISR 中执行耗时计算 | 低优先级中断永远得不到响应 | 把复杂逻辑移到任务中 |
| 忘记清除中断标志 | 中断反复触发，CPU 被 ISR 占满 | HAL 已处理，直接操作寄存器时要注意 |
| 回调函数未注册 | 收到数据但无人处理 | 初始化时确保注册了有效的回调函数 |
| 在回调中忘记重启 DMA 接收 | 只接收了一帧就停止 | UART 空闲回调末尾必须重调用 `ReceiveToIdle_DMA` |

---

## 6. 小结

| 要点 | 说明 |
|------|------|
| 回调机制 | ISR（固定入口）→ HAL 分发 → 用户回调（弱定义覆盖） |
| GSRL Driver 层 | 实现 HAL 回调，维护用户注册的回调函数指针 |
| 两种回调路径 | ISR 中直接回调（轻量）+ FreeRTOS 队列（重量级延迟处理） |
| 上半部/下半部 | ISR 只做数据搬运，任务线程做协议解析和业务逻辑 |
| UART 空闲中断 | 一帧数据接收完成后触发，处理后需重启 DMA 接收 |

GSRL 的 Driver 层（`drv_can.c`、`drv_uart.c`、`drv_spi.c`）正是围绕 HAL 回调机制设计的。它重写了 HAL 的 `__weak` 回调，在回调内部管理 FreeRTOS 队列和用户注册的回调链，把裸 HAL 的使用复杂度屏蔽在下面。

> 下一篇：[定时器与 PWM](./06-定时器与PWM.md)

> **作者**: [Qing](https://github.com/ZhangChuqing) | **修改日期**: 2026-07-26
