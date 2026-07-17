# GSRL 工程结构与运行机制

前面十二章讲解了嵌入式的基础知识。本章将所有知识点串联起来，从宏观到微观，像剥洋葱一样逐层揭示 GSRL 工程的完整架构：一个 [CAN](./11-CAN通信原理.md) 中断信号如何从硬件一路传递到电机的角度闭环控制，最终变成 CAN 总线上的控制指令。

---

## 1. 第一层：工程目录结构

从文件系统看 GSRL 工程的全貌：

```
GMStdRobotLib/                          ← 项目根
├── CubeMX_BSP/                         ← STM32CubeMX 生成的硬件初始化代码
│   ├── Inc/                            │   HAL 外设头文件 (main.h, can.h, spi.h...)
│   ├── Src/                            │   HAL 外设源文件 (main.c, can.c, freertos.c...)
│   │   ├── main.c                      │   ★ 程序入口：初始化 + 启动 FreeRTOS
│   │   ├── stm32f4xx_it.c              │   ★ ISR 集合：所有中断服务函数
│   │   ├── freertos.c                  │   ★ FreeRTOS 任务创建
│   │   ├── can.c / spi.c / uart.c ...  │   各外设的 HAL 初始化
│   │   └── stm32f4xx_hal_msp.c         │   NVIC 中断优先级配置
│   ├── Drivers/                        │   HAL 库 + CMSIS
│   └── CMakeLists.txt                  │   生成 stm32cubemx 静态库
│
├── GSRL/                               ← ★ 机器人软件库（核心）
│   ├── Include/                        │   GSRL 统一头文件入口
│   │   └── gsrl_common.h              │   引用 main.h + board_config + math_const...
│   ├── Driver/                         │   驱动层：封装 HAL 外设 API
│   │   ├── inc/                        │
│   │   │   ├── drv_can.h              │   CAN 驱动接口
│   │   │   ├── drv_uart.h             │   UART 驱动接口
│   │   │   ├── drv_spi.h              │   SPI 驱动接口
│   │   │   └── drv_misc.h             │   杂项驱动
│   │   └── src/                        │
│   │       ├── drv_can.c              │   CAN HAL 回调实现 + 队列管理
│   │       ├── drv_uart.c             │   UART DMA+IDLE 回调实现
│   │       └── drv_spi.c              │   SPI DMA 回调实现
│   ├── Device/                         │   设备层：外设对象封装
│   │   ├── inc/                        │
│   │   │   ├── dvc_motor.hpp          │   电机基类 + GM6020/M3508/DM4310...
│   │   │   ├── dvc_imu.hpp            │   IMU 基类 + BMI088
│   │   │   ├── dvc_remotecontrol.hpp  │   遥控器 (DR16/ET08A/VT13)
│   │   │   ├── dvc_referee.hpp        │   裁判系统
│   │   │   ├── dvc_rangefinder.hpp    │   测距模块
│   │   │   └── dvc_supercapacitor.hpp │   超级电容
│   │   └── src/                        │   对应 .cpp 实现
│   ├── Algorithm/                      │   算法层：数学与控制
│   │   ├── inc/                        │
│   │   │   ├── alg_pid.hpp            │   PID 控制器 (SimplePID + CascadePID)
│   │   │   ├── alg_ahrs.hpp           │   姿态解算算法
│   │   │   ├── alg_filter.hpp         │   滤波器 (低通/卡尔曼)
│   │   │   ├── alg_crc.hpp            │   CRC 校验
│   │   │   └── alg_general.hpp        │   通用数学工具
│   │   └── src/                        │   对应 .cpp 实现
│   ├── Dependence/                     │   第三方依赖
│   │   ├── CMSIS-DSP/                 │   ARM DSP 库
│   │   └── eigen/                     │   Eigen 矩阵运算库
│   └── CMakeLists.txt                  │   构建 GSRL 静态库
│
├── Task/                               ← 用户任务层（每个工程不同）
│   ├── inc/                            │   任务头文件
│   └── src/                            │
│       └── tsk_test.cpp               │   测试任务示例
│
├── cmake/                              │   CMake 工具脚本
├── build/                              │   编译输出
├── CMakeLists.txt                      ← ★ 顶层构建文件
├── CMakePresets.json                   │   构建预设
├── flash.cfg                           │   OpenOCD 烧录配置
└── RENAME_PROJECT.py                   │   项目重命名脚本
```

### 1.1 目录职责速查

| 目录 | 职责 | 谁修改 | 谁生成 |
|------|------|--------|--------|
| `CubeMX_BSP/` | 硬件初始化、ISR、[FreeRTOS](./01-嵌入式系统概述与HAL库.md) 配置 | **自动生成**（CubeMX） | CubeMX |
| `GSRL/Driver/` | HAL 外设的高级封装（回调、队列） | 手动编写 | — |
| `GSRL/Device/` | 外设对象的 C++ 封装（Motor、IMU...） | 手动编写 | — |
| `GSRL/Algorithm/` | 数学算法（PID、滤波、姿态解算） | 手动编写 | — |
| `Task/` | 具体的机器人控制任务 | **用户手动编写** | — |

---

## 2. 第二层：构建体系

### 2.1 CMake 构建层次

GMStdRobotLib 使用 CMake 管理编译，三层构建结构：

```
顶层 CMakeLists.txt
    │
    ├──→ add_subdirectory(CubeMX_BSP/cmake/stm32cubemx)
    │       └── 生成 stm32cubemx 库（HAL + CMSIS + BSP）
    │
    ├──→ add_subdirectory(GSRL)
    │       ├── Algorithm/  → 编译为 gsrl 库的一部分
    │       ├── Device/     → 编译为 gsrl 库的一部分
    │       ├── Driver/     → 编译为 gsrl 库的一部分
    │       ├── CMSIS-DSP/  → 独立子项目
    │       └── eigen/      → 独立子项目
    │
    └──→ target_sources(PRIVATE Task/src/tsk_test.cpp)
            ↑ 用户任务直接编译到可执行文件中

最终链接:
    GMStdRobotLib.elf = Task/ 源码 + gsrl 库 + stm32cubemx 库
```

```cmake
# 顶层 CMakeLists.txt 关键行
add_executable(${CMAKE_PROJECT_NAME})            # 可执行文件
add_subdirectory(CubeMX_BSP/cmake/stm32cubemx)   # HAL 库
add_subdirectory(GSRL)                           # GSRL 库
target_sources(... PRIVATE Task/src/tsk_test.cpp) # 用户任务
target_link_libraries(... stm32cubemx gsrl)       # 链接库
```

### 2.2 库的依赖关系

```
┌─────────────┐
│   Task/     │  ← 用户任务 (可执行文件的一部分)
│ tsk_test.cpp│
└──────┬──────┘
       │ 使用
       ▼
┌─────────────┐
│  GSRL 库    │  ← libgsrl.a (静态库)
│ Device/     │
│ Driver/     │
│ Algorithm/  │
└──────┬──────┘
       │ 使用
       ▼
┌─────────────────┐
│ stm32cubemx 库  │  ← HAL + CMSIS + BSP (CubeMX 生成)
│ CubeMX_BSP/     │
│ HAL Drivers/    │
└─────────────────┘
```

**依赖方向是单向的**：Task → GSRL → HAL。GSRL 不依赖 Task，HAL 不依赖 GSRL。这意味着 GSRL 可以在不同工程中复用。

---

## 3. 第三层：代码分层架构

### 3.1 五层洋葱模型

从内到外，逐层揭示：

```
        ┌─────────────────────────┐
        │  5. Task (用户任务)      │  ← 你的机器人控制逻辑
        │     PID + 控制循环       │
        ├─────────────────────────┤
        │  4. Device (设备层)      │  ← Motor、IMU、遥控器对象
        │     封装外设为 C++ 对象   │
        ├─────────────────────────┤
        │  3. Algorithm (算法层)   │  ← PID、滤波器、姿态解算
        │     纯数学，与硬件无关    │
        ├─────────────────────────┤
        │  2. Driver (驱动层)      │  ← CAN/UART/SPI 高级封装
        │     HAL 回调 + 队列管理  │
        ├─────────────────────────┤
        │  1. HAL (硬件抽象层)     │  ← STM32 外设寄存器操作
        │     CubeMX 生成代码      │
        └─────────────────────────┘
```

### 3.2 各层的职责边界

| 层 | 职责 | 不负责 | 示例 |
|----|------|--------|------|
| **HAL** | 操作寄存器、启动外设、ISR 入口 | 业务逻辑、数据解析 | `HAL_CAN_AddTxMessage()` |
| **Driver** | 封装 HAL 中断回调、管理 FreeRTOS 队列、注册用户回调 | 解析具体协议 | `CAN_Init(&hcan1, callback)` |
| **Algorithm** | 纯数学计算（PID、滤波、矩阵运算） | 硬件操作 | `myPID.controllerCalculate(set, fb)` |
| **Device** | 协议解析、状态管理、闭环控制接口 | 底层通信细节 | `motor.angleClosedloopControl()` |
| **Task** | 业务逻辑编排、控制循环、状态机 | 底层协议 | 1ms 循环中调用电机控制接口 |

### 3.3 关键依赖规则

1. **上层可以调用下层，下层不能调用上层**
2. **Algorithm 层是纯 C++，不包含任何 HAL 头文件**（`alg_pid.hpp` 只依赖 `gsrl_common.h`）
3. **Device 层使用 Driver 层的接口，不直接调用 HAL API**
4. **Task 层只调用 Device 和 Driver 的初始化函数，不直接操作 HAL 句柄**

> **为什么分离 Driver 层和 Device 层？** Driver 抽象的是通信接口（CAN/UART/SPI），Device 建模的是物理设备（电机/IMU/遥控器）。二者分离后，更换硬件只需要改一层：如果 IMU 从 SPI 接口换成 CAN 接口，只需替换 Driver 而不动 Device 层的姿态解算逻辑；如果从 GM6020 电机换成 DM4310 电机，只需替换 Device 派生类而 Driver 层 CAN 通信代码完全不变。这是经典的关注点分离（Separation of Concerns）——每一层只关心自己的抽象。
>
> **为什么 Algorithm 作为独立层？** PID 控制、滤波器、AHRS 姿态解算都是纯数学运算——它们不依赖任何硬件外设，理论上可以在 PC 上用 gtest 做单元测试，完全不需嵌入式硬件。这种"硬件无关性"还带来了复用性：同一份 PID 代码既用于电机速度环，又用于 IMU 温度控制的加热电阻 PWM，只需传入不同的参数。Algorithm 层不包含任何 HAL 头文件，从根本上杜绝了算法代码与硬件耦合。

---

## 第四层：单个中断的完整旅程

以 **CAN1 收到一帧电机反馈数据** 为例，追踪数据从硬件到控制决策的完整路径：

### 4.1 时间线

```
时刻 T0: 电机发送 CAN 帧（如 GM6020 的反馈帧，ID=0x205）
    │
    ▼ 硬件自动完成（ns 级）
T1: CAN 控制器收到数据帧，存入 RX FIFO 0
    │ CAN1_RX0 中断标志置位
    │
    ▼ NVIC 裁决（几个时钟周期）
T2: NVIC 根据优先级裁决，触发 CPU [中断](./04-中断与NVIC.md)
    │
    ▼ CPU 硬件响应
T3: CPU 自动压栈（R0-R3, R12, LR, PC, xPSR）
    │ 从中断向量表取出 CAN1_RX0_IRQHandler 地址
    │
    ▼ 跳转到 ISR（stm32f4xx_it.c）
T4: void CAN1_RX0_IRQHandler(void)
    {
        HAL_CAN_IRQHandler(&hcan1);  // 调用 HAL 分发函数
    }
    │
    ▼ HAL 中断分发（stm32f4xx_hal_can.c）
T5: void HAL_CAN_IRQHandler(CAN_HandleTypeDef *hcan)
    {
        // ① 读取中断状态寄存器，确认是 RX FIFO0 事件
        if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_RF0M))
        {
            // ② 清除中断标志
            __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_RF0M);
            // ③ 调用回调函数（__weak，已被 GSRL Driver 覆盖）
            HAL_CAN_RxFifo0MsgPendingCallback(hcan);
        }
    }
    │
    ▼ GSRL CAN Driver 回调（drv_can.c）
T6: void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
    {
        can_rx_message_t s_rx_msg;
        // ① 从 HAL 句柄读取原始 CAN 数据
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                            &s_rx_msg.header, s_rx_msg.data);

        // ② 放入 FreeRTOS 队列（给任务线程后续使用）
        xQueueSendToBackFromISR(canRxQueueHandle, &s_rx_msg, &woken);

        // ③ 调用用户注册的回调函数
        CAN_Manage_Object_t *obj = CAN_Get_Object(hcan);
        if (obj->rxCallbackFunction != NULL)
            obj->rxCallbackFunction(&s_rx_msg);

        // ④ 必要时触发任务切换
        portYIELD_FROM_ISR(woken);
    }
    │
    ▼ 用户注册的回调（tsk_test.cpp）
T7: void can1RxCallback(can_rx_message_t *pRxMsg)
    {
        // 调用电机对象的 ISR 解析方法
        motor.decodeCanRxMessageFromISR(pRxMsg);
    }
    │
    ▼ Motor 对象解析（dvc_motor.cpp）
T8: bool MotorDM4310::decodeCanRxMessage(const can_rx_message_t &rxMessage)
    {
        // ① 检查 CAN ID 是否匹配（确定是发给这个电机的）
        if (rxMessage.header.StdId != m_motorFeedbackMessageID)
            return false;

        // ② 解析字节数据 → 物理量
        //    Data[0:1] → 位置 (uint16 → float, PMAX归一化)
        //    Data[2:3] → 速度 (uint16 → float, VMAX归一化)
        //    Data[4:5] → 力矩 (uint16 → float, TMAX归一化)

        // ③ 更新电机内部状态
        m_currentAngle = decodedPosition;
        m_currentAngularVelocity = decodedVelocity;
        m_currentTorqueCurrent = decodedTorque;

        // ④ 更新在线状态（序列号判断）
        updateConnectionStatus();

        return true;
    }
    │
    ▼ ISR 返回（硬件自动）
T9: CPU 出栈恢复上下文 → 继续执行被中断的代码
    └── 可能是 FreeRTOS 的任务切换（如果 woken=true）
```

### 4.2 数据流全景图

```
                    ┌─────────────┐
                    │   GM6020    │
                    │   电机      │
                    └──────┬──────┘
                           │ CAN 总线上发出反馈帧 ID=0x205
                           ▼
┌──────────────────────────────────────────────────────────────┐
│ 硬件                                                          │
│  CAN 收发器 → CAN 控制器 → RX FIFO → 中断信号                  │
└────────────────────────────────┬─────────────────────────────┘
                                 │
      ┌──────────────────────────┴──────────────────────────┐
      │                                                      │
      ▼                                                      ▼
┌─────────────┐                                     ┌──────────────┐
│ ISR 路径    │                                     │ 任务线程路径  │
│ (实时响应)  │                                     │ (延迟处理)   │
│             │                                     │              │
│ can1RxCall- │                                     │ FreeRTOS     │
│ back()      │                                     │ 队列出队     │
│   ↓         │                                     │   ↓          │
│ motor.      │                                     │ motor.       │
│ decodeFrom  │                                     │ decodeFrom   │
│ ISR()       │                                     │ Queue()      │
│   ↓         │                                     │   ↓          │
│ 更新电机    │                                     │ 更新电机     │
│ 内部状态    │                                     │ 内部状态     │
└─────────────┘                                     └──────────────┘
      │                                                      │
      └──────────────────────┬───────────────────────────────┘
                             │
                             ▼
                  ┌─────────────────────┐
                  │  Motor 对象内部状态   │
                  │  - 当前角度           │
                  │  - 当前角速度         │
                  │  - 当前转矩电流       │
                  │  - 在线状态           │
                  └─────────────────────┘
```

---

## 5. 第五层：FreeRTOS 任务与整体运行

### 5.1 系统启动流程（回顾 + 细化）

```
上电/复位
    │
    ▼
Reset_Handler (汇编)
    │ ① 从 Flash 0x00 加载栈指针
    │ ② 跳转到 SystemInit()（配置 FPU、中断向量表偏移）
    │ ③ 跳转到 main()
    ▼
main() (CubeMX_BSP/Src/main.c)
    │
    ├─ HAL_Init()                           ← HAL 库初始化
    ├─ SystemClock_Config()                 ← 时钟 168MHz
    ├─ MX_GPIO_Init()                       ← GPIO 引脚初始化
    ├─ MX_DMA_Init()                        ← DMA 初始化
    ├─ MX_CAN1_Init()                       ← CAN1 控制器初始化
    ├─ MX_CAN2_Init()                       ← CAN2 控制器初始化
    ├─ MX_SPI1_Init()                       ← SPI1 控制器初始化
    ├─ MX_USART3_UART_Init()                ← UART3 初始化
    ├─ ... 其他 MX_XXX_Init()               ← 所有外设初始化
    │
    ├─ osKernelInitialize()                 ← FreeRTOS 内核初始化
    ├─ MX_FREERTOS_Init()                   ← 创建 FreeRTOS 对象
    │   └─ osThreadNew(test_task, ...)      ← ★ 创建用户任务
    │
    └─ osKernelStart()                      ← ★ 启动调度器
        │                                    从此，main() 不再执行
        │                                    调度器接管 CPU
        ▼
┌───────────────────────────────────────────────────┐
│               FreeRTOS 调度器运行中                 │
│                                                   │
│  ┌─────────────┐  ┌─────────────┐                │
│  │ test_task   │  │ 其他任务     │  ← 任务按优先级│
│  │ (1ms 周期)  │  │ (可扩展)    │     和时间片切换│
│  └─────────────┘  └─────────────┘                │
│                                                   │
│  中断可以打断任何任务:                              │
│    CAN IRQ ──→ ISR ──→ 回调 ──→ 队列              │
│    TIM IRQ ──→ ISR ──→ HAL_IncTick()              │
│    UART IDLE IRQ ──→ ISR ──→ 回调                  │
└───────────────────────────────────────────────────┘
```

> **为什么用 FreeRTOS 而非裸机（Bare Metal）？** 裸机编程需要把所有任务手动编排进一个超级循环，100ms 一次的遥测任务必须和 1ms 的控制计算挤在同一循环里，严重影响实时性。FreeRTOS 为每个任务赋予独立的执行时机和周期——控制任务以 1ms 周期高优先级运行，遥测任务以 100ms 周期低优先级运行，二者互不阻塞。优先级抢占机制保证：只要控制任务就绪，CPU 会立刻暂停当前低优先级任务，保证硬实时约束。

### 5.2 任务循环剖析

以 `tsk_test.cpp` 为例，一个典型的控制任务：

```cpp
extern "C" void test_task(void *argument)
{
    // ──── 初始化阶段（只执行一次）────
    CAN_Init(&hcan1, can1RxCallback);        // 初始化 CAN1 驱动
    UART_Init(&huart3, dr16ITCallback, 36);  // 初始化 UART 驱动

    TickType_t taskLastWakeTime = xTaskGetTickCount();  // 记录起始时间

    // ──── 主循环（1ms 周期）────
    while (1)
    {
        // ① 控制计算
        motor.openloopControl(
            sinf(2 * PI * 0.5f * xTaskGetTickCount() / 1000)
        );

        // ② 发送 CAN 控制帧给电机
        const uint8_t *data = motor.getMotorControlData();
        HAL_CAN_AddTxMessage(&hcan1,
            motor.getMotorControlHeader(), data, &send_mail_box);

        // ③ 等待到下一个 1ms 时刻（精确周期）
        vTaskDelayUntil(&taskLastWakeTime, 1);
    }
}
```

> **为什么任务周期是 1ms（1kHz）？** 大多数机械系统的带宽不超过 100Hz，按照奈奎斯特采样定理，控制频率至少需要 200Hz 才能稳定。1kHz 提供了 5 倍裕量，确保对底盘平衡、云台稳定等快速动态过程有充足的控制带宽。同时，1ms 足够 STM32F407 完成一次完整的姿态解算 + PID 计算 + CAN 通信——电机驱动器内部的电流环以 10~20kHz 运行，我们的 1kHz 位置/速度环是外环，这种分级控制架构是机器人领域的标准做法。
>
> **为什么 GSRL 优先使用静态分配？** 在 1ms 控制循环内，一次 `malloc`/`new` 可能耗时 50μs——这相当于控制周期的 5%，成为不可接受的抖动来源，破坏实时性。静态分配（全局变量、编译期确定大小的数组）在运行时零开销，时序完全可预测，这是硬实时嵌入式系统的铁律。此外，静态分配避免了内存碎片问题——一个运行数月不重启的机器人，堆碎片可能导致原本可用的 2KB 连续内存无法分配。

### 5.3 ISR 与任务的协作

CAN 数据的流向在一个控制周期内：

```
1ms 控制周期内发生的事件:

时刻 0ms (任务开始):
  task: 读取电机当前状态 → PID 计算 → 发送 CAN 控制帧

时刻 0~1ms (任务挂起，等待下一个周期):
  可能发生 1~N 次 CAN 中断:
    → ISR 接收反馈 → 更新 Motor 内部状态
    → 遥控器 UART 中断也可能在这期间触发

时刻 1ms (任务唤醒):
  task: 读取最新状态 → PID 计算 → 发送 CAN 控制帧
  ... 循环往复
```

```
时间线 (1ms 周期):

T=0ms         T=0.3ms       T=0.7ms       T=1ms
  │              │             │             │
  ├── 任务执行 ──┤             │             ├── 任务执行 ──→
  │  PID计算     │             │             │
  │  CAN发送     │             │             │
  └──────────────┘             │             │
                 ├─ CAN中断 ──┤├─ CAN中断 ──┤
                 │  接收反馈   ││  接收反馈   │
                 │  更新状态   ││  更新状态   │
                 └────────────┘└────────────┘
                    ↑ 任务挂起期间，中断仍在运行
```

**关键认知**：任务周期 1ms，但 CAN 数据的刷新率可能更高（如 1kHz）。任务每次被唤醒时，读到的电机状态是**最新一次** CAN 中断更新的值——不一定是 1ms 前的旧值。这是由 ISR 直接更新 Motor 内部状态实现的。

---

## 6. 第六层：GSRL 在工程中的角色定位

### 6.1 GSRL 是什么

GSRL（GMaster Standard Robot Library）是一个**跨工程复用的机器人控制中间件**。它将 STM32 [HAL](./01-嵌入式系统概述与HAL库.md) 库的底层细节封装为面向对象的 C++ 接口，向上提供：

- 统一的[电机](./07-舵机与电机.md)控制接口（`Motor` 基类 + 多品牌派生类）
- 统一的传感器驱动（[IMU](./12-IMU姿态传感器.md) → `BMI088`）
- 统一的控制算法（`SimplePID`, `CascadePID`, 滤波器）
- 统一的通信驱动（CAN/[UART](./08-UART通信原理.md)/[SPI](./09-SPI通信原理.md) Driver 层）

### 6.2 GSRL 的复用方式

不同兵种工程（步兵、英雄、哨兵、无人机...）都基于 GSRL：

```
GSRL (公共代码，不修改)
    │
    ├──→ 2025-Infantry-DL/  (步兵，使用 GSRL)
    ├──→ 2026_hero/         (英雄，使用 GSRL)
    ├──→ 2026_sentry/       (哨兵，使用 GSRL)
    ├──→ 2026_quadrotor/    (无人机，使用 GSRL)
    └──→ ...

每个工程的差异:
  - Task/ 目录下的任务代码不同（不同兵种的业务逻辑）
  - CubeMX_BSP/ 配置可能不同（不同主控板的引脚分配）
  - 可能使用不同版本的 GSRL（F4 vs H7）
```

> **为什么 GSRL 要做成独立库而非一个大工程？** 步兵、英雄、哨兵、无人机等多种机器人共用同一套电机驱动、IMU 读取和通信协议。将 GSRL 独立为静态库后，一次电机协议的 bug 修复就能自动惠及所有兵种工程，无需手动同步到 8 个项目。每个兵种工程只需编写自己独特的 Task 层控制逻辑（如步兵的底盘运动学 vs 云台跟踪算法），其余通用代码全部来自 GSRL——修改影响范围清晰可控，维护成本大幅降低。

### 6.3 GSRL 与 CubeMX_BSP 的边界

```
┌─────────────────────────────────────┐
│         CubeMX_BSP (自动生成)        │
│  - main.c        (不能改)           │
│  - stm32f4xx_it.c (ISR 入口，不能改)│
│  - can.c/spi.c   (外设初始化)        │
│  - freertos.c    (任务创建)          │
└────────────┬────────────────────────┘
             │ HAL 回调给 GSRL Driver
             ▼
┌─────────────────────────────────────┐
│         GSRL (手动维护)              │
│  - drv_can.c    (HAL CAN 回调实现)  │
│  - dvc_motor.cpp (电机协议解析)      │
│  - alg_pid.cpp  (PID 计算)          │
└─────────────────────────────────────┘
```

**边界规则**：
- `main.c` 通过 `/* USER CODE BEGIN */` 宏让用户在 Task 中写代码
- `stm32f4xx_it.c` 中的 ISR 只调用 `HAL_XXX_IRQHandler`，不做业务逻辑
HAL 的 `__weak` [回调](./05-回调机制与HAL中断处理.md)函数在 GSRL Driver 层被覆盖
- 用户的 Task 代码通过 GSRL 的 C++ API 间接使用硬件

---

## 7. 第七层：从零搭建一个控制任务的步骤

基于 GSRL 开发一个新兵种的典型步骤：

```
Step 1: CubeMX 配置
   - 配置引脚（CAN、UART、SPI、[GPIO](./03-GPIO与高低电平.md)...）
   - 配置[时钟树](./02-时钟树与总线架构.md)
   - 配置 FreeRTOS（任务数量、栈大小、优先级）
   - 生成代码 → CubeMX_BSP/

Step 2: 配置 board_config.h
   - 启用使用的外设宏：
     #define USE_CAN1
     #define USE_USART3
     #define USE_SPI1

Step 3: 编写 Task/ 任务代码
   - 声明电机对象（选择类型 + 设置 CAN ID + 挂载 PID）
   - 声明设备对象（IMU、遥控器）
   - 在 FreeRTOS 任务函数中编写控制循环
   - 实现 CAN/UART 的回调函数

Step 4: 编译 + 烧录
   - cmake --build build
   - openocd -f flash.cfg

Step 5: 调试
   - 通过 UART 打印调试信息
   - 通过 CAN 分析仪查看总线数据
```

---

## 8. 总结：完整知识地图

```
┌───────────────────────────────────────────────────────────────┐
│                      GMStdRobotLib                            │
│                                                               │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │              Task (tsk_test.cpp)                        │ │
│  │  控制循环: PID计算 → CAN发送 → 等待1ms → ...            │ │
│  └────────────┬───────────────────────┬────────────────────┘ │
│               │                       │                       │
│               ▼                       ▼                       │
│  ┌──────────────────┐   ┌──────────────────────┐            │
│  │  Device (设备层)   │   │  Algorithm (算法层)   │            │
│  │  Motor / IMU /    │   │  PID / Filter /     │            │
│  │  RemoteControl    │   │  AHRS / CRC         │            │
│  └────────┬─────────┘   └──────────────────────┘            │
│           │                                                   │
│           ▼                                                   │
│  ┌──────────────────────────────────────────────────────┐    │
│  │              Driver (驱动层)                          │    │
│  │  CAN_Init() / UART_Init() / SPI_Init()              │    │
│  │  HAL回调实现 / FreeRTOS队列管理 / 用户回调注册        │    │
│  └────────┬─────────────────────────────────────────────┘    │
│           │                                                   │
│           ▼                                                   │
│  ┌──────────────────────────────────────────────────────┐    │
│  │    CubeMX_BSP + HAL (硬件抽象层)                      │    │
│  │    main.c / stm32f4xx_it.c / can.c / spi.c ...      │    │
│  │    FreeRTOS 内核                                      │    │
│  └────────┬─────────────────────────────────────────────┘    │
│           │                                                   │
│           ▼                                                   │
│  ┌──────────────────────────────────────────────────────┐    │
│  │    硬件: STM32F407 + CAN收发器 + 电机 + 传感器        │    │
│  └──────────────────────────────────────────────────────┘    │
└───────────────────────────────────────────────────────────────┘

数据流:
  传感器/电机 → CAN总线 → CAN控制器 → 中断 → ISR → HAL回调
  → Driver层入队列 → Device层解析 → 任务中PID计算
  → Device层生成控制数据 → Driver层CAN发送 → 电机
```

### 关键要点

| 概念 | GSRL 中的体现 |
|------|-------------|
| **分层架构** | HAL → Driver → Algorithm/Device → Task |
| **中断处理** | ISR(固定) → HAL分派 → Driver回调(弱定义覆盖) → 用户回调 |
| **上半部/下半部** | ISR 中直接回调 + FreeRTOS 队列延迟 |
| **面向对象封装** | Motor 基类定义接口，派生类适配不同 CAN 协议 |
| **控制器模式** | Controller 抽象接口 → SimplePID / CascadePID 实现（位于 Algorithm 算法层） |
| **代码复用** | GSRL 作为独立库，多工程共享 |
| **实时性保证** | FreeRTOS 任务精准周期 + ISR 快速响应 |

这就是 GSRL 工程的完整图景——从硬件引脚上的一个电平跳变，到电机的精确角度控制，每一层都有清晰的职责边界和调用关系。
