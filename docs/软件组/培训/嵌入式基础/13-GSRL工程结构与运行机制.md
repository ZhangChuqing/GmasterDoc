# GSRL 工程结构与运行机制

前面十二章各自讲清了一个嵌入式知识点。本章把它们串起来，从宏观到微观逐层剥开 GSRL 工程的整体架构，追踪一个 [CAN](./11-CAN通信原理.md) 中断信号如何从硬件一路传到电机的角度闭环控制，最终变回 CAN 总线上的一帧控制指令。

---

## 1. 第一层：工程目录结构

先从文件系统看 GSRL 工程的全貌。

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

这套结构的核心分界，是"谁生成、谁修改"。`CubeMX_BSP/` 由 CubeMX 自动生成，管硬件初始化、ISR 入口和 [FreeRTOS](./01-嵌入式系统概述与HAL库.md) 配置，一般不手改；`GSRL/` 下的 Driver、Device、Algorithm 三层是手写的通用库，分别负责 HAL 外设的高级封装、外设对象的 C++ 封装、以及纯数学算法；`Task/` 是用户为具体机器人编写的控制逻辑，每个工程各不相同。

| 目录 | 职责 | 谁修改 | 谁生成 |
|------|------|--------|--------|
| `CubeMX_BSP/` | 硬件初始化、ISR、[FreeRTOS](./01-嵌入式系统概述与HAL库.md) 配置 | 自动生成 | CubeMX |
| `GSRL/Driver/` | HAL 外设的高级封装（回调、队列） | 手动编写 | — |
| `GSRL/Device/` | 外设对象的 C++ 封装（Motor、IMU...） | 手动编写 | — |
| `GSRL/Algorithm/` | 数学算法（PID、滤波、姿态解算） | 手动编写 | — |
| `Task/` | 具体的机器人控制任务 | 用户手动编写 | — |

---

## 2. 第二层：构建体系

### 2.1 CMake 构建层次

工程用 CMake 管理编译，分成三层：顶层 `CMakeLists.txt` 先把 CubeMX 生成的 HAL 和 CMSIS 编成 `stm32cubemx` 库，再把 GSRL 的三层加上两个第三方依赖编成 `gsrl` 库，最后把用户的 Task 源码和这两个库一起链接成最终的 `.elf`。

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

这三者的依赖是严格单向的：Task 用 GSRL，GSRL 用 HAL，反向不成立。GSRL 不知道 Task 的存在，HAL 也不知道 GSRL 的存在。正是这个单向依赖，让 GSRL 能作为一个独立的库在不同工程间复用。

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

---

## 3. 第三层：代码分层架构

### 3.1 五层洋葱模型

GSRL 的代码从底层硬件到顶层业务分成五层，像洋葱一样一层包一层。最内是 CubeMX 生成的 HAL，直接操作寄存器；往外依次是封装通信接口的 Driver、纯数学的 Algorithm、把外设建模成 C++ 对象的 Device，最外是用户编写控制逻辑的 Task。

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

每一层只做自己该做的事，把更底层的细节挡在外面。

| 层 | 职责 | 不负责 | 示例 |
|----|------|--------|------|
| HAL | 操作寄存器、启动外设、ISR 入口 | 业务逻辑、数据解析 | `HAL_CAN_AddTxMessage()` |
| Driver | 封装 HAL 中断回调、管理 FreeRTOS 队列、注册用户回调 | 解析具体协议 | `CAN_Init(&hcan1, callback)` |
| Algorithm | 纯数学计算（PID、滤波、矩阵运算） | 硬件操作 | `myPID.controllerCalculate(set, fb)` |
| Device | 协议解析、状态管理、闭环控制接口 | 底层通信细节 | `motor.angleClosedloopControl()` |
| Task | 业务逻辑编排、控制循环、状态机 | 底层协议 | 1ms 循环中调用电机控制接口 |

### 3.3 关键依赖规则

分层之所以有意义，靠的是几条被严格遵守的调用规则：上层可以调用下层，下层绝不反过来调用上层；Algorithm 层是纯 C++，不含任何 HAL 头文件（`alg_pid.hpp` 只依赖 `gsrl_common.h`）；Device 层通过 Driver 接口访问硬件，不直接调用 HAL API；Task 层只调用 Device 和 Driver 的初始化函数，不碰 HAL 句柄。

这里有两个划分特别值得说明。一是为什么要把 Driver 和 Device 分成两层。Driver 抽象的是通信接口（CAN/UART/SPI），Device 建模的是物理设备（电机、IMU、遥控器），二者分开后，换硬件只需要动一层：IMU 从 SPI 换成 CAN，只改 Driver 而 Device 层的姿态解算不动；电机从 GM6020 换成 DM4310，只改 Device 派生类而 Driver 层的 CAN 代码原样不变。这是典型的关注点分离，每层只关心自己的抽象。

二是为什么 Algorithm 要单独成层。PID、滤波器、AHRS 姿态解算全是纯数学，不依赖任何外设，可以直接在 PC 上用 gtest 做单元测试，不需要嵌入式硬件。硬件无关还带来复用：同一份 PID 代码既能用于电机速度环，也能用于 IMU 温控加热电阻的 PWM，只是传入的参数不同。把它与 HAL 头文件彻底隔离，就从根上杜绝了算法和硬件的耦合。

---

## 4. 第四层：单个中断的完整旅程

抽象的分层讲完，接下来用一个具体事件把它们贯穿起来：**CAN1 收到一帧电机反馈数据（GM6020 反馈帧，ID=0x205）**，看这份数据如何从硬件一路走到控制决策。

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

这条链路清晰地对应着分层：ISR 入口（T4）在 HAL 层且固定不变，HAL 分发（T5）识别事件后调用 `__weak` 回调，这个回调被 GSRL Driver 覆盖（T6）完成入队和转发，再交给用户注册的回调（T7），最终由 Device 层的 Motor 对象把字节解析成物理量并更新自身状态（T8）。每一步都只跟相邻层打交道。

### 4.2 数据流全景图

值得注意的是，同一帧数据在 T6 分了两条路：一条走 ISR 里的用户回调，实时更新电机状态；另一条进 FreeRTOS 队列，留给任务线程后续处理。前者响应快，后者适合放耗时较长的处理。

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

### 5.1 系统启动流程

从上电复位到任务跑起来，主线是这样：汇编的 `Reset_Handler` 加载栈指针、配置 FPU 和向量表偏移，跳到 `main()`；`main()` 依次初始化 HAL、时钟、各个外设，然后初始化 FreeRTOS 内核、创建用户任务，最后 `osKernelStart()` 启动调度器。调度器一接管，`main()` 就不再往下执行，CPU 从此交给各个任务和中断。

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

之所以用 FreeRTOS 而不是裸机的超级循环，是因为不同任务对时机的要求差别很大。裸机下，100ms 一次的遥测和 1ms 一次的控制计算得挤在同一个循环里，彼此拖累。FreeRTOS 给每个任务独立的执行时机和周期——控制任务以 1ms 高优先级运行，遥测任务以 100ms 低优先级运行，互不阻塞。抢占式优先级还保证了硬实时：只要控制任务就绪，CPU 会立刻打断正在跑的低优先级任务。

### 5.2 任务循环剖析

一个典型的控制任务分两段：初始化只执行一次，主循环按固定周期反复执行。以 `tsk_test.cpp` 为例，初始化里注册 CAN 和 UART 的回调，主循环里做控制计算、发出 CAN 控制帧，然后用 `vTaskDelayUntil` 精确等到下一个 1ms 时刻。

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

这里的 1ms 周期不是随意定的。大多数机械系统的带宽在 100Hz 以内，按奈奎斯特采样定理控制频率至少要 200Hz 才稳，1kHz 留出了 5 倍裕量，对底盘平衡、云台稳定这类快速动态过程足够。同时 1ms 也足够 STM32F407 跑完一次姿态解算加 PID 加 CAN 通信。需要说明的是，电机驱动器内部的电流环运行在 10~20kHz，我们的 1kHz 位置/速度环只是它的外环，这种分级控制是机器人领域的标准做法。

任务循环里还藏着一条实时性铁律：优先用静态分配。在 1ms 的控制周期内，一次 `malloc` 或 `new` 可能耗时 50μs，相当于整个周期的 5%，会成为不可接受的抖动来源。静态分配（全局变量、编译期定尺寸的数组）运行时零开销、时序可预测，还避免了内存碎片——一台连续跑几个月不重启的机器人，堆碎片可能让原本可用的 2KB 连续内存分不出来。

### 5.3 ISR 与任务的协作

任务和中断是并行运作的两条线。任务在自己的时刻做控制计算并发帧，而在它挂起等待下一个周期的那段时间里，CAN 中断仍可能触发一到多次，持续把电机最新状态写进 Motor 对象。

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

由此得出一个重要认知：任务周期是 1ms，但 CAN 数据的刷新率可能更高（如 1kHz）。任务每次被唤醒读到的，是最近一次 CAN 中断更新的电机状态，而不一定是 1ms 前的旧值——因为状态是由 ISR 直接写入 Motor 对象的。

---

## 6. 第六层：GSRL 在工程中的角色定位

### 6.1 GSRL 是什么

GSRL（GMaster Standard Robot Library）是一个跨工程复用的机器人控制中间件。它把 STM32 [HAL](./01-嵌入式系统概述与HAL库.md) 库的底层细节封装成面向对象的 C++ 接口，向上提供统一的[电机](./07-舵机与电机.md)控制接口（`Motor` 基类加多品牌派生类）、统一的传感器驱动（[IMU](./12-IMU姿态传感器.md) → `BMI088`）、统一的控制算法（`SimplePID`、`CascadePID`、滤波器），以及统一的通信驱动（CAN/[UART](./08-UART通信原理.md)/[SPI](./09-SPI通信原理.md) Driver 层）。

### 6.2 GSRL 的复用方式

步兵、英雄、哨兵、无人机等不同兵种工程都建立在同一份 GSRL 之上，公共代码不动，各工程只在自己的部分做区分。

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

把 GSRL 做成独立库而不是塞进每个大工程，收益在维护上很直接：多种机器人共用同一套电机驱动、IMU 读取和通信协议，一次电机协议的 bug 修复就自动惠及所有兵种，不必手动同步到八个项目里。每个工程只写自己独特的 Task 层逻辑（比如步兵的底盘运动学、云台跟踪算法），其余通用代码全来自 GSRL，改动的影响范围清晰可控。

### 6.3 GSRL 与 CubeMX_BSP 的边界

GSRL 和自动生成的 CubeMX_BSP 之间靠 HAL 回调衔接。CubeMX_BSP 里的 `main.c`、`stm32f4xx_it.c` 等文件不改，其中 ISR 只调用 `HAL_XXX_IRQHandler`，不掺业务逻辑；HAL 里的 `__weak` [回调](./05-回调机制与HAL中断处理.md)函数在 GSRL Driver 层被覆盖，业务代码经由 `/* USER CODE BEGIN */` 宏和 GSRL 的 C++ API 间接接触硬件。

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

---

## 7. 第七层：从零搭建一个控制任务

基于 GSRL 开发一个新兵种，大致按下面五步走：先在 CubeMX 里配好引脚、[时钟树](./02-时钟树与总线架构.md)和 FreeRTOS 并生成代码，再在 `board_config.h` 里启用要用的外设宏，然后在 `Task/` 下声明电机和设备对象、编写控制循环和回调，最后编译烧录并调试。

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

## 8. 完整知识地图

把全章串起来，一份控制指令的完整旅程是这样：电机的反馈帧经 CAN 总线到达 CAN 控制器，触发中断进入 ISR，HAL 分发后交给 Driver 层入队并回调，Device 层解析出物理量更新电机状态；任务在 1ms 周期里读取最新状态、做 PID 计算，再把结果经 Driver 层发回 CAN 总线，驱动电机。

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

贯穿全章的几个概念，在 GSRL 里都有明确落点：分层架构（HAL → Driver → Algorithm/Device → Task）、中断的分派链（固定 ISR → HAL 分派 → Driver 弱定义覆盖 → 用户回调）、中断上半部与下半部的划分（ISR 直接回调加 FreeRTOS 队列延迟）、面向对象封装（Motor 基类定义接口、派生类适配不同 CAN 协议）、控制器模式（Controller 抽象接口下的 SimplePID 与 CascadePID）、以及作为独立库的代码复用和 FreeRTOS 带来的实时性保证。

| 概念 | GSRL 中的体现 |
|------|-------------|
| 分层架构 | HAL → Driver → Algorithm/Device → Task |
| 中断处理 | ISR(固定) → HAL分派 → Driver回调(弱定义覆盖) → 用户回调 |
| 上半部/下半部 | ISR 中直接回调 + FreeRTOS 队列延迟 |
| 面向对象封装 | Motor 基类定义接口，派生类适配不同 CAN 协议 |
| 控制器模式 | Controller 抽象接口 → SimplePID / CascadePID 实现（位于 Algorithm 算法层） |
| 代码复用 | GSRL 作为独立库，多工程共享 |
| 实时性保证 | FreeRTOS 任务精准周期 + ISR 快速响应 |

从硬件引脚上的一次电平跳变，到电机的精确角度控制，中间的每一层都有清晰的职责边界和调用关系，这就是 GSRL 工程的完整图景。

> **作者**: [Qing](https://github.com/ZhangChuqing) | **修改日期**: 2026-07-26
