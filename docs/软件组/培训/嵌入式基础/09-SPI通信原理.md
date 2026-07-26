# SPI 通信原理与实践

SPI（Serial Peripheral Interface，串行外设接口）是一种高速、全双工的同步串行通信协议。GSRL 中 [IMU](./12-IMU姿态传感器.md)（惯性测量单元）的加速度计和陀螺仪都通过 SPI 与 MCU 通信。本章从 SPI 为高速通信付出了什么代价讲起，说明它的四根信号线、工作模式和传输流程，最后落到 BMI088 的实际读写。

> 上一篇：[UART 通信原理与实践](./08-UART通信原理.md)

---

## 1. 高速同步通信

SPI 与 UART 是一组鲜明的对照。UART 异步、全双工、只要两根数据线，省线很省，但省线的代价是双方必须精确匹配波特率，任何一端的时钟偏差都会导致采样错误，速度上限因此通常卡在几 Mbps。SPI 的思路正相反：它用一根独立的时钟线（SCLK）由主设备统一发出节拍，从设备不必再猜测采样时刻，这一根线换来了 10~50 倍的速率（几十 Mbps）；此外它还用一根 CS（片选）线直接选中要通信的从设备，省去了在数据流里嵌入地址的开销。代价是连线更多——SPI 至少需要四根线。

```
UART (异步)               SPI (同步)
                          
MCU_A      MCU_B          MCU (Master)      Sensor (Slave)
 TX ─────→ RX              SCLK ────────────→ SCLK
 RX ←───── TX              MOSI ────────────→ MOSI
                           MISO ←──────────── MISO
                           CS   ────────────→ CS   (低有效)
```

SPI 采用严格的主从（Master-Slave）架构。主设备产生时钟、发起和结束通信；从设备响应主设备的时钟来收发数据。一个主设备可以挂多个从设备，具体和哪个通信由 CS 信号决定。

---

## 2. 四根信号线

SPI 的四根线各司其职：SCLK 是主设备产生的时钟，MOSI 走主发从收的数据，MISO 走从发主收的数据，CS 则低电平有效地选中某个从设备。

| 信号 | 方向 | 全称 | 说明 |
|------|------|------|------|
| **SCLK** | Master → Slave | Serial Clock | 时钟信号，由 Master 产生 |
| **MOSI** | Master → Slave | Master Out, Slave In | 主设备发送的数据 |
| **MISO** | Slave → Master | Master In, Slave Out | 从设备发送的数据 |
| **CS** (或 SS) | Master → Slave | Chip Select / Slave Select | 低电平有效，选中从设备 |

在一主多从的拓扑里，SCLK、MOSI、MISO 三根线由所有从设备共享，每个从设备则各接一根独立的 CS。

```
典型连接拓扑（一主多从）:

         ┌──────────┐
         │  Master  │
         │  (MCU)   │
         └──┬──┬──┬─┘
            │  │  │
   SCLK ────┼──┼──┼──── 所有 Slave 共享
   MOSI ────┼──┼──┼──── 所有 Slave 共享
   MISO ────┼──┼──┼──── 所有 Slave 共享
            │  │  │
   CS1  ────┤  │  │    每个 Slave 独立 CS
   CS2  ───┼──┤  │
   CS3  ───┼──┼──┤

         ┌──┴──┐┌──┴──┐┌──┴──┐
         │Slave││Slave││Slave│
         │  1  ││  2  ││  3  │
         └─────┘└─────┘└─────┘
```

CS 是这套共享总线能工作的关键：只有被拉低 CS 的从设备才会响应总线上的 SCLK 和 MOSI，其余从设备的 MISO 引脚保持高阻态，不会干扰总线。用 CS 引脚而不是像 I2C 那样在数据里发地址，是简单性与引脚数之间的权衡——好处是数据流里不必编解码地址、通信帧更短，从设备也不需要地址匹配电路；坏处是每多一个从设备就要多一根 GPIO。不过嵌入式系统里 SPI 从设备通常只有两三个，这点代价可以接受。

MOSI 和 MISO 分成两条线，则是为了真正的全双工：主从可以在同一个时钟周期内同时收发。UART 虽然也全双工，但 SPI 的全双工不需要双方匹配波特率，因为时钟统一由主设备提供。读取 IMU 时这一点很实用——"发送读命令"和"接收数据"可以在同一次传输里一并完成。

---

## 3. 四种工作模式（CPOL 与 CPHA）

SPI 用两个参数描述时钟的行为：CPOL（Clock Polarity，时钟极性）决定时钟空闲时停在高电平还是低电平，CPHA（Clock Phase，时钟相位）决定在时钟的第一个还是第二个边沿采样数据。

| 参数 | 全称 | 含义 |
|------|------|------|
| **CPOL** | Clock Polarity | 时钟空闲时的电平：0=低，1=高 |
| **CPHA** | Clock Phase | 数据采样边沿：0=第一个边沿，1=第二个边沿 |

两个参数各有两种取值，组合出四种模式。最常用的是 Mode 0：

```
Mode 0 (CPOL=0, CPHA=0) —— 最常用
SCLK:     ──┐   ┌───┐   ┌───
            │   │   │   │
            └───┘   └───┘
MOSI/MISO: ──[D7][D6][D5]...  ← 在上升沿改变数据，下降沿采样
              ↑
          空闲为低

Mode 3 (CPOL=1, CPHA=1)
SCLK:     ┌───┐   ┌───┐   ┌──
          │   │   │   │   │
          ┘   └───┘   └───┘
MOSI/MISO: ──[D7][D6][D5]...  ← 在上升沿采样（第二个边沿）
```

| Mode | CPOL | CPHA | 空闲电平 | 采样边沿 |
|------|------|------|---------|---------|
| 0 | 0 | 0 | 低 | 上升沿（第1个边沿） |
| 1 | 0 | 1 | 低 | 下降沿（第2个边沿） |
| 2 | 1 | 0 | 高 | 下降沿（第1个边沿） |
| 3 | 1 | 1 | 高 | 上升沿（第2个边沿） |

之所以要保留四种模式，是因为不同厂商的芯片在时钟的哪个边沿锁存数据上并不一致，有的在上升沿、有的在下降沿。这四种组合让 SPI 能兼容任意芯片，只要一次配置正确就能对接，否则遇到极性不匹配的芯片就得外加反相器去翻转时钟。需要注意的是，主设备和从设备的 CPOL/CPHA 必须一致，否则数据会在错误的时刻被采样。

---

## 4. SPI 通信流程

一次完整的 SPI 传输是这样的：主设备先拉低 CS 选中从设备，在 SCLK 上产生时钟，随后主从通过各自的移位寄存器同时交换数据——主设备的 MOSI 数据逐位移入从设备，从设备的 MISO 数据逐位移入主设备，传输完成后主设备拉高 CS。

```
① Master 拉低 CS
② Master 在 SCLK 上产生时钟
③ Master 和 Slave 同时通过移位寄存器交换数据（全双工）
   - Master 的 MOSI 数据逐位移入 Slave
   - Slave 的 MISO 数据逐位移入 Master
④ 传输完成，Master 拉高 CS
```

```
Master 移位寄存器                 Slave 移位寄存器
┌──────────────────┐            ┌──────────────────┐
│ D7 D6 D5 ... D0  │──MOSI──→  │ D7 D6 D5 ... D0  │
│                  │←─MISO──   │                  │
└──────────────────┘            └──────────────────┘
      ↑ SCLK (由 Master 产生)
      每来一个时钟脉冲，Master 和 Slave 的寄存器
      都右移一位，同时最高位被发送出去
```

这里有一个由全双工机制带来的、初学时容易忽视的特性：SPI 的发送和接收是同步进行的，收发共用同一串时钟脉冲。因此即便只想读数据，也必须发送同样数量的字节（通常用 0x00 或 0xFF 作为空字节），才能把 MISO 上的数据"时钟出来"。

---

## 5. HAL 库的 SPI 操作

在 CubeMX 中配置 SPI，需要关注模式、片选方式、数据帧大小、时钟模式等几项。其中 CPOL/CPHA 要按从设备的数据手册来设，不能想当然。

| 参数 | 含义 | GSRL 典型值 |
|------|------|------------|
| Mode | 主/从模式 | Full-Duplex Master |
| NSS | 硬件/软件片选 | Software（用 GPIO 模拟） |
| Data Size | 数据帧大小 | 8 Bits |
| CPOL / CPHA | 时钟模式 | 根据从设备数据手册 |
| Baud Rate Prescaler | 时钟分频 | 影响 SCLK 频率 |
| First Bit | 先发 MSB 还是 LSB | MSB First |

传输 API 与 UART 类似，分阻塞、中断、DMA 三档；由于 SPI 是全双工，HAL 额外提供了同时收发的 `TransmitReceive` 系列，GSRL 推荐用其中的 DMA 版本以做到零 CPU 开销。

```c
// 阻塞模式：发送 + 接收（分别调用）
HAL_SPI_Transmit(&hspi1, txData, len, timeout);
HAL_SPI_Receive(&hspi1, rxData, len, timeout);

// 阻塞模式：全双工（同时收发，效率高）
HAL_SPI_TransmitReceive(&hspi1, txData, rxData, len, timeout);

// 中断模式：全双工
HAL_SPI_TransmitReceive_IT(&hspi1, txData, rxData, len);

// DMA 模式：全双工（GSRL 推荐，零 CPU 开销）
HAL_SPI_TransmitReceive_DMA(&hspi1, txData, rxData, len);
```

`drv_spi.c` 在这些 HAL API 之上做了统一封装，并实现了传输完成回调，把数据转交给用户注册的回调函数。

```c
// 初始化 SPI 驱动
void SPI_Init(SPI_HandleTypeDef *hspi, SPI_Call_Back rxCallbackFunction);

// DMA 全双工传输
HAL_StatusTypeDef SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi,
                                          uint8_t *pTxData, uint16_t exchangeDataLength);

// HAL 传输完成回调 → 调用用户注册的回调
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPI_Manage_Object_t *spi_obj = SPI_Get_Object(hspi);
    if (spi_obj->rxCallbackFunction != NULL) {
        spi_obj->rxCallbackFunction(spi_obj->rxBuffer, spi_obj->exchangeDataLength);
    }
}
```

---

## 6. GSRL 中的 SPI 应用：BMI088 IMU

BMI088 是一款 6 轴 IMU（3 轴加速度计加 3 轴陀螺仪），它的加速度计和陀螺仪其实是两颗独立的芯片，各有独立的 SPI 接口和 CS 引脚。因此在接线上，SCLK 和 MISO 由两者共用，加速度计和陀螺仪各接一根 CS，由 STM32 分别选中。

```
 STM32                      BMI088
┌────────┐              ┌──────────────┐
│        │──SCLK──┬─────│SCLK_ACC      │
│  SPI1  │──MOSI──┤     │              │
│        │──MISO──┼─────│MISO (共用)   │
│        │        │     │              │
│  GPIO  │──CS1───┤     │CS_ACC        │  ← 加速度计 CS
│  GPIO  │──CS2───┘     │CS_GYRO       │  ← 陀螺仪 CS
└────────┘              └──────────────┘
```

BMI088 采用典型的 SPI 寄存器读写协议，用地址字节的最高位区分读写。读寄存器时发送两个字节：第一个字节是地址、最高位置 1 表示读，第二个字节是任意值，只为把数据时钟出来，收到的第二个字节就是寄存器的值；写寄存器同样发两个字节，第一个字节地址、最高位置 0 表示写，第二个字节是要写入的值。

```
读寄存器格式（发送 2 字节，接收 1 字节有用数据）:
  Byte0: [0│R/W=1│A6 A5 A4 A3 A2 A1 A0]   ← 寄存器地址，bit7=1 表示读
  Byte1: 任意值（用于把数据时钟出来）
  收到:  Byte1 = 寄存器的值

写寄存器格式（发送 2 字节）:
  Byte0: [0│R/W=0│A6 A5 A4 A3 A2 A1 A0]   ← 寄存器地址，bit7=0 表示写
  Byte1: 要写入的值
```

GSRL 用 `BMI088` 类（`dvc_imu.hpp`）封装了这些细节。每颗芯片的 SPI 句柄和 CS 引脚记在一个 `SPIConfig` 里，类内部提供 `readSingleReg`、`readMultiReg` 等方法完成寄存器读写。

```cpp
struct SPIConfig {
    SPI_HandleTypeDef *hspi;      // SPI 句柄（如 &hspi1）
    GPIO_TypeDef *csGPIOGroup;    // CS 引脚 GPIO 端口
    uint16_t csGPIOPin;           // CS 引脚编号
};

class BMI088 : public IMU
{
    SPIConfig m_accelSPIConfig;   // 加速度计 SPI 配置
    SPIConfig m_gyroSPIConfig;    // 陀螺仪 SPI 配置

    // 内部实现
    void readSingleReg(const SPIConfig &cfg, uint8_t reg, uint8_t &rxData);
    void readMultiReg(const SPIConfig &cfg, uint8_t reg, uint8_t *pRxData, uint8_t len);
};
```

一次读寄存器的实际过程，正好把前面的协议、CS 时序和全双工传输串在一起：拉低 CS、发送带读标志的地址和一个空字节、拿回第二个字节作为结果、拉高 CS。

```c
void BMI088::readSingleReg(const SPIConfig &cfg, uint8_t reg, uint8_t &rxData)
{
    uint8_t txData[2] = {static_cast<uint8_t>(reg | 0x80), 0xFF};  // 读命令
    uint8_t rxDataBuf[2];

    HAL_GPIO_WritePin(cfg.csGPIOGroup, cfg.csGPIOPin, GPIO_PIN_RESET);  // 拉低 CS
    HAL_SPI_TransmitReceive(cfg.hspi, txData, rxDataBuf, 2, 100);       // 全双工传输
    HAL_GPIO_WritePin(cfg.csGPIOGroup, cfg.csGPIOPin, GPIO_PIN_SET);    // 拉高 CS

    rxData = rxDataBuf[1];  // 第 2 个字节是有效的寄存器值
}
```

---

## 7. SPI 与 UART 的对比

回到开头那组对照，把 SPI 和 UART 的差异并列起来看会更清楚：SPI 靠独立时钟线换来高速和全双工，适合 IMU、Flash、SD 卡这类近距离高带宽场合；UART 靠省线和异步，适合调试、遥控器、GPS 这类连接。

| 特性 | SPI | UART |
|------|-----|------|
| 时钟 | 独立时钟线（同步） | 无时钟（异步） |
| 连线数 | 4 根（SCLK/MOSI/MISO/CS） | 2 根（TX/RX） |
| 拓扑 | 一主多从 | 一对一 |
| 最大速率 | 几十 Mbps | 几 Mbps |
| 全双工 | 是 | 是 |
| 从设备选择 | CS 信号 | 地址协议（如 RS-485 地址位） |
| 应用 | IMU、Flash、SD 卡 | 调试、遥控器、GPS |

在 GSRL 中，BMI088 IMU 通过 SPI 通信，`drv_spi.c` 封装了 HAL SPI API，`dvc_imu.hpp` 里的 `BMI088` 类则在这层驱动之上实现了寄存器读写、校准和姿态解算。

> 下一篇：[I2C 通信原理与实践](./10-I2C通信原理.md)

> **作者**: [Qing](https://github.com/ZhangChuqing) | **修改日期**: 2026-07-26
