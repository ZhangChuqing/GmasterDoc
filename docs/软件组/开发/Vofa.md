# Vofa+ 调试指南

Vofa+ 是用于 PID 精调的串口数据可视化工具，支持实时波形显示和在线参数调节。

## 1. 下载

[Vofa+ 1.3.10 安装包](https://je00.github.io/downloads/vofa+_1.3.10_x64_installer.exe)

## 2. 集成到工程

在工程 `dev` 目录中加入以下文件：

- [dvc_vofa.cpp](dvc_vofa.cpp)
- [dvc_vofa.hpp](dvc_vofa.hpp)

配合 `tsk_test.cpp` 使用，调用初始化函数 `Init` 和写入函数 `WriteData` 即可。

### 快速集成示例

模板参数 `N` 为需要同时发送的数据通道数：

```cpp
#include "dvc_vofa.hpp"

Vofa<9> vofa;  // 9 个数据通道

extern "C" void test_task(void *argument)
{
    CAN_Init(&hcan1, can1RxCallback);
    UART_Init(&huart3, dr16ITCallback, 36);
    vofa.Init();
    vofa.AddParameterListener("data", [](fp32 *newValue) {
        data = *newValue;
        printf("Parameter updated: %f\n", *newValue);
    });
    TickType_t taskLastWakeTime = xTaskGetTickCount();
    while (1) {
        motor.openloopControl(0.0f);
        transmitMotorsControlData();
        vofa.writeData(data);
        vofa.sendFrame();
        vTaskDelayUntil(&taskLastWakeTime, 1);
    }
}
```

### 在线调参

使用 `AddParameterListener` 绑定参数名与回调函数：

```cpp
vofa.AddParameterListener("gimbal_p", [](fp32 *val) {
    gimbal_pid.kp = *val;
});
```

在 Vofa+ 串口助手中发送 `参数名:数值`（英文冒号，区分大小写），多条指令用换行分隔：

```text
gimbal_p:15.5
speed:0
```

## 3. 上位机配置

1. 协议选择：**FireWater**（即 JustFloat 协议）
2. 数据引擎：开启
3. 添加控件：拖入 **Waveform**（波形图）
4. 右键波形图 → 配置 → 绑定数据源，代码中 `BindFunction` 的调用顺序对应通道索引（第 1 个→ch0，第 2 个→ch1，以此类推）

## 4. 常见问题

### 4.1 有数据但内容不对

检查是否选择了正确的串口模式。

### 4.2 完全读不到数据

按顺序排查：

1. 波特率是否与代码一致（通常 115200 或更高）
2. 端口是否选择正确（与设备管理器一致）
3. TX/RX 是否接反（T 接 R，R 接 T）
4. 波形乱码或锯齿：确认协议为 FireWater

### 4.3 Vofa 启动异常（祖传 Bug）

如果以上全部确认无误仍然无法工作，这可能是 Vofa+ 的已知问题。解决方法：

1. 关闭 Vofa+
2. 删除 `C:\Users\[用户名]\AppData\Local\vofa+` 文件夹
3. 重新启动 Vofa+

> Vofa+ 已长期未更新，此 Bug 可能在任意时间随机触发，与你的电脑或代码无关。

### 4.4 在线调参无反应

- 参数名是否完全一致（含大小写）
- 是否发送了换行符
- `AddParameterListener` 的参数名是否为常量字符串

## 5. 补充参考

- [B站 Vofa 教程](https://www.bilibili.com/video/BV1q1421R7uK/)
- 通道数量上限由模板参数 `N` 决定

> **作者**: [Qing](https://github.com/ZhangChuqing) | **修改日期**: 2026-07-18
