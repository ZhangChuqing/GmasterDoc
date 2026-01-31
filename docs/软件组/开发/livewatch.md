# livewatch

Live Watch 是 Cortex-Debug 插件提供的实时变量监视功能，可在调试过程中高频刷新变量值，适合观察传感器数据、控制量等。

---

## 1. 安装 Cortex-Debug

在 VS Code 扩展商店搜索并安装 **Cortex-Debug**

---

## 2. 修改 `launch.json`

以下是 GSRL 库、ST-Link 适配的示例配置。  
**请根据你的本机环境修改 `gdbPath` 和 `executable` 路径。**

```bash
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Debug (OpenOCD)",
            "cwd": "${workspaceFolder}",
            "executable": "${workspaceFolder}/build/Debug/GMStdRobotLib.elf",
            "request": "launch",
            "type": "cortex-debug",
            "runToEntryPoint": "main",
            "servertype": "openocd",
            "gdbPath": "C:/[route]/arm-none-eabi-gdb.exe",
            "configFiles": [
                "interface/stlink.cfg",
                "target/stm32f4x.cfg"
            ],
            "showDevDebugOutput": "none",
            "liveWatch": {
                "enabled": true,
                "samplesPerSecond": 4
            }
        }
    ]
}
```
## 3. 常见问题排查
若仍然没有数据刷新，检查：
- 确认是否成功进入调试状态
- 确认 gdbPath 指向正确的 arm-none-eabi-gdb.exe
- 确认 .elf 文件路径正确（通常位于 build/Debug/）

---

## 4. 可选优化项
可根据需求调整刷新频率：
```bash
"liveWatch": {
    "enabled": true,
    "samplesPerSecond": （根据要求修改）
}
```
samplesPerSecond 越高，刷新越快。但是太高可能占用较多调试资源（但暂时没遇到类似问题）

---

如需支持 CMSIS-DAP 或其他烧录器，只需替换 configFiles 中的接口配置即可。