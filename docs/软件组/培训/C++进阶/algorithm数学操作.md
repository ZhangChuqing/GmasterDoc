# `<algorithm>` 中的数学操作

C++ 标准库 `<algorithm>` 提供了一些常用的数学工具函数，比如 `std::clamp`（限幅）、`std::min`/`std::max`、`std::abs`、`std::swap`。它们比手写版本更安全、更清晰、编译器优化也更好。

---

## 1. 各函数速览

| 函数 | 头文件 | 作用 | 示例 |
|------|--------|------|------|
| `std::clamp(v, lo, hi)` | `<algorithm>` (C++17) | 限幅：v < lo 则返回 lo，v > hi 则返回 hi | `clamp(1.5, 0, 1) → 1.0` |
| `std::min(a, b)` | `<algorithm>` | 取较小值 | `min(3, 5) → 3` |
| `std::max(a, b)` | `<algorithm>` | 取较大值 | `max(3, 5) → 5` |
| `std::abs(x)` | `<cstdlib>` / `<cmath>` | 绝对值（整数/浮点） | `abs(-5) → 5` |
| `std::swap(a, b)` | `<utility>` | 交换两个值 | `swap(x, y)` |

---

## 2. 工程代码中的实际使用（GSRL 源码）

### 2.1 `std::clamp` — 功率控制器中的衰减因子限幅

**出处：** `Chariot/src/crt_chassis.cpp:555-590`

底盘的功率控制器需要计算一个衰减因子 `alpha`（范围 [0, 1]），用来限制电机功率。计算过程中有多个步骤需要把结果限制在有效范围：

```cpp
void Chassis::powerController()
{
    // ... 省略参数计算 ...

    // A, B, C 是功率约束对应的二次方程系数：A*α² + B*α + C = 0
    fp32 A = k2 * (Rl * Rl + Rr * Rr);
    fp32 B = k0 * (Rl * omegaL + Rr * omegaR) +
             2.0f * k2 * (Ml * Rl + Mr * Rr);
    fp32 C = k0 * (Ml * omegaL + Mr * omegaR) +
             k1 * (fabsf(omegaL) + fabsf(omegaR)) +
             k2 * (Ml * Ml + Mr * Mr) + k3 - m_activePowerMaxW;

    fp32 alpha = 0.0f;

    if (fabsf(A) < 1e-8f) {
        // A ≈ 0：退化为线性方程 B*α + C = 0
        if (fabsf(B) > 1e-8f) {
            alpha = std::clamp(-C / B, 0.0f, 1.0f);   // ✅ 解限幅在 [0, 1]
        } else {
            alpha = (C <= 0.0f) ? 1.0f : 0.0f;
        }
    } else {
        // 正常二次方程
        fp32 delta = B * B - 4.0f * A * C;
        if (delta > 0.0f) {
            // ... 选取 [0, 1] 范围内最大的根 ...
        } else {
            // 无实数解：取抛物线顶点并限幅
            alpha = std::clamp(-B / (2.0f * A), 0.0f, 1.0f); // ✅ 顶点限幅
        }
    }

    // 低通滤波平滑衰减因子，防止力矩突变
    alpha = std::clamp(alpha, 0.01f, 1.0f);  // ✅ 下限 0.01 防止衰减因子为零
    m_powerDecayFactor = alpha * POWER_DECAY_FILTER_ALPHA
                       + m_powerDecayFactor * (1.0f - POWER_DECAY_FILTER_ALPHA);

    // 应用衰减后的力矩
    m_leftLeg.wheelTorque  += m_powerDecayFactor * Rl;
    m_rightLeg.wheelTorque += m_powerDecayFactor * Rr;
}
```

**伪代码——`std::clamp` 内部逻辑等价于：**

```bash
std::clamp(value, lo, hi) 等价于：
  if value < lo → return lo
  if value > hi → return hi
  else         → return value
```

**对比：手写 vs `std::clamp`**

```cpp
// ❌ 手写版本：三个分支，容易写错顺序
if (alpha < 0.0f) {
    alpha = 0.0f;
} else if (alpha > 1.0f) {
    alpha = 1.0f;
}

// ❌ 手写版本2：用三元运算符，一长串不好读
alpha = (alpha < 0.0f) ? 0.0f : (alpha > 1.0f) ? 1.0f : alpha;

// ✅ std::clamp：一行，意图清晰
alpha = std::clamp(alpha, 0.0f, 1.0f);
```

### 2.2 `std::clamp` — 腿长控制器防机械越界

**出处：** `Chariot/src/crt_chassis.cpp:605-627`

```cpp
void Chassis::legLengthController()
{
    // 左腿：目标长度必须限制在机械限位内
    fp32 targetLeftLegLength = std::clamp(
        m_targetLegLength - m_rollLengthCompensation,  // 计算值
        MIN_LEG_LENGTH_M,                               // 机械最小腿长
        MAX_LEG_LENGTH_M                                // 机械最大腿长
    );
    fp32 leftLengthError = m_leftLeg.l0 - targetLeftLegLength;
    fp32 leftLengthControlOutput = m_leftLegLengthPID.controllerCalculate(
        0.0f, &leftLengthError);
    m_leftLeg.virtualForce = leftLengthControlOutput;

    // 右腿同理
    fp32 targetRightLegLength = std::clamp(
        m_targetLegLength + m_rollLengthCompensation,
        MIN_LEG_LENGTH_M,
        MAX_LEG_LENGTH_M
    );
    // ...
}
```

```bash
原因：
  目标腿长 = 期望腿长 ± 侧倾补偿量

  如果机器人侧倾严重，补偿量可能让目标腿长超出机械限位
  → 电机强行驱动到不可达位置 → 堵转、过流

  std::clamp 强制把目标锁在机械允许范围内
```

### 2.3 `std::clamp` — 跳跃中的限速 lambda

**出处：** `Chariot/src/crt_chassis_behavior.cpp:670-676`

跳跃状态机中，用 lambda 封装了一个限速辅助函数：

```cpp
BTStatus ChassisBehaviorTree::jumpStateMachineCallback(void *context)
{
    auto *self    = static_cast<ChassisBehaviorTree *>(context);
    auto *chassis = self->m_chassis;

    // 限速辅助 lambda：修正速度到安全范围
    auto clampChassisSpeed = [chassis]() {
        // 先撤销上一帧的速度累积（position += speed * dt）
        chassis->m_targetPosition -= chassis->m_targetSpeed * chassis->m_loopDeltaTime;
        // 限幅速度
        chassis->m_targetSpeed = std::clamp(
            chassis->m_targetSpeed,
            -JUMP_MAX_SPEED_MPS,   // 最大后退速度
            JUMP_MAX_SPEED_MPS     // 最大前进速度
        );
        // 重新累积
        chassis->m_targetPosition += chassis->m_targetSpeed * chassis->m_loopDeltaTime;
    };

    // 在各个跳跃子阶段中反复调用
    switch (self->m_jumpPhase) {
        case JUMP_PHASE_CROUCH:    // 蹲下准备
            clampChassisSpeed();
            // ...
        case JUMP_PHASE_LAUNCH:    // 起跳
            clampChassisSpeed();
            // ...
        case JUMP_PHASE_RETRACT:   // 收腿
            clampChassisSpeed();
            // ...
        case JUMP_PHASE_EXTEND_LAND: // 伸腿着地
            clampChassisSpeed();
            // ...
    }
}
```

### 2.4 `std::min` / `std::max` — 行为树的角度约束

**出处：** `Chariot/src/crt_chassis_behavior.cpp:430-445`

恢复站立时的角度限幅：

```cpp
// 快速推进 phi0 向目标角度，但不能超过目标
self->m_targetPhi0Left += RECOVERY_STANDUP_SPEED * chassis->m_loopDeltaTime;
self->m_targetPhi0Left  = std::min(self->m_targetPhi0Left, RECOVERY_STANDUP_PHI0);
//                        ↑ 取较小值：如果超过了目标，就用目标值
```

爬楼梯时的角度下限：

```cpp
self->m_stairTargetPhi0Left -= CLIMBING_ASCEND_SPEED * chassis->m_loopDeltaTime;
self->m_stairTargetPhi0Left  = std::max(self->m_stairTargetPhi0Left, CLIMBING_ASCEND_TARGET_PHI0);
//                              ↑ 取较大值：如果跌破了目标，就用目标值
```

**结合 `std::clamp` 来理解这三者的关系：**

```bash
std::min(x, hi)   ≈  std::clamp(x, -∞, hi)    # 只限上界
std::max(x, lo)   ≈  std::clamp(x, lo, +∞)    # 只限下界
std::clamp(x, lo, hi)                          # 同时限上下界

典型用法：
  min → 增量累加时，防止超过上限
  max → 增量递减时，防止跌破下限
  clamp → 来自外部的值，直接锁死在区间内
```

### 2.5 `std::abs` — 卡尔曼滤波中的测量有效性判断

**出处：** `GSRL/Algorithm/inc/alg_filter.hpp:633-637`

```cpp
// KalmanFilter::performDynamicAdjustment()
for (int i = 0; i < MeasSize; ++i) {
    if (std::abs(measurement(i)) > EPSILON) {  // 绝对值 > 微小阈值 → 有效测量
        m_validMeasurementCount++;
    }
}
```

---

## 3. 动手写：基础用法示例

### 3.1 示例：PID 输出限幅

```cpp
#include <algorithm>

class SimplePID
{
public:
    struct PIDParam {
        fp32 Kp, Ki, Kd;
        fp32 outputLimit;     // 输出限幅值
        fp32 integralLimit;   // 积分限幅值
    };

    fp32 controllerCalculate(fp32 setPoint, const fp32 *feedBackData)
    {
        fp32 error = setPoint - *feedBackData;

        // P 项
        fp32 pOut = m_param.Kp * error;

        // I 项（带积分限幅）
        m_integral += error * m_dt;
        m_integral  = std::clamp(m_integral, -m_param.integralLimit, m_param.integralLimit);
        fp32 iOut   = m_param.Ki * m_integral;

        // D 项
        fp32 dErr   = (error - m_lastError) / m_dt;
        m_lastError = error;
        fp32 dOut   = m_param.Kd * dErr;

        // 总输出限幅
        fp32 output = pOut + iOut + dOut;
        return std::clamp(output, -m_param.outputLimit, m_param.outputLimit);
        //                    ↑ 一句话搞定上下限检查
    }

private:
    PIDParam m_param;
    fp32 m_integral = 0.0f;
    fp32 m_lastError = 0.0f;
    fp32 m_dt;
};
```

### 3.2 示例：传感器数据边界保护

```cpp
#include <algorithm>

// 处理激光测距数据
fp32 processRangeData(fp32 rawDistance)
{
    // 传感器有效范围 [0.05m, 12.0m]
    constexpr fp32 MIN_RANGE = 0.05f;
    constexpr fp32 MAX_RANGE = 12.0f;

    // 限幅到有效范围
    return std::clamp(rawDistance, MIN_RANGE, MAX_RANGE);
}

// 温度保护
fp32 applyTemperatureLimit(fp32 targetCurrent_A, fp32 currentTemp_C)
{
    constexpr fp32 MAX_TEMP      = 80.0f;
    constexpr fp32 DERATE_START  = 60.0f;
    constexpr fp32 MAX_CURRENT_A = 20.0f;

    // 温度超过降额起始点：线性降额
    if (currentTemp_C > DERATE_START) {
        fp32 factor = std::clamp(
            (MAX_TEMP - currentTemp_C) / (MAX_TEMP - DERATE_START),
            0.0f,   // 最高温度时因子为 0
            1.0f    // 降额起始时因子为 1
        );
        targetCurrent_A *= factor;
    }

    // 最终电流也要限幅
    return std::clamp(targetCurrent_A, 0.0f, MAX_CURRENT_A);
}
```

### 3.3 示例：`std::swap` 互换

```cpp
#include <utility>  // std::swap

// 两个滤波器交替工作（乒乓缓冲）
class DualKalmanFilter
{
    KalmanFilter2D m_filterA;
    KalmanFilter2D m_filterB;
    KalmanFilter2D *m_active;   // 指向当前使用的滤波器
    KalmanFilter2D *m_backup;   // 指向备用的滤波器

public:
    void swapFilters()
    {
        std::swap(m_active, m_backup);  // 交换两个指针，一行搞定
        // 等价于：
        // KalmanFilter2D *tmp = m_active;
        // m_active = m_backup;
        // m_backup = tmp;
    }
};

// 或者直接交换两个值
void toggleMode(int &modeA, int &modeB)
{
    std::swap(modeA, modeB);  // modeA 和 modeB 的值互换
}
```

### 3.4 示例：`std::abs` 比较 vs `fabsf`

```cpp
#include <cstdlib>   // std::abs (整数)
#include <cmath>     // fabsf (C 风格浮点绝对值)

// 卡尔曼滤波中判断测量值是否有效
bool isMeasurementValid(fp32 measurement)
{
    constexpr fp32 EPSILON = 1e-6f;

    // ✅ C++ 风格：std::abs 对浮点也能用（会调用重载版本）
    return std::abs(measurement) > EPSILON;

    // 等价于 C 风格：
    // return fabsf(measurement) > EPSILON;
}

// std::abs 的重载机制
int    absInt    = std::abs(-5);        // → 5 (int)
double absDouble = std::abs(-3.14);     // → 3.14 (double)
float  absFloat  = std::abs(-2.71f);    // → 2.71f (float)
// 一个函数名，编译器根据参数类型自动选版本
```

---

## 4. 对比：手写 vs 标准库

### 4.1 `std::clamp` vs 手写限幅

```cpp
fp32 value = getSomeValue();

// ❌ 手写版本1：容易把 > 和 < 写反
if (value > MAX) {
    value = MAX;
}
if (value < MIN) {
    value = MIN;
}

// ❌ 手写版本2：嵌套三元运算符，可读性差
value = (value < MIN) ? MIN : (value > MAX) ? MAX : value;

// ✅ 标准库：一行，意图明确
value = std::clamp(value, MIN, MAX);
```

### 4.2 `std::min`/`std::max` vs 三元运算符

```cpp
// ❌ 三元运算符：变量名出现两次，重复
int smaller = (a < b) ? a : b;
int larger  = (a > b) ? a : b;

// ✅ 标准库：简洁
int smaller = std::min(a, b);
int larger  = std::max(a, b);

// 更重要的是：初始化列表版本
int best = std::max({a, b, c, d});  // 四个值中取最大 —— 手写要嵌套三元
```

### 4.3 `std::abs` vs `fabsf`/`fabs`

```cpp
// C 风格：不同类型不同函数名，容易用错
int    v1 = abs(-5);           // int 绝对值
float  v2 = fabsf(-3.14f);    // float 绝对值  → 容易忘写 f
double v3 = fabs(-3.14);      // double 绝对值 → 容易和 abs 混淆

// C++ 风格：统一用 std::abs，自动重载
int    v1 = std::abs(-5);      // ✅
float  v2 = std::abs(-3.14f);  // ✅
double v3 = std::abs(-3.14);   // ✅
```

---

## 5. 总结

| 要做什么 | 用什么 | 头文件 |
|----------|--------|--------|
| 把值限制在 [lo, hi] | `std::clamp(v, lo, hi)` | `<algorithm>` (C++17) |
| 取两个值中较小的 | `std::min(a, b)` | `<algorithm>` |
| 取两个值中较大的 | `std::max(a, b)` | `<algorithm>` |
| 交换两个值 | `std::swap(a, b)` | `<utility>` |
| 取绝对值 | `std::abs(x)` | `<cstdlib>` 或 `<cmath>` |
| 多个值取最值 | `std::min({a,b,c})` / `std::max({a,b,c})` | `<algorithm>` |

**核心原则：能用标准库的，就不要手写。标准库版本更安全（类型检查）、更清晰（意图明确）、优化更好（constexpr + 编译器内建）。**

> **作者**: [Qing](https://github.com/ZhangChuqing) | **修改日期**: 2026-07-18
