# `<algorithm>` 中的数学操作

C++ 标准库的 `<algorithm>` 里有几个常用的数学工具函数：`std::clamp`（限幅）、`std::min` 与 `std::max`（取最值）、`std::abs`（绝对值）、`std::swap`（交换）。它们做的都是很朴素的事，与手写代码没有功能上的差别，但因为经过类型检查、语义明确、又能被编译器内建优化，写出来比手写的一串条件判断更不容易出错，也更好读。

这些函数各自的头文件和作用如下：

| 函数 | 头文件 | 作用 | 示例 |
|------|--------|------|------|
| `std::clamp(v, lo, hi)` | `<algorithm>`（C++17） | 把 v 限制在 [lo, hi]，越界就取边界 | `clamp(1.5, 0, 1) → 1.0` |
| `std::min(a, b)` | `<algorithm>` | 取较小值 | `min(3, 5) → 3` |
| `std::max(a, b)` | `<algorithm>` | 取较大值 | `max(3, 5) → 5` |
| `std::abs(x)` | `<cstdlib>` / `<cmath>` | 绝对值（整数或浮点） | `abs(-5) → 5` |
| `std::swap(a, b)` | `<utility>` | 交换两个值 | `swap(x, y)` |

其中 `std::clamp` 用得最多，也最能体现这类函数的价值，下面从工程代码入手来看。

## clamp 做的是限幅

`std::clamp(value, lo, hi)` 的逻辑只有一句话：value 小于下界 lo 就返回 lo，大于上界 hi 就返回 hi，落在区间内则原样返回。凡是"某个值不能超出某个范围"的场合，它都用得上。GSRL 的底盘控制里这样的场合很多。

底盘功率控制器要算一个衰减因子 `alpha`，取值范围是 [0, 1]，用来压低电机功率。计算过程中，二次方程的解、抛物线顶点、最终滤波后的值都可能算出区间外的数，每一处都用 `clamp` 锁回来（出处：`Chariot/src/crt_chassis.cpp:555-590`）：

```cpp
void Chassis::powerController()
{
    // A*α² + B*α + C = 0，A、B、C 为功率约束推出的系数
    // ... 省略系数计算 ...

    fp32 alpha = 0.0f;
    if (fabsf(A) < 1e-8f) {
        // A ≈ 0：退化为线性方程 B*α + C = 0
        if (fabsf(B) > 1e-8f) {
            alpha = std::clamp(-C / B, 0.0f, 1.0f);       // 解限幅在 [0, 1]
        } else {
            alpha = (C <= 0.0f) ? 1.0f : 0.0f;
        }
    } else {
        fp32 delta = B * B - 4.0f * A * C;
        if (delta > 0.0f) {
            // ... 选取 [0, 1] 内最大的根 ...
        } else {
            alpha = std::clamp(-B / (2.0f * A), 0.0f, 1.0f); // 无实根，取顶点并限幅
        }
    }

    // 下限 0.01 防止衰减因子归零，随后低通滤波平滑
    alpha = std::clamp(alpha, 0.01f, 1.0f);
    m_powerDecayFactor = alpha * POWER_DECAY_FILTER_ALPHA
                       + m_powerDecayFactor * (1.0f - POWER_DECAY_FILTER_ALPHA);
    m_leftLeg.wheelTorque  += m_powerDecayFactor * Rl;
    m_rightLeg.wheelTorque += m_powerDecayFactor * Rr;
}
```

腿长控制器是另一个例子（出处：`Chariot/src/crt_chassis.cpp:605-627`）。目标腿长等于期望腿长加减侧倾补偿量，机器人侧倾严重时补偿量可能把目标顶出机械限位，电机就会去够一个够不到的位置，导致堵转过流。`clamp` 把目标锁在 `[MIN_LEG_LENGTH_M, MAX_LEG_LENGTH_M]` 之内，避免这种情况：

```cpp
fp32 targetLeftLegLength = std::clamp(
    m_targetLegLength - m_rollLengthCompensation,
    MIN_LEG_LENGTH_M, MAX_LEG_LENGTH_M);
// 右腿同理，补偿量符号相反
```

跳跃状态机里，限速这件事被封成一个 lambda 反复用（出处：`Chariot/src/crt_chassis_behavior.cpp:670-676`）。它先撤销上一帧按速度累积的位置，把速度 `clamp` 到 `[-JUMP_MAX_SPEED_MPS, JUMP_MAX_SPEED_MPS]`，再重新累积，蹲下、起跳、收腿、着地各阶段都调用一次：

```cpp
auto clampChassisSpeed = [chassis]() {
    chassis->m_targetPosition -= chassis->m_targetSpeed * chassis->m_loopDeltaTime;
    chassis->m_targetSpeed = std::clamp(
        chassis->m_targetSpeed, -JUMP_MAX_SPEED_MPS, JUMP_MAX_SPEED_MPS);
    chassis->m_targetPosition += chassis->m_targetSpeed * chassis->m_loopDeltaTime;
};
```

## min、max 与 clamp 的关系

`std::min` 和 `std::max` 是只限一边的限幅。`min(x, hi)` 相当于只压上界的 `clamp(x, -∞, hi)`，`max(x, lo)` 相当于只托下界的 `clamp(x, lo, +∞)`。用法上有个规律：一个值在不断累加、怕它冲过上限时用 `min` 压顶；在不断递减、怕它跌破下限时用 `max` 托底；对来自外部、上下都不放心的值，直接用 `clamp` 锁在区间里。

底盘恢复站立时，`phi0` 向目标角度快速推进，用 `min` 防止冲过头（出处：`Chariot/src/crt_chassis_behavior.cpp:430-445`）：

```cpp
self->m_targetPhi0Left += RECOVERY_STANDUP_SPEED * chassis->m_loopDeltaTime;
self->m_targetPhi0Left  = std::min(self->m_targetPhi0Left, RECOVERY_STANDUP_PHI0);
```

爬楼梯时角度递减，则用 `max` 防止跌破下限：

```cpp
self->m_stairTargetPhi0Left -= CLIMBING_ASCEND_SPEED * chassis->m_loopDeltaTime;
self->m_stairTargetPhi0Left  = std::max(self->m_stairTargetPhi0Left, CLIMBING_ASCEND_TARGET_PHI0);
```

## abs 用一个名字覆盖各种类型

C 风格的绝对值函数按类型分了好几个名字：整数用 `abs`，`float` 用 `fabsf`，`double` 用 `fabs`，很容易忘写后缀或者用混。`std::abs` 是重载的，整数、`float`、`double` 都写 `std::abs`，编译器按参数类型自动挑对应版本，少了一层记名字、选后缀的负担。卡尔曼滤波里判断测量值是否有效就用它（出处：`GSRL/Algorithm/inc/alg_filter.hpp:633-637`）：

```cpp
for (int i = 0; i < MeasSize; ++i) {
    if (std::abs(measurement(i)) > EPSILON) {  // 绝对值超过阈值才算有效
        m_validMeasurementCount++;
    }
}
```

## swap 交换两个值

`std::swap(a, b)` 把两个变量的值互换，省去手写的临时变量三步走。指针也能交换，双缓冲切换就靠它：

```cpp
void swapFilters() {
    std::swap(m_active, m_backup);  // 交换两个指针
    // 等价于 tmp = m_active; m_active = m_backup; m_backup = tmp;
}
```

## 几个自己动手的例子

PID 控制器里，积分项和总输出都要限幅，`clamp` 各管一处：

```cpp
fp32 controllerCalculate(fp32 setPoint, const fp32 *feedBackData)
{
    fp32 error = setPoint - *feedBackData;
    fp32 pOut = m_param.Kp * error;

    m_integral += error * m_dt;
    m_integral  = std::clamp(m_integral, -m_param.integralLimit, m_param.integralLimit);
    fp32 iOut   = m_param.Ki * m_integral;

    fp32 dErr   = (error - m_lastError) / m_dt;
    m_lastError = error;
    fp32 dOut   = m_param.Kd * dErr;

    fp32 output = pOut + iOut + dOut;
    return std::clamp(output, -m_param.outputLimit, m_param.outputLimit);
}
```

传感器数据的边界保护也是同一个套路。激光测距只在 [0.05m, 12.0m] 内有效，超出范围就 `clamp` 回来；温度降额时，把降额因子 `clamp` 在 [0, 1] 内做线性插值：

```cpp
fp32 processRangeData(fp32 rawDistance) {
    return std::clamp(rawDistance, 0.05f, 12.0f);
}

fp32 applyTemperatureLimit(fp32 targetCurrent_A, fp32 currentTemp_C) {
    constexpr fp32 MAX_TEMP = 80.0f, DERATE_START = 60.0f, MAX_CURRENT_A = 20.0f;
    if (currentTemp_C > DERATE_START) {
        fp32 factor = std::clamp(
            (MAX_TEMP - currentTemp_C) / (MAX_TEMP - DERATE_START), 0.0f, 1.0f);
        targetCurrent_A *= factor;
    }
    return std::clamp(targetCurrent_A, 0.0f, MAX_CURRENT_A);
}
```

## 手写与标准库的对比

把标准库和手写版本摆在一起，就能看出前者的价值主要在可读和不易错。限幅若手写，容易把 `>` 和 `<` 写反，或者写成一长串嵌套三元运算符，而 `std::clamp` 一行就把意图交代清楚：

```cpp
// 手写：分支多，容易写反
if (value > MAX) value = MAX;
if (value < MIN) value = MIN;

// 手写：嵌套三元，读起来费劲
value = (value < MIN) ? MIN : (value > MAX) ? MAX : value;

// 标准库：一行，意图明确
value = std::clamp(value, MIN, MAX);
```

`min`、`max` 同理，三元运算符要让变量名出现两次，`std::min(a, b)` 则没有重复。多个值取最值时优势更明显，`std::max({a, b, c, d})` 一句话解决，手写就得嵌套一串三元。`std::abs` 的好处则是前面说的统一名字、自动重载，不必再为类型操心。

各函数的选用可以归到一张表里：

| 要做什么 | 用什么 | 头文件 |
|----------|--------|--------|
| 把值限制在 [lo, hi] | `std::clamp(v, lo, hi)` | `<algorithm>`（C++17） |
| 取两个值中较小的 | `std::min(a, b)` | `<algorithm>` |
| 取两个值中较大的 | `std::max(a, b)` | `<algorithm>` |
| 交换两个值 | `std::swap(a, b)` | `<utility>` |
| 取绝对值 | `std::abs(x)` | `<cstdlib>` 或 `<cmath>` |
| 多个值取最值 | `std::min({a,b,c})` / `std::max({a,b,c})` | `<algorithm>` |

这些函数功能都很基础，手写也能实现，但标准库版本经过类型检查、意图更清楚、也更利于编译器优化。能用标准库的地方，就不必自己再写一遍。

> **作者**: [Qing](https://github.com/ZhangChuqing) | **修改日期**: 2026-07-26
