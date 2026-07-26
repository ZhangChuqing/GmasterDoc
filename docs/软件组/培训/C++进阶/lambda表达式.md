# Lambda 表达式

Lambda 表达式是 C++11 引入的匿名函数对象。它最本质的作用，是让一段短小的逻辑就地写在用到它的地方，而不必为一次性的操作单独定义一个命名函数。

## 从"跳转"到"就地"

设想要把数组里的每个元素都平方。传统做法是先定义一个函数，再把它的名字交给算法：

```cpp
// 传统做法：定义一个命名函数
int square(int x) {
    return x * x;
}

// 在另一处调用
std::transform(data.begin(), data.end(), data.begin(), square);
//                                                       ^^^^^^
//                                             函数名要跳转到别处才能看到实现
```

`square` 的实现在别处，读代码时得跳过去看一眼；而如果这段逻辑只用这一次，为它专门起个名字也显得多余。lambda 把实现直接放在调用点：

```cpp
std::transform(data.begin(), data.end(), data.begin(),
    [](int x) { return x * x; }   // 就地定义，一眼看到实现
);
```

打个比方，传统函数像一篇独立的文章，有标题，读者要翻目录去找；lambda 则像正文里的脚注，就地解释，读完即走，也不占用命名空间。但归根到底，lambda 仍然是一个函数——有参数、有返回值、有一段可执行代码。它真正特殊的地方在于，可以直接捕获周围作用域里的变量。

## 语法与捕获

一个完整的 lambda 由四部分组成：

```cpp
//  捕获列表    参数列表          返回类型（可选）  函数体
//  ┌────┐   ┌─────────┐       ┌──────────┐   ┌───────────────┐
    [&x, y](int a, float b) -> bool          { return a > b && x > y; }
//          ───────────────────────────────────────────────────────────
//                            一个完整的 lambda
```

参数列表和返回值与普通函数一致，`-> 返回类型` 在编译器能推导时可以省略：

```cpp
auto add = [](int a, int b) { return a + b; };        // 返回 int，自动推导
auto div = [](int a, int b) -> float { return a / b; }; // 显式指定返回 float
```

真正区别于普通函数的是捕获列表。它决定 lambda 体内能用哪些外部变量，以及以什么方式访问。按值捕获会在 lambda 内部留一份拷贝，捕获发生在定义那一刻，此后外部变量再变也影响不到它；按引用捕获则持有原变量的引用，读到的始终是当前值，也能修改原变量。

```cpp
int x = 10;

auto byValue = [x]() { return x; };   // x 在这里被拷贝到 lambda 内部
x = 20;
byValue();  // 返回 10，不是 20——因为捕获发生在 lambda 定义时

auto byRef = [&x]() { return x; };    // 持有 x 的引用
x = 20;
byRef();    // 返回 20——读取的是 x 当前的值
```

```bash
┌───────────────┐        ┌───────────────────┐
│ 外部 int x=10 │        │ lambda byValue    │
│ (栈上)        │  拷贝   │  ┌──────┐         │
│               │──────→│  │ x:10 │ (独立副本)│
│               │        │  └──────┘         │
└───────────────┘        └───────────────────┘

┌───────────────┐        ┌───────────────────┐
│ 外部 int x=10 │        │ lambda byRef      │
│ (栈上)        │  引用   │  ┌──────┐         │
│               │←──────│  │ x ──→│ (指向外部x)│
└───────────────┘        └──│───┘────────────┘
                            │
                            └── 始终指向外部 x 的地址
```

捕获列表有几种写法，差别集中在捕获谁、以及能否改写外部变量这两点上：

| 捕获方式 | 写法 | 含义 | lambda 内能否修改外部变量 |
|----------|------|------|---------------------------|
| 不捕获 | `[]` | 不使用任何外部变量 | — |
| 按值捕获全部 | `[=]` | 外部变量在 lambda 内有一份拷贝 | 否（改的是拷贝，不影响原变量） |
| 按引用捕获全部 | `[&]` | 外部变量在 lambda 内是引用 | 是 |
| 按值捕获指定变量 | `[x]` | 只有 x 被拷贝进来 | 否 |
| 按引用捕获指定变量 | `[&x]` | 只有 x 以引用方式进来 | 是 |
| 混合捕获 | `[=, &y]` | x 按值，y 按引用 | y 可以改 |
| 捕获 this | `[this]` | 在成员函数内使用当前对象的成员 | 可以改 `this` 指向的成员 |
| 捕获 this + 局部变量 | `[this, &x]` | 对象成员 + x 的引用 | 都可以改 |

## GSRL 中的用法

GSRL 里的 lambda 主要出现在行为树回调函数内部，用来封装只在某个状态机阶段用到的辅助计算。把这类逻辑就地写成局部函数，读者能在同一屏内看到定义和调用，省去跳转。

离地检测是一个典型例子，它通过 `[this]` 捕获当前对象，从而直接访问 `m_imu` 等成员。源码位于 `Chariot/src/crt_chassis.cpp` 的 `Chassis::airborneController()`：

```cpp
void Chassis::airborneController()
{
    // 定义一个 lambda 用于判断单条腿是否离地
    // [this] 捕获当前对象指针，lambda 内可以访问 m_imu 等成员
    auto airborneDection = [this](LegState &leg) {
        fp32 fn;
        fn = leg.virtualForce * arm_cos_f32(leg.theta)
           + leg.virtualTorque * arm_sin_f32(leg.theta) / leg.l0
           + (WHEEL_WEIGHT_KG * 9.8f);
        // ... 利用 m_imu 的加速度信息修正 fn ...

        fp32 airborneThreshold = (leg.l0 < AIRBORNE_LEG_LENGTH_BOUNDARY_M)
            ? AIRBORNE_DETECTION_THRESHOLD_SHORT_N
            : AIRBORNE_DETECTION_THRESHOLD_LONG_N;

        if (fn < airborneThreshold) {
            leg.isAirBorne = true;
        }
        // ...
    };

    // 对左右腿调用同一个 lambda
    airborneDection(m_leftLeg);
    airborneDection(m_rightLeg);
}
```

```bash
流程：
  airborneDection = [this](LegState &leg) { ... }
        │
        ├── 调用 airborneDection(m_leftLeg)  → 传入左腿引用，修改 m_leftLeg.isAirBorne
        └── 调用 airborneDection(m_rightLeg) → 传入右腿引用，修改 m_rightLeg.isAirBorne
```

因为捕获了 `this`，lambda 内部可以直接用 `m_imu`、`WHEEL_WEIGHT_KG` 这些类成员，不必再通过参数一层层传进来。同一段逻辑对左右腿各调用一次，避免了重复。

跳跃状态机的回调则展示了混合捕获的场景。源码位于 `Chariot/src/crt_chassis_behavior.cpp` 的 `jumpStateMachineCallback()`：

```cpp
BTStatus ChassisBehaviorTree::jumpStateMachineCallback(void *context)
{
    auto *self    = static_cast<ChassisBehaviorTree *>(context);
    auto *chassis = self->m_chassis;
    // ...

    // 捕获 chassis 指针（按值），判断起跳姿态是否满足条件
    auto isJumpPostureOK = [chassis]() -> bool {
        return fabsf(chassis->m_leftLeg.theta)  < JUMP_POSTURE_THETA_MAX_RAD &&
               fabsf(chassis->m_rightLeg.theta) < JUMP_POSTURE_THETA_MAX_RAD &&
               fabsf(chassis->m_eulerAngle.x)   < JUMP_POSTURE_TILT_MAX_RAD  &&
               fabsf(chassis->m_eulerAngle.y)   < JUMP_POSTURE_TILT_MAX_RAD;
    };

    // 同时捕获 self 和 chassis，退出跳跃时重置状态
    auto exitJump = [self, chassis](bool ignoreArm) {
        self->m_jumpPhase        = JUMP_PHASE_IDLE;
        self->m_jumpLaunchActive = false;
        if (ignoreArm) {
            self->m_jumpArmIgnored = true;
        }
        chassis->m_leftJumpLegLengthPID.pidClear();
        chassis->m_rightJumpLegLengthPID.pidClear();
    };

    // 限制移动速度
    auto clampChassisSpeed = [chassis]() {
        chassis->m_targetPosition -= chassis->m_targetSpeed * chassis->m_loopDeltaTime;
        chassis->m_targetSpeed = std::clamp(chassis->m_targetSpeed,
                                            -JUMP_MAX_SPEED_MPS, JUMP_MAX_SPEED_MPS);
        chassis->m_targetPosition += chassis->m_targetSpeed * chassis->m_loopDeltaTime;
    };

    // 在 switch-case 状态机中按需调用
    switch (self->m_jumpPhase) {
        case JUMP_PHASE_LAUNCH:
            if (!isJumpPostureOK()) {           // ✅ 就地调用，语义清晰
                exitJump(true);
            }
            break;
        // ...
    }
}
```

这几个 lambda 都只在 `jumpStateMachineCallback` 内部使用，写成 lambda 让定义和调用挨在一起。`[chassis]` 按值捕获的其实只是一个 8 字节的指针，成本极低，同时也明确告诉读者这段逻辑只依赖 `chassis` 这一个外部量。

不捕获任何变量的 lambda 同样有用武之地。自起状态机里限制角度领先量的辅助函数，所有数据都通过参数传入，源码位于同一文件的 `recoveryStateMachineCallback()`：

```cpp
// 限制 phi0 目标相对实际角度的领先量
auto limitTargetPhi0Lead = [](fp32 &targetPhi0, const fp32 &actualPhi0) {
    fp32 targetLead = GSRLMath::normalizeDeltaAngle(targetPhi0 - actualPhi0);
    GSRLMath::constrain(targetLead, RECOVERY_PHI0_TARGET_LEAD_LIMIT);
    targetPhi0 = GSRLMath::normalizeDeltaAngle(actualPhi0 + targetLead);
};
```

它的捕获列表是空的 `[]`，`targetPhi0` 作为非 const 引用传入，因此 lambda 内的修改会作用到调用者的变量上。

## 在自己的代码里用 lambda

配合 STL 算法是 lambda 最常见的场景。谓词逻辑直接写在算法调用处，还能顺手捕获外部变量：

```cpp
#include <algorithm>
#include <array>

// 场景：检查所有电机电流是否在安全范围内
std::array<float, 4> motorCurrents = {2.5f, 3.1f, 1.8f, 12.0f};
const float MAX_CURRENT = 10.0f;

// 用 lambda 配合 std::any_of 检查是否有超标电机
bool hasOvercurrent = std::any_of(
    motorCurrents.begin(), motorCurrents.end(),
    [MAX_CURRENT](float current) { return current > MAX_CURRENT; }
);
// hasOvercurrent == true（第 4 个电机 12.0 > 10.0）
```

行为树条件节点也可以在注册处就地写逻辑，让条件与它被挂上的位置在同一行代码里：

```cpp
// 传统写法：条件逻辑散落在回调函数中
BTStatus myCondition(void *context) {
    auto *chassis = static_cast<Chassis *>(context);
    if (chassis->m_eulerAngle.x > 30.0f || chassis->m_eulerAngle.y > 30.0f) {
        return BT_FAIL;
    }
    return BT_SUCCESS;
}

// lambda 写法：在 setCondition 处就地定义，逻辑与注册在同一位置
myNode.setCondition(
    [](void *ctx) -> bool {
        auto *chassis = static_cast<Chassis *>(ctx);
        return fabsf(chassis->m_eulerAngle.x) < 30.0f &&
               fabsf(chassis->m_eulerAngle.y) < 30.0f;
    }
);
```

捕获方式一旦选错，会引出两类常见的坑。按值捕获的变量默认不可修改，因为 lambda 的 `operator()` 是 const 的；即便加了 `mutable`，改动的也只是副本，外部变量纹丝不动。若确实要改外部变量，应当按引用捕获。另一类坑更隐蔽：当 lambda 的存活时间超过了它按引用捕获的变量，引用就会悬垂。

```cpp
void badExample()
{
    int counter = 0;

    // 错误1：按值捕获后修改副本，外部不受影响
    auto badIncrement = [counter]() { counter++; };  // ❌ 编译错误
    // counter 是按值捕获的副本，默认不可修改（lambda 的 operator() 是 const）
    // 即使加 mutable，改的也是副本，外部 counter 不变

    // 正确：如果确实需要修改外部变量，用引用捕获
    auto goodIncrement = [&counter]() { counter++; }; // ✅

    // 错误2：lambda 存活时间超过捕获的引用
    std::function<void()> storedLambda;
    {
        int localVar = 42;
        storedLambda = [&localVar]() { return localVar; };
    }  // localVar 在此销毁
    // storedLambda();  // ❌ 悬垂引用！localVar 已经不存在了

    // 正确：如果 lambda 需要存活到作用域之外，按值捕获或确保引用有效
}
```

如果确实需要在 lambda 内改写按值捕获的副本，加上 `mutable` 即可。此时副本会在多次调用之间保留状态，而外部原变量始终不变：

```cpp
int x = 10;

// 默认情况下，按值捕获的变量在 lambda 内是 const
// auto f = [x]() { x++; };  // ❌ 编译错误

// 加 mutable 关键字后可以修改副本（外部原变量不受影响）
auto f = [x]() mutable { x++; return x; };
f();  // 返回 11
f();  // 返回 12——副本在每次调用间保持状态
// 外部 x 仍然是 10
```

## lambda 到底是什么

lambda 看似语法糖，底层其实是一个对象。编译器会把它翻译成一个匿名类，把函数体塞进这个类的 `operator()`：

```cpp
auto add = [](int a, int b) { return a + b; };
```

```cpp
// 编译器生成的等价代码（简化版）
class __lambda_add {
public:
    int operator()(int a, int b) const {
        return a + b;
    }
};
__lambda_add add;  // add 实际上是一个对象，不是一个函数指针
```

捕获列表里的变量，则成了这个匿名类的成员变量，在构造时被填入：

```cpp
// 如果捕获了变量：
int x = 10;
auto f = [x](int a) { return a + x; };

// 编译器大致生成：
class __lambda_f {
    int __x;  // 捕获的变量变成成员
public:
    __lambda_f(int x) : __x(x) {}
    int operator()(int a) const { return a + __x; }
};
```

这解释了 lambda 的几个特性：它是一个函数对象（functor）而非函数指针，`operator()` 被调用时执行函数体，捕获的变量以成员形式存活；每个 lambda 都有独一无二的类型，因此用 `auto` 接收最省事。

由此也能理解 lambda 与普通函数的分工。判断标准无非是复用性与生命周期：只用一次、就近可见的辅助逻辑适合 lambda；需要在多处复用的通用逻辑写成普通函数更划算；传给 STL 算法的谓词偏向 lambda；而行为树节点回调在当前 GSRL 中用普通函数注册，因为函数指针类型固定、注册简单。

| 场景 | 推荐方式 | 原因 |
|------|----------|------|
| 只用一次的辅助逻辑 | lambda | 就地定义，不需要命名和跳转 |
| 多处复用的通用逻辑 | 普通函数 | lambda 重复定义浪费代码，普通函数可复用 |
| 传给 STL 算法的谓词 | lambda | 简洁，捕获外部变量方便 |
| 行为树节点回调 | 普通函数（当前 GSRL 做法） | 函数指针稳定，类型已知，注册简单 |
| 回调内部的状态机辅助 | lambda | 只在该回调内部使用，封装性好 |
| 异步/线程任务 | lambda | 可以方便地捕获上下文数据 |

## 与 GSRL 的关系

GSRL 里行为树节点的回调通过函数指针注册（参考 `培训/using别名.md` 的行为树部分），这是历史设计选择——函数指针类型固定，注册简单，也不引入模板开销。但在回调函数内部，lambda 被广泛用来封装局部辅助逻辑：`airborneController()` 里 `[this]` 捕获的离地检测、跳跃状态机里 `[chassis]` 捕获的姿态验证、自起状态机里 `[]` 不捕获的角度限制，都是如此。外层用函数指针注册、内层用 lambda 组织逻辑，接口的简洁与内部实现的可读性由此兼顾。

> **作者**: [Qing](https://github.com/ZhangChuqing) | **修改日期**: 2026-07-26
