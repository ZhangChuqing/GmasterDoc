# using 别名

`using` 关键字用来给类型起一个别名，语法很直白：

```cpp
using 新名字 = 原类型;
```

它要解决的根本问题，是类型名字有时又长又难读，或者需要在不改动原类型的前提下换一个更贴近语境的称呼。要看清 `using` 好在哪，得先回到它之前的做法。

## 从 typedef 说起

在 `using` 出现以前，C 语言用 `typedef` 定义类型别名。对普通类型来说，两者做的事完全一样，只是排列顺序不同：`typedef` 把新名字放在原类型后面，`using` 则让新名字在等号左边、原类型在右边。

```cpp
// typedef 的语法：typedef 原类型 新名字;
typedef float fp32;         // fp32 等价于 float
typedef unsigned char u8;   // u8   等价于 unsigned char

fp32 angle = 3.14f;         // 等价于 float angle = 3.14f;
```

```cpp
// using 写同样的事：新名字在 = 左边，原类型在右边
using fp32 = float;
using u8   = unsigned char;
```

一旦类型变复杂，`typedef` 的写法就开始别扭。函数指针是最典型的例子——新名字被夹在返回类型和参数列表中间，阅读顺序不再是从左到右：

```cpp
// typedef 函数指针语法拆解：
//
//   typedef  返回类型  (*新名字)(参数列表);
//            ↑        ↑       ↑
//            返回int  是指针  参数(float,float,int)
//
typedef int (*ComputeFunc)(float a, float b, int mode);
//          ↑  * 表示这是一个指针类型，指向函数
// 解读：ComputeFunc 是一个指针类型，
//       指向 "参数为(float,float,int)、返回int" 的函数
```

读这行代码得先在中间找到 `ComputeFunc`，再向两边反推类型信息，签名越复杂越费劲。`using` 把新名字提到最左边，从左到右自然读下来即可：

```cpp
// using 写同一件事：新名字在最左边，然后 =，然后类型，从左到右自然阅读
using ComputeFunc = int (*)(float a, float b, int mode);
//                  ↑返回  ↑指针  ↑参数
```

`using` 与 `typedef` 之间还有一道本质的分界线，就是模板别名。`template` 是 C++ 的泛型机制，在定义里用占位符（如 `T`），编译器按调用时的实际类型生成具体代码：

```cpp
// 模板函数：T 是占位符
template <typename T>
T myMax(T a, T b) { return a > b ? a : b; }

myMax(3, 5);      // 编译器生成 int    myMax(int, int)
myMax(3.0, 5.0);  // 编译器生成 double myMax(double, double)
```

`using` 可以给一个模板类绑定部分参数后再起名，让另一个参数继续待定；`typedef` 做不到这一点：

```cpp
// ✅ using：模板参数 T 可以保留
template <typename T>
using Vec3 = Eigen::Matrix<T, 3, 1>;   // 三维列向量

Vec3<float>  vf;  // Eigen::Matrix<float,  3, 1>
Vec3<double> vd;  // Eigen::Matrix<double, 3, 1>

// ❌ typedef：不支持模板参数，无法表达 "T 待定" 的别名
```

正是这条分界线，让 `using` 在现代 C++ 里几乎全面取代了 `typedef`。

## GSRL 中的用法

GSRL 里 `using` 的用法可以归到几类，从简单的函数指针别名到复用整个类。

行为树用它给函数指针命名。`BTAction` 和 `BTCondition` 继承自 `BTNode`，它们不自己实现业务逻辑，而是通过函数指针让用户注册回调。源码位于 `GSRL/Algorithm/inc/alg_behavior_tree.hpp`：

```bash
BTNode（基类，定义虚函数 tick()）
  ├── BTAction : public BTNode
  │     定义  using ActionFunc = BTStatus (*)(void *context)
  │     存储  ActionFunc m_action     # 用户注册的函数指针
  │     重写  tick() → 调用 m_action(m_context)
  │
  └── BTCondition : public BTNode
        定义  using ConditionFunc = bool (*)(void *context)
        存储  ConditionFunc m_condition
        重写  tick() → 调用 m_condition(m_context) → 返回 成功/失败
```

```cpp
class BTAction : public BTNode
{
public:
    using ActionFunc = BTStatus (*)(void *context);  // 函数指针别名

private:
    ActionFunc m_action;  // 指向用户注册的回调函数
    void *m_context;      // 传给回调的上下文（通常是类实例指针）

public:
    BTAction(ActionFunc action, void *context = nullptr);
    BTStatus tick() override;  // 内部调用 m_action(m_context)
};

class BTCondition : public BTNode
{
public:
    using ConditionFunc = bool (*)(void *context);  // 函数指针别名

private:
    ConditionFunc m_condition;
    void *m_context;

public:
    BTCondition(ConditionFunc condition, void *context = nullptr);
    BTStatus tick() override;  // 内部调用 m_condition(m_context)
};
```

有了 `ActionFunc`、`ConditionFunc` 这两个短名，`ChassisBehaviorTree` 就能把 `BTAction` 和 `BTCondition` 当成员用，注册 3 个条件节点和 10 个动作节点。声明在 `Chariot/inc/crt_chassis_behavior.hpp`，构造时传入回调则在 `Chariot/src/crt_chassis_behavior.cpp`：

```cpp
// 声明（crt_chassis_behavior.hpp）
class ChassisBehaviorTree
{
    // 条件节点（使用 BTCondition）
    BTCondition m_fallDetection;          // 倒地检测
    BTCondition m_stairClimbingCondition; // 上台阶信号检测
    BTCondition m_jumpCondition;          // 跳跃信号检测

    // 动作节点（使用 BTAction）
    BTAction m_recoveryStateMachine; // 自起状态机
    BTAction m_stairClimbingAction;  // 上台阶执行
    BTAction m_jumpStateMachine;     // 跳跃状态机
    BTAction m_remoteTargetSetting;  // 遥控目标设定
    BTAction m_balanceLQR;           // LQR 平衡控制
    BTAction m_turnControl;          // 转向控制
    BTAction m_powerControl;         // 功率控制
    BTAction m_antiSplitCompensation;// 防劈叉补偿
    BTAction m_legLengthControl;     // 腿长控制
    BTAction m_rollControl;          // Roll 控制

    // 组合节点 ...
};

// 构造时传入回调函数（crt_chassis_behavior.cpp）
ChassisBehaviorTree::ChassisBehaviorTree()
    : m_fallDetection(fallDetectionCallback, this)       // 注册回调
    , m_stairClimbingCondition(stairClimbingConditionCallback, this)
    , m_recoveryStateMachine(recoveryStateMachineCallback, this)
    , m_balanceLQR(balanceLQRCallback, this)
    // ... 其余节点同理
{}
```

```bash
流程：
  ChassisBehaviorTree 构造 → 给每个 BTAction/BTCondition 传入回调函数指针
  → 行为树 tick() 遍历节点 → 节点调用存储的函数指针 → 执行实际业务逻辑
```

模板类型别名在滤波器里用得最充分。`KalmanFilter` 是模板类，继承自 `Filter<T>`，内部用 Eigen 矩阵库，而 Eigen 的矩阵类型名字很长，比如 `Eigen::Matrix<T, StateSize, StateSize>`。在类内用 `using` 起个短名，之后整个类内部就都用短名。源码位于 `GSRL/Algorithm/inc/alg_filter.hpp` 和 `GSRL/Algorithm/src/alg_filter.cpp`：

```cpp
template <typename T, int StateSize, int MeasSize, int ControlSize>
class KalmanFilter : public Filter<T>
{
public:
    // 用 using 将带模板参数的 Eigen 类型简化
    using StateVector   = Eigen::Vector<T, StateSize>;              // 状态向量
    using MeasVector    = Eigen::Vector<T, MeasSize>;               // 测量向量
    using StateMatrix   = Eigen::Matrix<T, StateSize, StateSize>;   // 状态矩阵
    using ObsMatrix     = Eigen::Matrix<T, MeasSize, StateSize>;    // 观测矩阵
    using GainMatrix    = Eigen::Matrix<T, StateSize, MeasSize>;    // 增益矩阵
    using ControlMatrix = Eigen::Matrix<T, StateSize, ControlSize>; // 控制矩阵

private:
    StateVector   m_state;        // 直接用短名
    StateMatrix   m_covariance;
    StateMatrix   m_transition;
    // ...
};
```

类外要用这些类型时，通过 `类名::别名` 拿到：

```cpp
KalmanFilter<fp32, 3, 1, 0> kf;                              // 三维状态、一维观测
KalmanFilter<fp32, 3, 1, 0>::StateVector state;              // 等价于 Eigen::Vector<fp32, 3>
```

除了简化类内类型名，还可以在类外再用一层 `using`，把常用的模板实例化组合固化成短名，这样调用者不必每次写出完整的模板参数：

```cpp
// alg_filter.cpp — 固定模板参数，命名常用维度的滤波器
using KalmanFilter1D = KalmanFilter<fp32, 1, 1, 0>;  // 一维
using KalmanFilter2D = KalmanFilter<fp32, 2, 1, 0>;  // 二维位置-速度
using KalmanFilter3D = KalmanFilter<fp32, 3, 1, 0>;  // 三维位置-速度-加速度

// alg_filter.hpp — RLS 滤波器同理
using RLSFilter2D = RLSFilter<fp32, 2>;
using RLSFilter3D = RLSFilter<fp32, 3>;
using RLSFilter4D = RLSFilter<fp32, 4>;
```

```bash
两层 using 的关系：

alg_filter.hpp 类内：
  KalmanFilter 模板类
    ├── using StateVector = Eigen::Vector<T, StateSize>   ← 简化类内使用的类型名
    └── ...

alg_filter.cpp 类外：
  using KalmanFilter2D = KalmanFilter<fp32, 2, 1, 0>     ← 固化模板参数
  → 用户只需写 KalmanFilter2D，不用每次写完整的模板参数
```

当一个类需要用到另一个类里定义的类型时，`using` 可以把那个类型引入进来，既缩短访问路径，也让外部用户能顺着当前类的接口直接找到类型定义。源码位于 `GSRL/Algorithm/inc/alg_pid.hpp` 和 `Chariot/inc/crt_chassis.hpp`。理解这一用法要先记得子类能访问父类的 public 成员（参考 `培训/面向对象编程.md`）：

```bash
class Parent {
public:
    int value;
    void foo();
};

class Child : public Parent {
    // Child 继承了 Parent 的 value 和 foo()
    // 外部可以通过 Child 实例访问 Parent 的 public 成员
};
```

`CascadePID` 继承自 `Controller`，内部用到 `SimplePID`。它通过 `using` 把 `SimplePID` 的类型暴露进自己的接口，用户写 `CascadePID::PIDParam` 即可：

```cpp
class SimplePID {
public:
    struct PIDParam { fp32 Kp, Ki, Kd, outputLimit, integralLimit; };
    struct PIDData  { fp32 integral, lastError; };
};

class CascadePID : public Controller   // CascadePID 是 Controller 的子类
{
public:
    using PIDParam = SimplePID::PIDParam;  // 用户写 CascadePID::PIDParam 即可
    using PIDData  = SimplePID::PIDData;

private:
    SimplePID m_outerLoop;  // 外环（成员变量，不是继承来的）
    SimplePID m_innerLoop;  // 内环

public:
    CascadePID(PIDParam &outerParam, PIDParam &innerParam);
};
```

`Chassis` 则是引入数学库的类型，在类内起短名，省得每次都写 `GSRLMath::` 前缀：

```cpp
class Chassis
{
public:
    using Vector3f  = GSRLMath::Vector3f;   // 不必每次写 GSRLMath::Vector3f
    using Matrix33f = GSRLMath::Matrix33f;
};
```

`using` 还有一个不太起眼却很实用的用法——等价替换。当两个类的实现完全一致时，不必把代码复制一遍，直接让一个名字成为另一个的别名即可。源码位于 `GSRL/Device/inc/dvc_motor.hpp`：

```cpp
// M2006 电机与 M3508 电机协议相同，直接复用
using MotorM2006 = MotorM3508;

// DM2325 电机与 DM4310 电机协议相同，直接复用
using MotorDM2325 = MotorDM4310;
```

```bash
对比：没有 using 的情况

做法A（重复代码）：
  class MotorM3508 { ... 全部实现 ... };
  class MotorM2006 { ... 复制粘贴一遍完全相同的代码 ... };
  → 占用双倍 ROM 空间，两处需要同步维护

做法B（using 别名）：
  class MotorM3508 { ... 全部实现 ... };
  using MotorM2006 = MotorM3508;  // M2006 就是 M3508，不产生新代码
  → 零额外 ROM 占用，一处维护
```

别名和原类型指向同一个类，不产生新代码，因此不额外占用 ROM，维护也只需改一处。

## 在自己的代码里用 using

最基础的用法是简化常用类型名，让声明更贴近语义：

```cpp
#include <cstdint>

// 简化常用类型，提高可读性
using fp32 = float;
using fp64 = double;
using u8   = uint8_t;
using u16  = uint16_t;
using u32  = uint32_t;

// 使用
fp32 voltage   = 24.0f;    // 等价于 float voltage
u8   motorId   = 1;        // 等价于 uint8_t motorId
u32  timestamp = 1000000;  // 等价于 uint32_t timestamp
```

函数指针别名常用来搭建状态机的跳转表——每种状态对应一个处理函数，用一个数组按状态索引查表调用：

```cpp
#include <cstdint>

// 状态枚举
enum class RobotState : uint8_t {
    IDLE,
    MOVING,
    JUMPING,
    STATE_COUNT   // 状态总数
};

class Robot;

// ✅ 用 using 定义状态处理函数的指针类型
//    解读：StateHandler 是 "指向 void 函数(Robot*) 的指针" 的别名
using StateHandler = void (*)(Robot *robot);

class Robot
{
public:
    // 跳转表：每种状态对应一个处理函数
    StateHandler stateTable[static_cast<uint8_t>(RobotState::STATE_COUNT)];
    RobotState currentState;

    void init()
    {
        // 注册各状态的处理函数
        stateTable[static_cast<uint8_t>(RobotState::IDLE)]    = handleIdle;
        stateTable[static_cast<uint8_t>(RobotState::MOVING)]  = handleMoving;
        stateTable[static_cast<uint8_t>(RobotState::JUMPING)] = handleJumping;
        currentState = RobotState::IDLE;
    }

    void update()
    {
        uint8_t idx = static_cast<uint8_t>(currentState);
        if (stateTable[idx] != nullptr) {
            stateTable[idx](this);  // 查表调用对应函数
        }
    }

private:
    static void handleIdle(Robot *robot)    { /* 空闲 */ }
    static void handleMoving(Robot *robot)  { /* 移动 */ }
    static void handleJumping(Robot *robot) { /* 跳跃 */ }
};
```

同样的函数指针，用 `typedef` 得把名字夹在中间，用 `using` 则一目了然：

```cpp
// ❌ typedef：新名字夹在中间
typedef void (*StateHandler)(Robot *robot);
//      返回 ↑指针  ↑名字      ↑参数

// ✅ using：新名字在左边，= 后面是类型
using StateHandler = void (*)(Robot *robot);
```

模板别名的差距更明显。`using` 一行搞定，`typedef` 只能用 `struct` 包一层，每次使用还要写 `::type`：

```cpp
#include <vector>

// ✅ using：一行定义模板别名
template <typename T>
using Vec = std::vector<T>;

Vec<int>    vi;               // 等价于 std::vector<int>
Vec<float>  vf;               // 等价于 std::vector<float>

// ❌ typedef：不支持模板别名。
//    变通方法是用 struct 包一层（每次使用都要写 ::type）：
template <typename T>
struct VecWrapper {
    typedef std::vector<T> type;   // 类型藏在 struct 里
};
VecWrapper<int>::type vi;          // 使用时要写 ::type，不直观
```

把这些场景放在一起，`using` 相对 `typedef` 的取舍就很清楚了：

| 场景 | typedef | using |
|------|---------|-------|
| 基础类型 | `typedef float fp32;` | `using fp32 = float;` |
| 函数指针 | `typedef void (*Func)(int);` | `using Func = void (*)(int);` |
| 模板别名 | ❌ 不支持 | `template<typename T> using Vec = std::vector<T>;` |
| 引入类成员类型 | 能写但不推荐 | `using PIDParam = SimplePID::PIDParam;` |
| 等价替换（复用类） | 不支持（只能复制代码） | `using MotorM2006 = MotorM3508;` |

> **作者**: [Qing](https://github.com/ZhangChuqing) | **修改日期**: 2026-07-26
