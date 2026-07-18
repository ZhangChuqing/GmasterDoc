# 模板（Template）

模板是 C++ 的泛型编程机制。它让你**写一次代码，适配多种类型**——编译器在编译期根据实际使用的类型自动生成对应的代码，不产生运行时开销。

---

## 1. 概念——它解决什么问题

### 1.1 没有模板时怎么做

假设你需要一个取最大值的函数。对于 `int` 和 `float`，逻辑完全相同，但类型不同，你不得不写两遍：

```cpp
// 不做泛型：每种类型写一份
int   maxInt(int a,   int b)   { return a > b ? a : b; }
float maxFloat(float a, float b) { return a > b ? a : b; }
// double、long、unsigned……每加一种类型就多一份代码
```

```bash
问题：
  maxInt(3, 5)        → 返回 5       ✅
  maxFloat(3.0f, 5.0f) → 返回 5.0f   ✅
  maxFloat(3, 5)      → 能工作，但隐式转换，类型不安全
  maxInt(3.0, 5.0)    → 截断为 int，精度丢失

写 N 种类型 = N 份几乎相同的代码，维护噩梦。
```

### 1.2 函数模板的做法

```cpp
// 写一次，适配所有类型
template <typename T>
T myMax(T a, T b) {
    return a > b ? a : b;
}

myMax(3, 5);          // T = int，    编译器生成 int    myMax(int, int)
myMax(3.0f, 5.0f);    // T = float，  编译器生成 float  myMax(float, float)
myMax(3.0, 5.0);      // T = double， 编译器生成 double myMax(double, double)
```

**关键理解：模板不是"运行时的多态"，而是"编译期的代码生成"。每当你用一种新类型调用模板函数，编译器就为你生成一份该类型的专属代码。这份生成的代码与手写的同类型函数完全等价，没有任何虚函数调用或运行时分发开销。**

### 1.3 类比

- 写模板 = 做一个印章（模具），`T` 是留空的部分
- 调用模板 = 用印章盖在纸上，`T` 被填入具体类型，盖出来的就是那份专属代码
- 盖 3 次产生 3 份代码，不盖就不产生——零开销原则（zero-overhead principle）

---

## 2. 语法基础

### 2.1 函数模板

```cpp
// 语法：template <typename 占位符名>  返回类型 函数名(参数列表)
template <typename T>
T add(T a, T b) {
    return a + b;
}

add(1, 2);       // T = int
add(1.5f, 2.5f); // T = float
```

可以有多个模板参数：

```cpp
// 两个不同类型的参数
template <typename T1, typename T2>
auto multiply(T1 a, T2 b) -> decltype(a * b) {
    return a * b;
}

multiply(3, 4.5f);   // T1 = int, T2 = float, 返回 float
multiply(2.0, 3);    // T1 = double, T2 = int, 返回 double
```

### 2.2 类模板

```cpp
// 一个容器，可以存放任意类型的单个值
template <typename T>
class Box {
private:
    T m_value;
public:
    Box(T value) : m_value(value) {}
    T get() const { return m_value; }
    void set(T value) { m_value = value; }
};

Box<int>    intBox(42);    // T = int，   编译器生成 Box<int>    类
Box<float>  floatBox(3.14f); // T = float，编译器生成 Box<float> 类
Box<fp32>   fpBox(1.0f);   // T = fp32，  编译器生成 Box<fp32>  类
```

```bash
模板类实例化过程：

  template <typename T> class Box { ... };    ← 模具（不占内存，不产生代码）

  使用 Box<int>       → 编译器生成 class Box<int>  { int m_value;    ... };
  使用 Box<float>     → 编译器生成 class Box<float>{ float m_value;  ... };

  Box<int> 和 Box<float> 是两个完全独立的类，它们之间没有继承关系。
```

### 2.3 非类型模板参数——值作为模板参数

除了 `typename`，模板参数还可以是**编译期常量值**（整数、枚举、指针等）：

```cpp
// N 不是类型，而是一个编译期确定的整数值
template <typename T, int N>
class FixedArray {
private:
    T m_data[N];  // 数组大小由模板参数 N 决定
public:
    constexpr int size() const { return N; }
    T& operator[](int i) { return m_data[i]; }
};

FixedArray<int, 5>  arr5;   // m_data 是 int[5]，20 字节
FixedArray<int, 10> arr10;  // m_data 是 int[10]，40 字节
// arr5 和 arr10 是不同类型，互不兼容
```

实际上 `std::array<T, N>` 就是这样实现的——`N` 是非类型模板参数（参考 `培训/std_array.md`）。

### 2.4 默认模板参数

```cpp
// 不指定 T 时，默认为 fp32
template <typename T = fp32>
class LowPassFilter : public Filter<T> {
    T m_alpha;
    T m_lastOutput;
    // ...
};

LowPassFilter<>      lp1;  // T = fp32（使用默认值）
LowPassFilter<float> lp2;  // T = float（显式指定）
```

多个参数都可以有默认值：

```cpp
template <typename T = fp32, size_t WindowSize = 10>
class MovingAverageFilter : public Filter<T> {
    T m_buffer[WindowSize];  // 大小由模板参数决定
    // ...
};

MovingAverageFilter<>        maf1;  // T = fp32, WindowSize = 10
MovingAverageFilter<float, 5> maf2;  // T = float, WindowSize = 5
```

---

## 3. 工程代码分析（GSRL 中的实际使用）

### 3.1 滤波器基类：类型占位

**源码位置：** `GSRL/Algorithm/inc/alg_filter.hpp`

```cpp
// 模板化的抽象接口——"我能处理任意类型 T 的数据"
template <typename T>
class Filter {
public:
    virtual ~Filter()                  = default;
    virtual T filterCalculate(T input) = 0;
    virtual void reset()               = 0;
};
```

```bash
Filter 模板实例化：

  Filter<float>  → filterCalculate(float)   → float
  Filter<double> → filterCalculate(double)  → double
  Filter<fp32>   → filterCalculate(fp32)    → fp32（fp32 就是 float，但类型名不同）

所有滤波器类（LowPassFilter、KalmanFilter、RLSFilter）都继承自 Filter<T>，
继承时把 T 传给基类，确保输入输出类型一致：
  LowPassFilter<T>     : public Filter<T>
  KalmanFilter<T, ...> : public Filter<T>
  RLSFilter<T, ...>    : public Filter<T>
```

### 3.2 卡尔曼滤波器：多维模板参数

**源码位置：** `GSRL/Algorithm/inc/alg_filter.hpp`

```cpp
template <typename T = fp32, int StateSize = 1, int MeasSize = 1, int ControlSize = 0>
class KalmanFilter : public Filter<T>
{
public:
    using StateVector   = Eigen::Vector<T, StateSize>;    // 状态向量
    using MeasVector    = Eigen::Vector<T, MeasSize>;     // 测量向量
    using StateMatrix   = Eigen::Matrix<T, StateSize, StateSize>; // 状态矩阵
    using ObsMatrix     = Eigen::Matrix<T, MeasSize, StateSize>;  // 观测矩阵
    using GainMatrix    = Eigen::Matrix<T, StateSize, MeasSize>;   // 增益矩阵
    using ControlMatrix = Eigen::Matrix<T, StateSize, ControlSize>;// 控制矩阵

private:
    StateVector m_state;
    StateMatrix m_covariance;
    StateMatrix m_transition;
    ObsMatrix   m_observation;
    GainMatrix  m_gain;
    ControlMatrix m_control;
    // ...
};
```

```bash
KalmanFilter 的模板参数含义：

  T             — 数值精度（fp32 / float / double）
  StateSize     — 状态向量维度（如：位置+速度 = 2）
  MeasSize      — 观测向量维度（如：单轴位置 = 1）
  ControlSize   — 控制输入维度（无控制 = 0）

三个维度参数都是 int，不是 typename——它们不是类型，而是决定矩阵大小的整数值。
ControlSize = 0 时，ControlMatrix 是 Eigen::Matrix<T, StateSize, 0>，
即零列矩阵，相关函数通过 SFINAE 在编译期移除（见 3.4 节）。

实例化示例：
  KalmanFilter<fp32, 2, 1, 0>   → 位置-速度 2D 滤波器，无控制输入
  KalmanFilter<fp32, 3, 1, 0>   → 位置-速度-加速度 3D 滤波器
  KalmanFilter<fp32, 4, 2, 2>   → 四维状态、二维观测、二维控制
```

工程中的类型别名（参考 `培训/using别名.md`）：

```cpp
using KalmanFilter1D = KalmanFilter<fp32, 1, 1, 0>;
using KalmanFilter2D = KalmanFilter<fp32, 2, 1, 0>;
using KalmanFilter3D = KalmanFilter<fp32, 3, 1, 0>;
```

### 3.3 行为树组合节点：整数模板参数

**源码位置：** `GSRL/Algorithm/inc/alg_behavior_tree.hpp`

```cpp
template <uint8_t MaxChildren = BT_DEFAULT_MAX_CHILDREN>
class BTComposite : public BTNode
{
protected:
    BTNode *m_children[MaxChildren];  // 子节点数组，大小由模板参数决定
    uint8_t m_childCount = 0;

public:
    bool addChild(BTNode *child) {
        if (m_childCount >= MaxChildren) return false;  // 编译期确定了上限
        m_children[m_childCount++] = child;
        return true;
    }
};

template <uint8_t MaxChildren = BT_DEFAULT_MAX_CHILDREN>
class BTSequence : public BTComposite<MaxChildren>  // 把参数传给基类
{
    uint8_t m_runningIndex = 0;
    BTStatus tick() override { /* 依次执行子节点 */ }
};

template <uint8_t MaxChildren = BT_DEFAULT_MAX_CHILDREN>
class BTSelector : public BTComposite<MaxChildren>
{
    uint8_t m_runningIndex = 0;
    BTStatus tick() override { /* 选择第一个成功的子节点 */ }
};
```

```bash
行为树模板参数的作用：

  BTSequence<8>    → m_children 是 BTNode*[8]，最多 8 个子节点
  BTSequence<16>   → m_children 是 BTNode*[16]，最多 16 个子节点

所有数组大小在编译期确定 → 整个节点对象在栈上，无堆分配。
不同 MaxChildren 值产生不同的类，互不兼容（这是编译期类型安全的一部分）。
```

### 3.4 SFINAE：编译期移除不适用的函数

**源码位置：** `GSRL/Algorithm/inc/alg_filter.hpp`

当 `ControlSize = 0`（无控制输入）时，带控制输入的 `predict`/`update`/`setControlMatrix` 不应存在——调用它们本身就是逻辑错误。

```cpp
// 仅在 ControlSize > 0 时，这个函数才存在
template <bool EnableControl = (ControlSize > 0)>
typename std::enable_if<EnableControl, void>::type
predict(const ControlVector &control)
{
    m_statePred = m_transition * m_state + m_control * control;
    // ...
}
```

```bash
机制解释（SFINAE = Substitution Failure Is Not An Error）：

  KalmanFilter<fp32, 2, 1, 0> kf;  // ControlSize = 0
  kf.predict();                      // ✅ 调用无控制输入的版本
  // kf.predict(controlVec);         // ❌ 编译错误！
  // std::enable_if<false, void>::type 替换失败，
  // 这个函数在编译期就被移除了，根本不存在

  KalmanFilter<fp32, 2, 1, 2> kf2; // ControlSize = 2 > 0
  kf2.predict();                     // ✅ 无控制输入版本
  kf2.predict(controlVec);           // ✅ 带控制输入版本（EnableControl = true 生效）
```

**为什么要这样做：** 如果 `ControlSize = 0`，`ControlMatrix` 是零列矩阵，`m_control * control` 在语义上无意义。与其在运行时报错，不如在**编译期直接让这个函数不存在**——调用者在 IDE 里就看不到这个重载，从根本上消除误用的可能性。

---

## 4. 动手写：在 GSRL 场景中使用模板

### 4.1 传感器数据缓冲区

```cpp
#include <array>

// 通用环形缓冲区——类型 T 和容量 N 都可以按需配置
template <typename T, size_t N>
class RingBuffer {
private:
    std::array<T, N> m_buffer;
    size_t m_head = 0;
    size_t m_count = 0;

public:
    void push(const T &value) {
        m_buffer[m_head] = value;
        m_head = (m_head + 1) % N;
        if (m_count < N) m_count++;
    }

    T average() const {
        if (m_count == 0) return T{};
        T sum = T{};
        for (size_t i = 0; i < m_count; i++) {
            sum += m_buffer[i];
        }
        return sum / static_cast<T>(m_count);
    }

    constexpr size_t size()     const { return N; }
    constexpr size_t count()    const { return m_count; }
    constexpr bool   isFull()   const { return m_count == N; }
};

// 使用——同一个模具，产生不同的具体类
RingBuffer<float, 10>  imuBuffer;    // 存储 10 个 IMU 读数
RingBuffer<fp32, 100>  motorBuffer;  // 存储 100 个电机电流值
RingBuffer<int, 5>     counterBuffer; // 存储 5 个计数值
```

### 4.2 配置化滤波器管理器

```cpp
#include <array>

// 模板参数决定：用什么滤波器、几个滤波器
template <typename FilterType, size_t FilterCount>
class FilterBank {
private:
    std::array<FilterType, FilterCount> m_filters;

public:
    void resetAll() {
        for (auto &f : m_filters) {
            f.reset();
        }
    }

    FilterType &operator[](size_t index) {
        return m_filters[index];
    }
};

// 实例化：4 个低通滤波器组，用于电机电流平滑
FilterBank<LowPassFilter<float>, 4> motorFilterBank;

// 实例化：2 个卡尔曼滤波器组，用于左右腿的状态估计
FilterBank<KalmanFilter2D, 2> legFilterBank;

// 调用
motorFilterBank[0].filterCalculate(2.5f);  // 第 1 个电机
legFilterBank[1].reset();                   // 重置右腿滤波器
```

### 4.3 常见错误

```cpp
// 错误1：模板定义和实现在不同文件，忘记显式实例化
// —— 头文件 myfilter.hpp ——
template <typename T>
class MyFilter {
public:
    T filterCalculate(T input);  // 只声明
};

// —— 源文件 myfilter.cpp ——
template <typename T>
T MyFilter<T>::filterCalculate(T input) {
    return input * 0.5f;
}
// ❌ 其他 .cpp 文件用 MyFilter<float> 时链接失败！
// 原因：模板在调用处才实例化，但调用处看不到实现。

// 解决方案 A：把实现全部放在头文件
// 解决方案 B：在 .cpp 末尾显式实例化常用类型
template class MyFilter<float>;
template class MyFilter<double>;

// 错误2：混淆"模板参数的类型"和"非类型参数的值"
template <typename int, int T>  // ❌ 第一个是 typename 但不能叫 int
class Bad {};
// 正确：
template <typename T, int N>    // ✅ T 是类型占位符，N 是值占位符
class Good {};

// 错误3：不同类型实例化出来的类不兼容
KalmanFilter<fp32, 2, 1, 0> kf2d;
KalmanFilter<fp32, 3, 1, 0> kf3d;
// kf2d = kf3d;  // ❌ 编译错误：这两个是完全不同的类型
// 即使它们看起来"差不多"，模板参数一不同就是不同的类
```

---

## 5. 本质——模板不是代码，是代码生成器

```cpp
template <typename T>
T myMax(T a, T b) { return a > b ? a : b; }
```

这行模板定义**本身不产生任何机器码**。只有当你写出 `myMax(3, 5)` 时，编译器才：

1. 推导出 `T = int`
2. 把模板中的 `T` 全部替换成 `int`
3. 生成 `int myMax(int a, int b) { return a > b ? a : b; }`
4. 编译这份生成的代码

```bash
调用                     编译器生成的等价代码
myMax(3, 5)         →    int    myMax(int a, int b)    { return a > b ? a : b; }
myMax(3.0f, 5.0f)   →    float  myMax(float a, float b)  { return a > b ? a : b; }
myMax(3.0, 5.0)     →    double myMax(double a, double b) { return a > b ? a : b; }

三份独立代码，各自针对具体类型优化。没有虚表，没有运行时分支。
```

---

## 6. 模板 vs 虚函数（多态）

| 特性 | 模板（编译期多态） | 虚函数（运行时多态） |
|------|-------------------|---------------------|
| 分发时机 | 编译期 | 运行时 |
| 运行时开销 | 零 | 虚表查找（一次指针解引用） |
| 代码体积 | 每种类型一份代码（膨胀） | 只有一份代码 |
| 类型安全 | 强（编译期检查） | 较弱（基类指针可能指向未知子类） |
| 灵活性 | 类型必须编译期已知 | 类型可以运行时动态决定 |
| GSRL 中的使用 | 滤波器、行为树组合节点 | 行为树基类 `BTNode::tick()` |

GSRL 的做法：**用模板做编译期类型适配（滤波器维度），用虚函数做运行时行为分发（行为树节点）。二者各取所长。**

---

## 7. 总结

| 要点 | 说明 |
|------|------|
| 核心概念 | 编译期的代码生成器——写一次逻辑，适配多种类型 |
| 函数模板 | `template <typename T> T func(T a);` |
| 类模板 | `template <typename T> class MyClass { T data; };` |
| 非类型参数 | `template <int N>` ——用值而非类型作参数（如 `std::array<T, N>` 的 N） |
| 默认参数 | `template <typename T = fp32>` ——不指定时使用默认值 |
| 实例化 | 调用时编译器生成具体类型的代码，不同的模板参数 = 不同的类/函数 |
| SFINAE | 利用模板替换失败不是错误，在编译期条件性地启用/禁用函数 |
| 头文件 | 模板实现通常放在头文件，否则需要显式实例化 |
| 与虚函数的关系 | 模板 = 编译期多态（零开销），虚函数 = 运行时多态（灵活），GSRL 中两者配合使用 |

---

## 8. 与 GSRL 的关系

GSRL 广泛使用模板来实现"静态多态"——在编译期确定类型和维度，避免运行时开销：

- **滤波器泛型**：`Filter<T>` 定义了统一的滤波器接口，`LowPassFilter`、`KalmanFilter`、`RLSFilter`、`MedianFilter` 通过模板参数适配不同的数值精度和维度
- **行为树容量控制**：`BTComposite<MaxChildren>` 的子节点数组大小由模板参数控制，编译期确定上限，无需动态分配
- **编译期安全**：`KalmanFilter` 用 SFINAE 在 `ControlSize = 0` 时移除控制相关的函数，避免无意义的调用
- **类型固化**：通过 `using` 别名（如 `KalmanFilter2D`）将常用模板实例化为具体类型，简化上层代码（详见 `培训/using别名.md`）
- **零开销抽象**：模板的所有"抽象"——类型推导、代码生成——都在编译期完成，运行时与手写具体类型代码无性能差异，符合嵌入式环境的性能要求

> **作者**: [Qing](https://github.com/ZhangChuqing) | **修改日期**: 2026-07-18
