# 模板（Template）

模板是 C++ 的泛型编程机制。它的根本，不是运行时的多态，而是编译期的代码生成：写一次逻辑，编译器在编译期按实际用到的类型自动生成对应的代码，运行时不产生任何额外开销。理解了这一点，模板的种种规则和用法都能顺着推出来。

## 没有模板时的困境

假设需要一个取最大值的函数。对 `int` 和 `float`，逻辑完全相同，但类型不同，就不得不各写一份。

```cpp
// 不做泛型：每种类型写一份
int   maxInt(int a,   int b)   { return a > b ? a : b; }
float maxFloat(float a, float b) { return a > b ? a : b; }
// double、long、unsigned……每加一种类型就多一份代码
```

这样做不仅冗余，还容易出错：`maxFloat(3, 5)` 能工作但发生了隐式转换，类型不安全；`maxInt(3.0, 5.0)` 会把小数截断成整数，精度丢失。有 N 种类型就有 N 份几乎相同的代码，是维护上的噩梦。

函数模板把类型抽出来作为占位符，一份代码即可适配所有类型。

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

每当用一种新类型调用模板，编译器就为这种类型生成一份专属代码，与手写的同类型函数完全等价，没有虚函数调用、也没有运行时分发。这就像刻一枚印章，`T` 是留空的部分；调用时把 `T` 填成具体类型，盖出来的才是那份真正的代码。盖三次产生三份代码，不盖就什么也不产生——这正是零开销原则（zero-overhead principle）的体现。

## 语法基础

函数模板的形式是在函数前面加一行 `template <typename 占位符名>`，占位符随后在参数和返回值里使用。

```cpp
template <typename T>
T add(T a, T b) {
    return a + b;
}

add(1, 2);       // T = int
add(1.5f, 2.5f); // T = float
```

模板参数可以有多个，各自独立推导：

```cpp
// 两个不同类型的参数
template <typename T1, typename T2>
auto multiply(T1 a, T2 b) -> decltype(a * b) {
    return a * b;
}

multiply(3, 4.5f);   // T1 = int, T2 = float, 返回 float
multiply(2.0, 3);    // T1 = double, T2 = int, 返回 double
```

类同样可以模板化。下面的 `Box` 能存放任意类型的单个值：

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

类模板本身不占内存、不产生代码，只有用到某个具体类型时编译器才生成对应的类。需要注意的是，`Box<int>` 和 `Box<float>` 是两个完全独立的类型，彼此之间没有继承关系，也不能互相赋值。

模板参数不一定是类型，也可以是编译期就能确定的常量值，比如整数、枚举、指针。这类非类型参数常用来固定数组大小：

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

标准库的 `std::array<T, N>` 正是这样实现的，`N` 就是非类型模板参数（参考 `培训/std_array.md`）。

模板参数还能带默认值，不显式指定时就用默认值，多个参数都可以这样处理：

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

template <typename T = fp32, size_t WindowSize = 10>
class MovingAverageFilter : public Filter<T> {
    T m_buffer[WindowSize];  // 大小由模板参数决定
    // ...
};

MovingAverageFilter<>        maf1;  // T = fp32, WindowSize = 10
MovingAverageFilter<float, 5> maf2;  // T = float, WindowSize = 5
```

## GSRL 中的模板

GSRL 用模板实现的是"静态多态"：在编译期确定类型和维度，把抽象的代价全部消化在编译阶段，运行时不留负担。几个典型场景可以看出它的用法。

滤波器基类用模板把数值类型抽象出来，声明"我能处理任意类型 T 的数据"（源码位置：`GSRL/Algorithm/inc/alg_filter.hpp`）：

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

所有具体滤波器都继承自 `Filter<T>`，继承时把 `T` 原样传给基类，从而保证输入输出类型一致：`LowPassFilter<T>`、`KalmanFilter<T, ...>`、`RLSFilter<T, ...>` 都是 `public Filter<T>`。其中 `fp32` 就是 `float`，只是类型名不同。

卡尔曼滤波器进一步用非类型参数描述维度（源码位置同上）：

```cpp
template <typename T = fp32, int StateSize = 1, int MeasSize = 1, int ControlSize = 0>
class KalmanFilter :
    public Filter<T>
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

这里 `T` 是数值精度（`fp32`/`float`/`double`），`StateSize`、`MeasSize`、`ControlSize` 三个都是 `int` 而非 `typename`——它们不是类型，而是决定矩阵尺寸的整数值。状态向量维度对应位置加速度这类量的个数，观测维度对应传感器读数的个数，控制维度为 0 表示没有控制输入。不同的维度组合生成完全不同的类，例如 `KalmanFilter<fp32, 2, 1, 0>` 是位置-速度的 2D 滤波器，`KalmanFilter<fp32, 3, 1, 0>` 则多了一维加速度。工程里再用 `using` 别名把常用组合固化成好记的名字（参考 `培训/using别名.md`）：

```cpp
using KalmanFilter1D = KalmanFilter<fp32, 1, 1, 0>;
using KalmanFilter2D = KalmanFilter<fp32, 2, 1, 0>;
using KalmanFilter3D = KalmanFilter<fp32, 3, 1, 0>;
```

行为树的组合节点用整数模板参数控制子节点数组的大小（源码位置：`GSRL/Algorithm/inc/alg_behavior_tree.hpp`）：

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

`BTSequence<8>` 的子节点数组是 `BTNode*[8]`，`BTSequence<16>` 则是 `BTNode*[16]`。数组大小在编译期确定，整个节点对象放在栈上，不做堆分配；不同的 `MaxChildren` 值产生互不兼容的类，这本身就是编译期类型安全的一部分。

模板还能在编译期决定某个函数是否存在。当 `ControlSize = 0`（无控制输入）时，`ControlMatrix` 是零列矩阵，`m_control * control` 在语义上没有意义，带控制输入的 `predict` 就不该存在。GSRL 用 SFINAE 让它在这种情况下直接消失（源码位置：`GSRL/Algorithm/inc/alg_filter.hpp`）：

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

SFINAE（Substitution Failure Is Not An Error，替换失败不是错误）的意思是：当模板参数替换失败时，编译器不报错，只是把这个候选函数从可选集合里悄悄剔除。于是对 `KalmanFilter<fp32, 2, 1, 0>`（`ControlSize = 0`），`kf.predict()` 正常调用，而 `kf.predict(controlVec)` 会编译报错，因为那个带参数的版本根本不存在；对 `KalmanFilter<fp32, 2, 1, 2>`，两个版本同时可用。与其在运行时才发现调用无意义，不如在编译期让这个重载彻底不出现——调用者在 IDE 里都看不到它，从根本上堵住了误用。

## 在 GSRL 场景中动手写

一个通用的环形缓冲区，类型和容量都作为模板参数，适合缓存固定数量的传感器读数：

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

模板参数甚至可以是另一个模板实例化出来的类型。下面的滤波器组用两个参数决定"用什么滤波器"和"几个滤波器"：

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

## 几个容易踩的坑

模板最典型的错误是把定义和实现拆到不同文件。模板要在调用处才实例化，而调用处必须看得到实现，否则链接失败。

```cpp
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
```

另外两类错误是混淆类型占位符和值占位符，以及误以为参数相近的实例化类型可以互换：

```cpp
// 混淆"模板参数的类型"和"非类型参数的值"
template <typename int, int T>  // ❌ 第一个是 typename 但不能叫 int
class Bad {};
template <typename T, int N>    // ✅ T 是类型占位符，N 是值占位符
class Good {};

// 不同类型实例化出来的类不兼容
KalmanFilter<fp32, 2, 1, 0> kf2d;
KalmanFilter<fp32, 3, 1, 0> kf3d;
// kf2d = kf3d;  // ❌ 编译错误：这两个是完全不同的类型
// 模板参数一不同就是不同的类，哪怕它们看起来"差不多"
```

## 模板与虚函数

回到最开始那句话：模板是编译期的代码生成器，而不是运行时的分发机制。这一点决定了它和虚函数的分工。虚函数在运行时通过虚表查找决定调用哪份实现，代价是一次指针解引用，好处是类型可以运行时才确定；模板则在编译期就把一切定死，运行时零开销，代价是每种类型各生成一份代码，且类型必须编译期已知。

| 特性 | 模板（编译期多态） | 虚函数（运行时多态） |
|------|-------------------|---------------------|
| 分发时机 | 编译期 | 运行时 |
| 运行时开销 | 零 | 虚表查找（一次指针解引用） |
| 代码体积 | 每种类型一份代码（膨胀） | 只有一份代码 |
| 类型安全 | 强（编译期检查） | 较弱（基类指针可能指向未知子类） |
| 灵活性 | 类型必须编译期已知 | 类型可以运行时动态决定 |
| GSRL 中的使用 | 滤波器、行为树组合节点 | 行为树基类 `BTNode::tick()` |

GSRL 让两者各取所长：用模板做编译期的类型和维度适配（滤波器的精度与状态维度、行为树的容量上限），用虚函数做运行时的行为分发（行为树节点的 `tick()`）。模板承担的抽象——类型推导、代码生成、SFINAE 的条件裁剪——全部在编译期完成，运行时与手写的具体类型代码没有性能差异，正契合嵌入式环境对性能的要求。

> **作者**: [Qing](https://github.com/ZhangChuqing) | **修改日期**: 2026-07-26
