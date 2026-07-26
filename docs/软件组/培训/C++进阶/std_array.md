# std::array

`std::array<T, N>` 是 C++11 引入的固定大小数组容器。它的定位很清楚：把 C 风格数组零额外开销的静态分配，和现代容器的安全性（边界检查、`size()` 方法）合到一起。要理解它的价值，先看 C 风格数组的短板。

## C 数组的短板与 std::array 的补法

```cpp
// 方式 A：C 风格数组
float motorPositions[4] = {0.0f, 0.0f, 0.0f, 0.0f};
```

这样一个数组用起来有四处不便，彼此独立又都很常见：

| 痛点 | 具体表现 |
|------|----------|
| **不知道自己的大小** | `sizeof(arr) / sizeof(arr[0])` 容易写错，传给函数后退化为指针后完全失效 |
| **不能直接赋值** | `arr2 = arr1;` 编译失败，只能用 `memcpy` |
| **不能按值传递/返回** | 函数参数中退化为指针，丢失大小信息 |
| **没有边界检查** | `arr[10]` 越过数组末尾，静默地破坏相邻内存 |

`std::array` 把它包成一个类，这四点随之解决：

```cpp
#include <array>

// 方式 B：std::array
std::array<float, 4> motorPositions = {0.0f, 0.0f, 0.0f, 0.0f};
```

关键在于，这层包装不带来任何运行时代价。`std::array` 的内存布局和 C 数组完全一致，数据同样紧挨着放在栈上，大小信息和成员函数都在编译期处理，一条多余指令都不产生。它是一次零开销抽象（zero-overhead abstraction），唯一多出来的东西，是在类型系统里携带了大小信息。

```bash
          C 风格数组                          std::array
┌─────────────────────────┐        ┌─────────────────────────┐
│ float[4]                │        │ std::array<float, 4>    │
│  ├── [0] 0.0f           │        │  ├── _M_elems[0] 0.0f   │
│  ├── [1] 0.0f           │        │  ├── _M_elems[1] 0.0f   │
│  ├── [2] 0.0f           │        │  ├── _M_elems[2] 0.0f   │
│  ├── [3] 0.0f           │        │  ├── _M_elems[3] 0.0f   │
│  └── 没有 size()        │        │  └── size()  → 4        │
│      不能赋值             │        │      可以直接赋值        │
│      退化为指针           │        │      传递时不退化        │
└─────────────────────────┘        └─────────────────────────┘

内存布局完全相同——数据紧挨着放在栈上，没有任何额外开销。
区别在于 std::array 在类型系统中携带了大小信息。
```

## 基本用法

声明时把元素类型和大小作为模板参数写出，初始化方式和普通数组类似。要注意默认初始化下元素值是不确定的，这一点与 C 数组一致，需要清零时用 `fill`：

```cpp
#include <array>

// 三种初始化方式
std::array<int, 3> a1 = {1, 2, 3};      // 列表初始化（C++11）
std::array<int, 3> a2{1, 2, 3};         // 直接列表初始化
std::array<int, 3> a3;                  // 默认初始化（元素值不确定，与 C 数组行为一致）
a3.fill(0);                             // 全部填 0
```

常用操作里，`[]` 和 C 数组一样不检查边界，`at()` 则会在越界时抛出 `std::out_of_range`；`size()`、`empty()` 都是编译期常量，`data()` 用来拿到原始指针对接 C API。赋值和比较这些 C 数组做不到的操作，`std::array` 直接支持：

| 操作 | 代码 | 说明 |
|------|------|------|
| 获取大小 | `arr.size()` | 返回 `N`，编译期常量 |
| 访问元素 | `arr[i]` | 不检查边界（与 C 数组行为一致） |
| 带检查的访问 | `arr.at(i)` | 越界时抛出 `std::out_of_range` |
| 首元素 | `arr.front()` | 等价于 `arr[0]` |
| 末元素 | `arr.back()` | 等价于 `arr[N-1]` |
| 获取原始指针 | `arr.data()` | 返回 `T*`，传给需要 C 数组的 API |
| 全部填充 | `arr.fill(value)` | 所有元素设为 `value` |
| 判空 | `arr.empty()` | 始终返回 `false`（`N > 0` 时）或 `true`（`N == 0` 时），编译期确定 |
| 赋值 | `arr2 = arr1;` | ✅ 支持！逐元素拷贝 |
| 比较 | `arr1 == arr2` | ✅ 支持！逐元素比较 |

遍历有三种写法，范围 for 循环最简洁，也有传统下标和迭代器两种：

```cpp
std::array<float, 4> currents = {2.5f, 3.1f, 1.8f, 4.2f};

// 方式 A：基于范围的 for 循环（推荐）
for (float c : currents) {
    // 处理 c
}

// 方式 B：传统下标
for (size_t i = 0; i < currents.size(); i++) {
    currents[i] *= 1.1f;
}

// 方式 C：迭代器
for (auto it = currents.begin(); it != currents.end(); ++it) {
    *it *= 1.1f;
}
```

## GSRL 中的用法

`std::array` 在 GSRL 里最贴切的场景，是大小在编译期已知、运行时不变的定长数据。卡尔曼滤波器就是这样一个例子。`KalmanFilter` 是模板类，观测维度 `MeasSize` 在编译期确定，几个映射数组的大小恰好依赖这个模板参数。源码位于 `GSRL/Algorithm/inc/alg_filter.hpp`：

```cpp
template <typename T, int StateSize, int MeasSize, int ControlSize>
class KalmanFilter : public Filter<T>
{
    // ...
    // 测量与状态的映射关系，大小由模板参数 MeasSize 决定
    std::array<int, MeasSize> m_measurementMap;   // 如 {1, 1, 4}
    std::array<T, MeasSize>   m_measurementDegree; // 如 {1.0f, 1.0f, 1.0f}
    std::array<T, MeasSize>   m_rDiagonalElements; // 如 {0.5f, 0.3f, 0.1f}
};
```

```bash
当 MeasSize = 3 时，这三个 array 的内存布局：

  m_measurementMap:     [1] [1] [4]          ← 3 个 int，12 字节
  m_measurementDegree:  [1.0][1.0][1.0]      ← 3 个 T，12 字节（T=float）
  m_rDiagonalElements:  [0.5][0.3][0.1]      ← 3 个 T，12 字节

全部在 KalmanFilter 对象内部，不涉及堆分配。
```

这里选 `std::array` 而非 `std::vector`，理由都落在"编译期已知"这一点上：`MeasSize` 是模板参数，不需要 `vector` 的动态扩容；`std::array` 的存储完全内嵌在 `KalmanFilter` 对象里，避免堆分配，契合嵌入式环境对实时性的要求；类型携带大小信息，编译器还能做循环展开一类的优化。

配置这些参数的接口也用 `std::array` 传入，通过 `const &` 避免拷贝（虽然数组本身很小，按值传也无妨）：

```cpp
// alg_filter.hpp 中的接口
void setDynamicAdjustmentParams(
    const std::array<int, MeasSize> &measurementMap,
    const std::array<T, MeasSize>   &measurementDegree,
    const std::array<T, MeasSize>   &rDiagonalElements
);
```

`std::array` 作为函数参数不会退化为指针，调用者传入的数组大小必须在编译期与 `MeasSize` 匹配，否则编译失败——这是 C 风格数组给不了的类型安全。

## 在自己的代码里用 std::array

电机电流监控是个直观的例子。电机数量固定，用 `std::array` 存读数，再配合 STL 算法查超标和求最大值，比手写循环清爽：

```cpp
#include <array>
#include <algorithm>

class MotorMonitor
{
public:
    static constexpr size_t MOTOR_COUNT = 4;
    static constexpr float  MAX_CURRENT = 10.0f;

    // 存储 4 个电机的电流读数
    std::array<float, MOTOR_COUNT> currents;

    MotorMonitor() {
        currents.fill(0.0f);
    }

    // 检查是否有电机电流超标
    bool hasOvercurrent() const {
        return std::any_of(
            currents.begin(), currents.end(),
            [](float c) { return c > MAX_CURRENT; }
        );
    }

    // 获取最大电流值
    float maxCurrent() const {
        return *std::max_element(currents.begin(), currents.end());
    }

    // 重置全部读数
    void reset() {
        currents.fill(0.0f);
    }
};
```

管理成组的 PID 参数也是一样的思路——内环加外环共两组，数量固定，用 `std::array` 存储后可以直接整组赋值：

```cpp
#include <array>

// 管理一组 PID 的参数——内环 + 外环共 2 组，每组 3 个参数
class PIDManager
{
public:
    struct PIDParams {
        float Kp, Ki, Kd;
    };

    static constexpr size_t PID_COUNT = 2;

    // 用 std::array 存储固定数量的 PID 参数组
    std::array<PIDParams, PID_COUNT> params;

    PIDManager() {
        // 全部初始化为零
        params.fill({0.0f, 0.0f, 0.0f});
    }

    // 批量设置参数
    void setParams(const std::array<PIDParams, PID_COUNT> &newParams) {
        params = newParams;  // ✅ 直接赋值，逐元素拷贝
    }
};
```

使用中容易踩的坑，多半和"大小固定在编译期"这条底线有关：初始化元素个数要对得上，大小必须是编译期常量，不能像 `vector` 那样 `push_back`，默认初始化也不保证清零。

```cpp
// 错误1：初始化元素个数不匹配
std::array<int, 3> arr1 = {1, 2};         // 第三个元素是 0（值初始化）
std::array<int, 3> arr2 = {1, 2, 3, 4};  // ❌ 编译错误：太多了

// 错误2：尝试用变量指定大小
int n = 10;
// std::array<int, n> arr;  // ❌ 编译错误：N 必须是编译期常量

// 正确：用 constexpr 或字面量
constexpr int N = 10;
std::array<int, N> arr3;                  // ✅
std::array<int, 10> arr4;                 // ✅

// 错误3：与 vector 混淆
std::array<int, 5> arr5;
arr5.push_back(42);                       // ❌ 编译错误：array 没有 push_back
// array 大小固定，不能动态添加元素

// 错误4：忘记 array 的元素默认不初始化（与 C 数组一致）
std::array<int, 5> arr6;                  // 元素值不确定！
arr6.fill(0);                             // 如果手写 for 循环也比直接假设全零安全
```

## 三者之间怎么选

`std::array` 夹在 C 数组和 `std::vector` 之间：它保留了 C 数组的零开销和栈上存储，又补齐了赋值、按值传递、边界检查这些能力；而与 `std::vector` 的分界线，就在大小是否需要在运行时改变。

| 特性 | `T[N]`（C 数组） | `std::array<T, N>` | `std::vector<T>` |
|------|-----------------|--------------------|--------------------|
| 大小 | 编译期固定 | 编译期固定 | 运行时可变 |
| 存储位置 | 栈 / 静态存储区（取决于声明位置） | 栈 / 静态存储区 | 堆（内部数据） |
| `.size()` | ❌ | ✅（返回 N） | ✅（返回当前元素数） |
| 赋值 `a = b` | ❌ | ✅ | ✅ |
| 按值传递 | ❌（退化为指针） | ✅ | ✅ |
| 传给 C API | 直接传数组名 | `arr.data()` | `vec.data()` |
| 边界检查 | ❌ | `.at()` 有检查 | `.at()` 有检查 |
| 额外内存开销 | 无 | 无 | 3 个指针（约 24 字节） |

## 为什么它是零开销

看一眼编译器眼里的 `std::array`，就明白零开销从何而来。它内部只有一个数据成员，就是一个 C 风格数组，其余全是 `inline` 函数：

```cpp
std::array<int, 4> arr = {1, 2, 3, 4};
```

```cpp
template <typename T, size_t N>
struct array {
    T _M_elems[N];  // 唯一的数据成员——就是一个 C 风格数组

    // 以下全是 inline 函数，编译期展开后消失：
    constexpr size_t size()     const { return N; }
    constexpr bool   empty()    const { return N == 0; }
    T& operator[](size_t i)           { return _M_elems[i]; }
    T* data()                         { return _M_elems; }
    // ...
};
```

`_M_elems` 是唯一的数据成员，所有方法都 `inline` 且在编译期求值，因此 `sizeof(std::array<T, N>)` 正好等于 `sizeof(T) * N`，与 C 数组占用完全相同；`arr.size()` 在编译期就被替换成字面量 `N`，运行时一条指令都不生成。

## 与 GSRL 的关系

在 GSRL 的嵌入式环境里，`std::array` 的静态分配特性与项目对实时性、内存可预测性的要求高度契合。`KalmanFilter` 用它存储与观测维度相关的映射数组，大小由模板参数决定，数据内嵌在滤波器对象中，不触发堆分配。电机控制、传感器滤波这类场景的数据维度通常在编译期就定死（4 个电机、3 轴 IMU），`std::array` 因而优于 `std::vector`。它还能配合 `std::any_of`、`std::max_element` 等 STL 算法减少手写循环，与 C API 交互时用 `.data()` 取原始指针，兼容性也没有损失。

> **作者**: [Qing](https://github.com/ZhangChuqing) | **修改日期**: 2026-07-26
