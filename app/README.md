# Robot Application Architecture

> 单板机器人应用层架构设计与开发规范

## 1. Architecture Overview

### 1.1 Overall Architecture

应用层（App）位于整个机器人软件架构的最上层，负责组合底层设备、通信服务、控制算法和 RTOS 任务，形成完整的机器人应用。

整体依赖关系：

```text
                    Application
                         │
                    ┌────┴────┐
                    │   App   │
                    └────┬────┘
                         │
             ┌───────────┴───────────┐
             │                       │
        Components                Robot
             │                       │
       ┌─────┴─────┐       ┌─────────┼─────────┐
       │           │       │         │         │
     Omni       Gimbal  Control   Tasks    Configs
       │           │       │         │
       └─────┬─────┘       └────┬────┘
             │                  │
             └────────┬─────────┘
                      │
                   Modules
                      │
                   Devices
                      │
                 BSP / Boards
                      │
                    HAL
                      │
                    Libs
```

各层职责：

| Layer | Responsibility |
|---|---|
| App | 应用初始化、依赖组合、线程创建 |
| Components | 可复用算法与控制组件 |
| Robot | 当前机器人业务逻辑、配置与任务调度 |
| Modules | 可复用的功能服务，如 IMU、Referee |
| Devices | 电机、遥控器、传感器等设备驱动封装 |
| BSP / Boards | 板级硬件资源、外设配置与适配 |
| Libs | 通用数学、通信、数据结构及基础工具 |

**依赖原则：上层使用下层，下层不依赖上层。**

---

## 2. Directory Structure

```text
app/
├── CMakeLists.txt
│
├── components/
│   ├── omni/
│   │   ├── include/
│   │   └── src/
│   │
│   └── gimbal/
│       ├── include/
│       └── src/
│
└── robot/
    ├── app.cpp
    │
    ├── configs/
    │   ├── build_config.hpp.in
    │   ├── thread_config.hpp
    │   └── robot_config.hpp
    │
    ├── control/
    │   ├── control.hpp
    │   ├── control.cpp
    │   └── control_thread.cpp
    │
    ├── vision/
    │   ├── protocol.hpp
    │   ├── protocol.cpp
    │   └── vision_thread.cpp
    │
    └── tasks/
        ├── ui_thread.cpp
        ├── debug_thread.cpp
        └── system_threads.cpp
```

不要求所有功能立即创建对应文件。按照实际代码规模逐步拆分。

### 2.1 app.cpp

作为整个应用的唯一启动入口（Composition Root）。

负责：

- 初始化设备和服务。
- 初始化 Control。
- 创建 ThreadX 线程。
- 管理线程栈、线程对象等资源。
- 根据编译配置决定是否创建可选线程。
- 检查初始化及线程创建结果。

不负责：

- 控制算法。
- 协议解析。
- 线程内部的复杂业务逻辑。
- 长时间运行的控制循环。

### 2.2 components/

放置可复用的算法和控制组件。

当前主要包括：

**Omni**

- 全向轮正运动学。
- 全向轮逆运动学。
- 轮组速度分配。
- 底盘运动学相关计算。

**Gimbal**

- 云台控制算法。
- 角度与角速度控制。
- 前馈、PID 等控制逻辑。
- 必要的坐标转换。

组件应满足：

1. 不依赖具体机器人实例。
2. 不依赖 ThreadX。
3. 不直接操作 HAL、CAN、UART 等硬件接口。
4. 通过配置和状态输入完成计算。
5. 不包含具体机器人使用的硬件 ID 和业务参数。

例如，Omni 组件定义 `omni::config`，具体轮半径、底盘尺寸由 `robot/configs` 提供。

### 2.3 robot/control/

负责整车控制逻辑。

`control.cpp`：

- 整车模式管理。
- 底盘与云台控制协调。
- 调用 Omni 和 Gimbal。
- PowerManager 功率约束。
- 故障降级与控制输入仲裁。
- 生成电机目标值。

`control_thread.cpp`：

- Control 周期调度。
- 获取最新输入状态。
- 调用 Control 更新。
- 更新电机控制输出。
- 执行必要的周期统计。

Control 算法应尽量与 ThreadX、HAL 解耦，方便后续进行 PC 单元测试。

### 2.4 robot/vision/

放置当前机器人的视觉通信协议。

`protocol.hpp / protocol.cpp`：

- Vision Command 数据结构。
- Vision Telemetry 数据结构。
- 数据包编码和解码。
- 帧头、帧长度、CRC 等协议常量。
- 数据包校验和解析。

`vision_thread.cpp`：

- 视觉通信调度。
- 遥测数据发送。
- 必要的数据缓冲与状态更新。

视觉协议属于通信契约，不属于线程实现，因此不能把所有编解码逻辑直接写进 `vision_thread.cpp`。

### 2.5 robot/tasks/

放置应用层独立线程的运行逻辑。

文件划分不要求一个线程对应一个文件。

| File | Responsibility |
|---|---|
| `ui_thread.cpp` | UI 更新与图形数据发送 |
| `debug_thread.cpp` | Debug 数据打包、遥测发送 |
| `system_threads.cpp` | 简单的应用级系统任务 |

如果某个功能逐渐变得复杂，再独立拆分。

---

## 3. Thread Architecture

### 3.1 Thread Overview

| Thread / Service | Priority | Frequency Reference | Responsibility |
|---|---|---:|---|
| Control | High | 1 kHz | 整车控制 |
| IMU | High | 传感器周期 | 采样与姿态解算 |
| Remoter | High | 事件驱动或短周期 | 遥控器接收与解析 |
| Referee | Medium | 事件驱动或短周期 | 裁判系统数据解析 |
| Vision | Medium | 按通信需求 | 视觉数据接收与发送 |
| ServiceAlive | Optional | 50–100 Hz | 服务在线状态监测 |
| Debug | Optional | 10–50 Hz | 调试数据输出 |
| UI | Optional | 10–20 Hz | 裁判系统图形交互 |
| SuperCap | TBD | 按通信方式 | 超电通信 |

以上频率为初始设计参考，不是固定要求。需要结合实际执行时间、通信负载和实时性要求测量调整。

IMU、Remoter、Referee 如果已经在 Modules 中实现线程或后台服务，App 只负责初始化和启动，不重复实现。

### 3.2 Control Thread

**核心控制线程，必须存在。**

推荐执行流程：

```text
Control Thread (1 kHz)
        │
        ▼
  Read Input States
        │
        ├── IMU State
        ├── Remote State
        ├── Motor Feedback
        ├── Vision State
        ├── Referee Data
        └── SuperCap State
        │
        ▼
  Robot Control
        │
        ├── Mode Management
        ├── Chassis Control
        │      └── Omni Kinematics
        │
        ├── Gimbal Control
        │      └── Gimbal Controller
        │
        └── Power Management
        │
        ▼
  Motor Setpoints
        │
        ▼
  Motor PID Update
        │
        ▼
  CAN Transmission
```

原则：

- 不在 Control 中解析 UART、USB 协议。
- 不执行阻塞式通信。
- 不执行 Debug、UI 等低优先级工作。
- 不直接处理复杂的数据接收流程。
- 每个控制周期检查关键输入的有效性和时间戳。

Motor 暂时不需要单独创建线程。由 Control 统一完成目标更新、PID 更新和 CAN 发送。

CAN 接收可以通过中断回调更新电机反馈。

前提是现有 Motor 接口支持由 Control 主动调用。如果设备层已有独立的周期调度，必须明确唯一的输出所有者，避免重复更新。

### 3.3 IMU

由现有 IMU Module 管理。

负责：

- 传感器采样。
- DMA / SPI / UART 数据接收。
- AHRS / ESKF 姿态解算。
- 姿态和角速度状态更新。
- 数据时间戳与有效性维护。

Control 只读取最新状态，不重新实现 IMU 算法。

### 3.4 Remoter

由 Remoter Service 负责。

负责：

- DR16 等遥控器数据接收。
- DMA RX-to-IDLE 数据处理。
- 协议解析。
- 遥控器离线检测。
- 发布最新 Remote State。

接收流程：

```text
UART DMA
    │
    ▼
RX Callback
    │
    ▼
FIFO
    │
    ▼
Remoter Service
    │
    ▼
Remote State
    │
    ▼
Control
```

Remoter 只负责提供摇杆、按键和在线状态，不负责整车模式仲裁。

### 3.5 Referee

由 Referee Module 负责。

负责：

- UART DMA 接收。
- FIFO 数据消费。
- 帧解析。
- CRC8 / CRC16 校验。
- Command ID 解码。
- 裁判系统数据缓存。
- 在线状态和时间戳维护。

典型数据：

- Game Status。
- Game Robot Status。
- Power Heat Data。
- Robot Position。
- Buff / Event 等。

Referee 负责通信与数据解析，不负责底盘功率分配。

UI 产生的图形命令应交给 Referee 的统一 TX 接口发送，避免多个线程直接竞争 UART。

### 3.6 Vision

负责当前机器人与视觉上位机之间的通信。

数据流：

```text
Vision Computer
       │
       ▼
    USB RX
       │
       ▼
 BSP USB Driver
       │
       ▼
Vision Protocol
       │
       ├── Frame Parsing
       ├── CRC Validation
       └── State Update
                │
                ▼
          Vision State
                │
                ▼
          Control Thread
```

注意：

- USB 驱动已经存在 RX 工作线程时，不要重复创建一个无意义的 RX 轮询线程。
- 如果回调只负责有界组帧和解析，可以直接更新状态。
- 如果解析负载较大，则使用 FIFO / Queue 将原始数据交给独立解析任务。
- USB TX 必须有明确的所有者。
- Vision 与 Debug 共用 CDC 通道时，必须统一发送调度，不能让多个线程随意并发写入。

### 3.7 ServiceAlive

默认不需要独立线程。

如果只是周期检查各个服务的数据是否过期，可以在 Control 中降频执行。

例如：

```cpp
if (++health_counter >= 10)
{
    health_counter = 0;
    update_service_status();
}
```

Control 为 1 kHz 时，对应 100 Hz。

负责监测：

- IMU 是否在线。
- Remote 是否在线。
- Referee 是否在线。
- Vision 数据是否过期。
- Motor Feedback 是否更新。
- SuperCap 是否在线。

ServiceAlive 只发布健康状态，不直接修改 PID 或电机输出。

如果未来健康监测发展为复杂的故障诊断与恢复服务，再独立成线程，必要时放入 `system_threads.cpp`。

### 3.8 Debug

可选线程。

负责：

- Debug 数据打包。
- USB / UART 遥测发送。
- 控制周期统计。
- 运行状态记录。
- 故障信息输出。

通过 Debug Snapshot 获取数据，不直接修改 Control 内部状态。

### 3.9 UI

可选线程。

负责：

- 裁判系统图形交互。
- 底盘状态显示。
- 云台状态显示。
- 故障状态显示。
- 超电状态显示。

UI 只读取机器人状态，生成图形命令，再交给 Referee TX 服务发送。

### 3.10 SuperCap 与 PowerManager

必须区分设备通信和功率管理算法。

**SuperCap：**

- 接收超电状态。
- 解析电压、电流、能量等信息。
- 发送功率限制指令。
- 管理通信在线状态。

是否需要独立线程取决于通信方式。CAN 中断接收和周期发送通常不需要额外线程；UART DMA 等复杂通信可以使用独立任务。

**PowerManager：**

- 读取裁判系统功率限制。
- 读取缓冲能量。
- 读取超电状态。
- 计算底盘功率约束。
- 执行轮组功率分配或限幅。
- 向 Control 提供限制结果。

PowerManager 默认作为普通 C++ 业务对象，由 Control 降频调用，不单独创建线程。

---

## 4. Configuration Management

配置按职责划分，不要把所有常量放进一个文件。

### 4.1 Build Configuration

`build_config.hpp.in`：

```cpp
#pragma once

#cmakedefine01 ENABLE_UI
#cmakedefine01 ENABLE_DEBUG
```

通过 CMake `configure_file()` 生成 `build_config.hpp`。

可选功能：

- `ENABLE_UI`
- `ENABLE_DEBUG`

在应用组合层使用：

```cpp
#if ENABLE_UI
// UI initialization and thread creation
#endif
```

使用 `#if`，不要使用 `#ifdef` 判断值为 0/1 的功能开关。

关闭可选功能时，CMake 应同时排除对应源文件，避免仅关闭线程创建而仍然编译整个功能。

### 4.2 Thread Configuration

`thread_config.hpp`：

```cpp
#pragma once

#include "tx_api.h"

namespace robot::cfg::threads
{

struct thread_cfg
{
    ULONG stack_size_bytes;
    UINT priority;
    UINT preemption_threshold;
};

inline constexpr thread_cfg control{
    2048, 5, 5
};

inline constexpr thread_cfg debug{
    2048, 10, 10
};

inline constexpr thread_cfg ui{
    2048, 8, 8
};

}
```

数值仅作为接口示例，实际配置需要根据任务 WCET、栈使用情况和系统优先级规划确定。

### 4.3 Robot Configuration

`robot_config.hpp` 或按实际规模拆分：

- Motor 参数。
- Omni 几何参数。
- Gimbal 参数。
- PID 参数。
- 滤波器参数。
- 机器人运行限制。

配置值属于当前机器人实例，组件只定义配置类型。

### 4.4 Constants

不同类型的常量放在对应位置：

| Constant | Location |
|---|---|
| Pi、2Pi | `libs/math/include/constants.hpp` |
| 角度转换 | `libs/math` |
| 协议帧头、长度 | 对应 `protocol.hpp` |
| CRC 参数 | 对应通信协议 |
| 轮半径、轮间距 | `robot/configs` |
| PID 参数 | `robot/configs` |
| Thread Stack / Priority | `thread_config.hpp` |

例如：

```cpp
#include "constants.hpp"

float target = math::pi / 2.0f;
float angle = math::degrees_to_radians(90.0f);
```

不重复定义已经存在于公共 Libs 中的数学常量。

---

## 5. Communication Protocol Rules

协议定义与线程实现必须分离。

例如 Vision：

```text
robot/vision/
    protocol.hpp
    protocol.cpp
    vision_thread.cpp
```

`protocol.hpp`：

- Command / Telemetry 数据结构。
- 固定帧头。
- 固定数据长度。
- 编解码接口。

`protocol.cpp`：

- 字节序列化。
- 字节流解析。
- CRC 校验。
- 数据合法性检查。

`vision_thread.cpp`：

- 通信调度。
- 接收数据处理。
- 发送遥测数据。

注意：

1. 不要直接将 C++ 结构体内存作为协议数据包发送。
2. 必须明确字节布局、大小端序和数据长度。
3. 不同线程之间传递解析后的状态，不要反复解析同一份原始数据。
4. 协议固定参数不属于机器人可调配置。
5. 可复用协议在确实需要跨项目使用时，再下沉到 Modules。

---

## 6. Thread Management Rules

### 6.1 File Organization

不要求一个线程对应一个 `.cpp`。

| Situation | Rule |
|---|---|
| 复杂业务逻辑 | 独立文件 |
| 独立控制周期 | 独立线程实现文件 |
| 同一子系统中的短任务 | 可以合并 |
| 简单周期检查 | 优先由现有线程降频调用 |
| 已由 Modules 管理的服务 | 不在 Robot 重复实现 |

推荐：

```text
control/
    control.cpp
    control_thread.cpp

vision/
    protocol.cpp
    vision_thread.cpp

tasks/
    ui_thread.cpp
    debug_thread.cpp
    system_threads.cpp
```

不要为了形式上的整齐创建大量只有几行代码的文件。

### 6.2 Thread Resource Ownership

线程对象和 Stack 必须具有足够长的生命周期。

如果由 `app.cpp` 统一创建线程，可以在文件作用域定义：

```cpp
TX_THREAD control_thread{};

ULONG control_stack[
    cfg::threads::control.stack_size_bytes / sizeof(ULONG)
]{};

static_assert(
    cfg::threads::control.stack_size_bytes % sizeof(ULONG) == 0
);
```

使用 `ULONG` 数组可以满足 ThreadX Stack 的类型和对齐要求。

创建线程时使用：

```cpp
sizeof(control_stack)
```

避免配置值与实际 Stack 大小不一致。

每个线程的入口函数由对应功能文件实现。少量入口可以在 `app.cpp` 中前置声明；入口较多时再统一使用 `thread_entries.hpp`，不必为每个线程建立单独的启动头文件。

### 6.3 ThreadX Priority

ThreadX 优先级数字越小，优先级越高。

初步优先级层次：

| Priority Level | Threads |
|---|---|
| High | Control、IMU |
| Medium-High | Remoter |
| Medium | Referee、Vision |
| Low | ServiceAlive |
| Lowest | Debug、UI |

具体优先级需要根据实时性分析和实际执行时间确定。

默认建议：

```cpp
preemption_threshold = priority;
```

只有明确需要限制抢占时才调整 Preemption Threshold。

### 6.4 Thread Creation

`app.cpp` 应检查：

```cpp
UINT status = tx_thread_create(...);

if (status != TX_SUCCESS)
{
    // Handle startup failure
}
```

不能忽略线程创建失败。

初始化顺序必须保证 Control 启动前，所依赖的服务和共享状态已经初始化。

可以先创建为 `TX_DONT_START`，完成全部初始化和线程创建后，再统一 Resume，避免线程提前执行。

---

## 7. Inter-thread Communication

线程之间不应随意共享和修改复杂对象。

基本原则：

**一个状态应当有明确的写入者，其他线程通过受控接口读取。**

推荐数据流：

```text
IMU Thread ───────┐
Remoter Service ──┤
Referee Service ──┤
Vision Service ───┼──> Shared State / Topic
Motor Feedback ───┤             │
SuperCap ─────────┘             ▼
                           Control Thread
                                 │
                                 ▼
                          Debug Snapshot
                                 │
                                 ▼
                           Debug Thread
```

根据数据性质选择同步机制：

| Mechanism | Use Case |
|---|---|
| Atomic | 简单原子状态、标志 |
| Mutex | 互斥访问共享资源 |
| Semaphore | 事件通知、计数同步 |
| Event Flags | 多种事件状态 |
| Message Queue | 传递完整消息 |
| FIFO / SPSC | 字节流与单生产者单消费者数据 |
| Snapshot / Double Buffer | 最新状态共享 |

注意：

- 中断回调中不能使用阻塞式 Mutex。
- 中断回调只执行必要的数据复制和通知。
- USB、UART、CAN 等资源必须有明确的访问所有者。
- 最新值 Topic 不保证每条消息都交付。
- Control 必须独立检查输入数据的有效性和时间戳，不能只依赖 ServiceAlive。

---

## 8. CMake Organization

应用层使用 `INTERFACE` Target 组织依赖与源文件。

`app/CMakeLists.txt`：

```cmake
option(ENABLE_UI "Enable UI thread" ON)
option(ENABLE_DEBUG "Enable Debug thread" OFF)

add_library(robot_app INTERFACE)

target_compile_features(robot_app INTERFACE
    cxx_std_17
)

target_include_directories(robot_app INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(robot_app INTERFACE
    modules
    devices
)

target_sources(robot_app INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/robot/app.cpp"

    "${CMAKE_CURRENT_SOURCE_DIR}/robot/control/control.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot/control/control_thread.cpp"

    "${CMAKE_CURRENT_SOURCE_DIR}/robot/vision/protocol.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot/vision/vision_thread.cpp"
)

if(ENABLE_UI)
    target_sources(robot_app INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/robot/tasks/ui_thread.cpp"
    )
endif()

if(ENABLE_DEBUG)
    target_sources(robot_app INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/robot/tasks/debug_thread.cpp"
    )
endif()
```

如果使用 `configure_file()` 生成 `build_config.hpp`，应将生成目录也加入 `target_include_directories()`。

根目录：

```cmake
add_subdirectory(devices)
add_subdirectory(modules)
add_subdirectory(app)

target_link_libraries(${PROJECT_NAME} robot_app)
```

依赖原则：

- App 自己声明直接依赖的 Modules、Devices 和 Components。
- 根目录只需要链接 `robot_app`。
- 不重复维护 App 的内部依赖。
- 不为了 App 创建不必要的 STATIC 库。

`INTERFACE + target_sources` 使应用源文件直接参与最终固件编译，也避免依赖静态库归档提取机制来引入强定义的 `app_start()`。

可复用 Components 可以分别建立独立的 INTERFACE Target，供其他机器人应用单独使用。

---

## 9. Development Rules

### 9.1 Responsibility Boundaries

| Code | Allowed Responsibility |
|---|---|
| `app.cpp` | 初始化、线程创建、资源组合 |
| `control.cpp` | 整车控制与模式管理 |
| `control_thread.cpp` | 控制周期调度 |
| `vision/protocol.cpp` | 视觉协议编解码 |
| `vision_thread.cpp` | 视觉通信调度 |
| `ui_thread.cpp` | UI 更新与图形发送 |
| `debug_thread.cpp` | 调试数据输出 |
| `components` | 可复用算法 |
| `configs` | 参数与功能配置 |

### 9.2 Avoid These Practices

- 不要把所有业务逻辑放入 `app.cpp`。
- 不要将每个短小任务强制拆成独立文件。
- 不要让 Control 直接解析 USB / UART 数据包。
- 不要让 Debug 和 UI 修改 Control 内部状态。
- 不要让 Components 依赖 ThreadX 或 HAL。
- 不要重复定义公共数学常量。
- 不要把协议常量和机器人可调参数混在一起。
- 不要为简单周期检查创建不必要的线程。
- 不要让多个线程直接竞争同一个通信发送接口。
- 不要假设所有服务上电后立即在线。
- 不要忽略线程创建、通信失败和数据超时。

### 9.3 Reliability

Control 必须考虑以下情况：

- IMU 尚未完成初始化。
- Remote 离线。
- Referee 数据过期。
- Vision 丢失目标或数据过期。
- Motor Feedback 超时。
- SuperCap 通信异常。
- 可选线程创建失败。
- 控制周期超时。

核心控制链路不能依赖 Debug、UI 等可选功能正常运行。

对不同故障制定明确的降级策略，而不是简单地将所有电机输出置零。

---

## 10. Final Design Principles

最终架构遵循以下原则：

1. **App 负责组合，Control 负责决策。**
2. **Components 负责可复用算法，不负责硬件和线程调度。**
3. **Modules 负责可复用服务，Robot 不重复实现已有功能。**
4. **Tasks 负责执行和调度，不承载不必要的业务逻辑。**
5. **Configs 管理配置，Protocol 管理通信契约。**
6. **线程数量由实时性需求决定，不由功能数量决定。**
7. **文件按照职责拆分，不强制一个线程一个文件。**
8. **共享状态通过明确的接口和同步机制传递。**
9. **可选功能由 CMake 和应用入口统一管理。**
10. **核心控制链路保持确定性、可测试性和故障隔离。**

### Architecture Summary

```text
libs
  └── 提供基础能力

devices / modules
  └── 提供硬件与通信服务

components
  └── 提供可复用算法

robot/configs
  └── 提供当前机器人的配置

robot/control
  └── 决定机器人应该做什么

robot/tasks
  └── 决定功能何时执行

robot/app.cpp
  └── 负责初始化和组合整个机器人应用
```

**核心目标：保持整车控制链路简单、可预测，同时让 Omni、Gimbal 等算法可以脱离当前机器人应用独立复用。**