# Robot Application Architecture

> 机器人应用层架构设计与开发规范；控制数据不绑定上下板或单板部署。

## 当前实现与数据依赖

`robot/control/control.hpp` 集中定义业务数据和接口；`control.cpp` 只组织两条控制链。
Remote、Gimbal、Shooter、Chassis 业务函数及三个任务入口已实现。
硬件采集、实际电机发送和视觉协议接入尚未完成，当前默认参数也不会使能电机。
下面第 1～31 节保留为架构设计参考，其中示例文件、API、CAN 编号不代表当前接线。

```text
逐帧遥控输入 → TaskRemoter → remote::command
                                  ├→ TaskGimbal  → Gimbal → 本周期 aim → Shooter
                                  └→ TaskChassis → Chassis → 四轮速度
IMU / Vision / 云台与发射反馈 ─────→ TaskGimbal
底盘电机反馈 ─────────────────────→ TaskChassis
```

- `gimbal_control::update(gimbal_input)` 返回 `gimbal_output`，包含云台、发射命令和瞄准状态。
  Shooter 使用本周期的 `gimbal::status`，不会读取另一个线程或上一周期的瞄准结果。
- `chassis_control::update(chassis_input)` 独立返回 `chassis::output`，不依赖云台输出。
  当前实现正方形 X 型全向轮逆运动学、统一比例限速及安装方向转换；轮速闭环和功率限制尚未接入。
- `remote::controller` 仅由 Remote 任务持有，两个控制任务各自持有控制器实例和命令副本，
  不共享 PID 状态、电机对象，也不重复处理按键边沿。
- `gimbal::controller` 通过两个非拥有的 `axis_control&` 调用 Yaw/Pitch 控制器；
  被引用对象必须活得更久。当前 `pid_axis` 单独实现串级 PID、预测和前馈，替换控制律不需要改 Gimbal。
  `axis_ref` 包含位置边界，替换控制律也须遵守；预测后的 Pitch 目标和向外前馈不能越过边界。
- 所有电机结果均为 `motor_command`，包含模式和命令值，默认 `relax`，不包含上下板协议。
  Roll 仅传递 `roll_requested`，实际 Roll 电机保持 `relax`。

### 遥控映射

| 输入 | 行为 |
|---|---|
| pause 按下沿 | 解锁/失能；重置摩擦轮、自瞄、Roll 请求 |
| fn1 按下沿 | 切换 Roll 请求，仅留接口 |
| fn2 按下沿 | 切换摩擦轮 |
| 拨轮离开中位 | 手动单发一次；回中后才接受下次动作，自瞄时忽略 |
| trigger（`reading.button`）按下沿 | 切换手动/自瞄 |
| 右摇杆、鼠标 | 沿用原来的 Yaw/Pitch 方向；摇杆是角速度，鼠标是新帧位移 |
| WASD、左摇杆 | 平移；SHIFT 或左拨杆上位进入旋转，旋转时忽略左摇杆横向量 |

底盘采用 x 向前、y 向左、wz 逆时针的坐标定义，因此 A/左移为正 y；
旧下板报文中向右为正的数值符号不直接沿用。旧腿长、跳跃等异构底盘命令没有套到全向轮底盘上。
旋转速度、键盘加速度及全部比例参数集中在 `cfg::remote`。

自瞄开火必须同时满足视觉目标新鲜且可达、`vision.fire`、Yaw/Pitch 误差范围、摩擦轮到速和热量余量。
手动单发也检查摩擦轮和热量；未满足条件的单发不排队等待。
拨弹按连续输出轴角度判断完成，卡住超过配置时间停止；持续自瞄请求必须撤销后才能重新尝试。
反转保留业务接口，但没有擅自绑定按键。退出自瞄会取消尚未完成的自动拨弹。

### 线程、采样与同步约定

- `app.cpp` 先初始化 topics，再创建三个 `TX_DONT_START` 任务，全部成功后启动；
  失败不进行部分启动。空的 Vision、Debug、UI 入口不加入构建，也不创建无业务的线程。
  现有 `build_config.hpp` 中这些开关仍需在对应实现完成后接入。
- 调度参数在 `cfg::gimbal_thread/chassis_thread/remote_thread`，默认周期 1 tick。
  这是名义调度间隔，不是硬实时测量结果；`dt_s` 使用实际经过时间。
- `now_ms`、所有 `received_ms` 使用 `HAL_GetTick()` 的同一毫秒时钟。
  读取缓存不能刷新源数据时间；电机快照时间与 IMU、Vision 各自接收时间分别保留。
- `remote_inputs`、`gimbal_inputs`、`chassis_inputs` 是当前接入点，每个 topic 仅一个写入者。
  输入发布者在任务上下文发布完整副本，不能在 ISR 使用依赖 mutex 的 `msg::topic`。
  Gimbal/Chassis 任务覆盖输入中的 `remote/now_ms/dt_s`，其余字段由对应采集方负责。
- `msg` 的 mutex 只保护快照复制，不跨越控制计算；忙时保留旧快照，由时间检查判断能否继续使用。
  `gimbal_outputs/chassis_outputs` 目前用于结果观察，发布竞争允许跳过一次；
  接电机时应在各自控制线程直接应用本周期输出，不将这些无时戳的缓存当成可靠电机发送队列。
- 遥控 `sequence` 必须逐解码帧递增，不能每次读缓存都加一。
  原始 latest-value 输入若跳帧，会失能并要求重新解锁，因为丢失的帧可能包含 pause。
  接入时应保证逐帧处理；现有 `receiver::read()` 还缺帧序号，需要补齐后接入。
- Remote 输出使用累计鼠标位移和单发计数，两条下游控制链频率不同时不会重复消费同一事件。
  新解锁会话重新建立累计量基线；延迟期间多个单发只合并为一次，不恢复后补打一串。
  这不意味着原始遥控 latest-value topic 能保存每一帧。
- IMU 使用 `imu::attitude::state`。姿态、陀螺仪和视觉角度必须先统一到相同坐标系；
  当前 Gimbal 按 ZYX 姿态把机体系角速度换成 Euler 角速度，不能把原始 gyro 分量直接当作轴角速度。
  BMI088/DMIMU 的轴向和安装关系仍需采集方统一。
- 电机采集方在 `health_tick(dt)` 后填写 `motor_feedback::state`。
  发射的 `heat_valid` 必须同时检查裁判热量包、热量上限和冷却参数的有效性及超时，
  `heat_sequence` 只在新热量观测时变化。本地按开始拨弹预扣热量，取消/失能不退还未确认的预扣。
- 角度、角速度、角加速度单位为 rad、rad/s、rad/s²；平移为 m/s。
  `pid_axis` 配置采用连续时间增益，调用现有 PID 时转换为 `ki * dt_s`、`kd / dt_s`，
  不能不换算就复制旧的离散 `ki/kd`。
  `current/voltage` 保留驱动原始命令单位，不是 A/V；执行端须选电机支持的模式。
  当前 DJI 驱动不支持 `torque` 模式。

### 必要保护与待接入项

固定参数只在控制器构造时检查；运行时保留数据失联/过期、周期超限、非有限外部数据、
机械目标/输出限幅，以及发射到速、热量和卡弹条件。保护处注明原因，
不检查本模块不使用的反馈字段，不在 Gimbal、Shooter 内重复检查控制周期。
未解锁或数据失效返回 `relax`；视觉失效回到手动目标，不允许自动开火。

`robot/configs/robot_config.hpp` 中未确认的增益、速度、几何限制保持未使能默认值，
需要实测填写，不能直接上电试转。当前配置为 Gimbal FDCAN1、Chassis FDCAN2；
Roll/左摩擦轮的 ID，以及底盘左前/左后的 ID 存在重复，须结合电机型号确认和修正后才能接线。
IMU/遥控/视觉/裁判接入、电机模式适配、轮速闭环和功率限制、实际 CAN 发送仍是后续工作；
当前没有新增统一 Motor Thread。

### 验证

业务测试使用真实生产类型和源文件，不需要连接硬件：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File app/tests/run_control_tests.ps1
cmake --build --preset h723_mc02
```

测试脚本需要已配置的固件编译数据库和本机 g++；可通过 `-Compiler`、`-CompileDatabase` 指定路径。
覆盖遥控边沿、累计事件、串级 PID/前馈、Pitch 限位、视觉开火许可、热量/卡弹、
两条控制链独立失效与毫秒计数回绕。主机测试和编译不替代上板的方向、限位、时序及实机安全验证。

## 1. Architecture Overview

### 1.1 Overall Architecture

应用层（App）位于整个机器人软件架构的最上层，负责组合底层设备、通信服务、控制算法和 RTOS 任务，形成完整的机器人应用。

整体依赖关系：

```text
                           Application
                                │
                              App
                                │
                 ┌──────────────┼──────────────┐
                 │              │              │
            Components        Robot         Modules
                 │              │              │
          ┌──────┴──────┐       │         Services
          │             │       │
        Omni         Gimbal     │
          │             │       │
          └─────────────┴───────┘
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
| Robot | 当前机器人业务逻辑、控制任务与配置 |
| Modules | 可复用功能服务，如 IMU、Referee、通信服务 |
| Devices | 电机、遥控器、传感器等硬件抽象 |
| BSP / Boards | 板级硬件资源、外设配置与适配 |
| Libs | 通用数学、通信、数据结构及基础工具 |

**依赖原则：上层使用下层，下层不依赖上层。**

---

## 2. Core Design Principles

### 2.1 Thread ≠ Module ≠ Device

三者必须明确区分：

- **Thread**：独立的实时执行上下文。
- **Module**：可复用的功能服务。
- **Device**：硬件抽象接口。

一个 Device **不因为存在就必须拥有一个 Thread**。

例如：

```text
Chassis Control Thread
        │
        ▼
   Chassis Motor
      Device
        │
        ▼
       CAN1
```

这里 `Motor` 是 Device，而不是 `Motor Thread`。

只有当某个功能具有独立的调度需求时，才创建独立 Thread。

---

### 2.2 Control 与 Thread 分离

核心原则：

> **Control 决定“怎么算”，Thread 决定“什么时候算”。**

例如：

```text
Chassis Control
    │
    ├── Omni Kinematics
    ├── PID
    ├── Power Limit
    │
    ▼
Motor Command
```

而：

```text
Chassis Control Thread
    │
    ├── 周期调度
    ├── 读取状态
    ├── 调用 Controller
    └── 输出 Command
```

控制算法本身尽量不依赖 ThreadX、HAL 和具体硬件。

---

### 2.3 Thread 数量由实时需求决定

不要遵循：

```text
一个功能 = 一个线程
```

而应遵循：

```text
一个独立实时执行需求 = 一个线程
```

例如：

- Chassis Control：需要 1 kHz 实时控制 → 独立线程。
- Gimbal Control：需要 1 kHz 实时控制 → 独立线程。
- PowerManager：只是控制中的一部分 → 不需要独立线程。
- ServiceAlive：低频健康检查 → 通常不需要独立线程。
- Debug：低优先级持续输出 → 可以独立线程。
- UI：低频持续更新 → 可以独立线程。

---

## 3. Directory Structure

推荐：

```text
app/
├── CMakeLists.txt
├── README.md
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
    │   ├── chassis/
    │   │   ├── chassis_control.hpp
    │   │   └── chassis_control.cpp
    │   │
    │   ├── gimbal/
    │   │   ├── gimbal_control.hpp
    │   │   └── gimbal_control.cpp
    │   │
    │   └── control_types.hpp
    │
    ├── vision/
    │   ├── protocol.hpp
    │   ├── protocol.cpp
    │   └── vision_thread.cpp
    │
    └── tasks/
        ├── chassis_control_thread.cpp
        ├── gimbal_control_thread.cpp
        ├── ui_thread.cpp
        ├── debug_thread.cpp
        └── system_threads.cpp
```

不要求所有功能立即拆成上述形式。

文件划分应该根据代码复杂度和职责进行，而不是为了形式上的整齐强制拆分。

---

## 4. `app.cpp`

`app.cpp` 是整个应用的 **Composition Root**。

主要负责：

- 初始化应用所需的设备和服务。
- 初始化 Controller。
- 创建 ThreadX 线程。
- 管理线程对象和线程 Stack。
- 根据编译配置创建可选线程。
- 检查初始化和线程创建结果。

不负责：

- 控制算法。
- 协议解析。
- 复杂业务逻辑。
- 长时间运行的控制循环。

推荐结构：

```text
app.cpp
│
├── 初始化 Devices / Modules
├── 初始化 Controllers
├── 创建 Chassis Control Thread
├── 创建 Gimbal Control Thread
├── 创建其他可选 Thread
└── 启动应用
```

---

# 5. Components

`components/` 用于放置可跨机器人复用的算法和控制组件。

当前主要包括：

```text
components/
├── omni/
└── gimbal/
```

### 5.1 Omni

负责：

- 全向轮运动学。
- 逆运动学。
- 轮速分配。
- 底盘几何相关计算。

例如：

```text
Robot Velocity
      │
      ▼
Omni Kinematics
      │
      ▼
Wheel Velocity
```

Omni 不应该知道：

- M3508 的 CAN ID。
- 当前机器人使用哪一个 CAN。
- ThreadX。
- UART / CAN API。
- 当前机器人模式。

---

### 5.2 Gimbal

负责可复用的云台控制算法，例如：

- Yaw / Pitch 控制。
- PID。
- Feedforward。
- ESO。
- 状态预测。
- 坐标转换。

Gimbal Component 不应该直接：

```cpp
CAN_Send(...);
```

而应该产生：

```cpp
gimbal_command
```

由 Robot 层负责连接到具体 Motor Device。

---

## 6. Robot Control Architecture

机器人控制由两个主要实时控制线程组成：

```text
                         Robot State
                              │
                 ┌────────────┴────────────┐
                 │                         │
                 ▼                         ▼
         Chassis Control             Gimbal Control
             Thread                      Thread
              1 kHz                       1 kHz
                 │                         │
                 ▼                         ▼
          Omni / PID                 Yaw / Pitch
                 │                         │
                 ▼                         ▼
        Chassis Motor Device      Gimbal Motor Device
                 │                         │
                CAN1                      CAN2
```

### 6.1 Chassis Control Thread

负责底盘实时控制。

典型流程：

```text
Chassis Control Thread
        │
        ▼
Read Robot State
        │
        ├── Remote
        ├── IMU
        ├── Wheel Feedback
        ├── Referee
        └── Power State
        │
        ▼
Mode Management
        │
        ▼
Omni Kinematics
        │
        ▼
PID / Controller
        │
        ▼
Power Limiting
        │
        ▼
Chassis Motor Command
        │
        ▼
Chassis Motor Device
        │
        ▼
CAN1
```

负责：

- 底盘模式。
- 遥控器输入。
- 全向轮运动学。
- 轮速控制。
- 功率限制。
- 底盘电机目标生成。
- 底盘电机发送。

不负责：

- 云台控制。
- Vision 协议解析。
- Debug 输出。
- UI 绘制。

---

### 6.2 Gimbal Control Thread

负责云台实时控制。

典型流程：

```text
Gimbal Control Thread
        │
        ▼
Read Robot State
        │
        ├── IMU
        ├── Gimbal Encoder
        ├── Vision
        └── Remote
        │
        ▼
Target / Mode Management
        │
        ▼
Yaw / Pitch Controller
        │
        ├── PID
        ├── Feedforward
        └── Prediction / ESO
        │
        ▼
Gimbal Motor Command
        │
        ▼
Gimbal Motor Device
        │
        ▼
CAN2
```

负责：

- Yaw / Pitch 控制。
- 云台模式。
- Vision 目标使用。
- IMU 状态使用。
- 编码器反馈。
- PID / FF / ESO 等控制算法。
- 云台电机目标生成。
- 云台电机发送。

不负责：

- 底盘运动学。
- UI。
- Debug。
- Referee 协议解析。

---

## 7. Motor Architecture

### 7.1 Motor 是 Device，而不是 Thread

Motor Device 负责：

- 电机对象。
- 电机 ID。
- CAN Frame 编码。
- 电机反馈解析。
- CAN TX / RX 接口。

例如：

```text
devices/
└── motor/
    ├── motor.hpp
    └── motor.cpp
```

控制线程直接调用：

```cpp
chassis_motor.send(command);
```

或：

```cpp
gimbal_motor.send(command);
```

不需要：

```text
Control
   ↓
Command Queue
   ↓
Motor Thread
   ↓
CAN
```

除非未来确实出现独立的 Motor Output 调度需求。

---

### 7.2 为什么不需要 Motor Thread

假设底盘和云台使用独立 CAN：

```text
Chassis Control ──> Chassis Motor ──> CAN1

Gimbal Control ───> Gimbal Motor ───> CAN2
```

两个控制器没有竞争同一 CAN TX 资源，因此没有必要再增加一个统一 Motor Thread。

如果 CAN Driver 本身使用：

- TX FIFO。
- Mailbox。
- Interrupt。
- DMA。

那么：

```cpp
motor.send(command);
```

通常只是将数据提交给 CAN 外设，不需要 Control Thread 阻塞等待整个 CAN 帧发送完成。

---

### 7.3 什么时候需要 Motor Output Thread

只有出现明确的独立调度需求时才考虑。

例如：

```text
Chassis ──┐
Gimbal ───┼── Shared CAN
Debug ────┘
```

或者需要：

- 统一输出时序。
- 多控制器共享一个 CAN Bus。
- CAN Bandwidth Management。
- TX Queue。
- Frame Priority。
- 多 CAN Bus 聚合。
- 独立输出周期。

此时可以引入：

```text
Controller
    │
    ▼
Motor Command Buffer
    │
    ▼
Motor Output Service
    │
    ▼
CAN
```

但这是**具体需求驱动的设计**，不是默认架构。

---

# 8. Thread Architecture

### 8.1 Thread Overview

| Thread / Service | 独立 Thread | 典型频率 | 主要职责 |
|---|---:|---:|---|
| Chassis Control | ✅ | 1 kHz | 底盘控制与电机输出 |
| Gimbal Control | ✅ | 1 kHz | 云台控制与电机输出 |
| IMU | 视 Module 实现 | 传感器周期 | 采样与姿态估计 |
| Remoter | 视实现 | 事件 / 短周期 | 遥控器接收 |
| Referee | 视实现 | 事件 / 短周期 | 裁判系统通信 |
| Vision | 视实现 | 按通信需求 | 视觉通信 |
| Debug | ✅ 可选 | 10–50 Hz | 调试输出 |
| UI | ✅ 可选 | 10–20 Hz | UI 更新 |
| ServiceAlive | ❌优先 | 50–100 Hz | 服务健康检查 |
| PowerManager | ❌ | 随 Control | 功率管理 |
| Motor | ❌默认 | — | Device，不单独建线程 |

频率只是设计参考，最终应根据实际 WCET、通信负载和实时性测试确定。

---

## 9. IMU

IMU 如果已经由 Module 管理，则 Robot App 不应该重新实现 IMU 线程。

负责：

- 传感器采样。
- DMA / SPI / UART 接收。
- AHRS / ESKF。
- 姿态和角速度状态更新。
- 时间戳。
- 在线状态。

Control 只读取：

```text
IMU State
```

而不是直接处理原始 IMU 数据。

---

# 10. Remoter

Remoter 负责：

- DR16 等遥控器数据接收。
- DMA RX-to-IDLE。
- 协议解析。
- 在线检测。
- Remote State 更新。

典型数据流：

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
   ├───────────────┐
   ▼               ▼
Chassis Control  Gimbal Control
```

Remoter 不负责：

- 底盘模式决策。
- 云台控制。
- 电机输出。

---

# 11. Referee

Referee Module / Service 负责：

- UART DMA 接收。
- FIFO。
- Frame Parsing。
- CRC8 / CRC16。
- Command ID 解码。
- 数据缓存。
- 在线状态。
- 时间戳。

典型数据：

- Game Status。
- Game Robot Status。
- Power Heat Data。
- Robot Position。
- Buff / Event。
- 其他裁判系统状态。

Referee 负责**通信和数据解析**，不负责底盘功率控制算法。

---

# 12. Vision

Vision 通信分为：

```text
Protocol
```

和：

```text
Vision Thread / Service
```

### 12.1 Protocol

放：

```text
robot/vision/
├── protocol.hpp
└── protocol.cpp
```

负责：

- 数据结构。
- Frame Header。
- Packet Length。
- Byte Layout。
- Encode。
- Decode。
- CRC。
- 数据合法性检查。

协议是通信契约，不应该和 ThreadX 调度逻辑混在一起。

---

### 12.2 Vision Thread

负责：

- USB 通信调度。
- 接收数据。
- 发送 Vision Telemetry。
- 更新 Vision State。

数据流：

```text
Vision Computer
       │
       ▼
    USB RX
       │
       ▼
 USB Driver
       │
       ▼
Vision Protocol
       │
       ├── Decode
       ├── CRC Check
       └── Validation
       │
       ▼
  Vision State
       │
       ▼
Gimbal Control
```

如果 USB Driver / Module 已经拥有独立接收线程，则 Robot App 不应该重复创建无意义的 RX Thread。

---

# 13. ServiceAlive

ServiceAlive 默认不需要独立线程。

如果只是检查：

- IMU 是否在线。
- Remote 是否在线。
- Referee 是否在线。
- Vision 是否超时。
- Motor Feedback 是否更新。
- SuperCap 是否在线。

可以由已有控制线程降频执行。

例如 Control 为 1 kHz：

```cpp
if (++health_counter >= 10)
{
    health_counter = 0;
    update_service_status();
}
```

即：

```text
1 kHz Control
      │
      └── 每 10 次检查一次
                 ↓
               100 Hz
```

如果未来发展为复杂故障诊断、恢复和状态机，再考虑独立 Service。

---

# 14. PowerManager

PowerManager 是**控制逻辑**，默认不是 Thread。

负责：

- 裁判系统功率限制。
- Buffer Energy。
- SuperCap 状态。
- 底盘功率限制。
- 电机输出缩放。
- 功率分配。

推荐：

```text
Chassis Control
      │
      ▼
PowerManager
      │
      ▼
Wheel Command
      │
      ▼
Motor
```

而不是：

```text
PowerManager Thread
      │
      ▼
Chassis Control Thread
```

除非未来 PowerManager 本身拥有独立通信或实时调度需求。

---

# 15. UI

UI 是可选低频线程。

负责：

- 裁判系统图形交互。
- 底盘状态。
- 云台状态。
- 故障状态。
- 超电状态。
- 其他机器人 UI。

UI 不应该修改 Control 内部状态。

推荐：

```text
Robot State
    │
    ▼
UI Thread
    │
    ▼
UI Command
    │
    ▼
Referee TX
```

UI 线程不直接修改控制器。

---

# 16. Debug

Debug 是可选低优先级线程。

负责：

- Debug 数据。
- Telemetry。
- 控制周期统计。
- CPU / Stack 使用情况。
- 故障信息。
- 调试变量。

推荐：

```text
Robot State
    │
    ▼
Debug Snapshot
    │
    ▼
Debug Thread
    │
    ▼
USB / UART
```

Debug 不应该直接修改 Control 状态。

---

# 17. Configuration Management

配置按职责管理。

推荐：

```text
robot/configs/
├── build_config.hpp.in
├── thread_config.hpp
└── robot_config.hpp
```

### 17.1 Build Configuration

`build_config.hpp.in`：

```cpp
#pragma once

#cmakedefine01 ENABLE_UI
#cmakedefine01 ENABLE_DEBUG
```

使用：

```cpp
#if ENABLE_UI
```

而不是：

```cpp
#ifdef ENABLE_UI
```

因为 `ENABLE_UI` 是明确的 0 / 1 配置。

CMake 同时应该根据功能开关决定是否编译对应源文件。

---

### 17.2 Thread Configuration

例如：

```cpp
#pragma once

#include "tx_api.h"

namespace robot::cfg::threads
{

struct thread_config
{
    ULONG stack_size_bytes;
    UINT priority;
    UINT preemption_threshold;
};

inline constexpr thread_config chassis_control{
    2048,
    5,
    5
};

inline constexpr thread_config gimbal_control{
    2048,
    5,
    5
};

inline constexpr thread_config debug{
    2048,
    10,
    10
};

inline constexpr thread_config ui{
    2048,
    10,
    10
};

}
```

具体 Stack 和 Priority 必须根据实际系统测量确定。

---

### 17.3 Robot Configuration

放置当前机器人实例相关参数：

- Omni 几何尺寸。
- Wheel Radius。
- Motor ID。
- PID。
- Gimbal 参数。
- Chassis 参数。
- Power Limit。
- 运行限制。

这些参数属于当前机器人，不应该放进通用 Component。

---

# 18. Constants

公共数学常量应该复用已有 Lib。

例如：

```text
libs/math/include/constants.hpp
```

放置：

- $\pi$
- $2\pi$
- 角度 / 弧度转换常量
- 其他通用数学常量

不要在 App 中重复：

```cpp
constexpr float pi = 3.1415926f;
```

协议常量则放在对应 Protocol：

```text
vision/protocol.hpp
```

例如：

- Frame Header。
- Packet Size。
- Command ID。
- CRC 参数。

机器人参数放：

```text
robot/configs/
```

线程参数放：

```text
robot/configs/thread_config.hpp
```

原则：

> **通用数学常量、协议常量、机器人参数、Build 配置、Thread 配置分别管理。**

---

# 19. Communication Protocol Rules

协议定义和线程实现必须分离。

例如：

```text
robot/vision/
├── protocol.hpp
├── protocol.cpp
└── vision_thread.cpp
```

`protocol.hpp / protocol.cpp`：

```text
数据结构
Frame Layout
Encode
Decode
CRC
Validation
```

`vision_thread.cpp`：

```text
通信调度
接收
发送
状态更新
```

不要直接将 C++ Struct 的内存布局作为协议发送：

```cpp
usb_write(&packet, sizeof(packet));
```

除非协议明确规定并且已经验证：

- Alignment。
- Padding。
- Endianness。
- Size。
- ABI。

更推荐显式编码：

```cpp
encode(packet, buffer);
```

---

# 20. Inter-thread Communication

线程之间应该通过明确的数据接口共享状态。

推荐：

```text
                     ┌───────────┐
                     │    IMU    │
                     └─────┬─────┘
                           │
                     IMU State
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
        ▼                  ▼                  ▼
 Chassis Control     Gimbal Control       Debug
        ▲                  ▲
        │                  │
     Remote             Vision
        │                  │
     Referee          Gimbal Encoder
```

基本原则：

> **一个状态应该具有明确的写入者，其他线程通过受控接口读取。**

根据数据类型选择同步机制：

| Mechanism | Use Case |
|---|---|
| Atomic | 简单状态、Flag |
| Mutex | 共享复杂对象 |
| Semaphore | 事件通知 |
| Event Flags | 多事件状态 |
| Message Queue | 完整消息传递 |
| FIFO / SPSC | 字节流 |
| Snapshot / Double Buffer | 高频状态共享 |

---

## 21. Control State 与 Command

建议区分：

```text
State
```

和：

```text
Command
```

例如：

```cpp
struct chassis_state
{
    float vx;
    float vy;
    float wz;

    float wheel_speed[4];

    bool remote_online;
};

struct chassis_command
{
    float wheel_current[4];
};
```

控制器：

```text
State
  │
  ▼
Controller
  │
  ▼
Command
```

Motor Device：

```text
Command
  │
  ▼
CAN Frame
```

这样可以避免 Control 和 Device 强耦合。

---

# 22. Thread Resource Ownership

Thread Object 和 Stack 必须具有足够长的生命周期。

例如：

```cpp
TX_THREAD chassis_control_thread{};

alignas(ULONG)
uint8_t chassis_control_stack[
    cfg::threads::chassis_control.stack_size_bytes
]{};
```

需要保证 Stack 大小满足 ThreadX 要求。

建议检查：

```cpp
static_assert(
    cfg::threads::chassis_control.stack_size_bytes % sizeof(ULONG) == 0
);
```

线程创建时不要忽略返回值：

```cpp
UINT status = tx_thread_create(...);

if (status != TX_SUCCESS)
{
    // Handle startup failure
}
```

---

# 23. ThreadX Priority

ThreadX 中：

> **Priority 数字越小，优先级越高。**

初步设计：

```text
High
 │
 ├── Chassis Control
 ├── Gimbal Control
 └── IMU
 │
 ├── Remote
 ├── Referee
 └── Vision
 │
 ├── ServiceAlive
 │
 └── Debug / UI
Low
```

实际 Priority 必须根据：

- WCET。
- 周期。
- 数据依赖。
- 中断延迟。
- 系统负载。

进行调整。

默认：

```cpp
preemption_threshold = priority;
```

只有明确需要限制抢占时才调整 Preemption Threshold。

---

# 24. Thread Startup

Control Thread 不应该在系统依赖尚未初始化时提前运行。

推荐：

```text
System Init
    │
    ├── Devices
    ├── Modules
    ├── Controllers
    └── Shared State
    │
    ▼
Create All Threads
    │
    ▼
Check Creation Results
    │
    ▼
Start Control Threads
```

可以先：

```cpp
TX_DONT_START
```

创建线程，完成全部初始化后再统一：

```cpp
tx_thread_resume(...)
```

避免出现：

```text
Chassis Thread
     ↓
读取 IMU
     ↓
IMU 还没初始化
```

---

# 25. CMake Organization

应用层可以使用 `INTERFACE` Target 管理源文件和依赖。

例如：

```cmake
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

    "${CMAKE_CURRENT_SOURCE_DIR}/robot/control/chassis/chassis_control.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot/control/gimbal/gimbal_control.cpp"

    "${CMAKE_CURRENT_SOURCE_DIR}/robot/tasks/chassis_control_thread.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot/tasks/gimbal_control_thread.cpp"

    "${CMAKE_CURRENT_SOURCE_DIR}/robot/vision/protocol.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot/vision/vision_thread.cpp"
)
```

可选功能：

```cmake
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

根目录：

```cmake
add_subdirectory(devices)
add_subdirectory(modules)
add_subdirectory(app)

target_link_libraries(${PROJECT_NAME} robot_app)
```

依赖关系：

```text
Firmware
    │
    ▼
robot_app
    │
    ├── components
    ├── modules
    └── devices
          │
          ▼
      BSP / Boards
```

根 CMake 不需要重复维护 App 内部依赖。

---

# 26. File Organization Rules

不要求：

```text
one thread = one file
```

而应该根据代码复杂度拆分。

推荐：

```text
control/
├── chassis/
│   ├── chassis_control.hpp
│   └── chassis_control.cpp
│
└── gimbal/
    ├── gimbal_control.hpp
    └── gimbal_control.cpp

tasks/
├── chassis_control_thread.cpp
├── gimbal_control_thread.cpp
├── ui_thread.cpp
├── debug_thread.cpp
└── system_threads.cpp
```

简单功能可以合并：

```text
system_threads.cpp
```

例如：

- ServiceAlive。
- 简单系统监测。
- 简单低频任务。

当某个功能明显变复杂，再独立拆分。

**不要为了“一个线程一个文件”制造大量只有十几行代码的文件。**

---

# 27. Reliability

Control 必须考虑以下情况：

- IMU 尚未完成初始化。
- Remote 离线。
- Referee 数据过期。
- Vision 数据过期。
- Motor Feedback 超时。
- SuperCap 通信异常。
- 可选线程创建失败。
- 控制周期超时。

核心控制链路不应该依赖：

- Debug。
- UI。
- ServiceAlive。

正常运行。

故障处理应该在控制层制定明确的降级策略，而不是让底层通信线程直接修改电机输出。

---

# 28. Responsibility Boundaries

| Code | Responsibility |
|---|---|
| `app.cpp` | 初始化、资源组合、线程创建 |
| `control/chassis` | 底盘控制算法 |
| `control/gimbal` | 云台控制算法 |
| `chassis_control_thread.cpp` | 底盘控制周期调度 |
| `gimbal_control_thread.cpp` | 云台控制周期调度 |
| `components/omni` | 可复用全向轮运动学 |
| `components/gimbal` | 可复用云台算法 |
| `motor device` | 电机硬件抽象、CAN 编解码 |
| `vision/protocol` | 视觉协议 |
| `vision_thread` | 视觉通信调度 |
| `ui_thread` | UI 更新 |
| `debug_thread` | 调试输出 |
| `configs` | 当前机器人配置 |
| `modules` | 可复用功能服务 |

---

# 29. Avoid These Practices

不要：

- 把所有业务逻辑放进 `app.cpp`。
- 把 Chassis、Gimbal 强行塞进同一个 Control Thread。
- 因为存在 Motor 就创建 `Motor Thread`。
- 让 Control 直接解析 USB / UART 协议。
- 让 Components 依赖 ThreadX / HAL。
- 让 Debug / UI 修改 Control 状态。
- 让多个线程无保护地访问同一个共享对象。
- 让多个线程直接竞争同一个 CAN TX 资源。
- 重复定义公共数学常量。
- 把协议常量和机器人可调参数混在一起。
- 为简单低频任务创建不必要的 Thread。
- 忽略线程创建失败。
- 忽略数据时间戳和在线状态。
- 假设系统启动后所有服务立即有效。

---

# 30. Final Architecture

最终推荐架构：

```text
                               app
                                │
                ┌───────────────┼────────────────┐
                │               │                │
             Modules         Robot          Components
                │               │                │
        ┌───────┼───────┐       │          ┌─────┴─────┐
        │       │       │       │          │           │
       IMU   Remote  Referee    │        Omni        Gimbal
        │       │       │       │
        └───────┴───────┴───────┘
                    │
              Robot State
                    │
          ┌─────────┴─────────┐
          │                   │
          ▼                   ▼
  Chassis Control       Gimbal Control
      Thread                 Thread
       1 kHz                  1 kHz
          │                   │
          ▼                   ▼
   Chassis Controller   Gimbal Controller
          │                   │
          ▼                   ▼
   Chassis Motor        Gimbal Motor
      Device                Device
          │                   │
         CAN1                CAN2


          Low Frequency / Optional
                    │
          ┌─────────┼─────────┐
          ▼         ▼         ▼
         UI       Debug   ServiceAlive
```

核心数据流：

```text
Sensors / Services
        │
        ▼
    Robot State
        │
   ┌────┴────┐
   ▼         ▼
Chassis    Gimbal
Control    Control
   │         │
   ▼         ▼
Command    Command
   │         │
   ▼         ▼
Motor      Motor
Device     Device
   │         │
  CAN1      CAN2
```

---

# 31. Final Design Principles

最终架构遵循以下原则：

1. **App 负责组合，Control 负责决策。**
2. **Chassis Control 与 Gimbal Control 是并列的实时控制任务。**
3. **Control Module 与 Control Thread 分离。**
4. **Motor 是 Device，不是默认 Thread。**
5. **拥有独立硬件资源的控制器可以直接驱动对应 Device。**
6. **只有共享硬件资源或存在独立调度需求时才增加 Output Service / Thread。**
7. **Components 负责可复用算法，不负责硬件和线程调度。**
8. **Modules 负责可复用服务，Robot 不重复实现已有功能。**
9. **Protocol 管理通信契约，Thread 管理通信调度。**
10. **Configs 管理当前机器人实例的配置。**
11. **线程数量由实时需求决定，而不是由功能数量决定。**
12. **文件按照职责和复杂度拆分，而不是机械地一线程一文件。**
13. **共享状态必须具有明确的所有权和同步机制。**
14. **核心控制链路不依赖 Debug、UI 等可选功能。**
15. **控制路径应保持确定性、低延迟、可测试和故障隔离。**

最终目标：

> **让 Chassis、Gimbal 等控制模块能够复用，同时让线程边界真实反映机器人的实时执行结构，而不是为了抽象而抽象。**
