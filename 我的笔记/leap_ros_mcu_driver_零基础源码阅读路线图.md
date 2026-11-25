# 在 Windows 上从零读懂 leap_ros_mcu_driver

这是一份源码导读。打开一个代码片段，弄清它负责什么、谁调用它、数据接下来去哪，再顺着链接读下一段。整条路线可以在 Windows 上的 VS Code 里读完。路线基于固件 `806d2fa` 和配套上位机 `8043c44`，链接行号对应本地文件。

## 从哪儿打开

- 在 VS Code 打开本文件，按 `Ctrl+Shift+V` 打开 Markdown 预览。点击链接文字，即可打开对应的本地源码标签页并定位到指定行。在 Markdown 编辑界面也可以按住 `Ctrl` 点击链接。
- 固件源码在 `github参考/leap_ros2`，上位机参考文件在 `github参考/chuyun_ws`。涉及原版时间同步和雷达识别的两个文件位于 `github参考/leap_ros_mcu_driver-806d2fa`。按 `Ctrl+Shift+F` 搜函数名能找到谁用过它。
- **阅读节奏**：一天读一个小片段也可以。每节末尾有一个“可以继续”的判断，只需能用自己的话回答。初次阅读把 `tests/`、`managed_components/`、运动学公式和 PID 推导留到以后。碰到宏或底层库函数，先弄清它在眼前这一行起什么作用。

## 先认清这三个地方

| 地方 | 在整套系统里做什么 | 你会看到的名字 |
|---|---|---|
| 你的 Windows 电脑 | 打开并阅读两端的源文件 | VS Code |
| 车上的 ESP32-S3 | 运行烧进芯片的程序，也就是固件。ESP-IDF 提供开发框架，FreeRTOS 安排板上任务运行 | `app_main`、`xTaskCreate`、GPIO、PWM |
| 项目里的 Linux 上位机 | 运行 micro-ROS Agent 和 ROS 2 应用。Agent 把板端通信接入 ROS 2；上位机应用接收、加工或发送消息 | `main`、`rclcpp`、launch 文件 |

先看 **谁在执行代码**，再看函数名。同一个“发布里程计”功能会跨越板上任务、通信链路和 Linux 程序，它们分别有自己的入口。

## 第一遍的路径图

| 顺序 | 带着这个问题打开源码 | 这一步学到什么 |
|---|---|---|
| 1 | 板子开机，项目自己写的第一段应用代码在哪？ | C/C++ 函数、头文件、声明与调用 |
| 2 | ESP32 的一个引脚，怎么在代码里变成“超声波传感器”？ | GPIO、类、对象、构造与初始化 |
| 3 | 距离是谁测的？其他任务怎么拿到读数？ | 结构体、FreeRTOS 任务、队列、按值复制 |
| 4 | 板子里的厘米数，怎样成为电脑可订阅的 ROS 话题？ | 发布者、消息类型、时间戳、micro-ROS |
| 5 | 反过来，电脑发来的速度指令在哪几行被底盘接收？ | 订阅、指针、命令与状态、任务分工 |
| 6 | 电机控制器怎样把目标变成实际输出？ | 编码器、闭环、PWM、硬件驱动 |
| 7 | Linux 上是谁接走板子发回的里程计？ | 可执行文件、进程、ROS 节点、回调与 launch |

## 1　找到固件应用入口｜第一次只读三个位置

**① 第一段只有几行。** 打开 [main/app_main.cpp · 1–5 行](../github参考/leap_ros2/main/app_main.cpp#L1-L5)。

```cpp
#include "app_runtime.h"

extern "C" void app_main(void) {
    app_runtime_startup();
}
```

- `#include "app_runtime.h"` 把头文件里的声明引入当前源文件，第 4 行由此知道有这样一个函数可以调用。
- `void app_main(void)` 的前一个 `void` 表示没有返回值，括号里的 `void` 表示没有参数。ESP-IDF 完成前面的系统启动后，由主任务调用这个应用入口。
- `extern "C"` 让这个 C++ 文件里的入口使用 C 语言链接约定，让框架能按约定的名字找到它。
- `app_runtime_startup();` 是一次函数调用。到这里，你已经知道下一步该搜哪个名字。[ESP-IDF v5.4.4 启动说明](../../../esp/v5.4.4/esp-idf/docs/en/api-guides/startup.rst#L156-L163)

**② 看同一个名字在两个文件里有什么区别。** [main/app_runtime.h · 9–10 行](../github参考/leap_ros2/main/app_runtime.h#L9-L10) 的函数名后面直接跟分号，告诉调用者“有这个函数”。[main/app_runtime.cpp · 170–180 行](../github参考/leap_ros2/main/app_runtime.cpp#L170-L180) 的函数名后跟花括号，里面才是执行步骤。第一段还有个能提前返回的 `if`，作用是避免重复启动。

**③ 只圈三个动作。** 在 [main/app_runtime.cpp · 182–199 行](../github参考/leap_ros2/main/app_runtime.cpp#L182-L199) 找 `board_init()`、`create_runtime_queues()`、`xTaskCreate(ultrasonic_task, ...)`。先把它们译成“准备硬件”“准备任务间传数据的地方”“创建测距任务”。今天读到第 199 行就可以关掉文件。

**可以继续**：能说出“框架调用谁，它又调用谁”，还能指出函数**声明**和**定义**分别在哪个文件。三个启动动作里，第 2 节先展开 `board_init()`。

## 2　从引脚读到对象｜C++ 在这里解决什么问题

这节只追超声波传感器。它有两根信号线，TRIG 发出触发脉冲，ECHO 返回一段脉冲，脉冲时长用于计算距离。

1. **引脚怎么命名？** [main/config/board_config.h · 22–23 行](../github参考/leap_ros2/main/config/board_config.h#L22-L23) 给 GPIO21 和 GPIO47 起了有用途的名字。读到 `constexpr`，先理解为“编译时已确定的值”。这样其他文件可以写“超声波触发引脚”，接线信息集中在一处。
2. **对象能做什么？** [components/ultrasonic_sensor/ultrasonic_sensor.h · 10–26 行](../github参考/leap_ros2/components/ultrasonic_sensor/ultrasonic_sensor.h#L10-L26) 定义了 `UltrasonicSensor` 类。`public` 下的构造函数接收两根引脚，`Init()` 初始化外设，`GetDistanceCm(...)` 返回厘米数；`private` 下保存对象自己的引脚等状态。把这个头文件读成一张可调用功能表。
3. **具体是哪一个传感器？** [main/platform/board.cpp · 19–21 行](../github参考/leap_ros2/main/platform/board.cpp#L19-L21) 的 `UltrasonicSensor ultrasonic(...)` 用刚才那两个引脚创建名为 `ultrasonic` 的对象。再看 [main/platform/board.cpp · 27–36 行](../github参考/leap_ros2/main/platform/board.cpp#L27-L36)，`board_init()` 调用 `ultrasonic.Init()` 来初始化硬件。这里同时能看清“创建对象”和“启动这块外设”是两个动作。

**可以继续**：遮住右侧注释，指出 `UltrasonicSensor`、`ultrasonic`、`GetDistanceCm()` 中哪个是类型、哪个是代表具体设备的对象、哪个是它提供的方法。下一节会看到谁用这个对象。

## 3　读一个完整的小任务｜FreeRTOS 和队列

**先读数据，再读任务。** [main/msg/ultrasonic_msg.h · 4–7 行](../github参考/leap_ros2/main/msg/ultrasonic_msg.h#L4-L7) 的 `struct` 把相关字段放进一个类型。`float distance_cm` 装距离。打开 [main/tasks/sensors/ultrasonic_task.cpp · 5–13 行](../github参考/leap_ros2/main/tasks/sensors/ultrasonic_task.cpp#L5-L13)，从头到尾只追下面四个动作。

| 位置 | 读成一句人话 |
|---|---|
| 第 6 行 `UltrasonicMsg msg;` | 创建一份这个任务用来装读数的消息。 |
| 第 7–8 行 `while (1)`、`ultrasonic.GetDistanceCm(25.0f)` | 任务反复向刚才的对象取距离，并把结果写进 `msg.distance_cm`。按方法声明，25 是温度参数，表示本次用 25℃ 做声速补偿。 |
| 第 10 行 `xQueueOverwrite(..., &msg)` | 把这份消息写进队列。`&msg` 给出源数据的地址，队列按创建时设置的大小复制内容。 |
| 第 11 行 `vTaskDelay(...100...)` | 当前任务等待约 100 ms，其他就绪任务可以运行。一次循环还包括测距所花的时间。 |

**现在回头读创建代码。** [main/app_runtime.cpp · 152–156 行](../github参考/leap_ros2/main/app_runtime.cpp#L152-L156) 中，`xQueueCreate(1, sizeof(UltrasonicMsg))` 指定只存一条，每条占一个消息结构体的大小。[main/app_runtime.cpp · 193–198 行](../github参考/leap_ros2/main/app_runtime.cpp#L193-L198) 的 `xTaskCreate(ultrasonic_task, ...)` 将函数作为任务入口交给调度器。启动函数还能继续创建电机等其他任务。最新距离来了就覆盖那一个位置，发布端稍后取走一份当前读数，采集和发送各按自己的节奏工作。

**可以继续**：说明 `ultrasonic_task` 为什么可以一直循环，`app_runtime_startup()` 仍能往下走。然后回答“先测到 30 cm，又测到 31 cm，发布任务这时才来读，会读到哪个值？”

**想看硬件底层时再展开**：[GetDistanceCm 的触发与等待](../github参考/leap_ros2/components/ultrasonic_sensor/ultrasonic_sensor.cpp#L121-L141) 翻转 TRIG 引脚并等待通知；[回波中断中的边沿与通知](../github参考/leap_ros2/components/ultrasonic_sensor/ultrasonic_sensor.cpp#L74-L100) 记录回波的起止边沿，再通知等结果的任务。这里是理解“外设事件、中断、任务”三者关系的一个实物例子。

## 4　把 45 cm 追到 ROS 话题｜第一条完整数据流

假设板子刚测到 45 cm。**话题**是双方约定的数据流名称，**消息类型**规定这一条数据有哪些字段，**发布者**负责把填好的消息发出去。

1. [main/tasks/network/microros/microros_runtime.cpp · 134–141 行](../github参考/leap_ros2/main/tasks/network/microros/microros_runtime.cpp#L134-L141) 创建发布者。找到三个信息就停：它属于 `s_node`、使用 `sensor_msgs/msg/Range` 类型、使用 `kUltrasonicTopic` 这个名字。[main/config/ros_config.h · 23–29 行](../github参考/leap_ros2/main/config/ros_config.h#L23-L29) 里能找到名字 `ultrasonic`。项目使用的完整话题名是 `/ultrasonic`。
2. [main/tasks/network/microros/microros_publishers.cpp · 175–187 行](../github参考/leap_ros_mcu_driver-806d2fa/main/tasks/network/microros/microros_publishers.cpp#L175-L187) 是最值得逐行读的十几行。`xQueuePeek` 读取并保留队列里的值；`set_stamp` 填时间戳；`distance_cm / 100.0f` 把 45 cm 变成 0.45 m；`rcl_publish` 发布消息。这里的局部变量 `ultrasonic` 装着一份读数，它的类型是 `UltrasonicMsg`。第 2 节的同名对象类型是 `UltrasonicSensor`，看声明就能分清。
3. **谁决定什么时候发？** 同文件 [196–206 行](../github参考/leap_ros_mcu_driver-806d2fa/main/tasks/network/microros/microros_publishers.cpp#L196-L206)。定时回调先检查同步时间，再按计数决定这一轮是否发布超声波。上一节的采集任务只负责更新读数。

**可以继续**：用自己的话讲完“GPIO 对应的传感器 → 采集任务 → 单槽队列 → ROS 发布者 → 电脑可订阅的距离”。顺便指出消息在哪里从厘米变成米。现在已经读通这个工程的一项完整功能。

## 5　反方向读速度指令｜ROS 订阅和板内消息

换方向看。上位机向 `/cmd_vel` 发布 `Twist`，其中 `linear.x` 表示前进速度，`angular.z` 表示转向角速度。这次只跟 `linear.x`，一直跟到 `robot.Drive()`。

1. **看到什么消息，会叫谁来处理？** [microros_runtime.cpp · 154–163 行](../github参考/leap_ros2/main/tasks/network/microros/microros_runtime.cpp#L154-L163) 建立 `Twist` 订阅；同文件 [371–378 行](../github参考/leap_ros2/main/tasks/network/microros/microros_runtime.cpp#L371-L378) 取出一条消息，交给 `handle_cmd_vel()`。
2. **逐行读一次消息转换。** [main/tasks/network/microros/microros_subscribers.cpp · 5–18 行](../github参考/leap_ros2/main/tasks/network/microros/microros_subscribers.cpp#L5-L18)。参数 `const ... *msg` 是指向只读输入消息的指针，`msg->linear.x` 读取其中的字段。`cmd` 是本地创建的对象，所以写成 `cmd.target_vx`。0.2 m/s 被换成 200 mm/s，再通过 `motion_cmd` 队列交给运动任务。可以回 [MotionMsg · 16–27 行](../github参考/leap_ros2/main/msg/motion_msg.h#L16-L27) 看目标字段与当前状态字段。
3. **到哪一行开始控制车？** [motion_task.cpp · 49–64 行](../github参考/leap_ros2/main/tasks/control/motion_task.cpp#L49-L64) 取出指令、在导航与手柄指令间选择。先按手柄没有接管来读，再向上跳到 [同文件 27–32 行](../github参考/leap_ros2/main/tasks/control/motion_task.cpp#L27-L32)。`control_mode == 0` 时，进入 `robot.Drive(...)`。`auto apply_command = [&](...)` 是函数内部定义的一小段可调用代码，初读时把它叫作“执行所选指令”就够了。

**可以继续**：能指出“接收外部消息”和“决定现在执行哪条命令”分别在哪个函数。这样设计的效果是，通信层负责消息格式，运动任务可以统一处理不同来源的命令。

## 6　追到真实的电机｜控制器、驱动和反馈

先看接口的分工。此处只要知道运动学把车身目标换成左右轮目标，公式可以以后再读。建议分两次看。

**第一次，只追输出。** [board.cpp · 13–19 行](../github参考/leap_ros2/main/platform/board.cpp#L13-L19) 把两个电机和两个编码器交给控制器 `robot`。[motion_controller.cpp · 105–121 行](../github参考/leap_ros2/main/control/motion_controller.cpp#L105-L121) 的 `Drive()` 接下目标，并经 `CalculateKinematics()` 存成目标轮速。在 [motion_task.cpp · 82–92 行](../github参考/leap_ros2/main/tasks/control/motion_task.cpp#L82-L92) 看见运动任务反复调用 `robot.Update(...)`。跳到 [Update() · 254–266 行](../github参考/leap_ros2/main/control/motion_controller.cpp#L254-L266)，找到 PID 的输出、死区补偿和最终的 `SetSpeed(final_pwm)`。最后打开 [motor_driver.cpp · 55–82 行](../github参考/leap_ros2/components/motor_driver/motor_driver.cpp#L55-L82)，看到正负号决定哪路输出 PWM。PWM 通过改变一个周期里输出有效的时间比例来调节驱动。这个 `SetSpeed` 参数实际装的是带方向的驱动量，读函数体比只凭名字猜更准确。

**第二次，只追反馈。** [encoder_driver.cpp · 78–87 行](../github参考/leap_ros2/components/encoder_driver/encoder_driver.cpp#L78-L87) 从 ESP32 的脉冲计数外设读数。回 [Update() · 131–141 行](../github参考/leap_ros2/main/control/motion_controller.cpp#L131-L141)，找到“本次计数减上次计数”、换成实际 RPM、滤波三个动作。再看 [pid_controller.cpp · 21–28 行](../github参考/leap_ros2/components/pid_controller/pid_controller.cpp#L21-L28) 的 `error = setpoint - measured_value`。轮子受阻后，同样的目标对应不同的实测速度，控制器因此有依据调整输出。这就是这里引入反馈的理由。

**可以继续**：用“目标轮速、编码器实测轮速、PWM”三个词解释两件事：控制器拿什么比较？电机驱动实际收到什么？能答出来即可继续到 Linux 上位机。

## 7　跨到 Linux｜同一条里程计由谁接收

本节在 VS Code 里同时看本地固件和 **chuyun_ws 上位机源码**。跟着同一条 `/odom` 看两端，能把 Linux 进程、ROS 节点、消息回调和板上的 FreeRTOS 任务区分开。

1. **板上从哪里发？** [固件 motion_task.cpp · 94–107 行](../github参考/leap_ros2/main/tasks/control/motion_task.cpp#L94-L107) 把实际运动状态写入 `motion_state`。[固件 publish_odom() · 95–113 行](../github参考/leap_ros_mcu_driver-806d2fa/main/tasks/network/microros/microros_publishers.cpp#L95-L113) 取出状态，填位置、方向和速度，发布里程计。micro-ROS Agent 连接板端和上位机的 ROS 2 通信。
2. **Linux 启动哪个程序？** [上位机 launch 文件 · 48–64 行](../github参考/chuyun_ws/chuyun_bringup/launch/chuyun_bringup.launch.py#L48-L64) 里，`executable='ekf_input_adapter'` 指定可执行程序。再看 [上位机 CMakeLists.txt · 38–44 行](../github参考/chuyun_ws/chuyun_bringup/CMakeLists.txt#L38-L44)，同名程序由 `src/ekf_input_adapter.cpp` 构建。可以把两个文件接成一句话：“这个 launch 配置要启动的，就是这份 C++ 源码构成的程序。”
3. **程序怎么持续接收消息？** [上位机 ekf_input_adapter.cpp · 88–94 行](../github参考/chuyun_ws/chuyun_bringup/src/ekf_input_adapter.cpp#L88-L94) 是 Linux 应用的 `main()`。`rclcpp::init` 初始化 ROS 2，`make_shared<EkfInputAdapter>()` 创建节点对象，`spin` 等待并处理事件。这一例里，一个 Linux 进程运行着这个 ROS 节点。
4. **收到一条消息后做什么？** 同文件 [39–58 行](../github参考/chuyun_ws/chuyun_bringup/src/ekf_input_adapter.cpp#L39-L58) 先创建发布者，再订阅 `/odom`。消息到达时，`[this](...){...}` 里的代码被执行。它复制消息、修改子坐标系名称和测量不确定度，再向 `/odom/ekf_input` 发布。后续的 EKF 节点用这些输入做状态估计，适配器本身负责整理输入。

**可以继续**：试着给三个名字各写一行注释。FreeRTOS 的 `motion_task` 是板上调度的任务；Linux 的 `ekf_input_adapter` 可执行程序启动后形成进程；进程里的 ROS 节点注册订阅并处理消息。项目在这一层能教你 Linux 应用是怎样组织与启动的。

**想再读一点 Linux 系统接口**：[上位机 Wi-Fi 配置程序 · 68–71 行](../github参考/chuyun_ws/chuyun_wifi_config/chuyun_wifi_config/wifi_config_applier.py#L68-L71)。只看 `subprocess.run(...)`：这个 ROS 应用可以启动系统工具 `nmcli` 来查询网络连接，并读回输出。这样就把“使用 ROS API”和“使用操作系统提供的程序”对应到了两段真实代码。

## 主线读完后的几个分支

按兴趣选一条。每一行都对应前面已经读过的一项设计。

| 想了解 | 打开哪里 | 带走什么 |
|---|---|---|
| 工程怎么拼成固件 | [根目录 CMakeLists.txt](../github参考/leap_ros2/CMakeLists.txt#L1-L4)，再看 [main/CMakeLists.txt](../github参考/leap_ros2/main/CMakeLists.txt#L30-L44) | 从项目名称、源文件列表和头文件搜索目录，认出前面读过的模块怎样进入同一工程。 |
| UART、Wi-Fi UDP 和 micro-ROS 的关系 | [setup_udp_transport / setup_uart_transport](../github参考/leap_ros2/main/tasks/network/microros/microros_transport.cpp#L181-L214) | 两种传输分别注册打开、关闭、读、写函数。上层仍能发布同一种 ROS 消息。 |
| ROS 服务与断电保存 | [SetSpeedPid.srv](../github参考/leap_ros2/extra_ros_packages/leap_interfaces/srv/SetSpeedPid.srv#L1-L8) → [设置 PID 的服务](../github参考/leap_ros2/main/tasks/network/microros/microros_services.cpp#L31-L57) → [保存参数](../github参考/leap_ros2/main/config/pid_config.cpp#L52-L64) | `.srv` 的横线上方是请求、下方是响应；服务处理代码同时涉及运行时参数和写入 ESP32 的 NVS 非易失存储。 |
| I²C 传感器和雷达状态机 | [IMU 的 I²C 与雷达的 UART 引脚](../github参考/leap_ros2/main/config/board_config.h#L33-L37)、[IMU 采集任务](../github参考/leap_ros2/main/tasks/sensors/imu_task.cpp#L11-L24)、[雷达识别状态更新](../github参考/leap_ros_mcu_driver-806d2fa/main/tasks/sensors/lidar_detection.h#L21-L39) | 比较周期读取传感器与持续等待串口数据的两种处理思路。雷达着重看“探测、识别、超时后重新探测”。 |

## 卡住时，怎样问 AI

复制**当前函数和路径**，再说具体卡在哪个词。对着源码一次解决一个疑问，答案更容易核对。

> 我在 Windows 上静态阅读 `leap_ros_mcu_driver`，刚学到第 3 节。下面是 `main/tasks/sensors/ultrasonic_task.cpp` 第 5–13 行。我已经理解循环会反复测距，现在卡在 `xQueueOverwrite(..., &msg)`。请结合这段代码解释：为什么传地址？队列保存的究竟是什么？要判断这一点，我下一步只需要看哪一处源码？先讲这段，不用给运行命令。

如果 AI 的解释还太抽象，继续问“请把这个名字指给我看，它在本段哪一行被创建、哪一行被使用？”每次结束只记一句：**我能解释哪一行，下次从哪个函数接着读。**
