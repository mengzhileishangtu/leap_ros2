# 从零读懂 leap_ros_mcu_driver

适用于 Ubuntu 24.04、VS Code 和 ESP32-S3 小车项目。源码以 `main` 分支的 v2.2、提交 `806d2fa` 为准。主线可以在电脑上学完。建议一次留 20～40 分钟；第一次完成阶段 0 和阶段 1 的直行例子就够了。[本路线对应的源码](https://github.com/czu963889306-dev/leap_ros_mcu_driver/tree/806d2fa02de9085c8ef413f59dc9e421c88d51f5)

## 你最后要能讲清什么

假设电脑发出“以 0.2 m/s 前进，同时向左转”的指令。读完后，你应该能打开源码，指出**哪个函数收到了指令、谁计算左右轮目标、谁知道轮子实际转了多快、谁把结果发回电脑**。然后换一个没讲过的传感器，自己追出它的采集和发布路径。

| 进度 | 这次解决的问题 | 留下一个能检查的结果 |
|---|---|---|
| □ 0　打开工程 | 哪些代码在电脑上跑，哪些在板子上跑？ | 打开仓库，找到两个入口 |
| □ 1　运动学 | 同一个“左转”，两个轮子各转多少？ | 手算轮速，运行第一个测试 |
| □ 2　启动与任务 | ESP32 开机后，谁让控制和采集同时进行？ | 标出一个任务和它使用的队列 |
| □ 3　接收命令 | 电脑发出的速度怎样到达控制器？ | 追出 `linear.x` 的值和单位 |
| □ 4　驱动电机 | 怎么知道车轮达到了目标转速？ | 画出一只轮子的反馈回路 |
| □ 5　读回里程计 | 电脑上的位置和速度从哪里来？ | 解释轮子空转时里程计会怎样 |
| □ 6　独立追外设 | 一个距离读数怎样变成 ROS 话题？ | 独立追完超声波，再任选一项拓展 |
| □ 7　复盘与迁移 | 换一台车，哪些假设需要重新验证？ | 写出一个故障的排查顺序和移植清单 |

## 阶段 0　从一个空文件夹开始

先认识电脑和小车的分工。Ubuntu 上的程序发运动指令、接收状态。ESP32-S3 上的固件驱动电机、读编码器和传感器。运行在电脑侧的 **micro-ROS Agent** 让板上的 micro-ROS 程序接入 ROS 2。你现在读的仓库主要是板上的固件，另有少量可以在普通电脑运行的测试。

**已有仓库**：直接用 VS Code 打开包含 `README.md`、`main/`、`components/` 和 `tests/` 的那一层文件夹。**第一次下载**：在 Ubuntu 终端执行下面几行，把参考源码放在独立学习目录。后续编译命令都在仓库根目录执行。

```bash
mkdir -p ~/leap-study
cd ~/leap-study
git clone https://github.com/czu963889306-dev/leap_ros_mcu_driver.git
cd leap_ros_mcu_driver
git rev-parse --short HEAD
code .
```

如果后续版本与本文不同，文末的源码链接仍指向固定版本。已有本地修改时，也可以直接对照这些链接阅读。若终端找不到 `code`，用 VS Code 的“文件 → 打开文件夹”打开同一目录即可。

第一次只打开三个文件：

1. `README.md`：找出主控芯片、底盘类型、支持的两类通信协议。看到陌生的传感器型号，先留下书签。
2. `main/app_main.cpp`：找到固件入口 `app_main()`，记下它调用的 `app_runtime_startup()`。到这里就停。
3. `tests/differential_kinematics_test.cpp`：找到普通 C++ 的 `int main()`。下一阶段在电脑上跑的就是它。

读代码时用两个动作就够了：VS Code 按 `Ctrl+P` 输入文件名，按 `Ctrl+Shift+F` 全工程搜索函数名。要找“谁调用了它”，搜索函数名并区分声明、定义和调用位置；还没配好 C++ 跳转功能也能这样读。

**过关**：关掉文件后，能重新找到板子的入口，以及电脑上运动学测试的入口。能用自己的话说出两个入口各自在什么设备上运行。

## 阶段 1　在电脑上跑通一件能算对的事

**问题**：两轮车原地向左转时，左右轮应该同向还是反向？先在纸上画车头和两只轮子，再开代码。

按顺序只看下面四处：

1. 在 `tests/differential_kinematics_test.cpp` 搜 `Positive yaw`。测试给出输入，也写出了作者认为正确的左右轮输出。这里的“正角速度”表示向左转。
2. 在 `main/control/differential_kinematics.h` 找 `Inverse` 和 `Forward`。前者由车身速度求轮速，后者由轮速反推车身速度。先把函数看成“给它什么、它算出什么”。
3. 在同名的 `.cpp` 里读两个函数的实现。这里只要认识 `wheel_mms[0]` 是左轮，`[1]` 是右轮。调用 `Forward(wheels, &vx, &vy, &wz)` 时，三个 `&` 把变量地址交给函数，让它把结果写回来。
4. 在 `main/config/motion_config.h` 找 `kTrackWidthMm`。两轮中心间距为 131.7 mm，所以半轮距是 65.85 mm。把它代入算式。

| 车身输入 | 左轮目标 | 右轮目标 |
|---|---:|---:|
| 前进 200 mm/s，无转动 | 200 mm/s | 200 mm/s |
| 不前进，向左转 2 rad/s | −131.7 mm/s | +131.7 mm/s |
| 前进 200 mm/s，同时向左转 1 rad/s | 134.15 mm/s | 265.85 mm/s |

在仓库根目录编译并运行原仓库的测试。两处 `-I` 告诉编译器去哪里找头文件，`-o` 指定输出程序的位置。如系统提示缺少 `g++`，可先运行 `sudo apt install g++`。

```bash
g++ -std=c++17 -Wall -Wextra -Werror -I main/config -I main/control \
  tests/differential_kinematics_test.cpp main/control/differential_kinematics.cpp \
  -o /tmp/leap_kinematics
/tmp/leap_kinematics && echo "运动学测试通过"
```

`ExpectNear(actual, expected)` 在误差很小时通过，超出范围就触发断言。原测试成功时本身没有输出，最后一句让你能看到成功提示。

**过关**：把输入改成“前进 200 mm/s、向右转 1 rad/s”，自己算一次，再对照答案：左轮 265.85 mm/s、右轮 134.15 mm/s。能解释符号为什么互换，今天就可以收工。[运动学实现](https://github.com/czu963889306-dev/leap_ros_mcu_driver/blob/806d2fa/main/control/differential_kinematics.cpp) · [原仓库测试](https://github.com/czu963889306-dev/leap_ros_mcu_driver/blob/806d2fa/tests/differential_kinematics_test.cpp)

## 阶段 2　回到开机那一刻，认识任务和队列

**问题**：如果运动控制和超声波采集都有一个 `while` 循环，为什么两个都能运行？

1. 从 `app_main()` 进入 `main/app_runtime.cpp` 的 `app_runtime_startup()`。只标出 `board_init()`、`create_runtime_queues()` 和创建 `motion_task`、`ultrasonic_task` 的两行 `xTaskCreate(...)`。
2. 在 `main/platform/board.cpp` 搜 `MotionController robot`。你会看到 `robot` 拿着左右电机和两个编码器。以后看到 `robot.Drive(...)`，就知道它控制哪几个硬件对象了。
3. 打开 `main/tasks/sensors/ultrasonic_task.cpp`。这个任务反复测距，把结果写进 `ultrasonic_state`，然后等待。等待期间，FreeRTOS 可以调度其他就绪任务。再回 `create_runtime_queues()`，确认这个队列长度为 1。

| 源码里遇到的操作 | 可以怎样理解 |
|---|---|
| `xTaskCreate` | 交给 FreeRTOS 一个任务入口，由调度器安排执行 |
| `xQueueOverwrite` | 把单槽队列的内容更新为最新值 |
| `xQueuePeek` | 查看当前值并保留，另一个读取者仍可查看 |
| `xQueueReceive` | 取出队列里的消息，这条消息随即被移出 |

**过关**：连续写入 30 cm、31 cm，发布任务之后才来看，能看到哪一次？答案是 31 cm。顺着 `ultrasonic_state` 搜一遍，给创建、写入、读取三个位置各加一个书签。[启动代码](https://github.com/czu963889306-dev/leap_ros_mcu_driver/blob/806d2fa/main/app_runtime.cpp) · [超声波任务](https://github.com/czu963889306-dev/leap_ros_mcu_driver/blob/806d2fa/main/tasks/sensors/ultrasonic_task.cpp)

**顺手看懂工程怎样编起来**：打开根目录 `CMakeLists.txt`，找到 ESP-IDF 的工程配置和项目名；再看 `main/CMakeLists.txt` 的 `APP_SRCS` 与 `idf_component_register`，找出刚读过的两个任务文件。主机运动学测试只编两个 C++ 文件；整套固件则要把任务、硬件驱动和通信组件一起交给 ESP-IDF 构建。

## 阶段 3　追一条下行命令

**问题**：ROS 2 消息里的 `0.2`，在哪一行变成了固件里的 `200`？

先认识三个词。**节点**是参与通信的程序实体。**话题**是一类消息使用的名字。**消息类型**规定里面有哪些字段。本项目订阅的话题叫 `/cmd_vel`，类型叫 `geometry_msgs/msg/Twist`。本例只关心前进速度 `linear.x` 和转动速度 `angular.z`。

如果你的 Ubuntu 已装好 ROS 2 Jazzy，可以用两个终端体验一次。两个终端都先运行 `source /opt/ros/jazzy/setup.bash`。下面用单独的练习话题，在电脑上就能看见消息。

```bash
# 终端 A，开始监听
ros2 topic echo /leap_study/cmd_vel geometry_msgs/msg/Twist

# 终端 B，查看字段，然后发一条消息
ros2 interface show geometry_msgs/msg/Twist
ros2 topic pub --once /leap_study/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 1.0}}'
```

在 A 中应看到 `linear.x` 为 0.2、`angular.z` 为 1.0，按 `Ctrl+C` 结束。还没安装 ROS 2 时，直接在下面的源码中跟踪这两个数，也能完成本阶段。[ROS 2 Jazzy 官方话题教程源码](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.rst)

| 打开的位置 | 只回答一个问题 |
|---|---|
| `main/tasks/network/microros/microros_runtime.cpp` 的 `spin_once()` | `rcl_take` 取到消息后调用了谁？ |
| 同目录 `microros_subscribers.cpp` 的 `handle_cmd_vel()` | 哪个字段乘以 1000？结果写入哪个队列？ |
| `main/msg/motion_msg.h` 的 `MotionMsg` | 指令字段 `target_vx` 与状态字段 `vx` 各表示什么？ |
| `main/tasks/control/motion_task.cpp` 的 `apply_command` | `control_mode == 0` 时，送给 `robot.Drive()` 的三个量是什么？ |

**过关**：关掉表格，用全文搜索从 `handle_cmd_vel` 走到 `robot.Drive`。能说出中间靠 `motion_cmd` 队列接力，线速度由 m/s 变为 mm/s，角速度保持 rad/s。[订阅函数](https://github.com/czu963889306-dev/leap_ros_mcu_driver/blob/806d2fa/main/tasks/network/microros/microros_subscribers.cpp) · [运动任务](https://github.com/czu963889306-dev/leap_ros_mcu_driver/blob/806d2fa/main/tasks/control/motion_task.cpp)

## 阶段 4　只盯一只轮子，读懂速度闭环

**问题**：左轮目标是 134.15 mm/s，如果地面阻力变大，哪段代码会发现轮子转慢了？

这次集中在 `main/control/motion_controller.cpp`，分两次读。第一次跟目标值，第二次跟实测值。

1. **跟目标值。** 从 `Drive()` 进 `CalculateKinematics()`。找到阶段 1 学过的 `kinematics_.Inverse(...)`，再找 `kMmSToRpm`。轮子的目标线速度在这里换成每分钟转数，存入 `final_target_vel_`。
2. **跟实测值。** 打开 `Update()`，从 `GetCount()` 向下读。两次编码器计数之差除以时间得到每秒计数，再用每轮一圈的计数换算成 `raw_rpm`，经过滤波得到 `filtered_rpm_`。配置给出的标称控制周期为 20 ms。
3. **在两条线相遇处停住。** 搜 `pid_vel_[i].Calculate`，看看它吃进的目标、实测和时间间隔。计算结果经过 `ApplyDeadband()`，最后传给 `motors_[i]->SetSpeed(final_pwm)`。PWM 是让驱动器改变电机输出的控制量。读到这里再打开 `components/pid_controller/pid_controller.cpp`，先认出 `error = setpoint - measured_value`。

**算一次**：假设方向已经校正，20 ms 内多了 78 个计数，每圈 4680 个计数。原始转速是多少？`78 ÷ 0.02 ÷ 4680 × 60 = 50 RPM`。现在你能把一次编码器读数一直追到 PID 的实测输入。刚开始转动时，滤波后的数值可能低于这次的原始值。

**过关**：在纸上画出“目标轮速、实测轮速、PID、PWM、电机、编码器”六项，并把实测轮速接回 PID。若需要深入某个硬件，再从 `GetCount()` 或 `SetSpeed()` 进入 `components/encoder_driver/` 或 `components/motor_driver/`。[控制器](https://github.com/czu963889306-dev/leap_ros_mcu_driver/blob/806d2fa/main/control/motion_controller.cpp) · [PID 实现](https://github.com/czu963889306-dev/leap_ros_mcu_driver/blob/806d2fa/components/pid_controller/pid_controller.cpp)

## 阶段 5　反向追踪，弄明白里程计

**问题**：电脑收到 `/odom`，其中的速度是怎么得来的？这次从电脑看到的结果往回找。

1. 在 `microros_publishers.cpp` 找 `publish_odom()`，看它从 `motion_state` 读到什么，哪几项除以 1000。
2. 全文搜 `queues.motion_state`，定位到 `motion_task.cpp` 中的写入位置。向上几行，找 `GetVelocity()` 和 `GetOdometry()`。
3. 回 `motion_controller.cpp`，在 `GetVelocity()` 找 `kinematics_.Forward(...)`。这就是阶段 1 的另一个函数。再到 `Update()` 找 `odom_yaw_` 和 `pos_x_`，看车身速度怎样随时间累积成航向与位置。

做一个思想实验。把驱动轮架空，假设编码器能读到两轮等速向前转。固件算出的前进里程会增加，车身在房间里的位置却没有变。这能同时解释“里程计为什么有用”和“里程计为什么会漂”。源码中的 `imu_yaw_rel_` 还用于相对转向控制；追踪这里发布的轮式航向时，看 `odom_yaw_ += local_vw * dt` 这一行。

**过关**：有人问“把轮子卡住但持续发前进命令，目标值和里程计速度各会怎样？”你能引用 `target_vx`、编码器反馈和 `publish_odom()` 三处解释，区分指令与测量。[里程计计算](https://github.com/czu963889306-dev/leap_ros_mcu_driver/blob/806d2fa/main/control/motion_controller.cpp#L143) · [话题发布](https://github.com/czu963889306-dev/leap_ros_mcu_driver/blob/806d2fa/main/tasks/network/microros/microros_publishers.cpp#L95)

## 阶段 6　用超声波练一次独立阅读

**题目**：传感器测到 45 cm，电脑会在哪个话题、哪个字段看到多少？先自己找，遇到断点再看下面这行提示。

**搜索顺序**：`GetDistanceCm` → `distance_cm` → `queues.ultrasonic_state` → `publish_ultrasonic` → `kUltrasonicTopic`。

**对照答案**：采集任务把 45 存入 `UltrasonicMsg.distance_cm`。发布函数读队列、除以 100，把 0.45 写进 `sensor_msgs/msg/Range` 的 `range` 字段，通过 `/ultrasonic` 发出。读数小于等于 0 时，这个字段写入 `NaN`，表示无有效距离。它在同步时间可用时才发布。[采集端](https://github.com/czu963889306-dev/leap_ros_mcu_driver/blob/806d2fa/main/tasks/sensors/ultrasonic_task.cpp) · [发布端](https://github.com/czu963889306-dev/leap_ros_mcu_driver/blob/806d2fa/main/tasks/network/microros/microros_publishers.cpp#L175)

接下来挑一个你最想弄明白的问题，自行做第二遍。以后再回来补其他分支。

| 想研究的问题 | 从哪里进 | 怎样证明自己看懂了 |
|---|---|---|
| 松开手柄后，之前的导航指令还在吗？ | `main/tasks/io/gamepad_i2c_task.cpp` 和 `motion_task.cpp` | 给出“导航前进、手柄接管、手柄回中”三个时刻生效的指令。用当前源码核对 README 中的断连描述。 |
| 为什么板子已经连上，但暂时收不到带时间戳的传感器数据？ | `microros_time_sync.cpp`、`publish_state_timer()`、`tests/time_sync_pro_test.cpp` | 找出首次同步前和上次成功同步超过 30 秒后，发布函数走哪条路径。 |
| 换一种雷达，固件靠什么识别？ | `main/tasks/sensors/lidar_detection.h`、`lidar_task.cpp`、`tests/lidar_driver_test.cpp` | 先从测试找识别和掉线场景，再回代码解释有效帧数和超时分别起什么作用。 |
| 改完速度 PID，重启后为什么还能用？ | `extra_ros_packages/leap_interfaces/srv/`、`microros_services.cpp`、`main/config/pid_config.cpp` | 从服务请求找到保存参数的位置，再回启动函数找到重新加载的位置。 |
| MAVLink 和 micro-ROS 两种指令如何共用电机控制？ | `app_runtime.cpp` 的 `start_comm_task()`、`main/tasks/network/mavlink/mavlink_common.cpp`、`doc/protocols.md` | 选一条 MAVLink 速度指令，找到它在哪里与阶段 3 的 ROS 路径汇合，再指出两者各自处理协议的文件。 |

想验证雷达和时间同步，可以在仓库根目录分别运行对应的主机测试。雷达测试使用 `g++`；时间同步的命令还用到 CMake 和 CTest。它们用测试数据模拟外设输入与时钟行为。

```bash
bash tests/run_lidar_tests.sh

cmake -S tests -B /tmp/leap-time-sync-tests
cmake --build /tmp/leap-time-sync-tests
ctest --test-dir /tmp/leap-time-sync-tests --output-on-failure
```

## 阶段 7　验收，准备迁移到自己的车

先合上这份指南，用源码回答三道题。

- [ ] **讲一次完整运动。** 输入为 `linear.x = 0.2`、`angular.z = 1.0`。写下 ROS 消息、板内指令、左右轮目标、编码器、PWM、里程计分别出现在哪个文件。能说清哪些是目标值，哪些来自测量。
- [ ] **查一次故障。** 上位机一直发前进指令，两个轮子都不动。按你会实际检查的顺序列三个观察点，例如回调是否收到消息、运动任务是否选中了这条命令、最终 PWM 是否为零。每个观察点旁写下“若正常，我下一步看哪里”。
- [ ] **说明一个改动会传到哪里。** 假设把“每圈编码器计数”填成实际值的两倍。用阶段 4 的数字算出轮速读数会减半，再说明这会怎样影响速度反馈和里程计。能做到这一项，就已经在用源码推理。

最后带着源码去看你的三轮车：在纸上标出每只轮子有没有电机、能否主动转向、有没有编码器。如果是“左右两个驱动轮＋被动支撑轮”，可以沿用刚学的差速公式，先核对驱动芯片、引脚、轮径、轮距、编码器方向与每圈计数。如果是舵机转向或三个全向轮，保留 ROS 消息与任务组织的思路，重新推导底盘运动学。你可以等到能画出自己底盘的轮子布局，再进入这一阶段。

## 读到哪里就记到哪里

每次读完，用三行写下今天能证明的事情。比如：

```text
今天的问题：0.2 m/s 在哪里变成 200 mm/s？
代码证据：handle_cmd_vel() 把 linear.x 乘 1000，写入 MotionMsg.target_vx。
接下来查：motion_task 何时读取 motion_cmd，谁会临时接管这条命令？
```

时间少时，只追一个变量、记下一个函数，就可以在这里停。下次从“接下来查”继续。
