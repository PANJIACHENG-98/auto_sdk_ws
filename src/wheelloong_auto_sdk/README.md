# wheelloong_auto_sdk

`wheelloong_auto_sdk` 是面向 Python 任务工程的 Wheelloong/Shiloong AUTO 模式 SDK。
任务代码只使用 `Robot`、普通 Python 数据类和统一异常，不需要构造
`arm_common`、`system_common`、`voice_common` 或 Nav2 的 ROS 消息。

当前运行行为以这份正式 Demo 为核对基准：

```text
/home/niic/wheelloong/install/system/lib/system/auto_client.py
```

SDK 当前提供：

- 双臂 `MoveJ`、`MoveL`、`MoveP` 和 Hold。
- 头部俯仰/转动、腰部俯仰/升降。
- 左、右或双夹爪位置控制及可选状态确认。
- 单点/多点导航、软暂停/恢复、清图、当前位姿、反馈和取消。
- 文本播报和机器人本地音频文件播放。
- 系统状态、AUTO/IDLE 模式和六组运动轴使能。
- 自动进入和退出 AUTO 的 `auto_session` 上下文。
- 无机器人单元测试使用的 `MockBackend`。

## 1. 安全说明

本 SDK 可以真实移动双臂、头部、腰部、夹爪和底盘。运行运动样例前必须：

1. 启动并确认机器人系统、伺服、机械臂和所需导航/语音节点正常。
2. 清空机器人运动范围，确认急停可用且操作员能够立即接管。
3. 核对单位、绝对/增量模式、左右臂选择以及升降方向。
4. 首次运行时缩短持续时间，并在低风险姿态下验证。

`auto_oscillation_demo.py` 会真实运动。`auto_session_demo.py` 和
`existing_node_demo.py` 不发送目标运动，适合先验证生命周期和环境。

## 2. 分层设计

```text
任务业务代码
  -> Robot
     -> System / Arms / Body / Gripper / Navigation / Voice
        -> RobotBackend（稳定、无 ROS 类型的语义接口）
           -> ROS2Backend（Service / Topic / Action）
           -> MockBackend（内存模拟，用于测试）
```

公共层负责参数名称、单位、范围校验和异常语义。`ROS2Backend` 负责把公共参数
转换成当前机器人安装环境中的 ROS 请求。例如：

```python
robot.body.move(waist_lift_mm=-100.0)
```

公共调用不会暴露 `JntsCtrl`。Backend 会转换为以下固定顺序：

```text
index/value[0] = 头部俯仰，rad
index/value[1] = 头部转动，rad
index/value[2] = 腰部俯仰，rad
index/value[3] = 腰部升降，mm
```

未传入的头腰轴对应 `index=False`，不会被自动命令到零位。

## 3. 安装

### 3.1 前提

- Ubuntu/ROS 2 Humble 机器人环境。
- 主系统已安装在 `/home/niic/wheelloong/install`。
- SDK 源码位于 `/home/niic/auto_sdk_ws/src/wheelloong_auto_sdk`。
- Python 3.10、`colcon` 和 `pytest` 可用。

### 3.2 推荐安装方式：colcon symlink install

```bash
cd /home/niic/auto_sdk_ws
source /opt/ros/humble/setup.bash
source /home/niic/wheelloong/install/setup.bash

colcon build \
  --symlink-install \
  --packages-select wheelloong_auto_sdk

source /home/niic/auto_sdk_ws/install/setup.bash
python3 -c "from wheelloong_auto_sdk import Robot; print('SDK import OK')"
```

`--symlink-install` 让 Python 源码修改后通常无需重新复制文件；修改包元数据或安装
结构后仍建议重新构建。新终端每次都要 source：

```bash
source /opt/ros/humble/setup.bash
source /home/niic/wheelloong/install/setup.bash
source /home/niic/auto_sdk_ws/install/setup.bash
```

不建议脱离 ROS 环境直接 `pip install`，因为 `ROS2Backend` 依赖主系统生成的 ROS
Python 接口包。纯业务单元测试可以直接使用 `MockBackend`。

### 3.3 VS Code

用 VS Code 打开：

```text
/home/niic/auto_sdk_ws
```

根目录的 `.vscode` 配置会补充 ROS/SDK 的 Python 搜索路径。推荐安装 Python、
Pylance 和 ROS 扩展；终端选择 `Wheelloong ROS Bash` 配置后会自动 source 环境。

## 4. 最小使用示例

### 4.1 SDK 独立拥有 ROS 运行时

适合普通 Python 脚本。SDK 创建私有 ROS Context、Node、两线程 Executor 和后台
spin 线程，退出 `with` 时自动释放：

```python
from wheelloong_auto_sdk import Robot

with Robot.standalone(node_name="my_auto_task") as robot:
    state = robot.state()
    print(state.control_mode, state.errors)
```

### 4.2 任务工程已有 Node

```python
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from wheelloong_auto_sdk import Robot

rclpy.init()
node = Node("existing_task_node")
executor = MultiThreadedExecutor(num_threads=2)
executor.add_node(node)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()

robot = Robot.from_node(node)
try:
    print(robot.state().control_mode)
finally:
    robot.close()  # 不销毁借用的 Node，也不 shutdown rclpy
    executor.shutdown()
    spin_thread.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()
```

借用 Node 时任务工程必须持续 spin。同步 SDK 方法若从 ROS 回调中调用，需要至少
两线程 `MultiThreadedExecutor`，否则当前回调可能占住唯一执行线程而等不到响应。

## 5. AUTO 生命周期

### 5.1 推荐：自动管理

```python
from wheelloong_auto_sdk import AxisSelection, Robot

axes = AxisSelection(
    left_arm=True,
    right_arm=True,
    head_pitch=True,
    head_yaw=True,
    waist_pitch=True,
    waist_lift=True,
)

with Robot.standalone(node_name="task") as robot:
    with robot.auto_session(
        required_axes=axes,
        state_timeout_sec=10.0,
        service_timeout_sec=3.0,
    ):
        # 所有任务动作放在这里。
        pass
```

进入 `auto_session` 时依次执行：

1. 获取新鲜 `/system/get_info`，拒绝带活动错误的状态。
2. 请求 AUTO，并等待状态话题确认 AUTO。
3. 请求 `required_axes` 的完整使能值，并等待状态确认。
4. 只要选择了任一机械臂，就发送双臂 Hold。

退出时（正常、异常或 Ctrl+C）依次执行：

1. 取消 SDK 仍在管理的导航目标。
2. 选择过机械臂时再次 Hold。
3. 请求 IDLE 并等待确认。
4. 请求六组轴全部去使能并等待确认。

### 5.2 独立调用每一步

正式 Demo 的模式和使能操作也都有独立接口：

```python
from wheelloong_auto_sdk import AxisSelection, ControlMode

axes = AxisSelection.all()
robot.system.set_control_mode(ControlMode.AUTO, timeout_sec=3.0)
robot.system.wait_for_control_mode(ControlMode.AUTO, timeout_sec=10.0)
robot.system.set_enabled(axes, timeout_sec=3.0)
robot.system.wait_for_enabled(axes, timeout_sec=10.0)

# ...任务动作...

robot.system.set_control_mode(ControlMode.IDLE, timeout_sec=3.0)
robot.system.wait_for_control_mode(ControlMode.IDLE, timeout_sec=10.0)
robot.system.disable_all(timeout_sec=3.0)
robot.system.wait_for_all_disabled(timeout_sec=10.0)
```

`set_*` 只等待服务结果；`wait_for_*` 独立等待 SystemInfo 状态。拆开后可以准确知道
失败发生在“请求未成功”还是“请求成功但实际状态未确认”。

## 6. 公共 API

### 6.1 Robot：统一入口

| 接口 | 作用 |
|---|---|
| `Robot.standalone(node_name="wheelloong_auto_sdk", ros_args=None)` | 创建并拥有私有 ROS 运行时。 |
| `Robot.from_node(node)` | 借用任务工程已有且持续 spin 的 Node。 |
| `Robot.with_backend(backend)` | 注入 Mock 或未来 Server Backend。 |
| `robot.state(max_age_sec=0.5, wait_timeout_sec=2.0)` | 获取无 ROS 类型的 `SystemState`，含当前双臂 TCP 位姿。 |
| `robot.auto_session(required_axes=None, state_timeout_sec=5.0, service_timeout_sec=5.0, max_state_age_sec=0.5)` | 创建 AUTO 生命周期上下文；`None` 选择全部轴。 |
| `robot.close()` | 取消活动导航并按所有权规则释放资源；可重复调用。 |

能力对象通过 `robot.system`、`robot.arms`、`robot.body`、`robot.gripper`、
`robot.navigation` 和 `robot.voice` 访问。

### 6.2 System：状态、模式和使能

| 接口 | 默认值与作用 |
|---|---|
| `state(max_age_sec=0.5, wait_timeout_sec=2.0)` | 获取年龄不超过 0.5 秒的状态。 |
| `set_control_mode(mode, timeout_sec=3.0)` | 请求 AUTO=2 或 IDLE=99。 |
| `wait_for_control_mode(mode, timeout_sec=10.0)` | 等待模式实际生效。 |
| `set_enabled(axes, timeout_sec=3.0)` | 一次设置六组轴的完整使能值。 |
| `wait_for_enabled(axes, timeout_sec=10.0)` | 等待六组实际值与目标完全一致。 |
| `enable_all()` / `disable_all()` | 全部上使能/去使能。 |
| `wait_for_all_disabled()` | 等待全部去使能。 |

六组轴是：左臂、右臂、头部俯仰、头部转动、腰部俯仰、腰部升降。

### 6.3 Arms：双臂

```python
robot.arms.hold(timeout_sec=5.0)

robot.arms.move_j(
    left=[0.0] * 7,
    right=[0.0] * 7,
    speed_rad_s=0.0,
    acceleration_rad_s2=0.0,
    relative=False,
    wait=True,
    timeout_sec=60.0,
)
```

| 接口 | 关键默认值 | 目标和速度单位/约束 |
|---|---|---|
| `hold()` | `timeout_sec=5.0` | 停止双臂现有运动并保持当前关节。 |
| `move_j()` | 速度/加速度 `0.0`，绝对，阻塞，超时 `30.0 s` | 关节 rad；速度 `[0, 3.14] rad/s`。 |
| `move_l()` | 速度/加速度/psi `0.0`，绝对，阻塞，超时 `30.0 s` | TCP mm；速度 `[0, 1000] mm/s`。 |
| `move_p()` | 速度/加速度 `0.0`，参考关节 `None`，阻塞，超时 `30.0 s` | TCP mm；关节速度 `[0, 3.14] rad/s`。 |

共同规则：

- `left=None` 或 `right=None` 表示不选择该侧；两侧不能同时为 `None`。
- `relative=False` 是绝对模式，`True` 是增量模式。
- `wait=True` 等运动结束；超时时 SDK 会尽力发送 Hold 后再抛异常。
- 速度/加速度传 `0.0` 表示让底层驱动选择默认值，不是零速运动。
- Shiloong 当前 `MoveJ/MoveP` 默认速度是 `0.628 rad/s`（最大值的 20%）。
- Shiloong 当前 `MoveL` 默认速度是 `200 mm/s`（最大值的 20%）。
- Shiloong 当前仅使用速度字段，加速度字段保留但暂未参与运动计算。
- Wheelloong 的 `MoveP` 需要提供相应侧的 7 关节逆解参考值；Shiloong 可省略。
- `CartesianPoseMM` 使用四元数 `qx/qy/qz/qw`，会自动归一化；当前底层忽略
  `frame_id`，任务仍应明确统一坐标约定。

### 6.4 Body：头部和腰部

```python
robot.body.move(
    head_pitch_rad=0.0,
    head_yaw_rad=0.0,
    waist_pitch_rad=0.0,
    waist_lift_mm=-100.0,
    speed_percent=0.0,
    wait=True,
    timeout_sec=30.0,
)
```

也可调用：

```python
robot.body.move_head(pitch_rad=0.1, yaw_rad=-0.1)
robot.body.move_waist(pitch_rad=0.1, lift_mm=-100.0)
```

规则：

- 头部和腰部俯仰/转动单位为 rad，升降单位为 mm。
- 升降是绝对位置；当前两种机器人约定向下为负值，零点是最高位置。
- 至少提供一个目标；未提供的轴不会被选择，也不会自动回零。
- `speed_percent=0.0` 与正式 Demo 一致；`JntsCtrl.speed` 当前是预留字段。
- 阻塞请求的 `timeout_sec` 范围是 `(0, 30]`。

### 6.5 Gripper：夹爪

```python
from wheelloong_auto_sdk import ArmSide

robot.gripper.set(
    ArmSide.DUAL,
    0.5,
    speed=0.0,
    torque=0.0,
    wait=True,
    tolerance=0.02,
    timeout_sec=10.0,
)
robot.gripper.open(ArmSide.LEFT)   # 1.0，最大行程
robot.gripper.close(ArmSide.LEFT)  # 0.0，最小行程
```

- `ArmSide.DUAL=-1`、`LEFT=0`、`RIGHT=1`。
- 位置范围 `[0, 1]`。
- `speed`、`torque` 范围上限为 1；小于等于 0 使用服务默认值。
- `wait=True` 会在服务成功后继续等待 SystemInfo 的实际位置进入容差。
- 正式振荡 Demo 为保持 5 Hz 下发，使用 `wait=False`。

### 6.6 Navigation：导航

```python
from wheelloong_auto_sdk import NavigationMode, NavigationPose

robot.navigation.clear_local_costmap()
robot.navigation.clear_global_costmap()
handle = robot.navigation.navigate_to(
    NavigationPose(x_m=1.0, y_m=0.5, yaw_rad=0.0),
    mode=NavigationMode.DEFAULT,
)
handle.pause()
handle.resume()
current_pose = robot.navigation.current_pose()
print(handle.feedback)
result = handle.wait(timeout_sec=120.0)
```

| 接口 | 作用 |
|---|---|
| `navigate_to(pose, ...)` | 下发一个 `NavigateToPose` 目标。 |
| `navigate_through(poses, ...)` | 按顺序下发多个目标。 |
| `handle.feedback` | 最近距离、预计剩余时间、耗时、恢复次数等。 |
| `handle.done` | 是否已经进入最终状态。 |
| `handle.paused` | 是否处于 SDK 软暂停状态。 |
| `handle.pause(...)` | 取消当前 Action，但保留原目标以便恢复。 |
| `handle.resume(...)` | 重新下发暂停时保存的原目标。 |
| `handle.wait(...)` | 等待成功；默认超时后取消并确认终态。 |
| `handle.cancel(...)` | 主动取消并等待最终状态。 |
| `navigation.clear_local_costmap()` | 清除完整局部代价地图。 |
| `navigation.clear_global_costmap()` | 清除完整全局代价地图。 |
| `navigation.current_pose()` | 读取 `map -> base_link` 当前 TF 位姿。 |
| `navigation.cancel_all()` | 尽力取消 SDK 管理的全部活动目标。 |

导航位置单位是 m，偏航角单位是 rad，默认坐标系是 `map`。精确导航模式依赖
`ROBOT_NAV_VERSION=weloong_nav` 或 `sloong_nav`；显式传入 `behavior_tree` 时优先使用
该路径。两个下发接口默认 `mode=DEFAULT`、`behavior_tree=""`、等待 Action Server
`5.0 s`；`handle.wait()` 默认不设完成截止时间，但设置截止时间后默认超时即取消；
取消确认默认等待 `5.0 s`。

`WLOONG` 等产品型号环境值不是行为树目录。SDK 不会再把它拼成 XML 路径，默认
导航会把空字符串交给 Nav2，由机器人启动配置选择正确的单点或多点行为树。

ROS 2 Humble/Nav2 没有导航原生暂停服务。`handle.pause()` 会安全取消当前 Action
并保留原目标，`handle.resume()` 再重新下发。单点会从暂停后的当前位置重新规划到
原终点；多点会重新下发原始完整点位序列，已经经过的点也可能再次经过。

#### 存点和直接导航样例

存点程序读取 TF `map -> base_link`。导航系统和定位正常后运行：

```bash
ros2 run wheelloong_auto_sdk waypoint_recorder
```

- 按 `S`：立即保存当前位置，不需要按 Enter。
- 按 `Q`：退出。
- 源码开发模式默认继续使用 `waypoint/waypoints.txt`，已有点位不会迁移。
- 可用 `--file`、`--global-frame`、`--base-frame` 修改文件和 TF 坐标系。

运行时代码不再写死 `/home/niic`。可在启动样例或任务程序前设置统一点位文件：

```bash
export WHEELLOONG_AUTO_SDK_WAYPOINT_FILE=/data/robot/waypoints.txt
```

开发工作空间中的样例会继续选择源码工程的 `waypoint/waypoints.txt`；普通安装时
SDK 也会尝试定位同一 colcon 工作空间的源码点位。因此通过 `ros2 run` 启动以及
以 root/niic 运行都会读取同一份已保存点位。总体优先级依次为环境变量、已有的
样例源码回退文件、SDK 源码数据、colcon 工作空间源码数据、用户 XDG 数据目录。

TXT 每行格式如下，导航坐标单位为 m/rad：

```text
# id,x_m,y_m,yaw_rad,frame_id
1,1.250000000,-0.400000000,1.570796327,map
```

任务工程无需启动中间服务。直接导入 SDK，读取编号点位后调用导航接口：

```python
from wheelloong_auto_sdk import Robot, load_waypoints

waypoints = load_waypoints()
with Robot.standalone(node_name="task_navigation") as robot:
    # 一个目标点调用 NavigateToPose。
    robot.navigation.navigate_to(waypoints[1].pose).wait(timeout_sec=300.0)

    # 多个目标点按列表顺序调用 NavigateThroughPoses。
    poses = [waypoints[index].pose for index in (1, 2, 3)]
    robot.navigation.navigate_through(poses).wait(timeout_sec=300.0)
```

`waypoint_navigation_demo.py` 不增加导航包装函数，直接按以下顺序调用公共接口：
读取初始模式、切换并确认 AUTO、清局部图、清全局图、非阻塞下发并等待完成。
Demo 不调用 `set_enabled()`，所以不会给双臂、头部或腰部上电；如果初始状态不是
AUTO，结束或 Ctrl+C 取消后会恢复 IDLE，也不会调用去使能接口。
一个编号调用 `navigate_to()`，多个编号按输入顺序调用 `navigate_through()`：

```bash
ros2 run wheelloong_auto_sdk waypoint_navigation_demo 1
ros2 run wheelloong_auto_sdk waypoint_navigation_demo 1 2 3
```

Demo 不提供额外的导航包装函数。编号不存在、点位文件错误、Action Server
不可用、导航失败和超时都会抛出相应异常；任务工程按自身流程捕获和处理即可。

运行期间按 Ctrl+C 时，Demo 会对当前 `NavigationHandle` 显式调用 `cancel()`，
等待 Nav2 返回最终状态并打印 `navigation cancellation confirmed: CANCELED`。
随后退出 `Robot` 上下文，`cancel_all()` 会再次兜底处理其他活动目标；进程返回码为
130。若 Ctrl+C 发生在目标接受前，程序会说明当前没有可取消的导航目标。

### 6.7 Voice：语音

```python
robot.voice.speak(
    "任务开始",
    wait=True,
    speech_timeout_sec=30.0,
    call_timeout_sec=35.0,
)

robot.voice.play(
    "notice.wav",
    play_count=1,
    wait=True,
    timeout_sec=120.0,
)
```

- `speak()` 拒绝空文本。
- `play()` 只接收机器人语音服务目录中的纯文件名，不接收路径。
- 当前语音底层没有停止/取消 Service，因此 SDK 不提供虚假的 `voice.stop()`。

### 6.8 公共数据类型

| 类型 | 作用 |
|---|---|
| `AxisSelection` | 六组轴使能目标；`all()`/`none()` 快速构造。 |
| `ArmSide` | 左、右或双臂/双夹爪选择。 |
| `ControlMode` | SDK 支持的 AUTO 和 IDLE。 |
| `MotionMode` | 机械臂绝对或增量运动。 |
| `CartesianPoseMM` | 毫米制 TCP 位姿和四元数。 |
| `NavigationPose` | 米和弧度制平面导航目标。 |
| `SystemState` | SystemInfo 的无 ROS 类型状态快照。 |
| `CommandResult` | 成功服务调用的统一操作名、错误码和消息。 |
| `NavigationFeedback` | 导航过程反馈。 |
| `NavigationResult` | 导航最终状态和成功标志。 |

常用属性和构造辅助方法：

- `AxisSelection.all()`/`none()` 创建全选/全不选；`is_empty` 判断是否为空；
  `satisfied_by(state)` 判断所有被要求的轴是否已经使能。
- `CartesianPoseMM.validated()` 校验数值并归一化四元数；`from_rpy(...)` 用弧度制
  RPY 创建四元数位姿。
- `NavigationPose.validated()` 校验米、弧度和坐标系值。
- `SystemState.left_arm_pose/right_arm_pose` 是 `/system/get_info` 中当前双臂
  `arm_quat_left/right` 转换出的 `CartesianPoseMM`；无效的全零四元数对应 `None`。
- `CommandResult.ok` 判断错误码是否为 0；`SystemState.age_sec` 返回状态缓存年龄。

## 7. 正式 AUTO 振荡 Demo 对照

SDK 等价样例：

```text
examples/auto_oscillation_demo.py
```

它在正式 `auto_client.py oscillation-test` 流程后增加 MoveP/MoveL，形成九阶段：

1. 请求 AUTO 并确认。
2. 双臂、头部两轴和腰部两轴全部上使能并确认。
3. Hold 双臂；双臂进入各自工作姿态，头部、腰部和夹爪到零；等待 2 秒。
4. 双臂围绕工作姿态、头腰围绕零点，按 5 Hz 下发 20 秒振荡目标。
5. Hold；读取当前双臂 TCP 位姿，按左右六维偏移执行 MoveP。
6. Hold；重新读取实际 TCP 位姿，按左右六维偏移执行 MoveL。
7. Hold；头腰和夹爪回零；双臂到机器人对应的最终姿态。
8. 请求 IDLE 并确认。
9. 六组轴全部去使能并确认。

侍龙双臂振荡中心由工作姿态常量给出，单位为度：

```text
left  = [10, 77, -80, 20, -25, 10, 10]
right = [-10, 77, 80, 20, 25, -10, 10]
```

每个采样点分别计算 `target[j] = work_pose[j] + angle`，因此七个关节保留
各自不同的中心角，只共享正弦偏移量。结束阶段仍使用独立的 RETURN 姿态。

### 7.1 MoveP/MoveL 笛卡尔命令函数

四个正式语义函数均接收左右臂各自的六维序列，统一顺序为
`(x, y, z, roll, pitch, yaw)`；位置或位置偏移单位为 mm，RPY 或 RPY 偏移单位为
deg：

- `command_move_p_offset()`：读取当前 TCP，再叠加左右臂各自六维偏移。
- `command_move_l_offset()`：读取当前 TCP，再叠加左右臂各自六维偏移。
- `command_move_p_absolute()`：直接发送左右臂六维绝对笛卡尔位姿。
- `command_move_l_absolute()`：直接发送左右臂六维绝对笛卡尔位姿。

offset 函数中位置直接相加，姿态按 `q_base*q_delta` 绕当前 TCP 局部轴组合；
不保存或使用预设绝对起点。MoveP 同时把状态快照中的双臂关节角作为逆解参考。

两个函数都会在 Hold 后等待 `0.25 s`，再要求读取到年龄不超过 `0.2 s` 的状态，
因此不会把 Hold 前或上一段运动尚未完成时的缓存位姿当成增量起点。

MoveL 的左右臂共用 `speed_mm_s`，默认 `10 mm/s`。MoveP 最终执行关节运动，
因此左右臂共用的是 `speed_rad_s`，默认 `0.2 rad/s`，不能把 `10 mm/s` 填入
MoveP 速度字段。当前完整样例的 MoveP 偏移为左右 `(0,0,30,0,0,0)`；MoveL
左臂为 `(0,20,30,0,0,0)`，右臂为 `(0,-20,30,0,0,0)`。随后
`command_final_pose()` 再次 Hold，才执行最终 MoveJ。若振荡时按 Ctrl+C，两个
offset 命令都会跳过，程序直接执行最终姿态和清理。

### 7.2 当前基准默认值

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `--duration` | `20.0 s` | 采样持续时间；0 表示直到 Ctrl+C。 |
| `--period` | `4.0 s` | 一个完整振荡周期。 |
| `--rate` | `5.0 Hz` | 服务目标下发频率；测试上限 20 Hz。 |
| `--amplitude-deg` | `3.0°` | 叠加到双臂工作姿态及头腰零点的振幅。 |
| `--lumbar-lift-amplitude-mm` | `100.0 mm` | 正幅度；实际向下目标最低为 `-100 mm`。 |
| `--control-gripper` | `on` | 双夹爪在 0 和 1 间变化。 |
| `--work-hold` | `2.0 s` | 工作姿态到达后的稳定时间；兼容 `--zero-hold`。 |
| `--state-timeout` | `10.0 s` | 模式和使能状态确认。 |
| `--service-timeout` | `3.0 s` | 普通/采样服务响应。 |
| `--motion-timeout` | `60.0 s` | 工作姿态、MoveP/MoveL 和结束姿态。 |
| `--arm-velocity` | `0.0` | 发送 0，让 Nero 使用默认 `0.628 rad/s`。 |
| `--arm-acceleration` | `0.0` | 与正式 Demo 一致；当前 Nero 暂未使用。 |

当前 install 基准脚本的实际常量和公式对应 100 mm。若它的个别 CLI 帮助文本仍显示
“10 mm”，那只是帮助字符串未同步；SDK 按实际运行常量 `100.0` 和目标公式对齐。

### 7.3 默认轨迹数值

```text
angle = radians(3) * sin(2πt / 4)
cycle = (1 - cos(2πt / 4)) / 2
waist_lift_mm = -100 * cycle
gripper = cycle
```

| 周期时刻 | 双臂相对工作姿态的偏移/头两轴/腰俯仰 | 腰部升降 | 双夹爪 |
|---:|---:|---:|---:|
| `0 s` | `0°` | `0 mm` | `0.0` |
| `1 s` | `+3°` | `-50 mm` | `0.5` |
| `2 s` | `0°` | `-100 mm` | `1.0` |
| `3 s` | `-3°` | `-50 mm` | `0.5` |
| `4 s` | `0°` | `0 mm` | `0.0` |

Shiloong 最终双臂姿态（度）为：

```text
left  = [0, 85, -2, 3, -25, 0, 0]
right = [0, 85, -2, 3,  25, 0, 0]
```

运行：

```bash
cd /home/niic/auto_sdk_ws
source /opt/ros/humble/setup.bash
source /home/niic/wheelloong/install/setup.bash
source /home/niic/auto_sdk_ws/install/setup.bash

ros2 run wheelloong_auto_sdk auto_oscillation_demo \
  --robot shiloong
```

首次可缩短为一个周期，但仍会真实运动：

```bash
ros2 run wheelloong_auto_sdk auto_oscillation_demo \
  --robot shiloong \
  --duration 4 \
  --control-gripper off
```

振荡样例的 `command_test_pose()` 使用四元素 `body_targets`，固定顺序为
`[头部俯仰(rad), 头部旋转(rad), 腰部俯仰(rad), 腰部升降(mm)]`。四个
自由度在调用处逐项写出并分别传给 SDK；即使三个角度数值相同，也不隐式
共用单个 `body_angle_rad` 参数。

离线生成四组逗号分隔的目标 TXT，并绘制总览图：

```bash
cd /home/niic/auto_sdk_ws
ros2 run wheelloong_auto_sdk simulate_oscillation_targets
ros2 run wheelloong_auto_sdk plot_oscillation_targets
```

默认输出目录为当前工作目录下的 `simulated_targets/`。四个 TXT 分别保存左臂七轴、
右臂七轴、头部两轴和腰部两轴；第一列 `task_elapsed_sec` 从任务启动时的
`0.000000` 秒开始。绘图脚本输出 `oscillation_targets.png`，全程不连接 ROS，
也不会发送机器人命令。两个脚本均可通过 `--help` 查看输入、输出和轨迹参数。
默认按 `shiloong` 生成，双臂 TXT 同样保存“工作姿态 + 正弦偏移”的绝对目标。

只验证 AUTO 生命周期、不发送目标运动：

```bash
ros2 run wheelloong_auto_sdk auto_session_demo \
  --observe-seconds 2
```

该样例将生命周期显式拆成两个函数：

```python
enter_auto_stage(robot, args)  # 初始状态 → AUTO → 确认 → 上使能 → 确认 → Hold
exit_auto_stage(robot, args)   # 检查使能 → Hold → IDLE → 去使能 → 最终状态
```

`run_demo()` 只是依次调用这两个函数，并用异常处理保证阶段一中断时仍尝试阶段二。
阶段二会先读取双臂使能状态：双臂均已使能时才调用 Hold；若阶段一在使能前
被中断，则明确打印左右臂状态并跳过必然失败的 Hold，之后仍执行 IDLE 和全部
去使能。`--observe-seconds` 单位为秒、默认 `2.0`，传入 `0` 表示不等待。

样例对错误信息按原因分类，并标明失败阶段和具体步骤：

- 机器人拒绝命令：打印 operation、error_code 和底层原始信息；
- 服务或动作不可用：提示检查系统进程及 ROS 通信环境；
- 状态确认超时：说明命令可能未生效或状态话题未更新；
- 状态异常、参数错误和后端通信错误：分别打印对应原因；
- Ctrl+C：打印中断发生的步骤以及异常后清理是否完整完成。

## 8. 样例程序

完成 `colcon build` 并 source 工作空间后，所有样例统一通过
`ros2 run wheelloong_auto_sdk <可执行名>` 启动。`setup.py` 将可读的
`examples/*.py` 安装为 ROS 2 控制台入口，不维护第二份实现。

```bash
ros2 run wheelloong_auto_sdk auto_oscillation_demo
ros2 run wheelloong_auto_sdk auto_session_demo
ros2 run wheelloong_auto_sdk existing_node_demo
ros2 run wheelloong_auto_sdk plot_oscillation_targets
ros2 run wheelloong_auto_sdk simulate_oscillation_targets
ros2 run wheelloong_auto_sdk standalone_demo
ros2 run wheelloong_auto_sdk waypoint_navigation_demo 1
ros2 run wheelloong_auto_sdk waypoint_recorder
```

| `ros2 run wheelloong_auto_sdk ...` 可执行名 | 是否运动 | 作用 |
|---|---|---|
| `auto_oscillation_demo` | 是 | 九阶段振荡及 MoveP/MoveL 流程。 |
| `auto_session_demo` | 否 | 用两个独立函数显式执行进入和退出阶段。 |
| `simulate_oscillation_targets` | 否 | 离线生成四组目标 TXT。 |
| `plot_oscillation_targets` | 否 | 将四组目标 TXT 绘制为 PNG。 |
| `standalone_demo` | 是 | 展示 SDK 自有 Node；双臂到零并打开双夹爪。 |
| `existing_node_demo` | 否 | 展示借用已有 Node、外部 Executor 和资源所有权。 |
| `waypoint_recorder` | 否 | 按 S 保存 `map -> base_link` 当前点位。 |
| `waypoint_navigation_demo` | 是 | 演示清图、非阻塞下发、暂停/恢复和当前位姿。 |

每个样例均包含入口、默认值、关键步骤和清理行为注释。硬件样例在顶部明确标记。

## 9. ROS Backend 映射

| 公共能力 | ROS 接口 |
|---|---|
| 状态 | Topic `/system/get_info` |
| 模式 | Service `/system/set_ctrl_mode` |
| 六组轴使能 | Service `/system/enable` |
| 头部/腰部 | Service `/system/jnts_ctrl` |
| 夹爪 | Service `/system/gripper_ctrl` |
| 双臂 Hold | Service `/arm_driver/hold_on` |
| MoveJ | Service `/arm_driver/joint_move` |
| MoveL | Service `/arm_driver/linear_move` |
| MoveP | Service `/arm_driver/point_move` |
| 文本播报 | Service `/voice/speak_text` |
| 音频播放 | Service `/voice/play_audio` |
| 单点导航 | Action `/navigate_to_pose` |
| 多点导航 | Action `/navigate_through_poses` |
| 清局部代价地图 | Service `/local_costmap/clear_entirely_local_costmap` |
| 清全局代价地图 | Service `/global_costmap/clear_entirely_global_costmap` |
| 导航当前位姿 | TF `map -> base_link` |
| 导航软暂停/恢复 | 取消当前 Action，并在恢复时重新下发原目标 |

MoveJ 请求与正式 Demo 一样显式设置双臂模式、目标、速度结构、全零 effort 和
`zone=0`。头腰请求显式保留未选轴的 `index=False`。

## 10. 异常

所有公共异常继承 `WheelloongSdkError`：

| 异常 | 含义 |
|---|---|
| `ValidationError` | 参数在接触 Backend 前就不合法。 |
| `ServiceUnavailableError` | ROS Service 或 Action Server 不可用。 |
| `CommandTimeoutError` | 服务、状态或导航未在截止时间内完成。 |
| `RobotStateError` | 状态过期、存在活动错误或不适合当前动作。 |
| `RobotCommandError` | 服务返回非零机器人错误码。 |
| `BackendError` | ROS Context、Future 或 Backend 本身失败。 |
| `NavigationRejectedError` | Action Server 拒绝导航目标。 |
| `NavigationCancelledError` | 导航在成功前被取消。 |
| `NavigationError` | 导航中止或其他非成功终态。 |

任务程序通常只需要捕获 `WheelloongSdkError`，需要针对性恢复时再捕获子类。

## 11. 并发与控制仲裁

当前 `ROS2Backend` 直接连接机器人已有 Topic、Service 和 Action。它不提供跨进程
独占锁，因此不能阻止前端或另一任务同时下发命令。

- AUTO 期间如果前端切换 Manual，SDK 的 AUTO 状态检查会失败并停止继续采样。
- 直连手臂、夹爪或导航仍可能与其他进程竞争。
- 急停和操作员接管始终应拥有最高优先级。

未来可增加统一 Robot Server 和租约/仲裁机制。只要 Server 实现 `RobotBackend`，任务
代码仍保持 `robot.arms.move_j(...)` 等调用，仅创建 Robot 的位置需要更换 Backend。

## 12. 文件说明

以下列出包内全部人工维护文件；`__pycache__`、`*.pyc` 和 `.pytest_cache` 是运行生成
缓存，不属于 SDK 功能，可删除且不应提交。

### 12.1 根目录

| 文件 | 功能 |
|---|---|
| `README.md` | 安装、API、样例、默认值、文件职责和排错说明。 |
| `DELIVERY.md` | 双架构开发约束、源码保护方向和最终交付边界。 |
| `package.xml` | ROS 2 包名、版本、构建类型及运行/测试依赖。 |
| `setup.py` | setuptools/ament_python 安装元数据和 Python 包发现。 |
| `setup.cfg` | ROS 2 Python 可执行脚本安装目录配置。 |
| `pytest.ini` | pytest 搜索目录及禁用 launch 测试插件。 |
| `resource/wheelloong_auto_sdk` | ament index 包标记文件，内容为空是正常的。 |

### 12.2 公共 SDK 包

| 文件 | 功能 |
|---|---|
| `wheelloong_auto_sdk/__init__.py` | 稳定顶层导出、公共类型、异常和版本号。 |
| `wheelloong_auto_sdk/robot.py` | `Robot` 总入口、Backend 装配和资源所有权。 |
| `wheelloong_auto_sdk/system.py` | 状态、AUTO/IDLE、使能请求和状态确认。 |
| `wheelloong_auto_sdk/arms.py` | Hold、MoveJ、MoveL、MoveP、校验和超时 Hold。 |
| `wheelloong_auto_sdk/body.py` | 头部/腰部组合及便捷运动接口。 |
| `wheelloong_auto_sdk/gripper.py` | 左/右/双夹爪 set/open/close。 |
| `wheelloong_auto_sdk/navigation.py` | 下发、软暂停/恢复、清图、位姿和句柄管理。 |
| `wheelloong_auto_sdk/waypoints.py` | TXT 点位读取、校验、编号和追加保存。 |
| `wheelloong_auto_sdk/voice.py` | 文本播报和本地音频播放。 |
| `wheelloong_auto_sdk/session.py` | AUTO 会话进入、异常清理和正常退出。 |
| `wheelloong_auto_sdk/models.py` | 无 ROS 类型的数据类、枚举和数值校验。 |
| `wheelloong_auto_sdk/errors.py` | 公共异常层次及机器人错误信息封装。 |

### 12.3 Backend

| 文件 | 功能 |
|---|---|
| `wheelloong_auto_sdk/backend/__init__.py` | 导出 Backend 契约和 Mock 实现。 |
| `wheelloong_auto_sdk/backend/base.py` | `RobotBackend` 与导航句柄抽象契约。 |
| `wheelloong_auto_sdk/backend/ros2.py` | ROS Topic/Service/Action 转换和运行时管理。 |
| `wheelloong_auto_sdk/backend/mock.py` | 记录语义调用并模拟状态，用于无硬件测试。 |

### 12.4 样例

| 文件 | 功能 |
|---|---|
| `examples/auto_oscillation_demo.py` | 振荡、MoveP、MoveL 和清理硬件流程。 |
| `examples/auto_session_demo.py` | 两个独立函数组成的无目标运动 AUTO 生命周期样例。 |
| `examples/simulate_oscillation_targets.py` | 纯数学轨迹模拟及四组 TXT 生成器。 |
| `examples/plot_oscillation_targets.py` | 四组模拟目标的 2×2 PNG 绘图工具。 |
| `examples/standalone_demo.py` | SDK 独立运行及真实双臂/夹爪调用样例。 |
| `examples/existing_node_demo.py` | 已有 Node、Executor 和关闭顺序样例。 |
| `examples/waypoint_recorder.py` | TF 当前定位点位交互保存程序。 |
| `examples/waypoint_navigation_demo.py` | 完整导航管理公共接口调用样例。 |
| `waypoint/waypoints.txt` | 用户保存的编号导航点位，CSV 风格纯文本。 |

### 12.5 测试

| 文件 | 功能 |
|---|---|
| `test/test_public_api.py` | 公共校验、默认值、导航管理和系统步骤测试。 |
| `test/test_ros2_lifecycle.py` | ROS 所有权和真实生成请求字段测试；不发送。 |
| `test/test_auto_oscillation_example.py` | 姿态、增量和九阶段调用测试。 |
| `test/test_auto_session_example.py` | 确认纯会话样例不发送任何目标运动。 |
| `test/test_waypoint_navigation_examples.py` | 点位文件和完整导航接口顺序测试。 |
| `test/test_ros2_entry_points.py` | 确认所有样例均注册为可加载的 ROS 2 可执行入口。 |

### 12.6 VS Code 工作区配置

以下文件位于工作区根目录 `/home/niic/auto_sdk_ws/.vscode`：

| 文件 | 功能 |
|---|---|
| `.vscode/extensions.json` | 推荐 Python、Pylance 和 ROS 扩展。 |
| `.vscode/settings.json` | Python 解释器、搜索路径、pytest 和终端配置。 |
| `.vscode/python.env` | Pylance/调试时使用的主系统和 SDK `PYTHONPATH`。 |
| `.vscode/ros_bashrc` | 集成终端启动时 source ROS、主系统和 SDK。 |
| `.vscode/tasks.json` | `SDK: colcon build` 和 `SDK: colcon test` 任务。 |

## 13. 构建与测试

```bash
cd /home/niic/auto_sdk_ws
source /opt/ros/humble/setup.bash
source /home/niic/wheelloong/install/setup.bash

colcon build --symlink-install --packages-select wheelloong_auto_sdk
source /home/niic/auto_sdk_ws/install/setup.bash

colcon test \
  --packages-select wheelloong_auto_sdk \
  --event-handlers console_direct+
colcon test-result --verbose
```

也可直接在源码包目录运行：

```bash
cd /home/niic/auto_sdk_ws/src/wheelloong_auto_sdk
python3 -m compileall -q wheelloong_auto_sdk examples test
python3 -m pytest -q
```

测试使用 `MockBackend` 或截获生成的 ROS 请求，不会向真实机器人发送运动。

## 14. 开发与交付边界

当前工程继续使用纯 Python 源码和 `colcon --symlink-install` 开发，不生成 Wheel，
也不启用 Cython。正式平台以 `aarch64 + Python 3.10 + ROS 2 Humble` 为主，
同时保持相同源码和公共 API 在 `x86_64` 上兼容。运行时代码不得依赖具体用户名、
Home 目录或 CPU 架构；VS Code、测试和工作空间路径属于开发配置，不进入最终交付。

后续受保护构建、双架构产物和交付边界记录在 `DELIVERY.md`。当前阶段只维护源码，
不会生成或提交 `.whl`、`.so` 和 Cython 中间文件。

## 15. 常见问题

### 导入失败或 VS Code 无法跳转

确认打开 `/home/niic/auto_sdk_ws`，选择 `/usr/bin/python3`，然后重新 source 并
构建。Pylance 修改后可执行 “Developer: Reload Window”。

### 服务或状态不可见

确认机器人系统已启动，并且终端 source 了与运行系统相同的 ROS 环境、Domain 和
`ROS_LOCALHOST_ONLY` 设置。仅 source SDK 不会启动机器人节点。

### `speed=0.0` 是否表示不运动

不是。对双臂接口，0 表示由驱动代入默认速度；Shiloong 当前 MoveJ/MoveP 为
`0.628 rad/s`，MoveL 为 `200 mm/s`。对头腰 `JntsCtrl.speed`，该字段当前预留。

### 为什么升降传负数

当前运动层以最高点为 `0 mm`，向下为负值。正式振荡 Demo 的 100 mm 幅度实际发送
`0 → -100 → 0 mm`。

### 为什么提供 Backend 分层

它让任务程序不依赖 ROS 消息，并可用 Mock 测试。以后加入 C++/Python Server、跨进程
仲裁或高频控制时，可以更换 Backend，而不修改任务人员的动作调用方式。
