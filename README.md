# 侍龙 L4 AUTO SDK

当前工程用于 **侍龙 L4（Shiloong）** 轮式人形机器人，提供基于 ROS 2 Humble 的
Python 任务 SDK。SDK 封装了 AUTO 模式下的系统控制、双臂、头腰、夹爪、导航和
语音能力，业务代码无需直接构造 ROS Service、Topic 或 Action 消息。

工程目录、ROS 包名和部分底层接口仍使用 `wheelloong`，例如主系统目录
`/home/niic/wheelloong` 和包名 `wheelloong_auto_sdk`。这些是现有系统保留的兼容
名称，不代表当前机器人型号；本机运行涉及型号选择的 Demo 时应使用
`--robot shiloong`。

## 主要能力

- 双臂 MoveJ、MoveL、MoveP 与 Hold 控制
- 头部、腰部和左右夹爪控制
- AUTO/IDLE 模式切换、轴使能与安全退出流程
- Nav2 单点及多点导航、暂停、恢复、取消和代价地图清理
- 文本播报与机器人本地音频播放
- 自动创建并管理独立 ROS Context、Node 和 Executor
- ROS 2 真实后端与无硬件测试用 Mock 后端

## 工程结构

```text
auto_sdk_ws/
├── .vscode/                     # VS Code、ROS 终端、构建和测试任务
├── .gitignore                   # Git 忽略规则
├── README.md                    # 安装、接口、示例和安全说明
├── scripts/
│   └── source_env.sh            # 一次加载 ROS、主系统、语音和 SDK 环境
├── src/
│   └── wheelloong_auto_sdk/
│       ├── wheelloong_auto_sdk/
│       │   ├── robot.py         # Robot 总入口与 Backend 装配
│       │   ├── system.py        # 状态、模式和轴使能
│       │   ├── arms.py          # Hold、MoveJ/MoveL/MoveP 及局部偏移运动
│       │   ├── body.py          # 头部和腰部控制
│       │   ├── gripper.py       # 左、右和双夹爪控制
│       │   ├── navigation.py    # 导航下发、反馈、暂停和取消
│       │   ├── voice.py         # 文本播报和音频播放
│       │   ├── waypoints.py     # 点位文件读取与保存
│       │   ├── session.py       # AUTO 会话和退出清理
│       │   ├── profiles.py      # 机型标准姿态与机械限位
│       │   ├── models.py        # 公共数据类型与数值校验
│       │   ├── errors.py        # 公共异常体系
│       │   └── backend/
│       │       ├── base.py      # Backend 抽象接口
│       │       ├── ros2.py      # ROS Topic/Service/Action 实现
│       │       └── mock.py      # 无硬件测试实现
│       ├── examples/
│       │   ├── mobile_manipulation_demo.py # 导航、本体、语音完整流程
│       │   ├── separation_demo.py          # 八步分部件真机运动
│       │   ├── auto_oscillation_demo.py    # AUTO 全身振荡流程
│       │   ├── waypoint_navigation_demo.py # 编号点位导航
│       │   ├── waypoint_recorder.py        # 交互式记录导航点位
│       │   ├── voice_demo.py               # 文本与本地音频测试
│       │   ├── auto_session_demo.py        # AUTO 生命周期测试
│       │   └── simulated_targets/          # 离线目标生成、数据和绘图
│       ├── test/                # 公共 API、生命周期和示例测试
│       ├── waypoint/            # 编号导航点位文件
│       ├── package.xml          # ROS 2 包清单与依赖
│       ├── setup.py             # ament_python 安装和入口点
│       ├── setup.cfg            # ROS 2 可执行文件安装位置
│       └── pytest.ini           # pytest 配置
├── build/                       # colcon 生成，不纳入版本管理
├── install/                     # colcon 生成，不纳入版本管理
└── log/                         # colcon 生成，不纳入版本管理
```

## 安装

运行环境为 Ubuntu 22.04、系统 Python 3.10 和 ROS 2 Humble。侍龙 L4 主系统应已
安装在 `/home/niic/wheelloong/install`，语音系统位于
`/home/niic/wheelloong_voice/install`。

先通过桌面的 `Shiloong_start` 启动机器人主系统。构建 SDK 时建议退出 Conda 或
Miniforge 环境，避免其 Python 版本覆盖 ROS 2 使用的系统 Python。

```bash
cd /home/niic
git clone https://github.com/PANJIACHENG-98/auto_sdk_ws.git
cd auto_sdk_ws

source /opt/ros/humble/setup.bash
source /opt/wheelloong/wheelloong_env.sh
source /home/niic/wheelloong/install/setup.bash
source /home/niic/wheelloong_os_env/niic/deploy/resources/wheelloong_env.sh
source /home/niic/wheelloong_os_env/niic/deploy/resources/shiloong_env.sh
source /home/niic/wheelloong_voice/install/setup.bash
colcon build --packages-select wheelloong_auto_sdk
source install/setup.bash
```

上面是首次构建时需要的完整底层环境顺序。`source_env.sh` 还会在最后加载
本工作区的 `install/setup.bash`，因此它适合已完成至少一次构建后使用。

构建完成后，每次打开新终端只需执行下面一条命令，即可依次加载 ROS、机器人主系统、
语音系统和本工作区环境：

```bash
source /home/niic/auto_sdk_ws/scripts/source_env.sh
```

脚本会依次打印已加载的 ROS 2、平台公共环境、主系统、部署环境、
侍龙机型环境、语音系统和 AUTO SDK；任一文件不存在时会立即失败并给出路径。

如果 `colcon build` 报告 `build/install/log` 无写入权限，先确认三个目录中
没有需要保留的手工文件，再恢复当前用户所有权后重新构建：

```bash
sudo chown -R "$USER":"$USER" \
  /home/niic/auto_sdk_ws/build \
  /home/niic/auto_sdk_ws/install \
  /home/niic/auto_sdk_ws/log
colcon build --packages-select wheelloong_auto_sdk
source install/setup.bash
```

如果当前终端已进入 Conda/Miniforge 环境，可先执行 `conda deactivate`。导入检查可
明确使用系统 Python：

```bash
/usr/bin/python3 -c "from wheelloong_auto_sdk import Robot; print('SDK import OK')"
```

修改 SDK 后，可执行完整构建和测试：

```bash
cd /home/niic/auto_sdk_ws
colcon build --packages-select wheelloong_auto_sdk
source install/setup.bash
colcon test --packages-select wheelloong_auto_sdk --event-handlers console_direct+
colcon test-result --verbose
```

## 最小任务结构

普通任务程序只需要使用 `Robot.standalone()` 创建运行时。需要本体运动时，建议用
`auto_session()` 统一完成 AUTO、轴使能和退出清理；业务动作写在内层 `with` 中：

```python
from wheelloong_auto_sdk import AxisSelection, Robot

with Robot.standalone(node_name="my_shiloong_task") as robot:
    with robot.auto_session(required_axes=AxisSelection.all()):
        # 在这里按任务顺序调用 robot.arms/body/gripper/navigation/voice。
        pass
```

`auto_session()` 退出时会尽力取消导航、双臂 Hold、切换 IDLE 并去使能。由于双臂
去使能后没有抱闸，不能把该自动清理当成物理支撑；离开会话前应先把双臂放到可承托
位置或准备可靠外部支撑。

## 可用接口总览

<table>
  <thead><tr><th>分类</th><th>接口</th><th>作用</th><th>Demo 直接调用</th></tr></thead>
  <tbody>
    <tr><td rowspan="2">机器人入口</td><td><code>Robot.standalone()</code></td><td>创建并拥有私有 ROS Context、Node 和 Executor。</td><td>是：<code>mobile_manipulation_demo.py:542</code>、<code>separation_demo.py:249</code> 等</td></tr>
    <tr><td><code>Robot.with_backend()</code></td><td>注入 Mock、ROS 或扩展 Backend。</td><td>无</td></tr>
    <tr><td rowspan="4">生命周期</td><td><code>robot.node</code></td><td>访问后端使用的 ROS Node。</td><td>无</td></tr>
    <tr><td><code>robot.state()</code></td><td>获取无 ROS 类型的系统状态快照。</td><td>无</td></tr>
    <tr><td><code>robot.auto_session()</code></td><td>统一管理 AUTO、轴使能、Hold、导航取消、模式恢复和退出清理。</td><td>是：<code>auto_session_demo.py:65</code>、<code>separation_demo.py:250</code> 等</td></tr>
    <tr><td><code>robot.close()</code></td><td>取消活动导航并释放 SDK 拥有的 ROS 资源。</td><td>无（Demo 通过 <code>with</code> 自动调用）</td></tr>
    <tr><td rowspan="4">机型配置</td><td><code>get_robot_profile()</code></td><td>按机型名取得标准姿态和机械限位。</td><td>是：<code>auto_oscillation_demo.py:322</code></td></tr>
    <tr><td><code>RobotProfile.work_arm_joints()</code></td><td>取得左右臂标准工作位，单位 rad。</td><td>是：<code>auto_oscillation_demo.py:331</code></td></tr>
    <tr><td><code>RobotProfile.return_arm_joints()</code></td><td>取得左右臂安全结束位，单位 rad。</td><td>是：<code>auto_oscillation_demo.py:302</code></td></tr>
    <tr><td><code>RobotProfile.validate_arm_degrees()</code></td><td>按机型机械限位校验七关节度制目标。</td><td>是：<code>mobile_manipulation_demo.py:150</code>、<code>separation_demo.py:99</code></td></tr>
    <tr><td rowspan="3">笛卡尔位姿</td><td><code>CartesianPoseMM.from_rpy()</code></td><td>用毫米位置和弧度制 RPY 创建 TCP 位姿。</td><td>无</td></tr>
    <tr><td><code>CartesianPoseMM.from_rpy_degrees()</code></td><td>用毫米位置和度制 RPY 创建 TCP 位姿。</td><td>是：<code>mobile_manipulation_demo.py:321</code>、<code>separation_demo.py:197</code> 等</td></tr>
    <tr><td><code>CartesianPoseMM.offset_local()</code></td><td>叠加毫米平移和绕当前 TCP 局部轴定义的旋转偏移。</td><td>无（由 offset 运动接口内部调用）</td></tr>
    <tr><td rowspan="9">系统与使能</td><td><code>system.state()</code></td><td>获取满足新鲜度要求的系统状态。</td><td>是：<code>mobile_manipulation_demo.py:223</code>、<code>auto_session_demo.py:63</code></td></tr>
    <tr><td><code>system.set_control_mode()</code></td><td>请求切换 AUTO 或 IDLE。</td><td>是：<code>mobile_manipulation_demo.py:235</code>、<code>mobile_manipulation_demo.py:508</code></td></tr>
    <tr><td><code>system.wait_for_control_mode()</code></td><td>等待控制模式实际生效。</td><td>是：<code>mobile_manipulation_demo.py:239</code>、<code>mobile_manipulation_demo.py:512</code></td></tr>
    <tr><td><code>system.set_enabled()</code></td><td>精确设置六组轴的完整使能目标。</td><td>无（由 <code>auto_session()</code> 调用）</td></tr>
    <tr><td><code>system.wait_for_enabled()</code></td><td>等待实际使能状态与目标一致。</td><td>是：<code>mobile_manipulation_demo.py:247</code></td></tr>
    <tr><td><code>system.enable_all()</code></td><td>将六组运动轴全部设为使能。</td><td>是：<code>mobile_manipulation_demo.py:246</code></td></tr>
    <tr><td><code>system.disable_all()</code></td><td>请求关闭全部运动轴使能。</td><td>是：<code>mobile_manipulation_demo.py:522</code></td></tr>
    <tr><td><code>system.wait_for_all_disabled()</code></td><td>等待全部运动轴完成去使能。</td><td>是：<code>mobile_manipulation_demo.py:523</code></td></tr>
    <tr><td><code>system.require_ready()</code></td><td>一次确认 AUTO、所需轴使能和无活动错误。</td><td>是：<code>auto_oscillation_demo.py:254</code></td></tr>
    <tr><td rowspan="6">双臂</td><td><code>arms.hold()</code></td><td>停止双臂现有运动并保持当前位置。</td><td>是：<code>mobile_manipulation_demo.py:253</code>、<code>mobile_manipulation_demo.py:501</code></td></tr>
    <tr><td><code>arms.move_j()</code></td><td>执行单臂或双臂关节空间运动。</td><td>是：<code>mobile_manipulation_demo.py:276</code>、<code>separation_demo.py:156</code> 等</td></tr>
    <tr><td><code>arms.move_l()</code></td><td>执行绝对或增量笛卡尔直线运动。</td><td>无（由 offset 接口内部调用）</td></tr>
    <tr><td><code>arms.move_p()</code></td><td>对目标位姿求逆解后执行关节运动。</td><td>无（由 offset 接口内部调用）</td></tr>
    <tr><td><code>arms.move_p_offset()</code></td><td>Hold 并读取当前 TCP，按 arm_driver 位置增量和 TCP 局部旋转执行 MoveP。</td><td>是：<code>mobile_manipulation_demo.py:320</code>、<code>separation_demo.py:196</code> 等</td></tr>
    <tr><td><code>arms.move_l_offset()</code></td><td>Hold 并读取当前 TCP，按 arm_driver 位置增量和 TCP 局部旋转执行 MoveL。</td><td>是：<code>mobile_manipulation_demo.py:336</code>、<code>separation_demo.py:212</code> 等</td></tr>
    <tr><td rowspan="3">头腰</td><td><code>body.move()</code></td><td>在一次请求中控制任意头部或腰部轴。</td><td>是：<code>auto_oscillation_demo.py:210</code></td></tr>
    <tr><td><code>body.move_head()</code></td><td>控制头部俯仰和转动。</td><td>是：<code>mobile_manipulation_demo.py:300</code>、<code>separation_demo.py:176</code></td></tr>
    <tr><td><code>body.move_waist()</code></td><td>控制腰部俯仰和升降。</td><td>是：<code>mobile_manipulation_demo.py:310</code>、<code>separation_demo.py:186</code></td></tr>
    <tr><td rowspan="3">夹爪</td><td><code>gripper.set()</code></td><td>按官方比例设置夹爪位置：0 张开、1 闭合。</td><td>是：<code>auto_oscillation_demo.py:222</code></td></tr>
    <tr><td><code>gripper.open()</code></td><td>将指定夹爪张开到协议位置 0。</td><td>是：<code>mobile_manipulation_demo.py:352</code>、<code>separation_demo.py:228</code></td></tr>
    <tr><td><code>gripper.close()</code></td><td>将指定夹爪闭合到协议位置 1。</td><td>是：<code>mobile_manipulation_demo.py:360</code>、<code>separation_demo.py:236</code></td></tr>
    <tr><td rowspan="4">导航下发</td><td><code>navigation.navigate()</code></td><td>传入 NavigationPose 序列并自动选择单点或多点 Action。</td><td>无</td></tr>
    <tr><td><code>navigation.navigate_waypoints()</code></td><td>传入点位编号序列，自动读取文件并执行导航。</td><td>是：<code>mobile_manipulation_demo.py:262</code>、<code>mobile_manipulation_demo.py:374</code></td></tr>
    <tr><td><code>navigation.navigate_to()</code></td><td>下发单点 Nav2 导航目标。</td><td>无（由 <code>navigate()</code> 调用）</td></tr>
    <tr><td><code>navigation.navigate_through()</code></td><td>按顺序下发多点 Nav2 导航目标。</td><td>无（由 <code>navigate()</code> 调用）</td></tr>
    <tr><td rowspan="7">导航管理</td><td><code>NavigationHandle.feedback</code></td><td>获取最近一次导航反馈。</td><td>无</td></tr>
    <tr><td><code>NavigationHandle.done</code></td><td>判断导航是否已进入最终状态。</td><td>无</td></tr>
    <tr><td><code>NavigationHandle.paused</code></td><td>判断导航是否处于 SDK 软暂停状态。</td><td>无</td></tr>
    <tr><td><code>NavigationHandle.wait()</code></td><td>等待导航完成，并按配置处理超时。</td><td>是：<code>mobile_manipulation_demo.py:267</code>、<code>mobile_manipulation_demo.py:379</code></td></tr>
    <tr><td><code>NavigationHandle.pause()</code></td><td>取消当前 Action 并保存原目标。</td><td>无</td></tr>
    <tr><td><code>NavigationHandle.resume()</code></td><td>重新下发软暂停时保存的目标。</td><td>无</td></tr>
    <tr><td><code>NavigationHandle.cancel()</code></td><td>主动取消导航并等待最终状态。</td><td>无（Demo 由会话统一取消）</td></tr>
    <tr><td rowspan="4">导航辅助</td><td><code>navigation.clear_local_costmap()</code></td><td>清除完整局部代价地图。</td><td>是：<code>mobile_manipulation_demo.py:256</code>、<code>mobile_manipulation_demo.py:368</code></td></tr>
    <tr><td><code>navigation.clear_global_costmap()</code></td><td>清除完整全局代价地图。</td><td>是：<code>mobile_manipulation_demo.py:259</code>、<code>mobile_manipulation_demo.py:371</code></td></tr>
    <tr><td><code>navigation.current_pose()</code></td><td>读取当前 map → base_link TF 位姿。</td><td>是：<code>waypoint_recorder.py:49</code>、<code>waypoint_navigation_demo.py:65</code></td></tr>
    <tr><td><code>navigation.cancel_all()</code></td><td>尽力取消 SDK 管理的全部活动目标。</td><td>是：<code>mobile_manipulation_demo.py:494</code></td></tr>
    <tr><td rowspan="2">语音</td><td><code>voice.speak()</code></td><td>调用语音服务播报文本。</td><td>是：<code>mobile_manipulation_demo.py:480</code>、<code>voice_demo.py:85</code></td></tr>
    <tr><td><code>voice.play()</code></td><td>播放机器人语音目录中的本地音频。</td><td>是：<code>voice_demo.py:98</code></td></tr>
    <tr><td rowspan="4">点位文件</td><td><code>default_waypoint_path()</code></td><td>按环境和工作区规则选择默认点位文件。</td><td>是：<code>mobile_manipulation_demo.py:106</code>、<code>waypoint_navigation_demo.py:41</code> 等</td></tr>
    <tr><td><code>load_waypoints()</code></td><td>读取并校验编号点位。</td><td>是：<code>mobile_manipulation_demo.py:195</code>（启动前检查点位 1、2）</td></tr>
    <tr><td><code>next_waypoint_id()</code></td><td>计算下一个可用正整数点位编号。</td><td>无（由 <code>append_waypoint()</code> 调用）</td></tr>
    <tr><td><code>append_waypoint()</code></td><td>将点位追加保存到文件。</td><td>是：<code>waypoint_recorder.py:54</code></td></tr>
  </tbody>
</table>

### `system.set_enabled()` 与 `system.enable_all()` 的区别

- `system.set_enabled(axes)` 接收一个 `AxisSelection`，用于一次明确设置六组轴的完整
  状态。例如只使能双臂时，其余头部和腰部轴会被明确设为不使能。
- `system.enable_all()` 不接收轴选择，是
  `system.set_enabled(AxisSelection.all())` 的便捷写法，固定请求使能全部六组轴。
- 两者都只等待使能服务返回；要确认机器人状态已经真正生效，还需调用
  `system.wait_for_enabled(axes)`。`enable_all()` 当前没有对应的
  `wait_for_all_enabled()`，可使用 `wait_for_enabled(AxisSelection.all())`。

函数源码 docstring 中给出了参数、单位、默认值、返回值和异常说明。

## 公共数据类型

| 类型 | 主要成员 | 作用 |
|---|---|---|
| `AxisSelection` | `all()`、`none()`、`is_empty`、`satisfied_by()` | 描述左/右臂、头部和腰部六组轴的使能目标。 |
| `ArmSide` | `LEFT`、`RIGHT`、`DUAL` | 选择左侧、右侧或双侧手臂/夹爪。 |
| `ControlMode` | `AUTO`、`IDLE` | 表示 SDK 允许业务层请求的控制模式。 |
| `MotionMode` | `ABSOLUTE`、`INCREMENTAL` | 表示机械臂绝对或增量运动模式。 |
| `CartesianPoseMM` | `validated()`、`from_rpy()`、`from_rpy_degrees()`、`offset_local()` | 使用毫米位置和四元数描述机械臂 TCP 位姿。 |
| `RobotProfile` | `work_arm_joints()`、`return_arm_joints()`、`validate_arm_degrees()` | 保存机型工作位、结束位及机械限位。 |
| `NavigationMode` | `DEFAULT`、`PRECISE`、`PRECISE_FORWARD`、`PRECISE_BACKWARD` | 选择默认或三种精确导航策略。 |
| `NavigationPose` | `validated()` | 使用米、弧度和坐标系描述平面导航目标。 |
| `SystemState` | `age_sec` 及模式、轴、位姿、夹爪、错误字段 | 保存无 ROS 消息类型依赖的系统快照。 |
| `CommandResult` | `operation`、`error_code`、`message`、`ok` | 保存普通指令的返回结果。 |
| `NavigationFeedback` | 剩余距离、预计时间、恢复次数等字段 | 保存导航过程反馈。 |
| `NavigationResult` | `status`、`succeeded` | 保存导航最终结果。 |
| `Waypoint` | `waypoint_id`、`pose` | 组合点位编号与 `NavigationPose`。 |

## 公共异常

所有公共异常均继承 `WheelloongSdkError`。

| 异常 | 触发场景 |
|---|---|
| `WheelloongSdkError` | SDK 所有可预期错误的基类。 |
| `ValidationError` | 参数类型、范围、数量或组合不合法。 |
| `BackendError` | 通信后端发生通用错误。 |
| `ServiceUnavailableError` | ROS Service 或 Action Server 在截止时间前不可用。 |
| `CommandTimeoutError` | 服务、运动、状态确认或导航等待超时。 |
| `RobotStateError` | 状态缺失、过期、异常或不满足执行条件。 |
| `RobotCommandError` | 机器人服务明确拒绝命令或返回非零错误码。 |
| `NavigationError` | 导航错误的公共基类。 |
| `NavigationRejectedError` | Nav2 拒绝接受导航目标。 |
| `NavigationCancelledError` | 导航被取消且调用方要求成功结果。 |

> [!CAUTION]
> **双臂下电后没有抱闸。** 双臂去使能、驱动下电或整机断电后不能依靠
> `Hold` 保持姿态，可能在重力作用下下落。执行会话退出、`disable_all()`、
> 急停或关机前，必须先将双臂放到可承托的安全位置或使用可靠外部支撑，
> 并确保双臂下方及运动范围内无人员、线缆和易损设备。

## Demo 使用说明

运行任何 Demo 前，应完成以下准备：

- 通过桌面 `Shiloong_start` 启动侍龙 L4 主系统，并确认相关终端没有异常退出。
- 在当前终端执行 `source /home/niic/auto_sdk_ws/scripts/source_env.sh`。
- 确认机器人没有活动错误，急停可用，底盘路径和本体运动空间无人员及障碍物。
- 真机运动过程由操作员现场全程监护，不要仅依赖软件超时或 `Ctrl+C`。

可用 `ros2 run wheelloong_auto_sdk <示例名> --help` 查看当前安装版本实际接受的参数。

### 真机运动 Demo

| 示例 | 作用 | 运动部件 |
|---|---|---|
| `mobile_manipulation_demo` | 导航到点位 1 执行八步动作，再导航到点位 2 重复八步动作，播报后执行退出清理并去使能。 | 移动底盘、双臂、头部、腰部、夹爪 |
| `separation_demo` | 分八步验证左右臂 MoveJ、头、腰、MoveP、MoveL及双夹爪开合；每步完成后按回车继续。 | 左臂、右臂、头部、腰部、夹爪 |
| `auto_oscillation_demo` | 执行 AUTO 七阶段流程：工作姿态、正弦振荡、MoveP、MoveL、结束姿态和安全清理。 | 双臂、头部、腰部及可选夹爪 |
| `waypoint_navigation_demo` | 从点位文件读取编号点位，清理代价地图并执行单点或多点导航。 | 移动底盘 |

#### 移动操作播报完整流程

`mobile_manipulation_demo` 固定使用现有点位 1 和 2，启动前会检查两个
点位都存在。该文件不导入或调用任何其他 Demo，模式切换、使能、
导航、两轮 1–8 步、语音和退出清理均在任务函数中按顺序平铺调用。
两轮本体目标值与 `separation_demo` 一致；每个运动指令仍使用
`wait=True` 等待完成，相邻任务动作之间自动暂停 500 ms，不需要按回车。

```text
切换 AUTO 并使能全部本体轴
  → 导航到点位 1 → 八步动作
  → 导航到点位 2 → 八步动作
  → 语音播报
  → 取消活动导航 → 双臂 Hold → IDLE → 全部轴去使能
```

默认目标就是当前 `waypoint/waypoints.txt` 的 1、2 号点，完成后播报
“移动抓取任务已完成”：

```bash
ros2 run wheelloong_auto_sdk mobile_manipulation_demo \
  --robot shiloong \
  --file /home/niic/auto_sdk_ws/src/wheelloong_auto_sdk/waypoint/waypoints.txt \
  --speech-text "移动抓取任务已完成"
```

| 完整流程参数 | 默认值 | 说明 |
|---|---:|---|
| `--robot` | 必填：`shiloong` | 确认当前机器人为侍龙 L4。 |
| `--file` | SDK 运行时默认路径 | 必须同时包含点位 1 和 2。 |
| `--navigation-timeout` | `300` 秒 | 每段导航允许的最长时间。 |
| `--server-timeout` | `5` 秒 | Nav2 Action Server 和代价地图服务超时。 |
| `--speech-text` | `移动抓取任务已完成` | 最后传给 `voice.speak()` 的文本。 |
| `--speech-timeout` | `30` 秒 | 语音合成和播放超时。 |
| `--speech-call-timeout` | `35` 秒 | SDK 等待语音服务响应的最长时间。 |

其余双臂、头腰、夹爪、速度及超时参数与下方
`separation_demo` 相同。执行第二段导航时，双臂保持在第一轮 MoveL 返回后
的工作位、夹爪为闭合状态；运行前必须确认整条导航路径和两个点位周围
均无人员或障碍物。任意阶段发生异常或按下 `Ctrl+C` 都会触发同样的安全清理。
本处“下电”指全部本体运动轴去使能，不会关闭整机总电源或底盘主电源。

#### 分步部件运动

该示例的 `--robot` 必须填写 `shiloong`。八步依次为左臂 MoveJ、右臂 MoveJ、
头部、腰部、双臂 MoveP 偏移、双臂 MoveL 偏移、双夹爪张开和双夹爪
闭合。每一步都使用 `wait=True`，完成后按回车才继续下一步；最后一步直接结束。
MoveP 在 `arm_driver` 坐标轴使用左右对称的 `(y=±20, z=+30) mm` 位置增量，
MoveL 使用相反增量返回附近原位。程序不控制底盘，必须在交互式终端中运行。
任意阶段按 `Ctrl+C` 都会先
请求双臂 Hold，再切换 IDLE 并去使能：

```bash
ros2 run wheelloong_auto_sdk separation_demo --robot shiloong \
  --left-joints-deg 10 77 -80 20 -25 10 10 \
  --right-joints-deg -10 77 80 20 25 -10 10 \
  --head-pitch-deg 3 \
  --head-yaw-deg 3 \
  --waist-pitch-deg 3 \
  --waist-lift-mm -5
```

| 参数 | 必填/默认值 | 说明 |
|---|---|---|
| `--robot` | 必填：`shiloong` | 确认当前运行机型为侍龙 L4。 |
| `--left-joints-deg` | `10 77 -80 20 -25 10 10` | 左臂七关节绝对 MoveJ 目标，单位为度。 |
| `--right-joints-deg` | `-10 77 80 20 25 -10 10` | 右臂七关节绝对 MoveJ 目标，单位为度。 |
| `--head-pitch-deg` | `3` 度 | 头部俯仰目标。 |
| `--head-yaw-deg` | `3` 度 | 头部转动目标。 |
| `--waist-pitch-deg` | `3` 度 | 腰部俯仰目标。 |
| `--waist-lift-mm` | `-5` mm | 腰部升降绝对目标；最高位置为 0，向下为负。 |
| `--arm-velocity` | `0` rad/s | MoveJ 速度；0 使用驱动默认值。 |
| `--arm-acceleration` | `0` rad/s² | MoveJ 加速度；0 使用驱动默认值。 |
| `--body-speed-percent` | `0` | 头腰速度百分比；0 使用驱动默认值。 |
| `--move-p-speed` | `0.2` rad/s | MoveP 逆解后的关节速度。 |
| `--move-l-speed` | `10` mm/s | MoveL 的 TCP 直线速度。 |
| `--state-timeout` | `10` 秒 | AUTO、IDLE 和使能状态确认超时。 |
| `--service-timeout` | `3` 秒 | 模式、使能及 Hold 服务响应超时。 |
| `--motion-timeout` | `30` 秒 | 每一步阻塞运动的超时，最大 30 秒。 |

#### AUTO 振荡

`--robot` 是必填参数；当前机器人必须填写 `shiloong`。建议首次运行显式给出全部
主要轨迹参数：

```bash
ros2 run wheelloong_auto_sdk auto_oscillation_demo --robot shiloong \
  --duration 20 \
  --period 4 \
  --rate 5 \
  --amplitude-deg 3 \
  --lumbar-lift-amplitude-mm 10 \
  --control-gripper true
```

| 参数 | 必填/默认值 | 说明 |
|---|---|---|
| `--robot` | 必填：`shiloong` | 选择侍龙 L4 的双臂工作姿态和结束姿态。 |
| `--duration` | `20` 秒 | 振荡持续时间；设为 `0` 时持续运行到按下 Ctrl+C。 |
| `--period` | `4` 秒 | 一个完整正弦周期的时间，必须大于 0。 |
| `--rate` | `5` Hz | 目标服务下发频率，最大 20 Hz。 |
| `--amplitude-deg` | `3` 度 | 双臂、头部和腰部共用的角度振幅，最大 10 度。 |
| `--lumbar-lift-amplitude-mm` | `100` mm | 腰部从零点向下的行程，范围 0～100 mm；上例显式使用 10 mm。 |
| `--control-gripper` | `true` | 是否让双夹爪在 0～1 之间随轨迹运动，可填 `true/false`、`on/off`、`yes/no` 或 `1/0`。 |
| `--work-hold` | `2` 秒 | 到达工作姿态后、开始振荡前的保持时间；`--zero-hold` 是同义参数。 |
| `--state-timeout` | `10` 秒 | 确认 AUTO、IDLE、使能和去使能状态的超时。 |
| `--service-timeout` | `3` 秒 | 每次普通服务调用的等待超时。 |
| `--motion-timeout` | `60` 秒 | 工作姿态和结束姿态阻塞运动的超时。 |
| `--arm-velocity` | `0` rad/s | MoveJ 速度；`0` 表示使用驱动默认值。 |
| `--arm-acceleration` | `0` rad/s² | MoveJ 加速度；`0` 表示使用驱动默认值。 |

#### 点位导航

至少需要提供一个正整数点位编号。Demo 只把编号序列传给
`navigation.navigate_waypoints()`；SDK 自动读取点位文件，并按数量选择单点或
多点导航：

仓库当前的 `src/wheelloong_auto_sdk/waypoint/waypoints.txt` 包含以下示例点位；
坐标均属于当前地图，换地图、定位重置或现场环境变化后必须重新核验，不能仅凭编号
判断目标安全：

| 点位 | x（m） | y（m） | yaw（rad） | frame |
|---:|---:|---:|---:|---|
| 1 | -2.592143711 | 0.183998449 | 3.010553199 | `map` |
| 2 | -3.823100147 | 0.301051218 | 2.851035871 | `map` |
| 3 | -3.085393559 | 0.269291467 | 3.039728904 | `map` |

```bash
# 导航到 1 号点位
ros2 run wheelloong_auto_sdk waypoint_navigation_demo 1

# 依次导航到 1、2、3 号点位
ros2 run wheelloong_auto_sdk waypoint_navigation_demo 1 2 3 \
  --navigation-timeout 300 \
  --server-timeout 5

# 使用指定点位文件
ros2 run wheelloong_auto_sdk waypoint_navigation_demo 1 \
  --file /absolute/path/to/waypoints.txt
```

| 参数 | 必填/默认值 | 说明 |
|---|---|---|
| `waypoint_ids` | 至少一个正整数 | 要执行的点位编号，可连续提供多个。 |
| `--file` | SDK 运行时默认路径 | 点位文件路径；可由 `WHEELLOONG_AUTO_SDK_WAYPOINT_FILE` 指定，否则安装环境使用用户数据目录。 |
| `--navigation-timeout` | `300` 秒 | 导航结果等待超时，必须大于 0。 |
| `--server-timeout` | `5` 秒 | Nav2 Action Server 和相关服务等待超时，必须大于 0。 |

### 独立语音 Demo

`voice_demo` 用自己的 ROS 2 Node 独立调用语音服务，不切换 AUTO、
不使能轴，也不会下发任何运动目标。运行前要确认
`Shiloong_start` 打开的 `Voice` 终端中 `ros2 launch voice voice.launch.py`
仍在运行；只执行 `source_env.sh` 不会启动语音节点。默认只验证文本播报：

```bash
ros2 run wheelloong_auto_sdk voice_demo \
  --text "侍龙 L4 语音接口测试"
```

要在一次运行中依次验证 `voice.speak()` 和 `voice.play()`，先将音频
文件部署到语音节点的 `audio_dir` 中；默认目录是
`$WHEELLOONG_HOME/voice`。`--audio-file` 只能提供文件名，不能带目录路径：

```bash
ros2 run wheelloong_auto_sdk voice_demo \
  --text "文本播报测试" \
  --audio-file notice.wav \
  --play-count 1
```

只验证本地音频播放：

```bash
ros2 run wheelloong_auto_sdk voice_demo \
  --skip-speak \
  --audio-file notice.wav
```

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `--text` | `侍龙 L4 语音接口测试` | 传给 `voice.speak()` 的非空文本。 |
| `--skip-speak` | 关闭 | 跳过文本播报；启用时必须同时提供 `--audio-file`。 |
| `--audio-file` | 无 | 语音服务目录中的纯文件名；提供后调用 `voice.play()`。 |
| `--play-count` | `1` | 音频播放次数，必须大于等于 1。 |
| `--wait` | `true` | 是否等待每项播放完成，支持 `true/false`、`on/off`。 |
| `--speech-timeout` | `30` 秒 | 传给文本合成/播放服务的超时。 |
| `--call-timeout` | `35` 秒 | SDK 等待文本播报服务响应的最长时间。 |
| `--audio-timeout` | `120` 秒 | SDK 等待音频播放服务响应的最长时间。 |

### 状态与生命周期 Demo

`auto_session_demo` 不发送目标轨迹，但会切换 AUTO/IDLE、使能全部轴、执行双臂
Hold 并在退出时去使能，因此仍需按真机操作要求进行监护。

```bash
ros2 run wheelloong_auto_sdk auto_session_demo \
  --observe-seconds 2 \
  --state-timeout 10 \
  --service-timeout 3
```

| `auto_session_demo` 参数 | 默认值 | 说明 |
|---|---:|---|
| `--observe-seconds` | `2` 秒 | 进入 AUTO 并完成使能后保持观察的时间，可为 0。 |
| `--state-timeout` | `10` 秒 | 模式和使能状态确认超时。 |
| `--service-timeout` | `3` 秒 | 系统控制及 Hold 服务响应超时。 |

### 离线轨迹 Demo

离线生成器不连接 ROS、不控制机器人，默认按侍龙 L4 工作姿态生成四组 TXT 文件；
绘图程序随后将这些数据输出为 PNG：

```bash
ros2 run wheelloong_auto_sdk simulate_oscillation_targets \
  --robot shiloong \
  --duration 20 \
  --period 4 \
  --rate 5 \
  --amplitude-deg 3 \
  --lumbar-lift-amplitude-mm 10 \
  --output-dir /tmp/shiloong_targets

ros2 run wheelloong_auto_sdk plot_oscillation_targets \
  --input-dir /tmp/shiloong_targets \
  --output /tmp/shiloong_targets/oscillation_targets.png
```

生成器参数与 AUTO 振荡 Demo 中同名参数含义一致；`--robot` 默认为 `shiloong`，
`--output-dir` 默认为当前目录下的 `simulated_targets`。绘图程序的 `--input-dir`
默认指向该目录，`--output` 默认生成其中的 `oscillation_targets.png`。

### 点位记录 Demo

该示例不驱动底盘，只读取 `map -> base_link` TF。启动后按 `S` 保存当前位置，按
`Q` 退出，必须在交互式终端中运行：

```bash
ros2 run wheelloong_auto_sdk waypoint_recorder \
  --file /home/niic/auto_sdk_ws/src/wheelloong_auto_sdk/waypoint/waypoints.txt \
  --global-frame map \
  --base-frame base_link \
  --tf-timeout 1
```

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--file` | SDK 运行时默认路径 | 点位追加保存路径；可由 `WHEELLOONG_AUTO_SDK_WAYPOINT_FILE` 指定。 |
| `--global-frame` | `map` | 全局坐标系名称。 |
| `--base-frame` | `base_link` | 机器人底盘坐标系名称。 |
| `--tf-timeout` | `1` 秒 | 每次查询 TF 的等待超时。 |

## 安全与许可

本项目能够向真实机器人发送运动命令。运行示例前，请确认运动空间、急停、控制
模式、单位和目标姿态均安全，并由操作员现场监护。`wait=True` 只表示等待运动完成，
不代表机械制动；`Hold` 也只在驱动保持使能时有效。双臂去使能或断电前，必须先处理
无抱闸导致的重力下落风险。

当前软件包声明为 **Proprietary**。公开仓库仅代表源代码可见，不授予复制、修改、
分发或商业使用权；正式对外发布前应补充完整许可证文件。
