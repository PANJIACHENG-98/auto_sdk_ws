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
│       │   ├── arms.py          # Hold、MoveJ、MoveL 和 MoveP
│       │   ├── body.py          # 头部和腰部控制
│       │   ├── gripper.py       # 左、右和双夹爪控制
│       │   ├── navigation.py    # 导航下发、反馈、暂停和取消
│       │   ├── voice.py         # 文本播报和音频播放
│       │   ├── waypoints.py     # 点位文件读取与保存
│       │   ├── session.py       # AUTO 会话和退出清理
│       │   ├── models.py        # 公共数据类型与数值校验
│       │   ├── errors.py        # 公共异常体系
│       │   └── backend/
│       │       ├── base.py      # Backend 抽象接口
│       │       ├── ros2.py      # ROS Topic/Service/Action 实现
│       │       └── mock.py      # 无硬件测试实现
│       ├── examples/            # 7 个可通过 ros2 run 启动的示例
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
git clone git@github.com:PANJIACHENG-98/auto_sdk_ws.git
cd auto_sdk_ws

source /opt/ros/humble/setup.bash
source /home/niic/wheelloong/install/setup.bash
colcon build --packages-select wheelloong_auto_sdk
source install/setup.bash
```

语音接口需要额外加载语音工作区：

```bash
source /home/niic/wheelloong_voice/install/setup.bash
```

构建完成后，每次打开新终端只需执行下面一条命令，即可依次加载 ROS、机器人主系统、
语音系统和本工作区环境：

```bash
source /home/niic/auto_sdk_ws/scripts/source_env.sh
```

如果当前终端已进入 Conda/Miniforge 环境，可先执行 `conda deactivate`。导入检查可
明确使用系统 Python：

```bash
/usr/bin/python3 -c "from wheelloong_auto_sdk import Robot; print('SDK import OK')"
```

## 可用接口总览

<table>
  <thead><tr><th>分类</th><th>接口</th><th>作用</th><th>Demo 直接调用</th></tr></thead>
  <tbody>
    <tr><td rowspan="2">机器人入口</td><td><code>Robot.standalone()</code></td><td>创建并拥有私有 ROS Context、Node 和 Executor。</td><td>是：<code>standalone_demo.py:17</code> 等</td></tr>
    <tr><td><code>Robot.with_backend()</code></td><td>注入 Mock、ROS 或扩展 Backend。</td><td>无</td></tr>
    <tr><td rowspan="4">生命周期</td><td><code>robot.node</code></td><td>访问后端使用的 ROS Node。</td><td>无</td></tr>
    <tr><td><code>robot.state()</code></td><td>获取无 ROS 类型的系统状态快照。</td><td>是：<code>auto_oscillation_demo.py:335</code></td></tr>
    <tr><td><code>robot.auto_session()</code></td><td>创建自动管理 AUTO、使能、Hold 和退出清理的上下文。</td><td>是：<code>standalone_demo.py:19</code></td></tr>
    <tr><td><code>robot.close()</code></td><td>取消活动导航并释放 SDK 拥有的 ROS 资源。</td><td>无（Demo 通过 <code>with</code> 自动调用）</td></tr>
    <tr><td rowspan="3">系统控制</td><td><code>system.state()</code></td><td>获取满足新鲜度要求的系统状态。</td><td>是：<code>waypoint_navigation_demo.py:93</code>、<code>auto_session_demo.py:239</code></td></tr>
    <tr><td><code>system.set_control_mode()</code></td><td>请求切换 AUTO 或 IDLE。</td><td>是：<code>auto_session_demo.py:247</code> 等</td></tr>
    <tr><td><code>system.wait_for_control_mode()</code></td><td>等待控制模式实际生效。</td><td>是：<code>auto_session_demo.py:254</code> 等</td></tr>
    <tr><td rowspan="5">轴使能</td><td><code>system.set_enabled()</code></td><td>精确设置六组运动轴的完整使能目标。</td><td>是：<code>auto_session_demo.py:263</code>、<code>auto_oscillation_demo.py:749</code></td></tr>
    <tr><td><code>system.wait_for_enabled()</code></td><td>等待实际使能状态与目标完全一致。</td><td>是：<code>auto_session_demo.py:270</code>、<code>auto_oscillation_demo.py:750</code></td></tr>
    <tr><td><code>system.enable_all()</code></td><td>便捷调用：将六组运动轴全部设为使能。</td><td>无</td></tr>
    <tr><td><code>system.disable_all()</code></td><td>请求关闭全部运动轴使能。</td><td>是：<code>auto_session_demo.py:349</code>、<code>auto_oscillation_demo.py:713</code></td></tr>
    <tr><td><code>system.wait_for_all_disabled()</code></td><td>等待全部运动轴完成去使能。</td><td>是：<code>auto_session_demo.py:354</code>、<code>auto_oscillation_demo.py:714</code></td></tr>
    <tr><td rowspan="4">双臂</td><td><code>arms.hold()</code></td><td>停止双臂现有运动并保持当前位置。</td><td>是：<code>auto_session_demo.py:279</code>、<code>auto_oscillation_demo.py:556</code> 等</td></tr>
    <tr><td><code>arms.move_j()</code></td><td>执行单臂或双臂关节空间运动。</td><td>是：<code>standalone_demo.py:22</code>、<code>auto_oscillation_demo.py:382</code></td></tr>
    <tr><td><code>arms.move_l()</code></td><td>执行单臂或双臂笛卡尔直线运动。</td><td>是：<code>auto_oscillation_demo.py:525</code></td></tr>
    <tr><td><code>arms.move_p()</code></td><td>对目标位姿求逆解后执行关节运动。</td><td>是：<code>auto_oscillation_demo.py:492</code></td></tr>
    <tr><td rowspan="3">头腰</td><td><code>body.move()</code></td><td>在一次请求中控制任意头部或腰部轴。</td><td>是：<code>auto_oscillation_demo.py:392</code></td></tr>
    <tr><td><code>body.move_head()</code></td><td>控制头部俯仰和转动。</td><td>无</td></tr>
    <tr><td><code>body.move_waist()</code></td><td>控制腰部俯仰和升降。</td><td>无</td></tr>
    <tr><td rowspan="3">夹爪</td><td><code>gripper.set()</code></td><td>设置左、右或双夹爪位置。</td><td>是：<code>auto_oscillation_demo.py:404</code></td></tr>
    <tr><td><code>gripper.open()</code></td><td>将指定夹爪打开到最大行程。</td><td>是：<code>standalone_demo.py:31</code></td></tr>
    <tr><td><code>gripper.close()</code></td><td>将指定夹爪关闭到最小行程。</td><td>无</td></tr>
    <tr><td rowspan="2">导航下发</td><td><code>navigation.navigate_to()</code></td><td>下发单点 Nav2 导航目标。</td><td>是：<code>waypoint_navigation_demo.py:142</code></td></tr>
    <tr><td><code>navigation.navigate_through()</code></td><td>按顺序下发多点 Nav2 导航目标。</td><td>是：<code>waypoint_navigation_demo.py:150</code></td></tr>
    <tr><td rowspan="7">导航管理</td><td><code>NavigationHandle.feedback</code></td><td>获取最近一次导航反馈。</td><td>无</td></tr>
    <tr><td><code>NavigationHandle.done</code></td><td>判断导航是否已进入最终状态。</td><td>是：<code>waypoint_navigation_demo.py:164</code></td></tr>
    <tr><td><code>NavigationHandle.paused</code></td><td>判断导航是否处于 SDK 软暂停状态。</td><td>无</td></tr>
    <tr><td><code>NavigationHandle.wait()</code></td><td>等待导航完成，并按配置处理超时。</td><td>是：<code>waypoint_navigation_demo.py:155</code></td></tr>
    <tr><td><code>NavigationHandle.pause()</code></td><td>取消当前 Action 并保存原目标。</td><td>无</td></tr>
    <tr><td><code>NavigationHandle.resume()</code></td><td>重新下发软暂停时保存的目标。</td><td>无</td></tr>
    <tr><td><code>NavigationHandle.cancel()</code></td><td>主动取消导航并等待最终状态。</td><td>是：<code>waypoint_navigation_demo.py:168</code></td></tr>
    <tr><td rowspan="4">导航辅助</td><td><code>navigation.clear_local_costmap()</code></td><td>清除完整局部代价地图。</td><td>是：<code>waypoint_navigation_demo.py:131</code></td></tr>
    <tr><td><code>navigation.clear_global_costmap()</code></td><td>清除完整全局代价地图。</td><td>是：<code>waypoint_navigation_demo.py:135</code></td></tr>
    <tr><td><code>navigation.current_pose()</code></td><td>读取当前 <code>map -&gt; base_link</code> TF 位姿。</td><td>是：<code>waypoint_navigation_demo.py:118</code></td></tr>
    <tr><td><code>navigation.cancel_all()</code></td><td>尽力取消 SDK 管理的全部活动目标。</td><td>无</td></tr>
    <tr><td rowspan="2">语音</td><td><code>voice.speak()</code></td><td>调用语音服务播报文本。</td><td>无</td></tr>
    <tr><td><code>voice.play()</code></td><td>播放机器人语音目录中的本地音频。</td><td>无</td></tr>
    <tr><td rowspan="4">点位文件</td><td><code>default_waypoint_path()</code></td><td>按环境和工作区规则选择默认点位文件。</td><td>是：<code>waypoint_recorder.py:48</code>、<code>waypoint_navigation_demo.py:53</code></td></tr>
    <tr><td><code>load_waypoints()</code></td><td>读取并校验编号点位。</td><td>是：<code>waypoint_recorder.py:102</code>、<code>waypoint_navigation_demo.py:67</code></td></tr>
    <tr><td><code>next_waypoint_id()</code></td><td>计算下一个可用的正整数点位编号。</td><td>无</td></tr>
    <tr><td><code>append_waypoint()</code></td><td>将点位追加保存到文件。</td><td>是：<code>waypoint_recorder.py:122</code></td></tr>
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

| 类型 | 作用 |
|---|---|
| `AxisSelection` | 描述左/右臂、头部和腰部六组轴的使能目标。 |
| `ArmSide` | 选择左侧、右侧或双侧手臂/夹爪。 |
| `ControlMode` | 表示 SDK 支持的 AUTO 和 IDLE 控制模式。 |
| `MotionMode` | 表示机械臂绝对或增量运动模式。 |
| `CartesianPoseMM` | 使用毫米位置和四元数描述机械臂 TCP 位姿。 |
| `NavigationMode` | 选择默认或精确导航模式。 |
| `NavigationPose` | 使用米、弧度和坐标系描述平面导航目标。 |
| `SystemState` | 保存系统模式、使能、错误、夹爪和手臂位姿状态。 |
| `CommandResult` | 保存服务操作名、错误码和返回消息。 |
| `NavigationFeedback` | 保存剩余距离、预计时间和恢复次数等导航反馈。 |
| `NavigationResult` | 保存导航最终状态、消息和成功标志。 |
| `Waypoint` | 组合点位编号与 `NavigationPose`。 |

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

## Demo 使用说明

以下命令均要求侍龙 L4 主系统已经启动，并且当前终端已加载 ROS、机器人主系统和
本工作区环境。可用 `ros2 run wheelloong_auto_sdk <示例名> --help` 查看源码当前
接受的参数。

### 真机运动 Demo

| 示例 | 作用 | 运动部件 |
|---|---|---|
| `auto_oscillation_demo` | 执行 AUTO 九阶段流程：进入工作姿态、正弦振荡、MoveP、MoveL、结束姿态和安全清理。 | 双臂、头部、腰部及可选夹爪 |
| `standalone_demo` | 演示 SDK 自建 ROS Context、Node 和 Executor。 | 双臂运动到零位并打开双夹爪 |
| `waypoint_navigation_demo` | 从点位文件读取编号点位，清理代价地图并执行单点或多点导航。 | 移动底盘 |

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

#### 独立运行时控制

该示例没有命令行参数，会真实下发双臂和夹爪命令：

```bash
ros2 run wheelloong_auto_sdk standalone_demo
```

#### 点位导航

至少需要提供一个正整数点位编号。一个编号调用单点导航，多个编号按给定顺序调用
多点导航：

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
| `--file` | 包内 `waypoint/waypoints.txt` | 点位文件路径，建议需要自定义时使用绝对路径。 |
| `--navigation-timeout` | `300` 秒 | 导航结果等待超时，必须大于 0。 |
| `--server-timeout` | `5` 秒 | Nav2 Action Server 和相关服务等待超时，必须大于 0。 |

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
| `--file` | 包内 `waypoint/waypoints.txt` | 点位追加保存路径。 |
| `--global-frame` | `map` | 全局坐标系名称。 |
| `--base-frame` | `base_link` | 机器人底盘坐标系名称。 |
| `--tf-timeout` | `1` 秒 | 每次查询 TF 的等待超时。 |

## 安全与许可

本项目能够向真实机器人发送运动命令。运行示例前，请确认运动空间、急停、控制
模式、单位和目标姿态均安全，并由操作员现场监护。

当前软件包声明为 **Proprietary**。公开仓库仅代表源代码可见，不授予复制、修改、
分发或商业使用权；正式对外发布前应补充完整许可证文件。
