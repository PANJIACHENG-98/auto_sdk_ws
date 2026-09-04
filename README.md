# Wheelloong AUTO SDK

面向 Wheelloong/Shiloong 轮式人形机器人的 ROS 2 Python 任务 SDK。项目通过稳定的
Python API 封装机器人 AUTO 模式下的系统控制、双臂、头腰、夹爪、导航与语音能力，
业务代码无需直接构造 ROS Service、Topic 或 Action 消息。

## 主要能力

- 双臂 MoveJ、MoveL、MoveP 与 Hold 控制
- 头部、腰部和左右夹爪控制
- AUTO/IDLE 模式切换、轴使能与安全退出流程
- Nav2 单点及多点导航、暂停、恢复、取消和代价地图清理
- 文本播报与机器人本地音频播放
- 支持独立 ROS 运行时和复用已有 ROS Node
- ROS 2 真实后端与无硬件测试用 Mock 后端

## 工程结构

```text
auto_sdk_ws/
├── .vscode/                     # Linux/ROS 2 开发配置
├── src/
│   └── wheelloong_auto_sdk/     # ROS 2 Python 包、示例与测试
├── build/                       # colcon 生成，不纳入版本管理
├── install/                     # colcon 生成，不纳入版本管理
└── log/                         # colcon 生成，不纳入版本管理
```

完整安装方法、API、安全说明和示例见
[SDK 详细文档](src/wheelloong_auto_sdk/README.md)。

## 快速验证

```bash
cd /home/niic/auto_sdk_ws
source /opt/ros/humble/setup.bash
source /home/niic/wheelloong/install/setup.bash

colcon build --symlink-install --packages-select wheelloong_auto_sdk
source install/setup.bash
colcon test --packages-select wheelloong_auto_sdk
colcon test-result --verbose
```

## 安全与许可

本项目能够向真实机器人发送运动命令。运行示例前，请确认运动空间、急停、控制
模式、单位和目标姿态均安全，并由操作员现场监护。

当前软件包声明为 **Proprietary**。公开仓库仅代表源代码可见，不授予复制、修改、
分发或商业使用权；正式对外发布前应补充完整许可证文件。
