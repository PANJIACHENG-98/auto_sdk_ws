#!/usr/bin/env python3
"""演示由 SDK 独立创建 ROS 运行时并发送一次双臂和夹爪指令。

该程序会真实移动机器人。双臂先以官方 Demo 相同的默认速度语义到达
七关节零位，然后双夹爪打开到 1.0；运行前必须确认运动空间安全。
"""

from wheelloong_auto_sdk import ArmSide, AxisSelection, Robot


def main() -> None:
    """创建独立 Robot、进入 AUTO 会话并在退出时自动清理。"""
    # 本例只命令双臂；头部和腰部不使能，也不会收到目标值。
    axes = AxisSelection(left_arm=True, right_arm=True)

    # standalone 创建私有 Context、Node 和两线程 Executor；外层 with 负责关闭。
    with Robot.standalone(node_name="standalone_auto_task") as robot:
        # auto_session 依次确认状态、切 AUTO、使能双臂并发送 Hold。
        with robot.auto_session(required_axes=axes):
            # 七个 0.0 都是绝对关节角，单位 rad；左右臂同时运动。
            # speed/acceleration=0.0 与官方 Demo 一致，表示使用驱动默认值。
            robot.arms.move_j(
                left=[0.0] * 7,
                right=[0.0] * 7,
                speed_rad_s=0.0,
                acceleration_rad_s2=0.0,
                wait=True,
                timeout_sec=30.0,
            )
            # DUAL=-1；位置 1.0 表示最大行程，默认等待实际状态到达。
            robot.gripper.open(ArmSide.DUAL)

        # 离开 auto_session 后已经 Hold、切回 IDLE 并确认全部轴去使能。


if __name__ == "__main__":
    main()
