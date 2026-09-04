#!/usr/bin/env python3
"""演示把任务工程已有的 ROS Node 交给 SDK 使用。

该程序只读取一次 SystemInfo 状态，不切换模式、不上使能，也不发送运动、
夹爪、导航或语音指令；Node、Executor 和 ROS Context 始终由调用方拥有。
"""

import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from wheelloong_auto_sdk import Robot


def main() -> None:
    """创建外部 ROS 运行时、读取状态并按正确所有权顺序释放。"""
    # Context、Node 和 Executor 都由任务工程创建并负责最终销毁。
    rclpy.init()
    node = Node("existing_task_node")

    # 同步 SDK 接口等待 ROS Future，至少需要另一个执行线程处理响应。
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # from_node 只借用 node，不会调用 rclpy.shutdown 或 destroy_node。
    robot = Robot.from_node(node)
    try:
        # state 默认要求缓存不超过 0.5 秒，并最多等待 2 秒的新状态。
        state = robot.state()
        node.get_logger().info(f"control mode: {state.control_mode}")
    finally:
        # 先关闭 SDK，再停止 Executor，最后由任务工程销毁 Node 和 Context。
        robot.close()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
