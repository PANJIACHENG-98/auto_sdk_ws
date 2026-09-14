#!/usr/bin/env python3
"""独立验证文本播报和本地音频播放接口。

默认只调用 voice.speak()；提供 --audio-file 时，会在文本播报后
继续调用 voice.play()。语音接口不需要切换 AUTO 模式或使能运动轴。
"""

import argparse
import sys

from wheelloong_auto_sdk import Robot, WheelloongSdkError
from wheelloong_auto_sdk._cli import boolean_switch, positive_integer, positive_number


DEFAULT_TEXT = "侍龙 L4 语音接口测试"


def build_parser() -> argparse.ArgumentParser:
    """创建文本播报、音频播放及超时参数解析器。"""
    parser = argparse.ArgumentParser(
        description="Test the standalone voice speak and audio playback APIs.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--text",
        default=DEFAULT_TEXT,
        help="text passed to voice.speak",
    )
    parser.add_argument(
        "--skip-speak",
        action="store_true",
        help="skip text-to-speech; --audio-file is then required",
    )
    parser.add_argument(
        "--audio-file",
        help="optional plain file name passed to voice.play (no directory path)",
    )
    parser.add_argument(
        "--play-count",
        type=positive_integer,
        default=1,
        help="number of times to play --audio-file",
    )
    parser.add_argument(
        "--wait",
        type=boolean_switch,
        default=True,
        metavar="BOOL",
        help="wait for each selected voice operation to finish",
    )
    parser.add_argument(
        "--speech-timeout",
        type=positive_number,
        default=30.0,
        help="speech synthesis/playback timeout passed to voice.speak",
    )
    parser.add_argument(
        "--call-timeout",
        type=positive_number,
        default=35.0,
        help="maximum service response wait for voice.speak",
    )
    parser.add_argument(
        "--audio-timeout",
        type=positive_number,
        default=120.0,
        help="maximum service response wait for voice.play",
    )
    return parser


def validate_arguments(
    parser: argparse.ArgumentParser,
    args: argparse.Namespace,
) -> None:
    """确保选择了至少一项语音操作。"""
    if args.skip_speak and not args.audio_file:
        parser.error("--audio-file is required when --skip-speak is used")


def run_demo(robot: Robot, args: argparse.Namespace) -> None:
    """按参数顺序调用文本播报和可选的音频播放。"""
    if not args.skip_speak:
        print(f"text-to-speech: {args.text}")
        robot.voice.speak(
            args.text,
            wait=args.wait,
            speech_timeout_sec=args.speech_timeout,
            call_timeout_sec=args.call_timeout,
        )
        print("text-to-speech request succeeded")

    if args.audio_file:
        print(
            f"audio playback: {args.audio_file} "
            f"(count={args.play_count})"
        )
        robot.voice.play(
            args.audio_file,
            play_count=args.play_count,
            wait=args.wait,
            timeout_sec=args.audio_timeout,
        )
        print("audio playback request succeeded")


def main() -> int:
    """创建独立 ROS 2 运行时并执行语音接口测试。"""
    parser = build_parser()
    args = parser.parse_args()
    validate_arguments(parser, args)
    try:
        with Robot.standalone(node_name="sdk_voice_demo") as robot:
            run_demo(robot, args)
        print("voice demo completed successfully")
        return 0
    except KeyboardInterrupt:
        print("voice demo interrupted", file=sys.stderr)
        return 130
    except WheelloongSdkError as exc:
        print(f"voice demo failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
