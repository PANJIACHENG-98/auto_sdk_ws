"""验证语音 Demo 只按命令行选项调用公开语音接口。"""

import importlib.util
from pathlib import Path

import pytest

from wheelloong_auto_sdk import Robot
from wheelloong_auto_sdk.backend import MockBackend


EXAMPLE_PATH = Path(__file__).resolve().parents[1] / "examples" / "voice_demo.py"


def load_example_module():
    """从 examples 目录加载语音样例而不执行 main。"""
    spec = importlib.util.spec_from_file_location("voice_demo", EXAMPLE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_voice_example_speaks_by_default():
    """确认默认只执行阻塞文本播报。"""
    example = load_example_module()
    args = example.build_parser().parse_args([])
    backend = MockBackend()

    example.run_demo(Robot.with_backend(backend), args)

    assert backend.calls == [
        ("speak", {"text": example.DEFAULT_TEXT, "wait": True})
    ]


def test_voice_example_speaks_then_plays_audio():
    """确认提供音频时依次调用 speak 和 play。"""
    example = load_example_module()
    args = example.build_parser().parse_args(
        [
            "--text",
            "hello",
            "--audio-file",
            "notice.wav",
            "--play-count",
            "2",
            "--wait",
            "false",
        ]
    )
    backend = MockBackend()

    example.run_demo(Robot.with_backend(backend), args)

    assert backend.calls == [
        ("speak", {"text": "hello", "wait": False}),
        (
            "play_audio",
            {"file_name": "notice.wav", "play_count": 2, "wait": False},
        ),
    ]


def test_voice_example_supports_audio_only():
    """确认 --skip-speak 可只测试音频播放。"""
    example = load_example_module()
    parser = example.build_parser()
    args = parser.parse_args(["--skip-speak", "--audio-file", "notice.wav"])
    example.validate_arguments(parser, args)
    backend = MockBackend()

    example.run_demo(Robot.with_backend(backend), args)

    assert [name for name, _ in backend.calls] == ["play_audio"]


def test_voice_example_rejects_no_selected_operation():
    """确认跳过播报时必须提供音频文件名。"""
    example = load_example_module()
    parser = example.build_parser()
    args = parser.parse_args(["--skip-speak"])

    with pytest.raises(SystemExit) as error:
        example.validate_arguments(parser, args)

    assert error.value.code == 2
