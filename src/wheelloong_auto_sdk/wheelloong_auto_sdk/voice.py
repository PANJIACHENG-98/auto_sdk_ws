"""Public voice output API."""

import os

from .backend.base import RobotBackend
from .errors import ValidationError
from .models import CommandResult, finite_float


class Voice:
    """提供文本转语音和本地音频文件播放接口。"""

    def __init__(self, backend: RobotBackend) -> None:
        """使用指定机器人后端创建语音控制器。"""
        self._backend = backend

    def speak(
        self,
        text: str,
        *,
        wait: bool = True,
        speech_timeout_sec: float = 30.0,
        call_timeout_sec: float = 35.0,
    ) -> CommandResult:
        """通过底层语音服务播报非空文本。

        Args:
            text: 去除首尾空白后需要播报的文本。
            wait: 是否等待语音播放完成。
            speech_timeout_sec: 传给语音服务的合成/播放超时。
            call_timeout_sec: SDK 等待服务响应的最长秒数。
        Returns:
            成功指令结果。
        Raises:
            ValidationError: 文本为空或超时时间不合法。
        """
        content = str(text).strip()
        if not content:
            raise ValidationError("text must not be empty or whitespace")
        speech_timeout = finite_float(speech_timeout_sec, "speech_timeout_sec")
        call_timeout = finite_float(call_timeout_sec, "call_timeout_sec")
        if speech_timeout <= 0.0 or call_timeout <= 0.0:
            raise ValidationError("voice timeouts must be greater than zero")
        return self._backend.speak(
            content, bool(wait), speech_timeout, call_timeout
        )

    def play(
        self,
        file_name: str,
        *,
        play_count: int = 1,
        wait: bool = True,
        timeout_sec: float = 120.0,
    ) -> CommandResult:
        """播放语音服务目录中的一个本地音频文件。

        Args:
            file_name: 不包含路径分隔符的纯文件名。
            play_count: 播放次数，必须为大于等于 1 的整数。
            wait: 是否等待全部播放完成。
            timeout_sec: SDK 等待服务响应的最长秒数。
        Returns:
            成功指令结果。
        Raises:
            ValidationError: 文件名、次数或超时时间不合法。
        """
        name = str(file_name).strip()
        if not name or os.path.basename(name) != name or "/" in name or "\\" in name:
            raise ValidationError("file_name must be one plain file name without a path")
        if (
            isinstance(play_count, bool)
            or not isinstance(play_count, int)
            or play_count < 1
        ):
            raise ValidationError("play_count must be an integer greater than or equal to 1")
        timeout = finite_float(timeout_sec, "timeout_sec")
        if timeout <= 0.0:
            raise ValidationError("timeout_sec must be greater than zero")
        return self._backend.play_audio(
            name, int(play_count), bool(wait), timeout
        )
