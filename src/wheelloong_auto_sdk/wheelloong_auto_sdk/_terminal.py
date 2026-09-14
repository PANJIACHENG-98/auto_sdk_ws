"""Private interactive-terminal helpers used by command-line examples."""

from contextlib import contextmanager
import sys
import termios
import tty


@contextmanager
def cbreak_terminal():
    """临时进入单字符终端模式，并保证退出时恢复原设置。"""
    if not sys.stdin.isatty():
        raise RuntimeError("an interactive terminal is required")
    terminal_fd = sys.stdin.fileno()
    settings = termios.tcgetattr(terminal_fd)
    try:
        tty.setcbreak(terminal_fd)
        yield
    finally:
        termios.tcsetattr(terminal_fd, termios.TCSADRAIN, settings)


def read_key() -> str:
    """读取一个无需回车确认的终端字符。"""
    return sys.stdin.read(1)
