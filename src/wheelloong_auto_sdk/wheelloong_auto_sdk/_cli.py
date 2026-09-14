"""Examples shared argparse value parsers; not part of the public robot API."""

import argparse
import math


def finite_number(value: str) -> float:
    """解析有限浮点命令行参数。"""
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"expected a number: {value}") from exc
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError(f"expected a finite number: {value}")
    return parsed


def non_negative_number(value: str) -> float:
    """解析有限非负浮点命令行参数。"""
    parsed = finite_number(value)
    if parsed < 0.0:
        raise argparse.ArgumentTypeError(f"expected value >= 0: {value}")
    return parsed


def positive_number(value: str) -> float:
    """解析有限正浮点命令行参数。"""
    parsed = finite_number(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError(f"expected value > 0: {value}")
    return parsed


def positive_integer(value: str) -> int:
    """解析大于等于 1 的整数命令行参数。"""
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"expected an integer: {value}") from exc
    if parsed < 1:
        raise argparse.ArgumentTypeError(f"expected value >= 1: {value}")
    return parsed


def boolean_switch(value: str) -> bool:
    """解析 on/off、true/false、yes/no 或 1/0 开关。"""
    normalized = value.strip().lower()
    if normalized in ("on", "true", "1", "yes"):
        return True
    if normalized in ("off", "false", "0", "no"):
        return False
    raise argparse.ArgumentTypeError(f"expected on/off, got: {value}")
