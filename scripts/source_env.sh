#!/usr/bin/env bash
# 加载侍龙 L4 平台、部署、主系统、语音系统及 AUTO SDK 环境。
# 用法：source /home/niic/auto_sdk_ws/scripts/source_env.sh

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "请使用 source 执行此脚本：" >&2
    echo "  source ${BASH_SOURCE[0]}" >&2
    exit 1
fi

_auto_sdk_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_auto_sdk_setup_files=(
    "/opt/ros/humble/setup.bash"
    "/opt/wheelloong/wheelloong_env.sh"
    "/home/niic/wheelloong/install/setup.bash"
    "/home/niic/wheelloong_os_env/niic/deploy/resources/wheelloong_env.sh"
    "/home/niic/wheelloong_os_env/niic/deploy/resources/shiloong_env.sh"
    "/home/niic/wheelloong_voice/install/setup.bash"
    "${_auto_sdk_workspace}/install/setup.bash"
)
_auto_sdk_environment_names=(
    "ROS 2 Humble"
    "Wheelloong 平台公共环境"
    "侍龙 L4 主系统"
    "Wheelloong 部署公共环境"
    "侍龙 L4 机型环境"
    "侍龙 L4 语音系统"
    "Wheelloong AUTO SDK"
)

for _auto_sdk_index in "${!_auto_sdk_setup_files[@]}"; do
    _auto_sdk_setup_file="${_auto_sdk_setup_files[${_auto_sdk_index}]}"
    _auto_sdk_environment_name="${_auto_sdk_environment_names[${_auto_sdk_index}]}"
    if [[ ! -f "${_auto_sdk_setup_file}" ]]; then
        echo "环境加载失败，文件不存在：${_auto_sdk_setup_file}" >&2
        unset _auto_sdk_environment_name _auto_sdk_environment_names
        unset _auto_sdk_index _auto_sdk_setup_file _auto_sdk_setup_files
        unset _auto_sdk_workspace
        return 1
    fi
    if ! source "${_auto_sdk_setup_file}"; then
        echo "环境加载失败，执行异常：${_auto_sdk_setup_file}" >&2
        unset _auto_sdk_environment_name _auto_sdk_environment_names
        unset _auto_sdk_index _auto_sdk_setup_file _auto_sdk_setup_files
        unset _auto_sdk_workspace
        return 1
    fi
    echo "[已加载] ${_auto_sdk_environment_name}: ${_auto_sdk_setup_file}"
done

echo "环境加载完成，当前工作区：${_auto_sdk_workspace}"

unset _auto_sdk_environment_name _auto_sdk_environment_names
unset _auto_sdk_index _auto_sdk_setup_file _auto_sdk_setup_files
unset _auto_sdk_workspace
