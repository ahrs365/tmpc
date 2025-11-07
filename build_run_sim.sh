#!/usr/bin/env bash
# Helper script to configure, build, and run the pure C++ MPC simulation.

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-${PROJECT_ROOT}/build/pure_cpp}"
CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"

# You may export ACADOS_SOURCE_DIR before calling this script.
# If it is not set we try to fall back to a local third_party checkout.
DEFAULT_ACADOS_DIR="${PROJECT_ROOT}/third_party/acados"
ACADOS_DIR="${ACADOS_SOURCE_DIR:-${DEFAULT_ACADOS_DIR}}"

if [[ ! -d "${ACADOS_DIR}" ]]; then
    echo "[ERROR] ACADOS directory not found."
    echo "        Export ACADOS_SOURCE_DIR=/absolute/path/to/acados, or"
    echo "        clone acados into ${DEFAULT_ACADOS_DIR}."
    exit 1
fi

export ACADOS_SOURCE_DIR="${ACADOS_DIR}"
export LD_LIBRARY_PATH="${ACADOS_SOURCE_DIR}/lib:${LD_LIBRARY_PATH:-}"

# 尝试不同的 BLAS 库配置来避免符号冲突
if [[ -z "${LD_PRELOAD:-}" ]]; then
    # 优先尝试 OpenBLAS，如果不存在则使用 libblas
    for candidate in /usr/lib/x86_64-linux-gnu/libopenblas.so.0 /usr/lib/x86_64-linux-gnu/libblas.so.3; do
        if [[ -f "${candidate}" ]]; then
            export LD_PRELOAD="${candidate}"
            echo "[INFO] Using LD_PRELOAD=${LD_PRELOAD} to avoid BLAS symbol clashes"
            break
        fi
    done
fi

# 如果仍然崩溃，尝试取消 LD_PRELOAD
# export LD_PRELOAD=""

echo "[INFO] Using ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR}"
echo "[INFO] Updated LD_LIBRARY_PATH=${LD_LIBRARY_PATH}"
echo "[INFO] Current LD_PRELOAD=${LD_PRELOAD:-<unset>}"
echo "[INFO] Build directory: ${BUILD_DIR}"
echo "[INFO] Build type: ${CMAKE_BUILD_TYPE}"

cmake -S "${PROJECT_ROOT}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"

cmake --build "${BUILD_DIR}" --target mpc_planner_main -j"$(nproc)"

# 固定使用 mpc_planner_jackalsimulator/config 作为配置目录
CONFIG_PATH="${PROJECT_ROOT}/mpc_planner_jackalsimulator/config"

# 场景文件参数：用户只需输入场景名称（如 s1），自动补全路径和扩展名
SCENARIO_NAME="${1:-}"
SCENARIO_FILE=""

if [[ -n "${SCENARIO_NAME}" ]]; then
    # 自动补全场景文件路径
    SCENARIO_FILE="${PROJECT_ROOT}/scenarios/${SCENARIO_NAME}.json"

    if [[ ! -f "${SCENARIO_FILE}" ]]; then
        echo "[ERROR] Scenario file not found: ${SCENARIO_FILE}"
        echo "[HINT] Available scenarios:"
        for f in "${PROJECT_ROOT}/scenarios/"*.json; do
            if [[ -f "$f" ]]; then
                basename "$f" .json | sed 's/^/  - /'
            fi
        done
        exit 1
    fi

    echo "[INFO] Using scenario: ${SCENARIO_NAME} (${SCENARIO_FILE})"
fi

echo "[INFO] Launching simulation with config: ${CONFIG_PATH}"
echo "[HINT] Ensure your Python environment provides matplotlib (required by matplotlib-cpp)."

if [[ -n "${SCENARIO_FILE}" ]]; then
    "${BUILD_DIR}/mpc_planner_main" "${CONFIG_PATH}" "${SCENARIO_FILE}"
else
    echo "[INFO] No scenario specified, using default obstacles"
    "${BUILD_DIR}/mpc_planner_main" "${CONFIG_PATH}"
fi
