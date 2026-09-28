#! /usr/bin/env bash
# ============================================================================
# litearm-cpp 运行环境 (source 用) —— 让 examples / tests 免安装即可跑
#
#   source env.sh
#   ./run_example.sh 01_hello              # 推荐
#   "$LITEARM_BIN/01_hello"                # 等价
#
# 作用:
#   1) 把 build/ 里的可执行文件放进 PATH;
#   2) 没构建过就自动构建一次 (CMake + make);
#   3) LITEARM_PORT 可覆盖 CDC 端口; 不设则自动发现 (VID:PID 1d50:606f)。
#
# ⚠ SDK 本身**不读任何环境变量** —— LITEARM_PORT 只被样例/脚本这一层用。
# ============================================================================
set -euo pipefail

export LITEARM_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LITEARM_BUILD="${LITEARM_BUILD:-${LITEARM_REPO}/build}"

if [ ! -x "${LITEARM_BUILD}/01_hello" ]; then
    echo "[litearm-cpp env] 首次运行, 正在构建 -> ${LITEARM_BUILD}"
    cmake -S "${LITEARM_REPO}" -B "${LITEARM_BUILD}" -DCMAKE_BUILD_TYPE=Release >/dev/null
    cmake --build "${LITEARM_BUILD}" -j"$(nproc 2>/dev/null || echo 4)" >/dev/null
fi

export LITEARM_BIN="${LITEARM_BUILD}"
export PATH="${LITEARM_BIN}:${PATH}"
export LITEARM_PORT="${LITEARM_PORT:-}"

echo "[litearm-cpp env] repo=${LITEARM_REPO}"
echo "  build        = ${LITEARM_BUILD}"
echo "  LITEARM_PORT = '${LITEARM_PORT}'  (空=自动发现 1d50:606f)"
