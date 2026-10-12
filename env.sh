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
#
# ⚠ 本脚本要被 `source`, 所以**必须还原调用方原有的 shell 选项**: 否则 `set -euo
#   pipefail` 会泄漏进交互终端 —— 之后任何一条返回非 0 的命令 (例如没接机械臂时
#   样例返回 1) 都会把终端直接关掉。
# ============================================================================

# 在开 `-e` 之前把调用方选项记下来, 脚本结束时原样还原。
#
# ⚠ `set +o` 里**唯独 errexit 记不准**: bash 会在命令替换子 shell 里把 errexit 关掉
#   (手册明文), 于是 `"$(set +o)"` 无论调用方是否开着 `-e` 都记成 `set +o errexit`,
#   还原时就把调用方的 `-e` 弄丢了 —— 恰好是本分支要修的那个泄漏。所以 errexit 单独
#   用 `$-`(当前 shell 的活动短选项, 不走命令替换) 记。
_litearm_opts="$(set +o)"
case "$-" in *e*) _litearm_had_errexit=1 ;; *) _litearm_had_errexit=0 ;; esac
set -euo pipefail
_litearm_restore_opts() {
    eval "$_litearm_opts"
    if [ "$_litearm_had_errexit" = 1 ]; then set -e; else set +e; fi
    unset _litearm_opts _litearm_restore_opts _litearm_had_errexit
}

export LITEARM_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LITEARM_BUILD="${LITEARM_BUILD:-${LITEARM_REPO}/build}"

if [ ! -x "${LITEARM_BUILD}/01_hello" ]; then
    echo "[litearm-cpp env] 首次运行, 正在构建 -> ${LITEARM_BUILD}"
    if ! cmake -S "${LITEARM_REPO}" -B "${LITEARM_BUILD}" -DCMAKE_BUILD_TYPE=Release >/dev/null; then
        echo "[litearm-cpp env] cmake 配置失败" >&2
        _litearm_restore_opts
        return 1
    fi
    if ! cmake --build "${LITEARM_BUILD}" -j"$(nproc 2>/dev/null || echo 4)" >/dev/null; then
        echo "[litearm-cpp env] cmake --build 失败" >&2
        _litearm_restore_opts
        return 1
    fi
fi

export LITEARM_BIN="${LITEARM_BUILD}"
export PATH="${LITEARM_BIN}:${PATH}"
export LITEARM_PORT="${LITEARM_PORT:-}"

echo "[litearm-cpp env] repo=${LITEARM_REPO}"
echo "  build        = ${LITEARM_BUILD}"
echo "  LITEARM_PORT = '${LITEARM_PORT}'  (空=自动发现 1d50:606f)"

_litearm_restore_opts
