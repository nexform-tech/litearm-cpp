#! /usr/bin/env bash
# ============================================================================
# 运行单个 example (设置好环境再跑), 用法:
#
#   ./run_example.sh 01_hello
#   ./run_example.sh 02_movej --go --speed 0.2
#   LITEARM_PORT=/dev/ttyACM0 ./run_example.sh 01_hello
#
# 等价于: source env.sh && "$LITEARM_BIN"/<name> [args...]
# ============================================================================
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ $# -lt 1 ]; then
    echo "用法: $0 <example> [args...]"
    echo "样例:"
    ls "$DIR/examples/"*.cpp 2>/dev/null | sed 's#.*/##; s#\.cpp$##' | grep -v '^_' || true
    exit 1
fi

name="$1"; shift || true
source "$DIR/env.sh"

exe="$LITEARM_BIN/$name"
if [ ! -x "$exe" ]; then
    echo "找不到样例可执行文件: $exe"; exit 1
fi
exec "$exe" "$@"
