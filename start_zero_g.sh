
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bin="$root/build/08_zero_g_drag"
if [[ ! -x "$bin" ]]; then
  bin="$root/build/08_zero_g_drag"
fi
if [[ ! -x "$bin" ]]; then
  echo "找不到 08_zero_g_drag；先运行 cmake --build build -j\"\$(nproc)\"" >&2
  exit 1
fi

mass="${PAYLOAD_MASS:-1.3328}"
com="${PAYLOAD_COM:-0.0094,-0.0001,0.0105}"

echo "payload (wrist-only 20-pose ID, 2026-11-17): mass=${mass} kg com=[${com}] m"
echo "★ 零重力会立刻生效：臂会失重，先扶稳；回车或 ^C 结束。"
exec "$bin" --go --payload "$mass" --com "$com" "$@"
