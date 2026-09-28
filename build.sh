#! /usr/bin/env bash
# ============================================================================
# 构建 + 跑测试 (离线, 不碰硬件)。
#
#   ./build.sh              # Release 构建 + ctest
#   ./build.sh --werror     # 把编译警告当错误
#   ./build.sh --asan       # AddressSanitizer + UBSan (查越界/泄漏)
#   ./build.sh --tsan       # ThreadSanitizer (查数据竞争) —— ⚠ 见文件末尾的"TSan 现状"
#
# ⚠ **TSan 现状 (2026-09-24, 未定论)**: 它是**分诊用**的可选工具, **不是**准入门禁。
#   本机需要 `setarch -R` (关 ASLR) 才能跑 —— 内核 vm.mmap_rnd_bits 偏大时 TSan 会直接
#   `FATAL: unexpected memory mapping`, 还没走到被测代码。脚本已自动加上。
#   当前它会报**大量** `Ack::queues` 上的 data race (11/19 个套件)。逐条人工核对过:
#   `queues` 的**全部 6 处**访问点都在 `mu` 之下 (`deliver` / `drain_for` / `wait` /
#   `get_status_now` 的 done 回调与显式 lock_guard), 代码层面**找不到**未加锁的访问,
#   且 TSan 打印的 `mutexes:` 是**报告生成那一刻**的持锁集合、不是访问发生时的 ⇒
#   "两侧都显示 M28" 并不能证明两侧当时都在锁内。
#   ⇒ **未定论**: 既没有证据说它是真竞争, 也没有证据说它是 TSan 误报。**先记账, 别当绿。**
#   下一个要查的人从这里开始: 单用例 `--filter cmd_enable_and_disable_round_trip` 就能
#   复现 55 条告警, 最小化路径很短。
#
# ⚠⚠ **`exitcode=0` 有一个反噬, 别踩 (2026-09-28 实测)**: TSan 的**死锁检测器**只有一个
#   64 格的固定数组 (`sanitizer_deadlock_detector.h:67`), 程序里被它同时追踪的 mutex
#   超过 64 个就 `FATAL: CHECK failed (0x40, 0x40)` **当场自杀**。而自杀走的也是
#   `internal__exit(exitcode)` ⇒ **退出码 0** ⇒ **ctest 报 Passed**。
#   ⇒ 用例若在**同一次运行里创建几十个 `Arm`** (每个 `Ack` 带一套 mutex), 就会撞上它:
#     你会看到"全绿", 而实际上用例跑到一半就没了。
#   ⇒ 判据不是看 ctest, 是看**框架自己那行汇总** (`全部通过: N 通过 / M 失败`) 有没有出现,
#     以及 stderr 里有没有 `FATAL: ThreadSanitizer CHECK failed`。脚本已把两者都留在输出里。
#   绕过办法 (临时): `TSAN_OPTIONS=...:detect_deadlocks=0`。
#   ./build.sh --clean      # 先删掉 build/
# ============================================================================
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD="$DIR/build"
EXTRA=()
RUNNER=()
TSAN=0
CLEAN=0

for a in "$@"; do
    case "$a" in
        --werror) EXTRA+=("-DLITEARM_WERROR=ON") ;;
        --asan)   EXTRA+=("-DCMAKE_BUILD_TYPE=Debug"
                          "-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer") ;;
        --tsan)   TSAN=1
                  BUILD="$DIR/build-tsan"
                  EXTRA+=("-DCMAKE_BUILD_TYPE=Debug" "-DCMAKE_CXX_FLAGS=-fsanitize=thread -g") ;;
        # ⚠⚠ `--clean` 只**记意愿**, 删除留到循环之后 —— 参数按顺序处理, 而 `--tsan` 是在
        #   循环的**后面几个迭代**里才把 `BUILD` 改成 `build-tsan` 的。原先这里直接
        #   `rm -rf "$BUILD"` 会让 `./build.sh --clean --tsan` 删掉的是 **`build/`**
        #   (非 tsan 的构建目录)、而 `build-tsan/` 原封不动 —— 一个**删错目录**的哑错误:
        #   不报错、不提示, 只是另一个目录没了 (实测 2026-09-28: 这个坑让 `build/`
        #   凭空消失过两次, 每次都先怀疑是自己手滑)。
        --clean)  CLEAN=1 ;;
        -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
        *) echo "未知开关: $a"; exit 2 ;;
    esac
done

# ⚠ 删除排在这里 (循环之后) —— 此时 `BUILD` 才是**最终**那个目录。理由见 `--clean` 那条。
if [ "$CLEAN" = 1 ]; then
    rm -rf "$BUILD"
fi

# ⚠ TSan 在本机需要**关掉 ASLR** 才能跑: 内核的 vm.mmap_rnd_bits 偏大,
#   否则每个用例都直接 `FATAL: ThreadSanitizer: unexpected memory mapping` (还没到被测代码)。
#   `setarch -R` 就是"这一条命令关 ASLR"。
if [ "$TSAN" = 1 ]; then
    if setarch -R true >/dev/null 2>&1; then
        RUNNER=(setarch -R)
    else
        echo "⚠ setarch -R 不可用; TSan 可能直接 FATAL (见 build.sh 里的说明)"
    fi
    # 两个 TSan 开关, 缺一个都会让"用例通过"被误报成"用例失败":
    #   * `halt_on_error=0` —— 否则检测到一条报告就**当场 exit(66)**, 被测断言根本跑不到
    #     (fork 用例里的子进程就是这样整批挂掉的);
    #   * `exitcode=0` —— 否则即使跑完了, TSan 也会在进程退出时把退出码设成 66,
    #     而 ctest 只看退出码 ⇒ 打印着"**全部通过: 12 通过 / 0 失败**"的用例被判 Failed。
    #     (实测: 单跑 test_params 输出"全部通过"而 ctest 报 Failed, 就是这一条。)
    # 本仓的口径: TSan 是**分诊工具**, 不是门禁 —— 报告要看得见, 但不该改变被测行为。
    export TSAN_OPTIONS="halt_on_error=0:exitcode=0${TSAN_OPTIONS:+:$TSAN_OPTIONS}"
fi

cmake -S "$DIR" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release "${EXTRA[@]}"
cmake --build "$BUILD" -j"$(nproc 2>/dev/null || echo 4)"
cd "$BUILD"
if [ "$TSAN" = 1 ] && [ ${#RUNNER[@]} -gt 0 ]; then
    "${RUNNER[@]}" ctest --output-on-failure
else
    ctest --output-on-failure
fi
