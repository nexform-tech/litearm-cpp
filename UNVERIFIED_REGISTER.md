# 未验证登记册

> **交付纪律**: 「**未实测 ≠ 已验证**」。凡是「没验证的 / 已知不完美的 / 有意与参照实现分叉的」，
> 都**如实登记在这里** —— 报告里必须能分清「实测 / 只编译未运行 / 未做」。
>
> ⚠ 本册的**核心纪律只有一条，而且它本身有判据**：**每条必须带出处**。
>    「存在差异」这种没有出处的条目**等于没登记** —— 现场复核不了，就得从头再查一遍。
>    判据见 `tests/test_register.cpp`（五列、出处形状、id 恰好 1..N，三条都钉住）。

## 格式约定

表头固定五列：`| # | 项 | 性质 | 状态 | 出处 |`

- **出处**必须能命中 `文件.扩展名:数字` 或 7~40 位十六进制提交哈希。
  ⚠ **本仓当前不是 git 仓库**（`git rev-parse` 报「不是 git 仓库」）⇒ 今天只能用
  `文件:行号` 这一种。哈希那一支留着，等它进版本管理后即可用。
- ⚠ **行号会腐** ⇒ 能写**符号名**就写符号名（如 `Arm::refresh_limits`），行号只是"当时"的锚。
  看到行号对不上时**先按符号名找**，再回来把行号改对。
- ⚠ **id 恰好是 1..N，不留空号**（判据会逐个数）。删条目要重编号 —— 那是**响的**改动，
  不像悬空那样容易被忽略。

## 图例（性质）

| 性质 | 含义 |
|---|---|
| 与 Python 分叉 | 本仓**有意**与参照实现 `litearm-python` 行为不同 ⇒ 相关场景**不进**跨语言对拍 |
| 已知继承缺陷 | 参照实现**就有**，本仓**保留原语义**（移植纪律：不静默改语义） |
| 未验证项 | 离线**没有判据**能分辨 / 真机没跑 ⇒ **不许**为它编一条恒真判据充数 |
| 结构事实 | 不是缺陷也不是取舍，是"这个形状在运行期观测不到"这类机制性说明 |
| 有意取舍 | 有意识地选了某一侧，代价写清 |

## 登记册

| # | 项 | 性质 | 状态 | 出处 |
|---|---|---|---|---|
| 1 | **同帧节流整套删除**（原版有 `_tx_allowed` / `set_tx_repeat_min_interval`）。原版允许节流**任何**帧 ⇒ 一条等 ACK 的命令被丢掉后帧没上 USB、固件永不应答、调用方只能等满窗口后报"无应答"（归因反了）。本仓**不做这套机制** | 与 Python 分叉 | 有意 | `src/arm.cpp:480`（`Arm::raw_write` 的说明）+ README「与原版的差异」① |
| 2 | **客户端预检四条**：非有限目标 / 软限越限 / 速度 / `move_p` 的容差。全部在**发帧之前**本地拒发；原版直接把参数发走、由固件 `clampf` 静默钳 | 与 Python 分叉 | 有意 | `include/litearm/arm.hpp:311`（`precheck_q_`）+ `tests/test_precheck.cpp` |
| 3 | **`connect()` 期读 n 条 `0x24` 建软限缓存** —— 原版的 `connect()` **不读软限**。⇒ 连接期的下行两侧本来就不同名，跨语言对拍一律**从 `connect` 之后开始录** | 与 Python 分叉 | 有意 | `src/arm.cpp:1012`（`Arm::refresh_limits`）|
| 4 | **`move_p` 的 speed 预检** —— 六个运动入口里**唯一**原版没有的那只 | 与 Python 分叉 | 有意 | `src/arm.cpp:1121`（`Arm::move_p` 的 `precheck_speed_` 调用点）|
| 5 | **七个只读诊断访问器**（`is_connected` / `is_in_dfu` / `status_seq` / `msg_hz` / `banner_version` / `host_stats` / `options`）—— 参照侧**一个都没有** | 与 Python 分叉 | 有意 | `include/litearm/arm.hpp:537` + `tests/test_accessors.cpp` |
| 6 | **可注入时间源**（`ArmOptions::clock`）—— 参照侧没有时间抽象。⚠ 本仓的 `Clock` **刻意不提供 `sleep_until`**（另一套实现有）：本仓的等待清一色是 `cv.wait_for`，靠 notify 唤醒；把睡眠搬进时钟会丢掉这条。代价：假钟只给**纯比较**类判据用，用在等待路径上会**挂死** | 与 Python 分叉 | 有意 | `include/litearm/clock.hpp:21` + README「与原版的差异」④ |
| 7 | **`port_string()` 报实际链路端口**（原版无此公开入口；原版私有的 `_port` 只是构造入参）。本仓先前也返回构造入参 ⇒ 自动发现时恒为空串，而**头文件注释承诺的是"或自动发现到的"**（注释与实现脱节，真机复现过） | 与 Python 分叉 | 已修 2026-09-28 | `include/litearm/arm.hpp:584` |
| 8 | **`get_status_now()` 在固件拒绝 `0x40` 时可能报成功**。结论判据后半条是"有任何新状态帧到"，而被动流与我们的 `0x40` 是两条独立的线 ⇒ 帧先落进窗口时，那条 `ERR` 还没投递，函数查一眼队列（空）便落到"返回成功"。**量化**：调用期间零帧交付 ⇒ 150/150 正确抛出；交付恰好 1 帧 ⇒ 150 次里 26~35 次报成功 | 已知继承缺陷 | 保留原语义 | `src/arm.cpp:586`（`Arm::get_status_now` 的 `done` 谓词）+ README「已知继承的差异/缺口」 |
| 9 | **同一根因让安全守卫 fail-open**：`Arm::reject_if_cart_in_flight()` 靠 `get_status_now()` 现取一帧判 `cart_busy`，取不到时本该**保守拒绝**；而上面那个窗口让它"误以为取到了" ⇒ 拿到陈旧状态 ⇒ `cart_busy=0` ⇒ **放行**。正是"轨迹中途进场靠摩擦滑停"那一侧 | 已知继承缺陷 | 保留原语义 | `src/arm.cpp:404`（`Arm::reject_if_cart_in_flight`）+ `tests/test_zero_g.cpp` 的 `zero_g_is_refused_conservatively_when_the_status_is_unavailable` |
| 10 | **`license()` 在低于 1.8.0 的固件上报"无应答超时"**而非"固件没有这条命令"：它调 `expect` 时没传 `echo_cmd` ⇒ `ERR{0x2F,0x00}` 落到别的队列，本入口看不到 | 已知继承缺陷 | 保留原语义 | `src/arm.cpp:1376`（`Arm::license`）+ README「已知继承的差异/缺口」 |
| 11 | **Windows 后端从未在 Windows 上跑过**（也没编译过）。改动它时**不许**声称已验证 | 未验证项 | 未做 | `src/transport.cpp:395`（`#else  // _WIN32` 起的整段）+ README:678（「平台」表） |
| 12 | **macOS 只共享代码路径，未实测**；且**端口自动发现不可用**（无 sysfs）| 未验证项 | 未做 | README.md:678（「平台」表）|
| 13 | **aarch64 只写了工具链文件，一次都没编过** —— 本机没装 `aarch64-linux-gnu-g++`。已实测的只有"工具链文件被正确解析、干净地失败在找不到编译器" | 未验证项 | 未做 | `cmake/toolchain-aarch64-linux-gnu.cmake:16`（`CMAKE_SYSTEM_NAME` 起）|
| 14 | **真机全功能验证已跑通**（2026-09-28，14 组 / 68 项断言 / 218 帧，**每个测试前后都回零**）。覆盖：只读访问器全家（含 `get_tcp`/`ik`/`host_stats`/`options`/`msg_hz`）、`params` 读回、`model` 读（probe/get_body/get_jm/status/get_gravity/**revert**）、`set_speed`/`set_motion_mode`/`park`/`clear_faults`/`reset`、`movej`/`movej_sync`、`move_p`、`move_js`/`send_mit`/`send_mit_all`、`move_l`/`move_c`/`move_path`、`zero_g` 启停、FF 全套（读原值写回）、`set_joint_param`/`set_joint_limits`（读原值写回）、`log().start/capture/stop`、`kin_bench`、`disable`/`enable` 往返、`emergency_stop` + 恢复。**逐帧审计：无任何不可逆命令**。⚠ **仍未在真机上跑**：四项不可逆（用户裁决排除）＋ `activate()`（本板已激活，调它只会回聚合档 0x02）＋ 真机上的**负路径**（不可达目标 / 拒发 / 超时只验到过一次 `move_p`，见下）| 未验证项 → **大部分已验** | 已实测 | `README.md:704`（真机验收范围）+ `/tmp/live_full.cpp`、`/tmp/live_recover.cpp`（**未收编进仓**）|
| 15 | **`test_protocol_sync` 从未对过真实固件源码** —— 固件仓挂在 Gitee 企业权限后，匿名取不到。它的正则与上游对齐过，但**只在构造出来的仿固件树上验过漂移检测** | 未验证项 | 未做 | `tests/test_protocol_sync.cpp:39`（`fw_dir()` 读 `LITEARM_FW_DIR`）|
| 16 | **真机只读复验的范围**（2026-09-28）：用 `/tmp/live_queries.cpp` 在 `Litearm1.8.0-7J` 上跑通 **38 帧**、全部查询类（白名单硬判据）；验到 0x43/0x42/0x34/0x35/0x38/0x39/0x2B/0x2C/0x2F/0x24/0x40 与四条预检。⚠ 机械臂**未接**时 `joint_fault=0x007F`（7 轴全锁存）、`mode=EMERGENCY`，接上后转 `0x0000`/`MOVE_J` —— 即该故障位就是"CAN 上收不到电机反馈" | 结构事实 | 已记录 | README.md:687 + `/tmp/live_queries.cpp:26`（白名单）|
| 17 | **真机探针不在仓里**（`/tmp/live_queries.cpp` 等）⇒ "下行帧白名单审计"这条保护**在仓内不可复现**。它验过的那几件事在仓内另有离线判据，但"真的只发了查询"这句话目前只有那一份输出为证 | 结构事实 | 未收编 | `/tmp/live_queries.cpp:26`（`kQueryOnly` 白名单；不在版本管理内）|
| 18 | **本仓不是 git 仓库** ⇒ 本册出处只能用 `文件:行号`，而**行号会腐**。这是当前状态下最弱的一环：判据只查**形状**，查不了"这个行号还指不指得到那句话" | 结构事实 | 已知局限 | `git rev-parse` 在本仓报「不是 git 仓库」；`tests/test_register.cpp:85`（`has_reference` 只验形状）|
| 19 | **`framing.hpp` 抽出来是为了消灭重复**（POSIX 与 Win32 从前各写一份分帧状态机）。**新增部分被编译被单测**，但 Win32 那两个**调用点**仍是 `#ifdef _WIN32` 里的代码 —— 在 Linux 上编不到 | 有意取舍 | 有意 | `include/litearm/framing.hpp:23` + `tests/test_framing.cpp` |
| 20 | **打包只对静态库实机验过**。`BUILD_SHARED_LIBS=ON` 会走同一套 install/export 规则，但**没有实测过共享产物**，也没有做符号可见性控制（`-fvisibility=hidden` + 导出宏）⇒ 共享库会把全部符号导出 | 未验证项 | 未做 | `CMakeLists.txt:124`（install 段）|
| 21 | **USB 重新枚举之后的第一次 `connect()` 会握手超时**（`get_firmware 无应答`），等几秒重试即好。真机实测（2026-09-28）：当天**4 次**重新插上/重新直通之后**每一次首次连接都失败**，重试全部成功；热链路下从未出现。**参照实现同样不重试**（`arm.py` 的 `connect()` 也是发一次 `0x41`、等 1.5s、失败就 `close()` 并抛）⇒ 属**继承行为**，不是移植引入的 | 已知继承缺陷 | 保留原语义（未修）| `src/arm.cpp:235`（`raw_write(proto::CMD_GET_FIRMWARE)`）+ README.md:620（「已知继承的差异/缺口」）|
| 22 | **掉电重启后的锁存怎么清 —— 已实测（2026-09-28，真机两次）**。⚠⚠ **本条的早期版本把结论下宽了**（据"只有 J4 锁存"那一例写成"根因是电气、软件清不掉"）；后一次全轴锁存的实测**推翻了那个推广**。**两次实测**：<br>**(a) 全 7 轴锁存** `joint_fault=0x007F` + `enabled=0` + `err=0`（7/7）+ 最大力矩幅值≈0.2：`clear_faults()` 清掉 `joint_fault`（`0x007F→0x0000`，但 `faulted` 仍为 1 —— 因为 `mode=EMERGENCY` 也算 faulted）→ `reset()` 清掉其余（`flags` 归零、`mode=INIT`）→ **`enable()` 成功，且电机随即开始报数**（`err=0` 的轴数 `7/7 → 0/7`）。⇒ **这条序列是可用的恢复路径**。<br>**(b) 单轴锁存** `joint_fault=0x0008`（J4，固件称 `drop_hold` 掉线刚性持位）：`clear_faults()` **逐位无变化**；`reset()` 清得掉**但一个检查周期内就重新锁上**。⚠ 两例的差别**尚未查清**（疑似 `drop_hold` 与普通 `joint_fault` 不是同一个位，或与"那次 enable 始终没成功、电机没上线"有关）—— **不编一套统一理论**，如实记两例。<br>**顺带实测**：`reset()` **会掉使能**（`enabled: 1→0`）；`err` 字节像是"该轴电机在不在报数"（健康=1；总线/未使能=0），但 SDK 只原样透传、**不解释它** | 未验证项 → **部分定性** | 已实测 | `TROUBLESHOOTING.md:304`（§17）+ `/tmp/live_recover.cpp`、`/tmp/live_reset.cpp` 的实测输出（**未收编进仓**）|

## 不在本册里的

- **修好且有判据守着的**东西（那是提交历史，不是"未验证"）。
- **线协议本身的细节**（帧格式、CRC、状态帧布局、双 ID）—— 那些有 `test_protocol.cpp` 逐条钉住，
  且经跨实现对拍验过 19/19，属"已验证"。

## 怎么用这本册子

1. 报告里说"测过了"之前，先扫一眼这里有没有相关条目。
2. 改上面任一条时：**同时**改这里的「状态」列与出处（出处是承重的，不是装饰）。
3. 新增一条"未验证"很容易，**删掉**一条要能说清"凭什么现在算验过了" ——
   说不清就别删。
