[English](UNVERIFIED_REGISTER.md) | 简体中文

# 未验证登记册

> **交付纪律**：“**没测量不等于已验证**”。任何“未验证 / 已知不完美 / 有意与参照实现
> 分叉”的东西都**如实登记在这里**——报告必须让读者分清“已测量 / 编译过但没跑过 / 没做”。
>
> ⚠ 本登记册有**一条核心规则，而且这条规则本身有检查**：**每条都必须带出处**。像“存在
> 差异”这样没有出处的条目**等于完全没登记**——现场没人能复核，整个调查只能从头再做。
> 检查在 `tests/test_register.cpp`（固定五列、出处格式、编号严格 1..N——三者都钉住）。
>
> ⚠ 本登记册引用的测试套件已于 2026-10-26 删除。下面对 `tests/...` 的引用属于历史；登记册
> 本身保留，作为“还有哪些未验证”的记录。

## 格式约定

表头固定五列：`| # | 项目 | 性质 | 状态 | 出处 |`

- **出处**必须匹配 `文件.扩展名:行号` 或 7–40 位十六进制提交哈希。
  ⚠ 本仓**现在已纳入 Git**，两种形式都可用；`文件:行号` 仍是首选，哈希分支是“没有哪一行
  能锚定”时的兜底。
- ⚠ **行号会漂。** 能写**符号名**时就写符号名（例如 `Arm::refresh_limits`）；行号只是
  “当时”的锚点。行号对不上时，**先按符号名搜索**，再回来修正行号。
- ⚠ **编号严格为 1..N，没有空缺**（检查会逐条数）。删除一条意味着重新编号——那是**响亮**
  的变更；相比之下，悬空的出处很容易被忽略。

## 图例（性质）

| 性质 | 含义 |
|---|---|
| 与 Python 分叉 | 本仓**有意**与参照实现 `litearm-python` 行为不同，因此相关场景**不进入**跨语言对比 |
| 继承缺陷 | 参照实现**也有**；本仓**保留原语义**（移植纪律：绝不静默改变语义） |
| 未验证项 | 没有离线**判据**能区分，或从未在真机上跑过，因此**禁止**为了填格子编造一个永远为真的空洞判据 |
| 结构性事实 | 既不是缺陷也不是取舍，而是机制性陈述，例如“这种形态在运行期不可观测” |
| 有意取舍 | 知情地选择了一边；代价写下来 |

## 登记表

| # | 项目 | 性质 | 状态 | 出处 |
|---|---|---|---|---|
| 1 | **同帧节流整套删除**（原版有 `_tx_allowed` / `set_tx_repeat_min_interval`）。原版允许节流**任何**帧，于是等 ACK 的命令可能被丢掉、帧根本没上 USB、固件永不应答，调用方只能等满整个窗口再报“无应答”——归因反了。本仓**不实现该机制** | 与 Python 分叉 | 有意 | `src/arm.cpp:480`（`Arm::raw_write` 处的注释）+ README “Four deliberate divergences”（1） |
| 2 | **四个客户端预检**：目标非有限值 / 越软限位 / 速度 / `move_p` 容差。四者都在**发帧之前本地拒绝**；原版把参数直接发走，让固件 `clampf` 静默钳制 | 与 Python 分叉 | 有意 | `include/litearm/arm.hpp:311`（`precheck_q_`）+ `tests/test_precheck.cpp` |
| 3 | **`connect()` 读 n 帧 `0x24` 来建软限位缓存**——原版的 `connect()` **不读软限位**。所以连接期间的下行流量在两侧并非同名；跨语言对比总是**从 `connect` 之后开始记录** | 与 Python 分叉 | 有意 | `src/arm.cpp:1012`（`Arm::refresh_limits`） |
| 4 | **`move_p` 的速度预检**——六个运动入口里**唯一**原版没有的 | 与 Python 分叉 | 有意 | `src/arm.cpp:1121`（`Arm::move_p` 里 `precheck_speed_` 的调用点） |
| 5 | **七个只读诊断访问器**（`is_connected` / `is_in_dfu` / `status_seq` / `msg_hz` / `banner_version` / `host_stats` / `options`）——参照侧**一个都没有** | 与 Python 分叉 | 有意 | `include/litearm/arm.hpp:537` + `tests/test_accessors.cpp` |
| 6 | **可注入时间源**（`ArmOptions::clock`）——参照侧没有时间抽象。⚠ 本仓的 `Clock` **刻意不提供 `sleep_until`**（另一套实现有）：这里所有等待都是被通知唤醒的 `cv.wait_for`，把睡眠搬进时钟会丢掉这个性质。代价：假钟只能用于**纯比较**判据；用在等待路径上会**挂死** | 与 Python 分叉 | 有意 | `include/litearm/clock.hpp:21` + README “Four deliberate divergences”（4） |
| 7 | **`port_string()` 报告活链路的端口**（原版没有这个公开入口；它的私有 `_port` 只是构造参数）。本仓以前也返回构造参数，于是自动发现时永远是空串，而**头文件注释却承诺“或自动发现的那个”**（注释与实现脱节；真机复现） | 与 Python 分叉 | 已修复 2026-09-28 | `include/litearm/arm.hpp:584` |
| 8 | **固件拒绝 `0x40` 时 `get_status_now()` 可能报成功。** 完成判据的后半句是“任意新状态帧到达”，而被动流和我们的 `0x40` 是两条独立线路，所以当窗口里先落下一帧时，那条 `ERR` 还没投递；函数瞥一眼队列（空）就落到“返回成功”。**量化**：调用期间零帧到达时，150/150 正确抛出；恰好 1 帧到达时，150 次里有 26–35 次报成功 | 继承缺陷 | 保留原语义 | `src/arm.cpp:586`（`Arm::get_status_now` 的 `done` 谓词）+ README “Known inherited gaps” |
| 9 | **同一根因让一个安全守卫 fail open**：`Arm::reject_if_cart_in_flight()` 调 `get_status_now()` 取新帧读 `cart_busy`，取不到时本该**保守拒绝**；上面的窗口反而让它“以为取到了”，于是读到陈旧状态、看到 `cart_busy=0`，**放行命令**。这正是“中途进场、靠摩擦滑停”的那一侧 | 继承缺陷 | 保留原语义 | `src/arm.cpp:404`（`Arm::reject_if_cart_in_flight`）+ `tests/test_zero_g.cpp` 用例 `zero_g_is_refused_conservatively_when_the_status_is_unavailable` |
| 10 | **在低于 1.8.0 的固件上，`license()` 报“无应答，超时”**，而不是“固件没有这条命令”：它调 `expect` 时没传 `echo_cmd`，于是 `ERR{0x2F,0x00}` 落到别的队列，本入口永远看不到 | 继承缺陷 | 保留原语义 | `src/arm.cpp:1376`（`Arm::license`）+ README “Known inherited gaps” |
| 11 | **Windows 后端从未在 Windows 上运行过**（也从未在那里编译过）。改动它时**不要**声称已验证 | 未验证项 | 未做 | `src/transport.cpp:395`（从 `#else  // _WIN32` 开始的整块）+ `README.md:789`（“Platforms” 表） |
| 12 | **macOS 只共享代码路径，从未测量**；**端口自动发现在那里不可用**（没有 sysfs） | 未验证项 | 未做 | `README.md:789`（“Platforms” 表） |
| 13 | **aarch64 只写了工具链文件，从未编译过一次**——本机没有 `aarch64-linux-gnu-g++`。只测量到“工具链文件能正确解析，并在找不到编译器时干净失败” | 未验证项 | 未做 | `cmake/toolchain-aarch64-linux-gnu.cmake:16`（来自 `CMAKE_SYSTEM_NAME`） |
| 14 | **全功能真机验证已通过**（2026-09-28，14 组 / 68 断言 / 218 帧，**每个测试前后都回零**）。覆盖：全套只读访问器（含 `get_tcp` / `ik` / `host_stats` / `options` / `msg_hz`）、`params` 读回、`model` 读取（probe / get_body / get_jm / status / get_gravity / **revert**）、`set_speed` / `set_motion_mode` / `park` / `clear_faults` / `reset`、`movej` / `movej_sync`、`move_p`、`move_js` / `send_mit` / `send_mit_all`、`move_l` / `move_c` / `move_path`、`zero_g` 启停、全套 FF（读原值、写回）、`set_joint_param` / `set_joint_limits`（读原值、写回）、`log().start/capture/stop`、`kin_bench`、`disable` / `enable` 往返，以及带恢复的 `emergency_stop`。**逐帧审计没有发现不可逆命令**。⚠ **仍未在真机上运行**：四条不可逆命令（用户裁决排除）、`activate()`（本板已激活，调用只会返回聚合档 `0x02`），以及真机上的**负路径**（不可达目标 / 拒绝 / 超时只在 `move_p` 上命中过一次，见下） | 未验证项，**大部分已验证** | 已测量 | `README.md:798`（真机验收范围）+ `/tmp/live_full.cpp`、`/tmp/live_recover.cpp`（**未入库**） |
| 15 | **`test_protocol_sync` 从未对真实固件源码核对过**——固件仓在 Gitee 企业权限后面，无法匿名拉取。它的正则已与上游对齐，但漂移检测**只对构造的假固件树验证过** | 未验证项 | 未做 | `tests/test_protocol_sync.cpp:39`（`fw_dir()` 读 `LITEARM_FW_DIR`） |
| 16 | **只读真机复核的范围**（2026-09-28）：`/tmp/live_queries.cpp` 在 `Litearm1.8.0-7J` 上跑了 **38 帧**，全是查询（一个硬编码白名单判据）；它覆盖了 `0x43` / `0x42` / `0x34` / `0x35` / `0x38` / `0x39` / `0x2B` / `0x2C` / `0x2F` / `0x24` / `0x40` 以及四个预检。⚠ 机械臂**断开**时，`joint_fault=0x007F`（七轴全锁存）、`mode=EMERGENCY`；连上后变为 `0x0000` / `MOVE_J`——也就是说，那些故障位表示“CAN 总线上没有电机反馈” | 结构性事实 | 已记录 | `README.md:798` + `/tmp/live_queries.cpp:26`（白名单） |
| 17 | **真机探测程序不在仓库里**（`/tmp/live_queries.cpp` 等），所以“下行帧白名单审计”这道保障**在仓库内不可复现**。它验证过的东西在这里有独立的离线判据，但“它真的只发了查询”目前只靠那一份输出 | 结构性事实 | 未入库 | `/tmp/live_queries.cpp:26`（`kQueryOnly` 白名单；不在版本控制下） |
| 18 | **行号会漂，而判据只检查形态。** 这是当前安排里最弱的一环：检查验证的是出处*存在且格式正确*，不是行号仍指向它声称的那句话 | 结构性事实 | 已知限制 | `tests/test_register.cpp:85`（`has_reference` 只检查形态）+ 本登记册第 11、12、14、16 条修正后的出处 |
| 19 | **`framing.hpp` 是为消除重复而抽出的**（POSIX 和 Win32 原先各带一份分帧状态机）。**新代码已编译并有单测**，但两个 Win32 **调用点**仍在 `#ifdef _WIN32` 里，Linux 上编不了 | 有意取舍 | 有意 | `include/litearm/framing.hpp:23` + `tests/test_framing.cpp` |
| 20 | **打包只在静态库上做过真机测量。** `BUILD_SHARED_LIBS=ON` 走同一套安装与导出规则，但**共享产物从未测量**，也没有符号可见性控制（`-fvisibility=hidden` 加导出宏），所以共享库会导出所有符号 | 未验证项 | 未做 | `CMakeLists.txt:124`（安装段） |
| 21 | **USB 重新枚举后的第一次 `connect()` 会在握手上超时**（`get_firmware` 无应答）；等几秒重试就行。真机实测（2026-09-28）：那天在 **4** 次重新插拔 / 重新透传后，**第一次连接每次都失败**，每次重试都成功；热链路从未发生。**参照实现也不重试**（它的 `connect()` 同样只发一条 `0x41`、等 1.5 s、然后 `close()` 抛出），所以这是**继承行为**，不是移植引入的 | 继承缺陷 | 保留原语义（未修） | `src/arm.cpp:235`（`raw_write(proto::CMD_GET_FIRMWARE)`）+ `README.md:690`（“Known inherited gaps”） |
| 22 | **断电后如何清除锁存——实测（2026-09-28，真机两次）**。⚠⚠ **本条目早期版本把结论下得太宽**（从“只有 J4 锁存”这一个案例得出“根因是电气问题、软件清不掉”）；后来全轴锁存的测量**推翻了那个概括**。**两次测量**：<br>**（a）七轴全锁存**，`joint_fault=0x007F` + `enabled=0` + `err=0`（7/7）+ 峰值力矩幅度约 0.2：`clear_faults()` 清掉 `joint_fault`（`0x007F` → `0x0000`，但此时 `faulted` 仍为 1，因为 `mode=EMERGENCY` 也算 faulted），然后 `reset()` 清掉其余（`flags` 归零、`mode=INIT`），然后 **`enable()` 成功、电机立刻开始报数**（`err=0` 的轴从 7/7 变为 0/7）。所以**这个顺序是可用的恢复路径**。<br>**（b）单轴锁存**，`joint_fault=0x0008`（J4，固件称 `drop_hold`）：`clear_faults()` **逐位都没有变化**；`reset()` 能清掉，**但一个检查周期内又锁上**。⚠ 两种情况之间的差异**尚未确定**（可能是 `drop_hold` 与普通 `joint_fault` 不是同一位，也可能与“那次 `enable()` 从未成功、电机从未上线”有关）——所以**不编造统一理论**；两种情况都按发生时的样子记录。<br>**顺带测量**：`reset()` **会掉使能**（`enabled: 1` → `0`）；`err` 字节看起来像“该轴电机是否在报数”（健康 = 1；总线断 / 未使能 = 0），但 SDK 只透传、**不解释** | 未验证项，**部分表征** | 已测量 | `docs/TROUBLESHOOTING.md:369`（第 17 节）+ `/tmp/live_recover.cpp` 与 `/tmp/live_reset.cpp` 的实测输出（**未入库**） |
| 23 | **真机傅里叶采集路径从未在硬件上运行过。** 它以 300 Hz 流式发送纯 PD，清 FF mask，对齐 100 Hz 状态流与 300 Hz 固件日志，并在发送迟延 / 跟踪误差 / 有效拍数不足时中止。测试套件只覆盖离线求解器和轨迹设计 | 未验证项 | 未做 | `src/ident/capture.cpp:88`（`capture_center`） |
| 24 | **辨识写回路径只通过假传输验证过。** `write_model` stage bodies 1..7（可选 jm）、commit `0x0FE`/`0x2FE` 并读回；假传输测试覆盖掩码/读回/拒绝路径。摩擦写回（`--write-friction`，0x26 items 9/10/11）和 `model.commit_jm()` 同样只做了离线验证。2026-11-25 的硬件测试（`Litearm1.12.3-7J`）在主机物理检查处停下，并手工验证了 jm 绕过（先 stage bodies、commit `0x2FE`、jm 读回误差 2.5e-9），但端到端 `write_model` 调用与 `get_gravity` 对比尚未在硬件上测量 | 未验证项，部分已表征 | 已测量 | `src/ident/export.cpp:477`（`write_model`）+ `src/model.cpp:107`（`commit_jm`） |
| 25 | **SN/HYY 导出器是面向行的补丁器**，不是通用 YAML 编辑器。它要求块式模板，且恰好有七个 `dynamics_parameters` 块和全部 16 个键；行内 map、锚点或额外块都会被拒绝。这是为了让 ident 模块只保留一个可选依赖（Eigen）而做的有意取舍 | 有意取舍 | 有意 | `src/ident/export.cpp:204`（`sn_template_yaml`） |
| 26 | **C++ 傅里叶系数生成器使用 `std::mt19937_64`，不是 NumPy 的旧版 MT19937。** 激励结构相同（5 次谐波、8 s、`1/j^2` 衰减、随机相位），但系数不是逐位相同；需要精确复现 Python 生成的日志时，用 `--input-csv` | 与 Python 分叉 | 有意 | `src/ident/trajectory.cpp:193` |
| 27 | **示例 11 的负载采集、基线采集和 `--apply` 从未在硬件上运行过。** 回归量已与 RNEA 交叉校验，求解器有合成离线测试，但静态力矩采样、固件 `set_payload` 读回和实时位姿预检尚未在真机上测量 | 未验证项 | 未做 | `examples/11_identify_payload.cpp:486`（`capture_series`） |
| 28 | **负载辨识仅限点质量（质量 + 质心）；不辨识负载惯量或负载摩擦。** 这与固件的 `set_payload` 模型一致，并让问题对四个参数保持线性；完整的惯性负载需要写模型，而不是 `set_payload` | 有意取舍 | 有意 | `include/litearm/ident/payload.hpp:31`（`PayloadOptions`） |

## 哪些内容不在本登记册

- **已修复且有判据守护的东西**——那是提交历史，不是“未验证”。
- **线协议本身的细节**（帧格式、CRC、状态帧布局、双 ID）——这些在 `test_protocol.cpp` 里逐条钉住，并经过 19/19 跨实现对比，所以算已验证。

## 如何使用本登记册

1. 在报告里说“测过了”之前，先在这里扫一眼有没有相关条目。
2. 修改上面任何条目时，**同时**修改它的**状态**列和**出处**——出处是承重的，不是装饰。
3. 添加“未验证”条目很容易。**删除**一条则需要能说清为什么它现在算已验证。说不清就不要删。
