# litearm-cpp 开发者指南

LiteArm 机械臂的 C++ SDK —— 经 USB 串口直连固件。

规划、运动学与动力学都在固件里; PC 侧只做三件事: **编解码帧**、**发命令**、**判到位**。

## 目录

1. [模块地图](#1-模块地图)
2. [线协议](#2-线协议)
3. [读路径: 唯一读者 + 分队列](#3-读路径-唯一读者--分队列)
4. [写路径: 唯一写口 + 两道钩子](#4-写路径-唯一写口--两道钩子)
5. [笛卡尔: FIFO 配对与吸收额度](#5-笛卡尔-fifo-配对与吸收额度)
6. [锁序](#6-锁序)
7. [安全不变量](#7-安全不变量)
8. [错误面](#8-错误面)
9. [测试](#9-测试)
10. [与 Python 版的对照](#10-与-python-版的对照)

---

## 1. 模块地图

| 头文件 | 对应 Python | 职责 |
| --- | --- | --- |
| `protocol.hpp` | `_protocol.py` | 帧编解码 / CRC16 / 常量 / 状态帧布局 / 版本解析 / 覆盖契约 |
| `errors.hpp` | `errors.py` | 异常层级 + `(cmd, code)` 语义表 + `ERR` -> 异常的映射 |
| `rot.hpp` | `_rot.py` | 旋转/位姿纯数学 (ZYX 内旋, 与固件 `kin.c` 同约定) + 位姿形态归一化 |
| `state.hpp` | `state.py` | `RobotState` / `JointState` —— 状态帧的面向对象视图 |
| `transport.hpp` | `transport.py` | 传输接口 + 真串口 (POSIX/Win32) + 端口发现 + 进程内端口登记 |
| `ack.hpp` | `arm.py` 的 `_Ack` | **唯一读者** + 按 (上行 id, 回显码) 分队列 + `expect` |
| `cart.hpp` | `cart.py` | 笛卡尔 FIFO 配对 / 吸收额度 / `CartPlan` / 三条入口 |
| `arm.hpp` | `arm.py` | `Arm` 主类 / `Msg` / `LicenseInfo` / 零重力会话 / CLI |
| `params.hpp` | `params.py` | 关节级参数 (0x22/0x23/0x24/0x36) |
| `model.hpp` | `model.py` | 动力学模型在线导入 (0x30..0x39) |
| `log.hpp` | `log.py` | 300Hz 采集 (0x2D/0x2E) |
| `diagnostics.hpp` | `diagnostics.py` | 固件自检 (0x49) + 文本解析 |
| `testing.hpp` | `testing.py` | 脚本化应答的假传输 —— 让全套测试离线可跑 |
| `msg.hpp` | `arm.py` 的 `Msg` | 返回值信封 `{value, hz, timestamp}` |

**设计边界** (刻意不做): 任意 `fk(q)` —— 固件 `CMD_GET_TCP` 只能算**当前反馈 q** 的位姿,
没有"给 q 求位姿"的下行命令; 阻抗控制 —— 高级场景走 pylitearm + server。
PC 侧不做规划、不做运动学, 重型计算本来就在固件里。

---

## 2. 线协议

```
帧:  SOF(0xA5)  CMD(1B)  LEN(1B)  PAYLOAD(0..255)  CRC16_LO  CRC16_HI
CRC: CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, 无末异或), 覆盖 [SOF..PAYLOAD]
```

上行 100Hz 状态帧 (两种布局, 尾部 `joint_fault u16` 是固件 1.5.0 起新增的):

| 布局 | 固件 | 长度 (7J / 1J) |
| --- | --- | --- |
| `4 + 21N` | ≤ 1.4.x | 151 / 25 |
| `6 + 21N` | ≥ 1.5.0 | 153 / 27 |

单关节 21B = `q f32 + dq f32 + tau f32 + t_mos f32 + t_coil f32 + err u8`。
`flags` 的位分配: **bit0..5 = 安全 flag**, **bit6..8 = mode**, **bit9 = enabled**,
**bit10 = cart_busy** —— 所以 `flag_names` 只看 bit0..bit5 (见 `protocol.cpp` 里那个
`for (k = 0; k < 6; ++k)`; 把它改大就会让 mode/enabled/cart_busy 一起变成"故障位")。

⚠ **两个双 ID**: `0x49` (下行 `CMD_KIN_BENCH` / 上行 `RSP_JOINT_PARAM`) 与
`0x41` (下行 `CMD_GET_FIRMWARE` / 上行 `RSP_DETAIL`, 后者是**死登记**)。
靠**方向**区分, 不可用单张 ID 表查。

覆盖契约 `command_coverage()` (49 条) 是"固件每条已实现命令都有 SDK 入口"的断言载体,
由 `tests/test_protocol.cpp` 逐条对表钉住。

### 2.1 分帧器 `FrameReader` (`include/litearm/framing.hpp`)

把字节流切成帧的那台状态机 (SOF 扫描 / 半帧窗口 / 噪声留痕) **单独一个类**,
`SerialTransport` 的两条后端 (POSIX 与 Win32) **共用同一份**。

⚠⚠ **它存在的唯一理由是消灭重复。** 从前两条后端**各写过一份**同样的状态机,
代码里自称"与 POSIX 侧同一段逻辑" —— 那是**会漂的拷贝**, 而 Windows 那条分支在
Linux 上**连编都编不了**, 漂了也不会有人当场发现。抽出来之后两条后端只剩 I/O,
而这段逻辑**被编译、被单测** (`tests/test_framing.cpp` 9 条), 不再只能经 pty 间接验。

⚠ **`feed()` 是"取走"语义, 不是"试试看"** —— 丢掉它的返回值就是丢掉那一帧, 症状极隐蔽
(数据在流, 却一帧都读不出来)。实测**在同一个下午踩了两次**。标准用法:

```cpp
if (auto f = r.feed(chunk.data(), chunk.size())) return *f;   // 这一批里可能就有整帧
while (auto f = r.feed(nullptr, 0)) { /* 处理 f */ }          // 再取空缓冲
```

⚠ `tick(now)` 返回**缓冲有没有被改动**: 改动过就要立刻重试装配 (刚丢掉的假帧头后面
很可能就跟着一条真帧)。少了这个返回值, 调用方要么漏帧、要么空转。

---

## 3. 读路径: 唯一读者 + 分队列

**全包只有一个地方碰 `transport->read_frame`**: `Ack::reader_loop`。它只有两条纪律:

1. **只投递, 不判定** —— "这帧是谁的"由**队列**决定, 不由线程决定;
2. **死了要响亮** —— 死因进 `reader_error` 并唤醒所有等待者, 绝不静默退出。

帧的**归属 = 它落在哪条队列**, 键是 `(上行 id, 回显码或 kNoEcho)`:

- 只有 `RSP_ACK` / `RSP_ERR` 的 `payload[0]` 是"原命令码", 所以只有它们需要第二维;
  别的帧的 `payload[0]` 是**数据** (关节号 / item 索引 / 状态字节)。
- 于是 `ACK{0x10}` 与 `ACK{0x11}` 天然落在**两条**队列里 —— 并发命令互吃应答这件事在
  **结构上不可能发生**。

三类特殊投递:

| 帧 | 去处 | 理由 |
| --- | --- | --- |
| `RSP_STATUS` | **单槽** `state` + `status_seq++` | 100Hz **广播**帧。进队列会在 0.6s 内撑满 `kQueueMax`; 而"取走就没"的队列语义天生不适合广播 |
| `RSP_CART_PLAN` | 锁**外**交给 `CartPending::on_reply` | 避免在全局锁里调另一个子系统的锁 |
| 其它 | `queues[(id, echo)]` 队列 | 每条队列封顶 `kQueueMax`, 越界丢**最旧**并计 `dropped` |

`expect(want, timeout, label, raise_on_err, echo_cmd, err_waits_for_ack)`:
`want` 是 `ACK`/`ERR` 时 **`echo_cmd` 必填** (不给就分不清"我的 ACK"与"别人的 ACK");

- `err_waits_for_ack=true`: 一条匹配 `echo_cmd` 的 `ERR` **不是终局** —— 继续等
  `ACK{echo_cmd}`; 找到 ⇒ 那条 ERR 是**别人的** (本命令其实已被受理, 不抛);
  窗口耗尽仍只有 ERR ⇒ 那条 ERR 就是本命令的, 抛它。
  代价: **真被拒时要等满整个窗口**。所以只有笛卡尔三条入口开这个开关。

⚠ **这条"受理必回 ACK、拒绝绝不补 ACK"的形状只在生成侧成立, 到达侧不是**:
受理那条 `ACK{cmd}` **自己也会丢** (固件应答 FIFO 满时丢最新)。

### 三种读者

| 类型 | 取帧口 | 例子 |
| --- | --- | --- |
| 队列等待者 | `Ack::wait` (在指定队列上等) | `expect` / `enable` / 参数读回 |
| 驱动型读者 | `Arm::pump_until` (**一条帧都不认领**) | `read_status` / `get_status_now` / `wait_settled` |
| 事件等待者 | `token` 上的条件变量 | `CartPending::wait` |

驱动型读者是"广播帧不进队列"这条设计的必然结果: 它只是**等单槽前进**, 于是多个并发
`get_state()` 不再互相饿。

---

## 4. 写路径: 唯一写口 + 两道钩子

`Arm::raw_write` 是**所有**下行帧的唯一出口 (含零重力保活线程, 它刻意绕过
`write_cmd` 的零重力守卫, 但**不该**绕过清队)。钩子顺序是承重的:

```
① fork 守卫 / 终态守卫   —— 结构性保证"子进程零下发""终态不再发帧"
② 清队 (cart_->clear_pending)  —— 必须在 drain_for **之前**
③ drain_for(cmd)         —— 必须在写**之前**
④ tr_->write_frame(...)
```

- **③ 为什么必须在写之前**: 我们的应答只可能在写**之后**到达, 故此刻清队不可能吃掉自己的
  应答。反过来 (先写后清) 会 100% 吃掉。

⚠⚠ **2026-09-28: 这里原来还有一道"同帧节流" (排在 ② 之前), 已整套删除。**
它当年排在清队之前有射程 (清队的理据是"固件**收到**这条命令才会作废在途规划", 而帧被丢掉时
固件状态一点没变)。删掉之后**那条射程自动消失** —— 唯一写口的新不变式是
「**进了这个函数就一定会写出去**」, "清队了但帧没发"这种半状态**不可构造**。
见 README「与原版的差异」①。

上层两个出口:

| 出口 | 零重力守卫 | 用途 |
| --- | --- | --- |
| `write_query` | 无 | **查询类** (get_tcp/get_ik/参数读回/采集/自检) —— 拖动示教期间仍应能读状态 |
| `write_cmd` | **有** (可 `guarded=false` 关掉) | **动作类**。`guarded=false` 只给降能量方向的动作 (急停/失能) —— 它们必须永远可达 |

---

## 5. 笛卡尔: FIFO 配对与吸收额度

固件原生笛卡尔 (`0x3A/0x3B/0x3E`) 的应答载荷里**没有命令 id**, 只能按**受理顺序**
FIFO 配对; 而固件那份应答只有**一个槽位** ⇒ 同一排空窗口内登记 ≥3 条时, 中段请求零应答。

三条入口 (`move_l` / `move_c` / `move_path`) **全程持串行锁**: 从登记 token 一直持到
`0x4E` 配对完成; `wait=true` 时**再持到停稳**。

### 两条守卫

**"多了一条"** (队列空却收到 `0x4E`): 计数 + 抛基类 `LiteArmError`。
不给它专门的异常类型是刻意的 —— 那是**配对模型与固件脱同步**, 内部不变量被破坏,
没有任何调用方能据它做出正确决定, 也**不该被专门 catch**。

**"少了一条"** (token 超时没收尾): 摘除该 token 并抛 `CartReplyLostError` (**不是**
`MotionTimeoutError` —— 这里要表达的是"未知结局", 混进通用超时会让调用方按"没生效"去重发)。

### 吸收额度

清队/放弃一条 token 时, 固件那条应答**可能已经在路上**。不留额度的话, 它一到就撞上
"队列空"判据 ⇒ `LiteArmError` 从**毫不相干的读路径**炸出来。

额度 = 摘掉的条数, 存活 `absorb_ttl = max(move_timeout, 12.0)` 秒。

⚠ **两个方向都会出错**:

- **取大** ⇒ 一条真·脱同步的应答被静默吸收 (硬错误降级成**静默**);
- **取小/过期** ⇒ 一条还在路上的应答撞上"队列非空"而被配给**新登记**的 token ⇒
  调用方拿到**别人那条**的结果并**报成功** = **假成功**。

两害相权必须**偏向取大**。下限 12.0s 的固件依据是三段相加 (规划 3s + 擦写停顿 8s +
链路 1s), 逐段出处见 `cart.hpp` 里 `kAbsorbTtlFloor` 的注释。

**清队的两条正确性条件**:

- `n == 0` 时**不刷新截止** —— 零重力保活每 40ms 发一条清队 opcode (几乎永远是空清队),
  若空清队也刷新截止, 额度就**永远不过期**;
- 额度**跨多次清队累加** —— 写成"重置为本次条数"的话, 上一次那条待吸收的迟到应答会退化
  成硬错误。

**`drop_and_absorb` 的唯一例外**: `timed_out == true` 且本条请求的窗口里**已经消费过额度**
⇒ **不补** (吸收**先于**配对, 所以被吃掉的就是它自己那条; 再补就会**自持** ——
实测改前能让新会话此后每一条笛卡尔命令都报"结局未知")。
⚠ 这条例外**只对超时支成立**: pump 支 (读链路当场就炸) 根本没等过窗口, 那句前提在它身上
没有依据。

---

## 6. 锁序

```
cart_serial_  →  CartPending::lock_  →  transport::wlock_
zg_lock_      →  CartPending::lock_  →  transport::wlock_
```

**反向无环**, 判据是"下层都不知道上层": `CartPending` 与 `SerialTransport` 都不引用
`cart_serial_` / `zg_lock_`。

- `cart_serial_` 是 **`std::recursive_mutex`** —— 嵌套点只有一个: `move_path` 把
  `BEGIN + ADD×n + RUN` 整段圈进本锁, 而 RUN 那一段本身也走 `request_and_wait`。
- `zg_lock_` 只串行化启停; **保活线程不取它** (它取 `zg_err_mu_`) ——
  否则会与"持着 `zg_lock_` 等 join"的 `zero_g_stop` 死锁。
- **`emergency_stop`/`disable`/`zero_g*`/`get_tcp` 自己不许获取 `cart_serial_`** ——
  降能量方向的动作与只读查询必须永远可达 (持锁者可能阻塞到 `move_timeout`)。
  ⚠ 但**入口在自己的临界区里调用 `get_tcp()` 是另一回事**: `move_c` 的起点校验就是那样。
- `Ack::mu` 是**叶子锁的加强版**: 只在 `deliver`/`wait` 内部持有, 且 `deliver` 里
  **不调用任何子系统** (`on_reply` 刻意留在锁外)。

---

## 7. 安全不变量

逐条都有测试钉着:

1. **fork 的子进程零下发** —— `ForkedSessionError` 挂在三个结构性收口
   (`require` / `Ack::wait` / `raw_write`) 上, 而不是逐入口枚举。
   子进程里的 `close()` **故意泄漏** `Ack` 与传输层两个句柄。三条**独立**的死锁,
   各自实测过: ① `SerialTransport::close()` 要取的读锁很可能正被父进程的读线程持着;
   ② `join()` 一条在子进程里**不存在**的线程会永久阻塞;
   ③ ⚠ 最隐蔽的一条 —— 析构一个**有人停在上面的** `std::condition_variable`
   (`Ack::stop_cv_`) 会永久阻塞 (glibc 组切换要取 condvar 的内部锁, 而那把锁与已不存在的
   等待者绑在一起)。③ 在**成员析构**里, 躲不掉 ⇒ 调用方必须**根本不析构**那个对象。
   见 `src/arm.cpp` 里 `close()` 的 fork 分支与 `Ack::~Ack()` (那里有最小复现的描述)。

   **推论 (写给下一个加线程的人)**: 任何"子进程里可能被析构、且持有 condvar"的对象都有
   这个形状。新增这类成员之前先想清楚它在 fork 之后会不会被谁析构。
2. **终态零下发** —— `enter_dfu` 确认设备消失后置位, 之后每个入口抛 `ArmIsInDfuError`。
3. **写失败 ⟹ 整帧未送达** —— `write_frame` 把 `write()` 抛与 `flush()` 抛**分开报**:
   前者抛 `TransportError`, 后者**不抛** (整帧已在驱动里, 失败收不回来), 只计
   `flush_failures`。这条不变量是 `CartPending::request` 能安全摘 token 的前提。
4. **`RSP_CART_PLAN` 应答里没有命令 id** ⟹ 任何 FIFO 方案必有一支判错 ⟹ 只能选
   **判错得安全**的那一支: 宁可报"结局未知", **绝不报"成功"**。
5. **到位 ≠ 停止** —— `CartPlan::settled` 要两条一起成立: 判到位收的尾 **且**
   到位判据满足**之后**回读的**实际 TCP** 与目标对得上。只看 `bit10` 会把"中途被扯断、
   TCP 根本不在目标上"报成到位。
6. **零重力期拒绝动作命令, 但放行查询与降能量动作** —— 两处抛出点共用**同一个常量**
   (`ZERO_G_GUARD_MESSAGE`), 改一处不会静默漂移。
7. **`0x00` 恒等于"固件没有这条命令"** —— 它是唯一稳定的哨兵, 比版本号可靠。
8. **不可逆命令不代劳** —— `save_params` / `reset_factory` / `model.commit` 都要求调用方
   自己先 `disable()`, 库不替调用方做安全决策。

---

## 8. 错误面

```
LiteArmError                     本包所有错误
├─ NotConnectedError             未连接 / 链路已关
│  └─ ForkedSessionError         子进程里沿用父会话 (fail-closed)
├─ TransportError                串口读写 / 帧 CRC / 链路丢失
├─ FirmwareMismatchError         版本不符合约定或过旧
├─ InvalidCommandError           参数/命令非法
├─ MotorFaultError               状态帧 FAULT / EMERGENCY / 单轴断轴
├─ MotionTimeoutError            move 超时未到位
├─ IKError                       IK 失败
├─ CommandRejectedError          固件显式 ERR{cmd, code}
│  └─ UnsupportedByFirmwareError ERR code == 0x00 ("固件没这条命令")
├─ CartesianPlanError            规划被拒 (IK/共线/超容量/越限) —— 臂一步没动
├─ MotionSupersededError         被新请求取代 (预期内的接管, **不是失败**)
├─ CartReplyLostError            0x4E 丢了 —— 结局未知
├─ ArmIsInDfuError               会话终态
└─ NotRemoteable / NotSupportedOnThisBackend / TeleopLockedError / TeleopBusyError
```

三条**刻意不继承**的关系 (都有测试钉着):

- `MotionSupersededError` **不**继承 `CartesianPlanError` —— 混进"规划失败"会让正常抢占
  走成故障分支;
- `CartesianPlanError` **不**继承 `CommandRejectedError` —— 固件这里回的是**规划结果**,
  不是 `ERR{cmd, code}` 形态, 硬套会凭空多出语义错误的 `cmd`/`code` 字段;
- `ArmIsInDfuError` **不**继承 `NotConnectedError` —— 后者语义是"连上即可", 而终态不是
  一次可恢复的掉线 (混在一起会让"断连就重连"的逻辑把 DFU 当普通掉线)。

`err_reason(cmd, code)` 是 `(cmd, code)` 的**唯一**解读处, 三级查找:

1. 具体档 `err_text()` 命中 ⇒ 用它 (同一个 `0x03` 在 `0x01`/`0x10`/`0x23` 上语义不同);
2. 通用档命中 ⇒ 用它 **并附上原始码**;
3. 都没有 ⇒ 明说"未登记", **带上原始 cmd/code**。

第 2/3 档之所以必须带原始码: 固件新增一档错误码时, 这条路径就是**唯一**会让上位机看见
"这是我没见过的码"的地方。

---

## 9. 测试

```bash
./build.sh                       # 构建 + ctest
./build.sh --asan                # AddressSanitizer + UBSan
ctest --test-dir build -R test_cart -V
./build/tests/... --filter movej # 单文件内按名字过滤 (每个 test_*.cpp 是独立可执行文件)
```

| 测试 | 覆盖 |
| --- | --- |
| `test_protocol` | CRC (标准检查值 + 独立参考实现) / 组帧解帧 / 版本解析 / 开机签名 / 两种状态帧布局 / 覆盖契约逐条对表 |
| `test_rot` | rpy<->矩阵 / 万向锁 / SO(3) 对数指数 / slerp / 姿态误差方向 / 三种位姿写法 |
| `test_errors` | 层级关系 / 三级查找 / 白名单 / `ERR` -> 异常的类型映射 |
| `test_state` | enabled/cart_busy/faulted/断轴/`drop_hold_inferred` |
| `test_transport` | **真 pty**: 逐字节往返 / 噪声跳过与留痕 / 坏帧重同步 / 半帧跨调用保留 / `timeout=0` 的"有就给我" / 假帧头超时丢弃 / 端点独占 / 失败路径释放登记 |
| `test_framing` | 分帧器**直接**单测 (不再只能经 pty 间接测): 一次喂多帧取空 / 跨调用半帧 / 噪声留痕与封顶 / 假帧头超时 / 坏帧重同步 / `reset` |
| `test_register` | 《未验证登记册》的格式守门: 五列 / 每条都有出处 / id 恰好 1..N / 没有丢了行首 `\|` 的隐形行 |
| `test_packaging` | 卖出去之前能守住的那几类: 版本单一口径 / 公开头的兄弟头都在装出去的目录里 / 打包模板在 |
| `test_arm_assembly` | 握手 / 版本拒绝 / 幂等 connect / 改靶 / 收尾 / DFU 终态四条路径 |
| `test_commands` | 使能重试白名单 / 安全命令 / 关节运动与到位 / 伺服透传 arity / 状态读取 / IK |
| `test_ff` | 0x26/0x27/0x28/0x31 + 0x2B/0x2C 读回 / item 表 |
| `test_params` | 0x22/0x23/0x24/0x36 + 武装门禁 |
| `test_model` | 探测 / staging-bank-commit 三层 / 掩码逐位 / 需失能 |
| `test_log` | 样本布局 / 游标续读 / 掉帧重试 / 游标不前进的死循环保护 / 逐拍记录模型 |
| `test_diagnostics` | 计时行 (名字与数字相连) / LINK 行 (**名字带数字的键**) / 两帧收齐 / 旧固件兼容 |
| `test_cart` | `CartPlan` / 异常映射 / 三条入口 / **FIFO 配对** / **吸收额度两个方向** / 清队 / 能力探测 |
| `test_zero_g` | 保活线程 / 双向守卫 / RAII / 保活中断可见 / close 停线程 |
| `test_license` | 26B 记录 / 未激活 / `0x02` 聚合档的回读定性 / 未授权时的使能门禁 |
| `test_threading` | 并发状态读者 / 并发笛卡尔入口串行 / close 唤醒等待者 / `Msg.hz` 语义 / 唯一写口无丢弃路径 |
| `test_precheck` | 客户端预检 (非有限值 / 软限 / 速度 / 容差) 与软限缓存的两条纪律 |
| `test_accessors` | 七个只读诊断访问器 (含终态下不抛、以及包装层转发链) |
| `test_clock` | 可注入时间源: 默认是真钟 / 注入可见 / 假钟下"超龄判失联"确定性可验 |
| `test_frame_ownership` | **帧归属**: 唯一读口 (源码扫描) / 归属判据 / 两线程互不吃应答 / 陈旧帧清队 / 队列封顶 / 守卫 |
| `test_full_coverage` | **命令覆盖动态哨兵** (每条已实现命令都真发出去过) / 无多余 id / 公开成员名单守卫 / 未连接时的错误类型全扫 |
| `test_protocol_sync` | **与固件头文件双向比对** —— **需 `LITEARM_FW_DIR`, 否则 SKIP** (见 §9 末) |
| `test_fork_guard` | 真 `fork()`: 子进程零下发 / 收尾不挂 |

**全部离线** —— 用 `testing::FakeTransport` (按命令脚本化应答的假传输)。
串口本身用**真 pty** 测: 桩会把 `timeout` 整个忽略掉, 于是"`read_frame(0)` 其实一个字节
都不碰"这类缺陷在桩上全绿 (上游就这么漏掉过一次真机故障)。

**唯一的例外**是 `test_protocol_sync`: 它要读**固件仓库**的源码, 找不到时**SKIP**
(`LITEARM_FW_DIR`, 默认 `~/litearm-stm32`)。帧归属那套里还有两处**源码扫描**判据
(唯一读口、公开成员名单) —— 它们用 `lt::repo_root()` 从 `__FILE__` 推路径, 与当前工作
目录无关 (ctest 的 cwd 是构建目录, 相对路径在那边打不开 —— 实测踩过)。

⚠ **SKIP 不是通过**: 报告里单独计数并逐条打印。看到"N 跳过"就说明那部分**没测**。

---

## 10. 与 Python 版的对照

移植是**逐条对照**的: 同一个常量值、同一句错误文案、同一个判据顺序。
语言层的差异只有这些:

| 主题 | Python | C++ |
| --- | --- | --- |
| 位姿入参 | 运行期判形态 | `rot::PoseInput` (三种写法各有具名工厂; 6 向量隐式转换) |
| 取值 | `None` | `std::nullopt` |
| 属性 | `arm.n` | `arm.n()` (只读) / 公开成员 (可调旋钮) |
| 上下文管理器 | `with arm.zero_g():` | RAII `ZeroGSession`, 析构不抛 |
| 重载 | 同名多态 | `Arm::write_frame(cmd, {payload})` 之类便捷重载 |
| 弱引用 | `weakref` (读线程不钉住 Arm) | 不适用: 读线程在 `close()` 里被 join, 生命周期由 join 顺序保证 |
| `__del__` | GC 兜底收尾 | 析构兜底收尾 (同样只委托给 `close()`) |

⚠ **两处已知的继承差异** (原版就有, 移植时刻意保留):

1. `license()` 不传 `echo_cmd`, 于是在**低于 1.8.0** 的固件上报"无应答超时"而不是
   "固件没有这条命令" (证据见 `tests/test_license.cpp`)。
2. `linux` 之外不做 CDC 自动发现 (macOS 无 sysfs; Windows 需要 SetupAPI 才能可靠读到
   VID:PID, 猜一个会给出假阳性)。
