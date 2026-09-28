# 样例

七个可执行的样例。**默认只读** —— 任何会动的都要显式加 `--go`。

```bash
cd ..
source env.sh
./run_example.sh 01_hello
./run_example.sh 02_movej --go
```

| 样例 | 内容 | 需要 `--go` |
| --- | --- | --- |
| `01_hello` | 握手 + 固件版本 + 读状态 (q/dq/tau/flags/mode) + 当前 TCP | 否 (只读) |
| `02_movej` | 关节运动: `enable` → `movej` → 等到位 → 读回 | **是** |
| `03_move_p` | 笛卡尔点到点: 读 TCP → `move_p` → 回读 TCP | **是** |
| `04_ik_tcp` | `get_tcp` + `ik(pose)` 自洽性检查 | 否 (只读) |
| `05_ff_tune` | 动力学/控制律调参: `ff_preset` / gs / is / gravity / payload | **是** |
| `06_cartesian` | 笛卡尔直线 / 圆弧 / 多路点 (固件规划) | **是** |
| `07_vel_jitter_trace` | 双路逐拍采集: 100Hz 状态流 + 300Hz 固件日志 | **是** |

## 共用开关

所有样例都认这三个:

```
--port PORT    串口 (默认自动找 1d50:606f, 也可用环境变量 LITEARM_PORT)
--go           真正 enable/运动/改参 (默认只读连接, 不上力)
--speed S      move 速度倍率 0~1 (默认 0.3)
```

各样例自己的开关 (用 `--help` 看):

- `02_movej` 位置参数: 7 个目标关节角 (省略则默认"当前位置 + J3 小步")
- `07_vel_jitter_trace`: `--dist` `--dur` `--dir` `--tag` `--ff-mask`

## 安全约定

- **默认只读**: 不加 `--go` 时样例只连接、只查询, 一个动作命令都不发。
- **小步可回退**: 需要动的样例默认都只走 1cm / 0.1rad 这种小步。
- **跑完刻意不 `disable()`** (02/03/06/07): 该位形失能会让臂因自重坠回, 保持使能持位才是
  安全终态。急停要在手边。
- **不可逆命令**: `05_ff_tune` 会调 `save_params()` (**写 flash**)。别在已标定的臂上随手跑。

## 从 Python 样例迁移

对应关系:

| Python | C++ |
| --- | --- |
| `arm.get_state().value.q` | `arm.get_state().value->q()` |
| `arm.movej([...], speed=0.3)` | `arm.movej({...}, 0.3)` |
| `arm.move_l(pose, speed=0.5)` | `arm.move_l(pose, 0.5)` |
| `arm.model.get_body(1)` | `arm.model().get_body(1)` |
| `with arm.zero_g():` | `{ auto zg = arm.zero_g(); ... }` |
| `for jp in arm.params.all_joint_params()` | `for (const auto& jp : arm.params().all_joint_params())` |

`07_vel_jitter_trace` 是唯一做了**有意删减**的一个: Python 版还带"方向 IK 预检 +
终点软限位余量门槛"和四段 A/B 序列 (`--legs`)。那些预检要用 PC 侧模型, 而本包的定位是
**不做 PC 侧运动学** (重型计算全在固件里) —— 故 C++ 版只保留"一次平移 + 双路采集"主干,
把方向选择交给调用方。
