[English](README.md) | 简体中文

# 示例

11 个可运行程序，用于在真机上验证 SDK；运行任何一个之前先读本页。

**默认只读。** 任何会运动的操作都必须显式加 `--go`。

```bash
cd ..
./run_example.sh 01_hello
./run_example.sh 02_movej --go
```

| 示例 | 作用 | 需要 `--go` |
| --- | --- | --- |
| `01_hello` | 握手、固件版本、状态读取（q/dq/tau/flags/mode）、当前 TCP | 否（只读） |
| `02_movej` | 关节运动：`enable` → `movej` → 等到位 → 读回 | **是** |
| `03_move_p` | 笛卡尔点到点：读 TCP → `move_p` → 读回 TCP | **是** |
| `04_ik_tcp` | `get_tcp` + `ik(pose)` 自洽校验 | 否（只读） |
| `05_ff_tune` | 动力学与控制律调参：`ff_preset` / gs / is / gravity / payload | **是** |
| `06_cartesian` | 笛卡尔直线 / 圆弧 / 多路点（固件规划） | **是** |
| `07_vel_jitter_trace` | 双通道逐拍采集：100 Hz 状态流 + 300 Hz 固件日志 | **是** |
| `08_zero_g_drag` | 拖动示教：设置末端负载（夹爪）→ `zero_g` → 手动拖动 → 退出前检查软限位 | **是** |
| `09_ik_accuracy` | 固件 IK 精度扫描：FK 模型自检 vs `get_tcp`、Halton 关节空间采样、PC 侧 FK 复算、不可达位姿拒绝、`--dump` CSV | 否（只读） |
| `10_identify` | 控制器 Craig/MDH 连杆系（`DH/litearm_dh.urdf`）下的动力学辨识：默认从 `examples/SN.yaml` 固定 70 个惯性参数，只辨识摩擦/电枢；`--full-inertia` 解全部 96 参数。离线 CSV 求解（`--input-csv`），默认两段激励 `examples/fourier_coeffs_default.txt` 24 s，或其他系数文件（`--coeffs`/`--coeffs-inline`，每段 77 个，按关节限位拟合且不修改文件，不做末端高度/奇异处理），SN 导出，可选 `--write-model`（需要 `LITEARM_BUILD_IDENT=ON`） | 采集/写回时 **是** |
| `11_identify_payload` | 从静态重力位姿辨识末端点质量负载（质量 + 质心），使用示例 10 的 DH 系模型：空载基线（`--capture-baseline`）、差分求解（`--baseline`）、绝对求解（`--absolute`）、离线 CSV 回放、可选 RAM-only `--apply`（需要 `LITEARM_BUILD_IDENT=ON`） | 采集/应用时 **是** |

## 示例 11 快速开始

`11_identify_payload` 会走过一组静态腕部位姿。用 `./build.sh --werror` 构建（它会打开
辨识模块），然后在仓库根目录执行：

```bash
# 只读规划：连接、打印位姿集/力矩余量/秩，不运动。
./build/11_identify_payload --port /dev/ttyACM0

# 快速绝对模式：20 个腕部位姿中的前 3 个，负载已装好。
./build/11_identify_payload --port /dev/ttyACM0 --go --absolute \
  --pose-count 3 --out /tmp/payload_test

# 差分模式（推荐；采基线时拆下负载）。
./build/11_identify_payload --port /dev/ttyACM0 --go \
  --capture-baseline /tmp/payload_baseline.csv --pose-count 20
./build/11_identify_payload --port /dev/ttyACM0 --go \
  --baseline /tmp/payload_baseline.csv --pose-count 20 --out /tmp/payload_test

# 把结果写入固件（只写 RAM；从不调用 save_params()）。
./build/11_identify_payload --port /dev/ttyACM0 --go \
  --baseline /tmp/payload_baseline.csv --pose-count 20 --apply
```

结果：`--out`（默认 `ident_result`）下的 `payload_loaded_<stamp>.csv` 和
`payload_result_<stamp>.yaml`，以及 `--capture-baseline` 指定路径的基线 CSV。

- 必须加 `--go` 才会运动；不加只连接并打印规划。
- 加载采集前会清掉固件负载前馈。若不加 `--apply`，运行结束后固件负载保持 0。
- 基线与加载运行必须使用相同的 `--pose-count` / `--pose-set`。
- 示例结束时会保持使能并停在末位姿；请自行回零或失能。
- 默认速度 0.10；第一次可传 `--speed 0.05` 更慢。
- 当前位姿不在软限位内时，先运行
  `./build/litearm-cpp --port /dev/ttyACM0 home`。

## 公共开关

所有示例都接受这三个：

```
--port PORT    串口（默认自动发现 1d50:606f，或使用 LITEARM_PORT 环境变量）
--go           真正使能 / 运动 / 写参数。不加时连接只读、不上力。
--speed S      运动速度倍率 0~1（默认 0.3）
```

各示例自己的开关（用 `--help` 查看）：

- `02_movej` 位置参数：7 个目标关节角。省略时默认“当前位置 + J3 一小步”。
- `07_vel_jitter_trace`：`--dist` `--dur` `--dir` `--tag` `--ff-mask`
- `08_zero_g_drag`：`--seconds`（时间上限，0 = 不限）`--payload`（kg）`--com X,Y,Z`（完整 ee 法兰质心，推荐）或 `--comz`（只给 z）。回车或 ^C 结束。
- `09_ik_accuracy`：`--n`（样本数）`--seed zero|target|prev` `--margin`（采样占关节范围的比例）`--dump FILE`（逐样本 CSV）`--unreach K`（不可达位姿拒绝探测）`--timeout` `--frame ee|j7` `--j6axis 1|-1`（强制某个模型变体，而不是自检选择）。
- `10_identify`：`--input-csv FILE`（离线求解，无机械臂）、`--cycles N`、`--period S`（默认 24）、`--send-hz H`、`--kp`/`--kd` 列表、`--coeffs FILE` / `--coeffs-inline LIST`（每段 77 个数，按关节优先 `a0,a1,b1,...,a5,b5`；按关节做中心平移 + 统一谐波缩放拟合到限位，不修改输入文件；不做末端高度/奇异处理；默认文件 `examples/fourier_coeffs_default.txt`）、`--fixed-model FILE`（默认 `examples/SN.yaml`；保留其 70 个惯性参数，只解摩擦/电枢）、`--full-inertia`（改为解全部 96 参数）、`--allow-nonphysical`（接受/导出非物理惯性块）、`--sn-template FILE`（默认：固定模型文件；结果写成 SN `dynamics_parameters` 配置）、`--physical-mass`（`--full-inertia` 模式下用 kg 质量而不是 m=1 的单位质量基准）、`--urdf`/`--urdf-sha256`（在 SN 头记录模型来源；使用 `DH/litearm_dh.urdf`）、`--write-model`、`--write-jm`、`--rollback-gravity`，以及 `--help` 里的 ridge/正则开关。需要 `-DLITEARM_BUILD_IDENT=ON`。
- `11_identify_payload`：`--capture-baseline FILE`（负载已物理拆下）、`--baseline FILE`（差分；推荐）、`--absolute`（无基线；使用固件 bank）、`--loaded-csv FILE` / `--baseline-csv FILE`（离线回放）、`--load-params FILE`（空臂模型覆盖；默认固件模型 bank，离线回放必需）、`--capture-loaded FILE`、`--apply`（仅 RAM）、`--pose-count N`（3..20，默认 20）、`--pose-set LIST`（显式 1 基位姿，例如 `1,10,20`）、`--samples N`、`--sample-interval S`、`--speed S`、`--max-pose-error RAD`、`--max-payload-kg KG`、`--max-com-radius M`、`--bootstrap-mass`/`--bootstrap-com`、`--condition-max`、`--huber-iterations`。需要 `-DLITEARM_BUILD_IDENT=ON`。

## 安全约定

- **默认只读。** 不加 `--go` 时示例只连接和查询，完全不发运动命令。
- **小步可退。** 会运动的示例默认走小步，例如 1 cm / 0.1 rad。
- **结束时刻意不 `disable()`**（02/03/06/07/08）。在这些位姿失能会让机械臂在自重下
  掉落，所以保持使能并托住位置是安全的收尾。急停保持在手边。
- **拖动示教是松的，不是停住的。** `08_zero_g_drag` 把臂置于零重力：它跟随重力而不是
  保持位姿，所以带 `--go` 运行前先扶住。任何关节在软限位外时它拒绝退出——零重力
  跳过越限锁存，但退出会立即重新武装它（见 `arm.hpp`:359），重启会重新锁存。再按一次
  回车（或 ^C 两次）可强制退出。
- **只写 RAM。** `05_ff_tune` 把 FF 参数写入 RAM，从不调用 `save_params()`；断电会恢复
  Flash 值。要固化到 Flash 是另一个不可逆操作，需手动执行。

## 从 Python 示例迁移

对应关系：

| Python | C++ |
| --- | --- |
| `arm.get_state().value.q` | `arm.get_state().value->q()` |
| `arm.movej([...], speed=0.3)` | `arm.movej({...}, 0.3)` |
| `arm.move_l(pose, speed=0.5)` | `arm.move_l(pose, 0.5)` |
| `arm.model.get_body(1)` | `arm.model().get_body(1)` |
| `with arm.zero_g():` | `{ auto zg = arm.zero_g(); ... }` |
| `for jp in arm.params.all_joint_params()` | `for (const auto& jp : arm.params().all_joint_params())` |

`07_vel_jitter_trace` 是唯一一个有**刻意省略**的示例。Python 版还带有“方向性 IK 预检
加末端软限位余量门”和四段 A/B 序列（`--legs`）。那些预检需要 PC 侧模型，而本包明确
不做 PC 侧运动学——重型计算全在固件里——所以 C++ 版只保留“一次平移加双通道采集”
的主干，方向选择交给调用方。
