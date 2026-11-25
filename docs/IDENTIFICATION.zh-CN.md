[English](IDENTIFICATION.md) | 简体中文

# 动力学辨识（可选）

本模块把 Python SDK 的 96 参数傅里叶辨识移植到 `litearm-cpp`。它**默认关闭**；
核心 SDK 仍然零外部依赖。

## 构建

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DLITEARM_BUILD_IDENT=ON -DLITEARM_BUILD_TESTS=ON -DLITEARM_BUILD_EXAMPLES=ON
cmake --build build -j"$(nproc)"
ctest --test-dir build -R 'test_ident_' --output-on-failure
```

这个可选目标需要 Eigen3（header-only）。`include/litearm/ident/` 下的公开头文件只暴露
`std` 类型；Eigen 只出现在 `src/ident/*.cpp` 里。

## 模型

辨识参数向量有 96 项：

| 切片 | 含义 |
| --- | --- |
| `[0,70)` | 每个连杆 10 个惯性参数：`m, m*cx, m*cy, m*cz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz`（绕关节原点的惯量） |
| `[70,77)` | 对称库仑 `a` |
| `[77,84)` | 粘性 `fv` |
| `[84,91)` | 非对称 `b` |
| `[91,96)` | 电枢 `jm3..jm7` |

`fc0 = a + b`，`fc1 = -a + b`。`jm1/jm2` 与 link1/link2 的 `Izz` 结构共线，固定为 0。
`fv2/fv3` 是 HYY 预留字段，保持 0。惯性块使用 Pinocchio 的 `toDynamicParameters`
约定；固件模型的 `set_body` 字段是绕质心的，因此 `phi_to_body10` 负责做平行轴转换。

**默认固定惯量方案。** `10_identify` 通常从 `examples/SN.yaml` 固定全部 70 个惯性
参数，只辨识摩擦/电枢块；见下文“固定惯量”。要用 `--full-inertia` 解全部 96 个参数。

**单位质量基准（`--full-inertia`）。** 控制器的出厂 bank 把每个臂连杆质量都写成 1.0
（厂商 SN 模板也一样）。因此 `10_identify` 在这个基准下辨识：每个质量列固定为 1，
其余惯性参数（`m*c` 和绕关节原点的惯量）重新拟合。这是**受约束的再辨识**，
**不是**把物理质量解做缩放——单纯缩放会让加速度回归量的质量项按 `1/m` 改变，
破坏力矩拟合（在参考采集上 J2 RMSE 会从约 0.6 Nm 涨到约 9 Nm）。单位质量拟合在真机
数据上保持与物理质量模型相同的 RMSE。需要辨识并导出 kg 质量模型时，传
`--physical-mass`。

主机模型使用**控制器的 Craig/MDH 连杆系**，由 `DH/litearm_dh.urdf` 重建——不是厂商
`assets/litearm/urdf/litearm.urdf` 的连杆系。关节 4/6/7 绕 -Z 旋转
（`axis = s_i * z`），固定的 `ee_link` 并入 link 7。因此 `phi_flat` 直接表达在 DH
连杆系里，可以不做任何逐连杆转换就与 `DH/dh_params.yaml` 比对；同一个 `phi` 就是
`phi_to_body10` 转换到固件 body10 布局的内容（也是 `--write-model` 发出的内容）。
`cad_phi()` 是把厂商 CAD 惯量
重新表达在这些 DH 系里，只作为求解器先验。70 列回归量由 10 个基参数解析构造；
`rnea()` 是独立的递归牛顿-欧拉实现，用于校验和重力检查。

要让 Python SDK 使用 C++ 结果，把它的 `model.urdf_path` 指向 `DH/litearm_dh.urdf`
（见 `DH/README.md` 第 0 节），然后用 `--urdf` / `--urdf-sha256` 记录该文件，使 YAML
绑定到同一个模型。

## 物理可行性

在 `--full-inertia` 模式下，`identify()` 在返回前把每个连杆投影到物理可实现的刚体
参数：

- 十个值都有限，`m > 1e-4 kg`，`|c| <= 1 m`；
- `I_com` 正定；
- 主惯量矩满足三角不等式
  （`lambda_max <= lambda_mid + lambda_min`）。

默认固定惯量模式跳过这个投影，原样保留提供的 bank，包括非物理值。

质量或质心不可用的连杆（NaN/Inf、太小、或超出质心半径）会被替换成它的 CAD 连杆。
其余连杆只调整 `I_com`——质量和一阶矩（`m`、`m*c`）保持不变，所以重力力矩不变。
这是必需的，因为 `jm1/jm2` 与 link1/link2 的 `Izz` 结构共线并折叠进去；辨识出的张量
因此可能看起来像细杆并违反三角不等式。`identified_yaml`、`sn_template_yaml` 和
`write_model` 会重新检查结果，拒绝非有限或非物理参数（除非调用方显式放弃检查）。
固定惯量路径是显式例外：它保留提供的 bank，在输出头标记 `# inertia: fixed from ...`，
并允许导出/写回该 bank（`write_model` 的 `allow_nonphysical`，`10_identify` 对固定
惯量输出会自动打开）。没有这个 opt-out 时，`write_model` 在 `disable()` 之前做检查，
所以坏文件永远不会让正在保持的臂掉下来。

物理可行性是主机侧条件。固件的 `model.commit` 可能施加额外范围检查；见
`docs/UNVERIFIED_REGISTER.md`。

## 离线使用（无机械臂）

```bash
./build/10_identify \
  --input-csv /path/to/fourier_log.csv \
  --output ident_result
```

它接受 Python 17 的 44 列 CSV（旧版 37 列文件也接受）。结果只写成
`SN_<YYYYMMDD_HHMMSS>.yaml`，即控制器的 SN 配置格式：替换每个关节的 16 个
`dynamics_parameters` 值，保留模板的运动学/轴/限位。模板默认是
`examples/SN.yaml`（固定模型）；它的序列号后缀不会重复出现在输出名里。每次运行都会
追加一个带时间戳的新文件，所以连续运行形成时间序列，而不是覆盖上一次结果。该路径
在存在时使用实测位置（`qm`）和实测力矩（`tau_m`）。在 `--full-inertia` 模式下质量
写成 1.0（单位质量基准）；要 kg 质量请加 `--physical-mass`。

文件以注释头开始，记录模板、DH 坐标系约定、可选的 `--urdf`/`--urdf-sha256` 来源，
以及采集参数。旧的 `identified_params_*.yaml` schema 不再写出（示例 10 和 11 仍可
用 `--load-result`/`--load-params` 读取它）。

在 Python 参考日志上，C++ 求解器复现 Python 结果：
`rank=69/96`，`rank_inertia=43/70`，逐关节 RMSE
`0.5264 / 0.7338 / 0.3649 / 0.2424 / 0.1271 / 0.0683 / 0.0799 Nm`。

## 真机采集

激励始终是系数文件。默认是随包的两段 `examples/fourier_coeffs_default.txt`
（每段 77 个系数，5 次谐波），按 **24 s** 周期采集。路径从源码树解析，所以二进制在
build 目录里运行也可以；`--coeffs FILE` 覆盖文件。

采集是纯 PD 且连续的：清除 FF mask，`send_mit_all` 以 300 Hz 流式发送傅里叶
`q/dq`，100 Hz 状态流提供整段轨迹的实测 `q/dq/tau`。`ddq` 由轨迹解析得到。不使用
固件日志，所以 24 s 周期不受 8 s 容量停顿影响；每段每周期大约得到 2400 个样本。
采集从最大 `|dq|` 最小的相位开始（不一定是相位 0），并在 `movej` 到达该位姿后等待
0.15 s，使 `movej` 到 PD 的交接以及最后回到 `movej` 的交接都不会让速度跳变。
发送截止时间滑移、关节故障、跟踪误差超限，或有效状态样本少于 `1800 × cycles` 时，
采集会中止。

```bash
# 只读规划（连接、读限位、不运动）
./build/10_identify

# 真机采集默认系数文件；急停保持在手边
./build/10_identify --go --cycles 1
./build/10_identify --go --cycles 4 --output ident_result
```

命令循环在发送截止时间滑移、关节故障或跟踪误差过大时中止。结束时机械臂保持使能并
停在末姿态。

`--home-first` 使用 60 s 回零窗口。默认 `Arm::home()` 超时（15 s）在机械臂折叠顶着
硬限位启动时太短：固件以固定 0.10 rad/s 回零，超过 1.5 rad 的行程可能超过 15 s，
即使臂已经到达零位。如果回零报告超时，先读状态再重试；臂可能已经在零位。

## 傅里叶系数文件

默认激励是 `examples/fourier_coeffs_default.txt`；`--coeffs FILE`（或
`--coeffs-inline LIST`）直接提供另一个文件。每段有 **77 个数 = 7 关节 × 11 系数**，
按关节优先排列：

```
J1: a0 a1 b1 a2 b2 a3 b3 a4 b4 a5 b5
...
J7: a0 a1 b1 a2 b2 a3 b3 a4 b4 a5 b5
```

其中

```
q_j(t) = a0 + sum_{n=1..5} [ a_n*cos(n*w*t) + b_n*sin(n*w*t) ]
w = 2*pi / period
```

`FourierCoefficients` 把 sin 系数存进 `a`，把 cos 系数存进 `b`，所以外部 `a_n`
映射到 `b[n-1][j]`，外部 `b_n` 映射到 `a[n-1][j]`。

文件规则：

- 数字用空白和/或逗号分隔；`#` 开始注释；忽略 UTF-8 BOM；
- 可选的 `# period: <秒>` 注释钉住设计周期。用不同的 `--period` 解析会被拒绝；格式
  错误或前后冲突的声明也会被拒绝。没有该注释的文件沿用调用方给的周期（向后兼容）；
  随包默认文件声明为 24 s；
- 每段恰好 77 个值；段之间用只含 `---` 的一行分隔，或当当前段已有 77 个值时用
  空行分隔；部分填充段内部的空行被忽略；
- 行是关节，列是系数。11×7 布局也包含 77 个数，但含义不同；
- 支持一到两段。它们按顺序采集（`capture_center` 移动到每段的 `t=0` 起点），样本
  合并进一次 `identify()` 求解；`--cycles` 作用于每一段。

**限位拟合。** 载入的轨迹会被拟合到关节位置限位，但不修改系数文件：每个关节的中心
（`a0`）只移动到需要的程度；当偏移仍大于允许范围时，该关节的所有谐波按同一个公共
因子缩放。拟合使用固件的 `q_min`/`q_max` 并保留 0.05 rad 余量，所以也在稍宽的
URDF 限位内；每段会打印施加的平移/缩放。没有末端高度抬升，也没有 `sigma_min`
拒绝；规划只把 `z_min` 和 `sigma` 作为信息打印。SN 头记录拟合后的轨迹。`--period`
必须与设计这些数时使用的周期一致（速度和加速度按 `1/T` 和 `1/T²` 缩放）；解析器会
强制文件里的 `# period:` 声明并拒绝不一致，所以随包的 24 s 文件不能在 `--period 48`
下运行。`capture_center` 在命令非有限、超出固件关节限位或跟踪误差过大时中止——这是
运行期硬件护栏，不是轨迹设计。

```bash
# 默认两段文件的只读规划
./build/10_identify

# 采集两个默认段
./build/10_identify --go --cycles 1 \
  --sn-template /path/to/SN<serial>.yaml

# 另一个系数文件
./build/10_identify --go --cycles 1 \
  --coeffs /path/to/other_coeffs.txt \
  --sn-template /path/to/SN<serial>.yaml

# 内联（154 个数 = 2 段）；第一个值为负时用 =LIST
./build/10_identify --go --cycles 1 \
  --coeffs-inline='0.1, 0.2, ...' \
  --sn-template /path/to/SN<serial>.yaml
```

`--coeffs` 与 `--coeffs-inline` 互斥，且只适用于真机采集；与 `--input-csv` 或
`--load-result` 组合是错误。

## 固定惯量：只重新辨识摩擦/电枢

`--fixed-model FILE` 载入 SN 配置或 `identified_params` 文件，把全部 **70 个惯性
参数**固定为该文件中的值，只解摩擦/电枢块：

- 对称库仑 `a` (7)
- 粘性 `fv` (7)
- 非对称 `b` (7)
- 电枢 `jm3..jm7` (5)

这是默认方案：`--fixed-model` 默认是 `examples/SN.yaml`，`--sn-template` 也默认
同一个文件。改用 `--full-inertia` 解全部 96 个参数，或用 `--fixed-model FILE`
使用另一个惯量 bank。

固定惯量原样保留，即使它并非物理可行（例如控制器 bank 的 `m=1` 和负的质心惯量）；
运行会打印警告。此时 `rank_fit` 是 26 个活跃列的秩，而 `rank` 仍描述完整的 96 列
回归量。`fv2/fv3` 保持 0，通常的非负/`|b|<=a` 投影仍作用于辨识出的摩擦；特别是，
想要负 `jm` 的解会被钳到 0。

```bash
# 默认方案：固定 examples/SN.yaml，只解摩擦/电枢
./build/10_identify --input-csv ident_result/fourier_log.csv

# 另一个固定 bank
./build/10_identify --input-csv ident_result/fourier_log.csv \
  --fixed-model path/to/SN.yaml --sn-template path/to/SN.yaml
```

非物理惯量通常会被 `load_result_yaml` 和 `sn_template_yaml` 拒绝；默认/固定模型
路径在本次运行中选择不检查的路径，导出的 SN 头记录 `# inertia: fixed from ...`。
用 `--load-result` 重新载入这样的输出时，会自动识别该头；其他非物理文件仍可用
`--allow-nonphysical`。

## 模型写回

`--write-model` 是显式的，需要一个受支持的机械臂（提交期间失能）。它暂存 bodies
1..7（`set_body`），可选 `jm`（`--write-jm`），提交 `0x0FE` 或 `0x2FE`，读回模型，
并可选地把 `get_gravity(q)` 与主机 RNEA 比较。**从不**调用 `save_params()`，所以
重启会恢复 Flash 模型。提交前请托住机械臂。

`--write-jm` 把 jm 并入同一次提交。只提交 jm 会被固件的零模型哨兵拒绝
（`ERR{0x32,0x02}`），因为 `stage_begin_jm()` 不会填充 body 组；请用
`arm.model().commit_jm(jm)`，它会先把当前 bodies 重新 stage 一遍，再提交 `0x2FE`。
`write_model` 每次提交都会 stage bodies 1..7，因此不需要这个绕过。

`--write-friction`（需要 `--write-model`）还会把辨识出的摩擦写到固件的 0x26 向量：
item 9 = `fv`，item 10 = `fc0`，item 11 = `fc1`。写之前会检查
`fv >= 0`、`fc0 >= 0`、`fc1 <= 0` 的契约。摩擦立即生效（没有 staging/commit）；
`--rollback-gravity` 也会恢复之前的摩擦值，并打印读回误差。

`--load-result` 把 `phi` 原样写进固件。只载入针对 DH 连杆系辨识的文件（本工具导出的
SN 配置，或声明 `phi_frame: craig_mdh_link_frames` 的旧版 `identified_params` 文件，
或 `model.urdf_path` 指向 `DH/litearm_dh.urdf` 时产生的 Python 结果）。针对厂商 URDF
产生的结果描述的是同样的刚体，但处在不同的逐连杆系里；载入它们会静默安装错误的
动力学。旧版文件没有声明 DH 系时，示例会打印警告。固定惯量输出带有
`# inertia: fixed from ...` 头，载入/写入时不做物理检查；其他非物理文件需要
`--allow-nonphysical`。

```bash
./build/10_identify --go --cycles 4 --write-model
./build/10_identify --go --cycles 4 --write-model --write-jm --write-friction
```

要写入已有结果而不重新采集，加 `--load-result`。不带 `--write-model` 时它只解析
文件并打印质量/摩擦摘要：

```bash
./build/10_identify --load-result ident_result/SN_20261117_143620.yaml
./build/10_identify --load-result ident_result/SN_20261117_143620.yaml --write-model
```

对 SN 文件，加载器读取七个 `dynamics_parameters` 块，并把 body10（绕质心）转换回
`write_model` 使用的关节原点 `phi`；`fc0`/`fc1` 恢复对称库仑项和 DC 偏置。旧版
`identified_params` 文件仍被接受，并读取其 `phi_flat`、`friction` 和 `armature`
块。写回不保存到 Flash：重启会恢复之前的模型。

## 末端负载辨识（示例 11）

`11_identify_payload` 在 EE 法兰系中辨识末端点质量和质心。固件的负载模型正是如此：
`set_payload` 接受质量和三元素质心（`ff_scalar` item 4/5）；它没有负载惯量。
**真机运行使用固件自己的动力学 bank 作为空臂模型**：读取
`arm.model().get_body(1..7)`（只读 bank，不合成负载，Craig/MDH 连杆系）并转换为
RNEA 使用的 `phi`。这是控制器的权威模型，不需要示例 10 的文件。`--load-params FILE`
用示例 10 的结果覆盖它（SN 配置，或声明
`phi_frame: craig_mdh_link_frames` 的旧版 `identified_params` 文件）；离线 CSV
回放没有机械臂，所以仍需要 `--load-params`。结果记录 `model_source`（固件 bank 或
文件路径）、`com_frame: ee_flange (DH/litearm_dh.urdf)` 以及质量和质心。

### 快速开始

用 `./build.sh --werror` 构建（它会打开 `LITEARM_BUILD_IDENT`）。连接机械臂并清空工作区。
以下命令都在仓库根目录执行。

```bash
# 1. 只读：连接、打印位姿集/力矩余量/秩，不运动。
./build/11_identify_payload --port /dev/ttyACM0

# 2A. 快速绝对模式：20 个腕部位姿中的前 3 个，负载已装好。
./build/11_identify_payload --port /dev/ttyACM0 --go --absolute \
  --pose-count 3 --out /tmp/payload_test

# 2B. 差分模式（推荐；负载必须可拆）。
# 拆下负载，采空载基线：
./build/11_identify_payload --port /dev/ttyACM0 --go \
  --capture-baseline /tmp/payload_baseline.csv --pose-count 20
# 装上负载，再用该基线辨识：
./build/11_identify_payload --port /dev/ttyACM0 --go \
  --baseline /tmp/payload_baseline.csv --pose-count 20 --out /tmp/payload_test

# 3. 把结果写入固件（只写 RAM；从不调用 save_params()）。
./build/11_identify_payload --port /dev/ttyACM0 --go \
  --baseline /tmp/payload_baseline.csv --pose-count 20 --apply
```

输出：`--out`（默认 `ident_result`）下的 `payload_loaded_<stamp>.csv` 和
`payload_result_<stamp>.yaml`，以及 `--capture-baseline` 指定路径的基线 CSV。

- 必须加 `--go` 才会运动；不加只连接并打印规划。
- 加载采集前会清掉固件负载前馈。若不加 `--apply`，运行结束时会恢复运行前的负载。
- 基线与加载运行必须使用相同的 `--pose-count` / `--pose-set`。
- 示例结束时会保持使能并停在末位姿；请自行回零或失能。
- 默认速度 0.10；第一次可传 `--speed 0.05` 更慢。
- 当前位姿不在软限位内时，先运行
  `./build/litearm-cpp --port /dev/ttyACM0 home`。

负载重力力矩对 `theta = [m, m*cx, m*cy, m*cz]` 是线性的：

```
Y_j,0    = -(a_j x (p_ee - p_j)) . g
Y_j,1+k  = -(a_j x (R_ee e_k)) . g       k = 0,1,2
tau      = Y(q) theta
```

负号与 `rnea()` 的保持力矩约定一致（基座加速度 `-g`）。`p_ee`/`R_ee` 是
`ee_link` 系（`kDhEe`），不是 link 7。重力竖直时，J1（竖直偏航轴）对负载完全
不敏感，所以默认使用 J2..J7；倾斜安装时用 `--joints` 覆盖。单个位姿秩为 3
（一阶矩中沿重力方向的分量不可观测）；10 位姿族给出秩 4，缩放条件数约 3.4。

推荐的差分流程（同一次会话的空载基线抵消空模型误差和静摩擦偏置）：

```bash
# 1. 拆下负载
./build/11_identify_payload --go --capture-baseline payload_baseline.csv
# 2. 装上负载（空臂模型 = 固件 bank）
./build/11_identify_payload --go --baseline payload_baseline.csv
```

基线会用 `rnea(q_loaded) - rnea(q_baseline)` 传输到加载位姿（当稳定位姿略有不同
时）；RNEA 模型是启动时读取的固件 bank（或 `--load-params`）。基线应在加载运行
附近重新采集；超过 24 h 的基线会打印警告。绝对模式使用空臂重力（默认固件 bank），
对模型误差和静摩擦更敏感：

```bash
./build/11_identify_payload --go --absolute
```

采集使用 5 个静态腕部位姿：J1 和 J3 保持零位，J2 固定 0.9，J4 固定 -0.8，只有
J5..J7 变化（20 个代表性腕部角/边/中心；秩 4，条件数 4.33，EE z 0.25–0.40 m）。
拟合只用 J5..J7，避开模型误差更大的前轴。每个位姿以 12 ms 采样 80 个样本，
`|dq| <= 0.03 rad/s`，带固定引导位姿，速度倍率 0.10（硬上限 0.20）。加载采集前，
示例会把固件负载前馈清零（`set_payload(0,0,0)`），因为有负载会偏置实测力矩。运行结束
时会恢复运行前的负载；要改为写入辨识出的负载请传 `--apply`（中断或失败时也会恢复）。
`--pose-count 3` 或 `--pose-set ...` 仍可在快速检查时给出
秩 4，但位姿更少意味着同样残差下方程更少——最终辨识请用全部 20 个。每个位姿都会
以 `complete: 0` 检查点写入 CSV；最终行集为 `complete: 1`，所以中断的采集会被
拒绝，而不会静默求解部分序列。加载采集总是写到
`<out>/payload_loaded_<stamp>.csv`（或 `--capture-loaded FILE`），这让每次真机
求解都可以在没有机械臂时回放：

```bash
./build/11_identify_payload \
  --loaded-csv payload_loaded_<stamp>.csv \
  --baseline-csv payload_baseline.csv \
  --load-params ident_result/SN_<stamp>.yaml
```

`--apply` 调用 `set_payload(mass, com)` 并读回数值；固件会静默钳制越界值。它只写
RAM，从不调用 `save_params()`，所以重启会恢复之前的负载。不加 `--apply` 时，示例会
恢复运行前生效的负载。

## 限制

- SN/HYY 模板使用面向行的补丁器；它支持块式模板，且必须恰好有七个
  `dynamics_parameters` 块和全部 16 个键。
- 非对称库仑（`b`）在当前傅里叶数据上弱可辨识；求解器用 `--dc-ridge`
  （默认 `1e5`）收缩它。`--fix-dc` 强制 `b=0`。真正的非对称估计需要低速/过零激励。
- **负载**辨识的静态重力位姿由示例 11 实现；完整的 96 参数静态重力辨识仍未实现。
- 负载辨识受限于固件的点质量模型：质量和质心，没有负载惯量和负载摩擦。完整的
  惯性负载需要写模型，而不是 `set_payload`。
- 真机采集、基线采集和 `--apply` **未**被离线测试套件验证。当前状态见
  `docs/UNVERIFIED_REGISTER.md`。
