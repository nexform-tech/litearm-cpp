// 样例 09 · 固件 IK 精度扫描 (只读, 不需 --go)。
//
// 目的: 在关节空间准随机遍历工作空间, 对每个可达目标位姿调用固件后台 IK
// (CMD_GET_IK 0x42, DLS), 再用 PC 侧 FK 复算末端位姿, 统计求解精度。
//
// 为什么精度只能在 PC 侧量: SDK 明说不做 PC 侧运动学 (没有 fk(q) 下行命令,
// 见 docs/DEVELOPER_GUIDE.md "Design boundaries"), 而 IK 应答只回 7 关节角 + 1 个
// ok 字节、**不带残差** (src/arm.cpp: `RSP_IK` 的 payload[7*4]==0 即 IKError) ——
// 固件自己只报"成不成", 不报"有多准"。于是复算 FK 的模型只能取自描述包:
//   ~/my_ws/src/litearm/urdf/litearm.urdf  (与 litearm.csv 同源导出)
// 下面 kJoints/kEe 就是它的逐行数值拷贝。**同一套 rpy 约定** (ZYX 内旋) 已在
// rot.hpp 里逐元素对齐固件 kin.c, 所以 PC 复算和固件算的是同一个东西。
//
// 流程:
//   0) 连接 (只读): 读当前 q / TCP / 各关节软限位 (0x24)
//   1) 模型自检: FK_pc(当前 q) vs 固件 get_tcp()。**两者不一致就谈不上测固件 IK**
//      —— 这里同时试 4 个变体 (末端帧 ee/j7 x j6 轴符号 +1/-1), 打印各自的偏差。
//      URDF 的 joint6 轴是 (0,0,-1) 而 CSV 写 (0,0,1), 两份厂商文件冲突, 用实测定。
//   2) Halton 准随机采样 N 个关节构型 (各自限位区间内) -> 目标位姿 = FK_pc(q_t)
//   3) 逐个 arm.ik(pose_t, seed) —— 记往返耗时 (含串口) / 成败 / 解
//   4) FK_pc(q_ik) vs 目标位姿 -> 位置误差 (mm) / 姿态误差 (deg); 另记关节空间偏差
//      (判解是不是落在别的分支上 —— 7 轴冗余, 换分支不算错, 只要笛卡尔准)
//   5) 汇总分位数 + 直方图 + 覆盖范围; --dump 出逐样本 CSV 供进一步分析
//
// seed 默认全零 (固件零点, 与 moveit initial_positions.yaml 一致)。另两种:
//   --seed target  用采样构型本身当 seed —— 误差下界≈f32 量化 + PC/固件模型差,
//                  可用来把"求解误差"和"模型差"分开看。
//   --seed prev    用上一个解当 seed (仿轨迹跟踪里 move_p 的真实用法)。
//
// 另做一组刻意不可达的笛卡尔目标 (半径 1.3m, 远超臂展 ~0.78m), 验证固件在
// 不可达时是否按契约回 ok=0 (SDK 侧表现为 IKError) 而不是给出胡说解。
//
// 运行: ./09_ik_accuracy --n 200 [--seed zero|target|prev] [--margin 0.9]
//                        [--dump ik_sweep.csv] [--unreach 5] [--timeout 3.0]
//                        [--bench]   (先跑固件 KIN_BENCH, 拿自报的 IK 迭代数/cycles)
//       [--frame ee|j7] [--j6axis 1|-1]   (强制模型变体; 默认取自检里最像的那个)
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "_common.hpp"

using namespace litearm;

namespace {

// ---------------------------------------------------------------- PC 侧 FK
//
// litearm.urdf 的关节链 (单位 m/rad), base_link 起:
//   j1..j7 皆 revolute, 轴默认 +Z; joint6 例外, URDF 记 (0,0,-1) (CSV 记 +1, 冲突)
// 下面数值原样抄自 URDF (1.5708/3.1416 这类近似值别"顺手改精确", 保原样)。
struct Origin {
    rot::Vec3 xyz;
    rot::Vec3 rpy;
};

const Origin kJoints[7] = {
    {{0.0, 0.0, 0.06975}, {0.0, 0.0, 0.0}},          // j1 <- base_link
    {{0.0, 0.0, 0.0635}, {1.5708, 0.0, -1.5708}},    // j2 <- j1
    {{0.0, 0.1135, 0.0}, {1.5708, 0.0, 3.1416}},     // j3 <- j2
    {{0.038, 0.0, 0.186}, {1.5708, 0.0, 0.0}},       // j4 <- j3
    {{-0.038, 0.08545, 0.0}, {1.5708, 0.0, 3.1416}}, // j5 <- j4
    {{0.0, 0.0, 0.13705}, {1.5708, 0.0, 0.0}},       // j6 <- j5
    {{0.0, 0.074, 0.0}, {1.5708, 0.0, 1.5708}},      // j7 <- j6
};
/// joint7 -> ee_link 的固定变换 (末端封盖, 0.05375m 前伸 + Ry(90))。
const Origin kEe{{0.05375, 0.0, 0.0}, {0.0, 1.5708, 0.0}};

struct Xform {
    rot::Mat3 R;
    rot::Vec3 p;
};

Xform compose(const Xform& A, const Xform& B) {
    Xform C;
    C.R = rot::mat_mul(A.R, B.R);
    const rot::Vec3 pb = rot::mat_vec(A.R, B.p);
    C.p = {A.p[0] + pb[0], A.p[1] + pb[1], A.p[2] + pb[2]};
    return C;
}

Xform origin_frame(const Origin& o) { return {rot::rpy_to_mat(o.rpy), o.xyz}; }

/// Rz(q) (关节轴皆为 Z 系; j6 的符号由 j6sign 给)。
Xform rot_z(double q) {
    const double c = std::cos(q), s = std::sin(q);
    Xform T;
    T.R[0] = {c, -s, 0.0};
    T.R[1] = {s, c, 0.0};
    T.R[2] = {0.0, 0.0, 1.0};
    T.p = {0.0, 0.0, 0.0};
    return T;
}

/// FK: q[7] -> 末端帧 (to_ee=false 时止于 joint7 法兰)。
Xform fk7(const std::vector<double>& q, bool to_ee, double j6sign) {
    Xform T{rot::eye3(), {0.0, 0.0, 0.0}};
    for (int i = 0; i < 7; ++i) {
        T = compose(T, origin_frame(kJoints[i]));
        T = compose(T, rot_z(i == 5 ? j6sign * q[size_t(i)] : q[size_t(i)]));
    }
    if (to_ee) T = compose(T, origin_frame(kEe));
    return T;
}

// ---------------------------------------------------------------- Halton
double halton(int idx, int base) {
    double f = 1.0, r = 0.0;
    while (idx > 0) {
        f /= base;
        r += f * (idx % base);
        idx /= base;
    }
    return r;
}
constexpr int kHaltonBases[7] = {2, 3, 5, 7, 11, 13, 17};

// ---------------------------------------------------------------- 统计小件
struct Stats {
    std::vector<double> v;
    void add(double x) { v.push_back(x); }
    double pct(double p) const {
        if (v.empty()) return 0.0;
        const size_t i = std::min(v.size() - 1,
                                  size_t(p / 100.0 * double(v.size() - 1) + 0.5));
        return v[i];
    }
    double mean() const {
        double s = 0.0;
        for (double x : v) s += x;
        return v.empty() ? 0.0 : s / double(v.size());
    }
    double max() const { return v.empty() ? 0.0 : v.back(); }
    void sort() { std::sort(v.begin(), v.end()); }
    void line(const char* name, const char* unit) const {
        if (v.empty()) return;
        std::printf("  %-10s %-4s: p50=%.4g  p90=%.4g  p99=%.4g  max=%.4g  mean=%.4g\n",
                    name, unit, pct(50), pct(90), pct(99), max(), mean());
    }
};

void histogram(const char* name, const std::vector<double>& v,
               const std::vector<double>& edges, const char* unit) {
    if (v.empty()) return;
    std::vector<int> bins(edges.size() + 1, 0);
    for (double x : v) {
        size_t b = 0;
        while (b < edges.size() && x > edges[b]) ++b;
        bins[b]++;
    }
    std::printf("  %s 分布 (%s):\n", name, unit);
    for (size_t b = 0; b < bins.size(); ++b) {
        const std::string lo =
            b == 0 ? std::string("0") : std::to_string(edges[b - 1]).substr(0, 6);
        const std::string hi =
            b == edges.size() ? std::string("inf") : std::to_string(edges[b]).substr(0, 6);
        std::printf("    [%-6s, %-6s] %5d  %s\n", lo.c_str(), hi.c_str(), bins[b],
                    std::string(std::min<size_t>(bins[b] * 60 / std::max<size_t>(1, v.size()),
                                                 60), '#').c_str());
    }
}

struct Row {
    std::vector<double> q_t, q_ik;
    rot::Vec3 pos_t{};
    double pos_mm = 0.0, ori_deg = 0.0, jdev_rad = 0.0, ms = 0.0;
    bool ok = false;
};

}  // namespace

int main(int argc, char** argv) {
    return example::run_example(
        argc, argv, "样例 09 · 固件 IK 精度扫描 (只读)",
        [](Arm& arm, const example::Args& args) -> int {
            if (arm.n() != 7) {
                std::printf("需要整臂 (7J) 才能扫全工作空间; 当前 n=%d\n", arm.n());
                return 1;
            }
            const auto st = arm.get_state().value;
            const auto tcp = arm.get_tcp().value;
            if (!st || !tcp) {
                std::printf("取不到状态或 TCP\n");
                return 1;
            }
            const std::vector<double> q_now = st->q();

            // ---- 软限位 (0x24 逐关节读回; 只读) ----
            // ⚠ SDK 的关节参数是 **0 基** (get_joint_param(0..n-1)), 别按 1..7 映射。
            std::vector<double> qmin(7), qmax(7);
            const auto jps = arm.params().all_joint_params();
            std::printf("软限位 (固件读回):\n");
            for (size_t i = 0; i < jps.size() && i < 7; ++i) {
                qmin[i] = jps[i].q_min;
                qmax[i] = jps[i].q_max;
                std::printf("  j%zu (idx=%d): [%+.4f, %+.4f]\n", i + 1, jps[i].idx,
                            jps[i].q_min, jps[i].q_max);
            }
            if (jps.size() != 7) {
                std::printf("关节参数只读回 %zu 个, 期望 7\n", jps.size());
                return 1;
            }
            for (int i = 0; i < 7; ++i) {
                if (!(qmin[size_t(i)] < qmax[size_t(i)])) {
                    std::printf("关节 %d 软限位异常: [%g, %g]\n", i + 1, qmin[size_t(i)],
                                qmax[size_t(i)]);
                    return 1;
                }
            }
            std::printf("\n");

            // ---- 0) 模型自检: 4 个变体各算一遍当前位姿 ----
            const rot::Vec3 tcp_p{tcp->at(0), tcp->at(1), tcp->at(2)};
            const rot::Mat3 tcp_R = rot::rpy_to_mat({tcp->at(3), tcp->at(4), tcp->at(5)});
            std::printf("固件: %s\n", arm.firmware().c_str());
            std::printf("当前 q   = %s\n", example::fmt(q_now).c_str());
            std::printf("固件 TCP = %s\n\n", example::fmt(*tcp, 4).c_str());
            std::printf("模型自检 (FK_pc(当前q) vs 固件 TCP):\n");

            struct Variant {
                bool to_ee;
                double j6sign;
                const char* name;
            };
            const Variant variants[4] = {
                {true, -1.0, "ee / j6轴-1 (URDF)"},
                {true, +1.0, "ee / j6轴+1 (CSV)"},
                {false, -1.0, "j7 / j6轴-1"},
                {false, +1.0, "j7 / j6轴+1"},
            };
            // 用户可强制某一变体, 否则取位置偏差最小的那个。
            bool force_frame = args.has("frame");
            bool want_ee = force_frame ? args.str("frame", "ee") == "ee" : true;
            double forced_j6 = args.has("j6axis") ? args.num("j6axis", -1.0) : 0.0;
            int best = -1;
            double best_pos = 1e9;
            for (int v = 0; v < 4; ++v) {
                const Xform F = fk7(q_now, variants[v].to_ee, variants[v].j6sign);
                const auto e = rot::pose_err(F.p, F.R, tcp_p, tcp_R);
                std::printf("  %-22s pos=%.4f mm  orient=%.4f deg\n", variants[v].name,
                            e.first * 1000.0, e.second * 180.0 / M_PI);
                if (e.first < best_pos) {
                    best_pos = e.first;
                    best = v;
                }
            }
            bool to_ee = variants[best].to_ee;
            double j6sign = variants[best].j6sign;
            if (force_frame) to_ee = want_ee;
            if (forced_j6 != 0.0) j6sign = forced_j6 > 0 ? 1.0 : -1.0;
            std::printf("  => 采用: %s (pos 偏差最小)\n", to_ee ? "ee" : "j7");
            std::printf("     j6 轴取 %+.0f; 当前 q6=%.4f rad, |q6| 太小时两个符号分不开\n",
                        j6sign, q_now[5]);

            // ---- 基线: 普通查询 (get_tcp) 的往返耗时, 用来给 IK 耗时定标 ----
            Stats base_ms;
            for (int i = 0; i < 10; ++i) {
                const auto t0 = std::chrono::steady_clock::now();
                (void)arm.get_tcp();
                const auto t1 = std::chrono::steady_clock::now();
                base_ms.add(std::chrono::duration<double, std::milli>(t1 - t0).count());
            }
            base_ms.sort();
            std::printf("基线 get_tcp 往返: p50=%.2f ms (IK 耗时的对照)\n\n", base_ms.pct(50));

            // ---- 可选: 固件自报运动学开销 (KIN_BENCH 0x49; 只读, 不运动) ----
            // 回执里 IK 是 (n_try, min_cyc, max_cyc) —— 固件自己的迭代数与单次迭代
            // cycles, 与下面实测的往返耗时对账用。LOOP1 = 控制拍周期 (cycles)。
            if (args.has("bench")) {
                const auto kb = arm.diag().kin_bench();
                std::printf("固件 KIN_BENCH (原文):\n%s\n", kb.value.raw.c_str());
                for (const auto& kv : kb.value.timings) {
                    std::printf("  %-6s:", kv.first.c_str());
                    for (long long x : kv.second) std::printf(" %lld", x);
                    std::printf("\n");
                }
                std::printf("  loop_max_kcycle=%lld overruns=%lld crc_errors=%lld "
                            "reply_dropped=%lld can_tx_fail=%lld\n\n",
                            kb.value.loop_max_kcycle(), kb.value.loop_overruns(),
                            kb.value.crc_errors(), kb.value.reply_dropped(),
                            kb.value.can_tx_fail());
            }

            // ---- 1) 采样 + 逐个 IK ----
            const int N = int(args.num("n", 200));
            const double margin = args.num("margin", 1.0);
            const std::string seed_mode = args.str("seed", "zero");
            const double timeout = args.num("timeout", 3.0);
            std::vector<double> qseed(size_t(7), 0.0);  // "zero"
            const bool seed_prev = seed_mode == "prev";

            std::vector<Row> rows;
            rows.reserve(size_t(N));
            Stats pos_mm, ori_deg, ms, jdev;
            int n_fail = 0, n_timeout = 0, n_other_branch = 0, n_out_of_limit = 0;
            double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9, zmin = 1e9,
                   zmax = -1e9, rmin = 1e9, rmax = -1e9;
            const auto t_sweep0 = std::chrono::steady_clock::now();

            std::printf("扫描 %d 个位姿 (Halton, margin=%.2f, seed=%s)...\n", N, margin,
                        seed_mode.c_str());
            for (int k = 1; k <= N; ++k) {
                std::vector<double> q_t(size_t(7));
                for (int i = 0; i < 7; ++i) {
                    const double mid = 0.5 * (qmin[size_t(i)] + qmax[size_t(i)]);
                    const double half = 0.5 * (qmax[size_t(i)] - qmin[size_t(i)]);
                    const double u = halton(k, kHaltonBases[i]) - 0.5;
                    q_t[size_t(i)] = mid + u * margin * 2.0 * half;
                }
                const Xform F_t = fk7(q_t, to_ee, j6sign);
                const std::array<double, 6> pose_t = rot::as_pose6(F_t.p, F_t.R);

                if (seed_mode == "target") qseed = q_t;
                const double t0 = std::chrono::duration<double>(
                                      std::chrono::steady_clock::now().time_since_epoch())
                                      .count();
                Row row;
                row.q_t = q_t;
                row.pos_t = F_t.p;
                try {
                    row.q_ik = arm.ik(pose_t, qseed, timeout);
                    row.ok = true;
                } catch (const IKError&) {
                    row.ok = false;
                    ++n_fail;
                } catch (const TransportError&) {
                    row.ok = false;
                    ++n_timeout;
                }
                const double t1 = std::chrono::duration<double>(
                                      std::chrono::steady_clock::now().time_since_epoch())
                                      .count();
                row.ms = (t1 - t0) * 1000.0;

                if (row.ok) {
                    const Xform F_i = fk7(row.q_ik, to_ee, j6sign);
                    const auto e = rot::pose_err(F_i.p, F_i.R, F_t.p, F_t.R);
                    row.pos_mm = e.first * 1000.0;
                    row.ori_deg = e.second * 180.0 / M_PI;
                    double jd = 0.0;
                    for (int i = 0; i < 7; ++i) {
                        jd = std::max(jd, std::fabs(row.q_ik[size_t(i)] - q_t[size_t(i)]));
                    }
                    row.jdev_rad = jd;
                    if (jd > 0.2) ++n_other_branch;  // 换分支(合法), 只是要计数
                    for (int i = 0; i < 7; ++i) {
                        if (row.q_ik[size_t(i)] < qmin[size_t(i)] - 1e-6 ||
                            row.q_ik[size_t(i)] > qmax[size_t(i)] + 1e-6) {
                            ++n_out_of_limit;  // 解出软限位外的构型 (固件没拦)
                            break;
                        }
                    }
                    pos_mm.add(row.pos_mm);
                    ori_deg.add(row.ori_deg);
                    ms.add(row.ms);
                    jdev.add(jd);
                    if (seed_prev) qseed = row.q_ik;
                    xmin = std::min(xmin, F_t.p[0]);
                    xmax = std::max(xmax, F_t.p[0]);
                    ymin = std::min(ymin, F_t.p[1]);
                    ymax = std::max(ymax, F_t.p[1]);
                    zmin = std::min(zmin, F_t.p[2]);
                    zmax = std::max(zmax, F_t.p[2]);
                    const double rr = rot::norm(F_t.p);
                    rmin = std::min(rmin, rr);
                    rmax = std::max(rmax, rr);
                }
                rows.push_back(std::move(row));
                if (k % 25 == 0 || k == N) {
                    std::printf("  %4d/%d  ok=%d fail=%d timeout=%d  max_pos=%.2f mm\r",
                                k, N, int(ms.v.size()), n_fail, n_timeout, pos_mm.max());
                    std::fflush(stdout);
                }
            }
            const double sweep_s =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t_sweep0)
                    .count();
            std::printf("\n");

            // ---- 2) 不可达位姿的拒绝测试 ----
            const int n_unreach = int(args.num("unreach", 5));
            std::printf("\n不可达目标拒绝测试 (半径 1.3m, 臂展 ~0.78m), %d 个:\n", n_unreach);
            int rejected = 0, not_rejected = 0;
            for (int k = 0; k < n_unreach; ++k) {
                const double th = 2.0 * M_PI * halton(k + 1, 2);
                const double zz = 2.0 * halton(k + 1, 3) - 1.0;
                const double rr = std::sqrt(std::max(0.0, 1.0 - zz * zz));
                const std::array<double, 6> bad = {1.3 * rr * std::cos(th),
                                                   1.3 * rr * std::sin(th), 0.2 + 1.3 * zz,
                                                   0.4, -0.6, 1.1};
                const auto t0 = std::chrono::steady_clock::now();
                try {
                    const auto q_bad = arm.ik(bad, std::vector<double>(7, 0.0), timeout);
                    const auto t1 = std::chrono::steady_clock::now();
                    std::printf("  [%d] 未拒绝! q=%s (%.0f ms)\n", k + 1,
                                example::fmt(q_bad).c_str(),
                                std::chrono::duration<double, std::milli>(t1 - t0).count());
                    ++not_rejected;
                } catch (const IKError&) {
                    const auto t1 = std::chrono::steady_clock::now();
                    std::printf("  [%d] 正确拒绝 (%.0f ms)\n", k + 1,
                                std::chrono::duration<double, std::milli>(t1 - t0).count());
                    ++rejected;
                }
            }
            std::printf("  正确拒绝 %d / 未拒绝 %d (期望全拒)\n", rejected, not_rejected);

            // ---- 3) 汇总 ----
            pos_mm.sort();
            ori_deg.sort();
            ms.sort();
            jdev.sort();
            const int n_ok = int(ms.v.size());
            std::printf("\n================ 精度汇总 ================\n");
            std::printf("样本          : %d 个, 成功 %d (%.1f%%), 固件拒绝 %d, 超时 %d\n", N,
                        n_ok, 100.0 * double(n_ok) / double(N), n_fail, n_timeout);
            std::printf("总墙钟        : %.1f s (%.1f ms/样本)\n", sweep_s,
                        sweep_s * 1000.0 / double(N));
            pos_mm.line("位置误差", "mm");
            ori_deg.line("姿态误差", "deg");
            ms.line("往返耗时", "ms");
            jdev.line("关节偏差", "rad");
            std::printf("  关节偏差 >0.2 rad 的解 %d 个 (7 轴冗余的合法换分支)\n",
                        n_other_branch);
            std::printf("  解出软限位外的 %d 个 (固件未拦截)\n", n_out_of_limit);
            std::printf("  工作空间覆盖: |p| %.2f~%.2f m, x[%.2f, %.2f] y[%.2f, %.2f] "
                        "z[%.2f, %.2f]\n\n",
                        rmin, rmax, xmin, xmax, ymin, ymax, zmin, zmax);
            histogram("位置误差", pos_mm.v, {0.01, 0.05, 0.1, 0.5, 1.0, 2.0, 5.0}, "mm");
            histogram("姿态误差", ori_deg.v, {0.01, 0.05, 0.1, 0.5, 1.0, 2.0}, "deg");

            // ---- 4) --dump CSV ----
            const std::string dump = args.str("dump", "");
            if (!dump.empty()) {
                if (FILE* f = std::fopen(dump.c_str(), "w")) {
                    std::fprintf(f, "k,ok,ms,pos_err_mm,ori_err_deg,jdev_rad,tx,ty,tz");
                    for (int i = 1; i <= 7; ++i) std::fprintf(f, ",qt%d", i);
                    for (int i = 1; i <= 7; ++i) std::fprintf(f, ",qik%d", i);
                    std::fprintf(f, "\n");
                    for (size_t r = 0; r < rows.size(); ++r) {
                        const Row& row = rows[r];
                        std::fprintf(f, "%zu,%d,%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", r + 1,
                                     int(row.ok), row.ms, row.pos_mm, row.ori_deg,
                                     row.jdev_rad, row.pos_t[0], row.pos_t[1], row.pos_t[2]);
                        for (double q : row.q_t) std::fprintf(f, ",%.6f", q);
                        if (row.ok) {
                            for (double q : row.q_ik) std::fprintf(f, ",%.6f", q);
                        } else {
                            for (int i = 0; i < 7; ++i) std::fprintf(f, ",");
                        }
                        std::fprintf(f, "\n");
                    }
                    std::fclose(f);
                    std::printf("\n逐样本数据 -> %s\n", dump.c_str());
                } else {
                    std::printf("\n打不开 dump 文件: %s\n", dump.c_str());
                }
            }
            std::printf("\n已断开 (全程只读, 未使能/未运动)\n");
            return 0;
        });
}
