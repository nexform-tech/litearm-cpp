#include "litearm/testing.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <thread>

#include "litearm/errors.hpp"

namespace litearm {
namespace testing {

const std::map<uint8_t, size_t> CART_MIN_LEN = {
    {0x3A, 28}, {0x3B, 52}, {0x3C, 5}, {0x3D, 25}, {0x3E, 0}};

const char* const KIN_BENCH_TEXT =
    "FK200 108 240 \n"
    "JAC200 220 610 \n"
    "IK100 2600 5200 \n"
    "LOOP14000 \n"
    "SC200 60 120 \n"
    "G200 300 700 \n"
    "RNEA200 800 1500 \n"
    "M200 900 2200 \n"
    "LAW200 50 90 \n"
    "LINK crc=3 ovf=11 rfd=1 txf=7 rxlen=5 rxl0=13 rxl1=17 rbd=19 "
    "cmode=127 cmb=23 tfe=1 tfd=2 tfc=3 tfs=0 tfm=1 tec=29 rec=31 psr=8 "
    "ebo=0 eep=0 eew=0 epa=0 epd=0 "
    " loop_k=2 ovr=4  flt=12345 /6 st=1 \n";

std::vector<std::string> split_kin_bench(const std::string& text, bool one_frame) {
    const size_t pos = text.find("LINK");
    if (pos == std::string::npos) return {text};
    const std::string head = text.substr(0, pos);
    const std::string tail = text.substr(pos);
    if (one_frame) return {head};
    return {head, tail};
}

std::vector<uint8_t> make_log(int n_ticks, int n) {
    std::vector<uint8_t> out;
    for (int i = 0; i < n_ticks; ++i) {
        const auto t = proto::pack_u32le(uint32_t(i));
        out.insert(out.end(), t.begin(), t.end());
        std::vector<double> qref, dq, tau;
        for (int j = 0; j < n; ++j) {
            qref.push_back(double(10 * j + 1));
            dq.push_back(0.5);
            tau.push_back(-0.25);
        }
        const auto a = proto::pack_f32s(qref);
        const auto b = proto::pack_f32s(dq);
        const auto c = proto::pack_f32s(tau);
        out.insert(out.end(), a.begin(), a.end());
        out.insert(out.end(), b.begin(), b.end());
        out.insert(out.end(), c.begin(), c.end());
    }
    return out;
}

std::vector<uint8_t> make_status(const std::vector<double>& q,
                                 const std::vector<double>& dq, int mode, uint16_t flags,
                                 uint16_t seq, uint16_t joint_fault, int n) {
    // 按固件 1.5.x 真实布局 (6+21N) 造状态帧 —— 尾部带 joint_fault u16。
    // 旧桩只造 4+21N, 导致"SDK 解不了真固件状态帧"这一 P0 在离线测试里全绿被掩盖。
    std::vector<uint8_t> body;
    const auto hdr = proto::pack_u16le(uint16_t(flags | (mode << 6)));
    body.insert(body.end(), hdr.begin(), hdr.end());
    const auto s = proto::pack_u16le(seq);
    body.insert(body.end(), s.begin(), s.end());
    for (int i = 0; i < n; ++i) {
        const double qv = i < int(q.size()) ? q[size_t(i)] : 0.0;
        const double dv = i < int(dq.size()) ? dq[size_t(i)] : 0.0;
        const auto v = proto::pack_f32s(std::vector<double>{qv, dv, 0.5, 30.0, 25.0});
        body.insert(body.end(), v.begin(), v.end());
        body.push_back(0);
    }
    const auto jf = proto::pack_u16le(joint_fault);
    body.insert(body.end(), jf.begin(), jf.end());
    return proto::pack_frame(proto::RSP_STATUS, body);
}

// The firmware reports ENABLE in status flags bit9. The SDK exposes it as
// RobotState::enabled() and the direct-USB component waits on it before its first MOVE_JS,
// so a double that never sets it turns that wait into a phantom firmware bug.
uint16_t status_flags(bool enabled) {
    return enabled ? uint16_t(1u << proto::FLAG_ENABLED_BIT) : uint16_t(0);
}

FakeTransport::FakeTransport(const std::string& port_in, double timeout_in,
                             const std::string& fw_in, int n_in)
    : port(port_in), timeout(timeout_in), fw(fw_in), n(n_in) {
    q.assign(size_t(n), 0.0);
    pose = {0.300, 0.0, 0.350, 0.0, 0.0, 0.0};
    path_last_pose = pose;
    model_body.assign(size_t(model_nbody),
                      std::vector<double>(10, 0.0));
    for (auto& b : model_body) b[0] = 1.0;      // "编译期常量": 全 1 质量
    model_body[size_t(model_nbody - 1)][0] = 0.0;   // body8 质量恒 0
    model_jm.assign(7, 0.0);
    for (int i = 0; i < n; ++i) {
        joint_params.push_back({{"kp", jp_kp},
                                {"kd", jp_kd},
                                {"tau_max", jp_tau_max},
                                {"q_min", jp_q_min},
                                {"q_max", jp_q_max}});
    }
    license_uid = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
                   0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B};
    kin_bench_text = KIN_BENCH_TEXT;
}

void FakeTransport::close() { closed = true; }

// 读线程会读这几个字段, 故写入必须持 `state_lock` (见 header 里的说明)。
void FakeTransport::set_q(std::vector<double> v) {
    std::lock_guard<std::mutex> sl(state_lock);
    q = std::move(v);
}
void FakeTransport::set_joint_limits(double q_min, double q_max) {
    std::lock_guard<std::mutex> sl(state_lock);
    for (auto& jp : joint_params) {
        jp["q_min"] = q_min;
        jp["q_max"] = q_max;
    }
}
void FakeTransport::set_pose(std::vector<double> v) {
    std::lock_guard<std::mutex> sl(state_lock);
    pose = std::move(v);
}
void FakeTransport::set_cart_busy(int seq) {
    std::lock_guard<std::mutex> sl(state_lock);
    cart_busy_seq = seq;
}
void FakeTransport::clear_cart_busy() {
    std::lock_guard<std::mutex> sl(state_lock);
    cart_busy_seq.reset();
}
bool FakeTransport::is_open() const { return !closed; }

// ---------------------------------------------------------------- FakeClock
std::shared_ptr<FakeClock> FakeClock::make() { return std::make_shared<FakeClock>(); }

double FakeClock::now_s() const {
    std::lock_guard<std::mutex> lk(mu_);
    return now_;
}

void FakeClock::advance(double s) {
    std::lock_guard<std::mutex> lk(mu_);
    now_ += s;
}

std::string FakeTransport::text_log() const {
    std::lock_guard<std::mutex> sl(state_lock);
    return noise_text;
}

void FakeTransport::set_text_log(std::string s) {
    std::lock_guard<std::mutex> sl(state_lock);
    noise_text = std::move(s);
}

void FakeTransport::set_flush_failures(uint64_t n) { flush_failures_.store(n); }

void FakeTransport::push(const std::vector<uint8_t>& frame) { resp_.push_back(frame); }

void FakeTransport::push_frame(uint8_t cmd, const std::vector<uint8_t>& payload) {
    std::lock_guard<std::mutex> sl(state_lock);
    push(proto::pack_frame(cmd, payload));
}

std::vector<std::pair<uint8_t, std::vector<uint8_t>>> FakeTransport::tx_snapshot() const {
    std::lock_guard<std::mutex> lk(tx_lock);
    return tx_log;
}

size_t FakeTransport::tx_count() const {
    std::lock_guard<std::mutex> lk(tx_lock);
    return tx_log.size();
}

std::vector<double> FakeTransport::stamps_of(uint8_t cmd) const {
    std::lock_guard<std::mutex> lk(tx_lock);
    std::vector<double> out;
    for (size_t i = 0; i < tx_log.size() && i < tx_stamps.size(); ++i) {
        if (tx_log[i].first == cmd) out.push_back(tx_stamps[i]);
    }
    return out;
}

int FakeTransport::log_recorded() const {
    if (log_hz <= 0.0 || log_t0 < 0.0) return log_target;
    const int by_time = int((now_s() - log_t0) * log_hz);
    return std::min(log_target, by_time);
}

bool FakeTransport::axis_outside(int idx, double qmin, double qmax) const {
    // 逐条镜像固件 ctrl_axis_outside 的可复现部分 (①②⑤)。
    if (idx < 0 || idx >= n) return true;      // ①
    if (!(qmin < qmax)) return true;           // ②
    return q[idx] < qmin || q[idx] > qmax;     // ⑤ (q_ref 的代理)
}

std::vector<uint8_t> FakeTransport::cart_plan_frame() const {
    if (cart_err_override) {
        // ok=0 时 n_wp / plan_us 一律 0 (固件同款: 失败时那两个数没有意义)
        std::vector<uint8_t> body{0, uint8_t(*cart_err_override)};
        const auto a = proto::pack_u16le(0);
        const auto b = proto::pack_u32le(0);
        body.insert(body.end(), a.begin(), a.end());
        body.insert(body.end(), b.begin(), b.end());
        return proto::pack_frame(proto::RSP_CART_PLAN, body);
    }
    std::vector<uint8_t> body{1, 0};
    const auto a = proto::pack_u16le(uint16_t(cart_n_wp));
    const auto b = proto::pack_u32le(cart_plan_us);
    body.insert(body.end(), a.begin(), a.end());
    body.insert(body.end(), b.begin(), b.end());
    return proto::pack_frame(proto::RSP_CART_PLAN, body);
}

std::vector<double> FakeTransport::cart_goal(uint8_t cmd,
                                             const std::vector<uint8_t>& payload) const {
    // 规划成功时桩把 pose 直接置成该命令的目标: 到位之后回读的 TCP 必须与目标对得上
    // (Arm::pose_near 那条判据), 否则每条 wait=true 的用例都会拿到 settled=false。
    // 桩不建模运动过程 (没有中间位姿), 只保证"收尾时 TCP 在目标上"。
    if (cmd == proto::CMD_MOVE_L) {
        return proto::unpack_f32s(payload.data(), 0, 6);
    }
    if (cmd == proto::CMD_MOVE_C) {
        return proto::unpack_f32s(payload.data(), 24, 6);
    }
    return path_last_pose;   // 0x3E RUN
}

void FakeTransport::cart_cmd(uint8_t cmd, const std::vector<uint8_t>& payload) {
    // 桩不建模使能/零重力/掉线锁存三道门禁: 要在离线用例里造"被门禁拒", 用 err_override 注入。
    if (!cart_supported) {
        // 固件关掉 LITEARM_CART_PLAN 时这 5 条整段不存在 -> default 分支
        push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x00}));
        return;
    }
    const auto it = CART_MIN_LEN.find(cmd);
    if (it != CART_MIN_LEN.end() && payload.size() < it->second) {
        push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x01}));
        return;
    }
    if (cmd == proto::CMD_CART_ADD) {
        path_last_pose = proto::unpack_f32s(payload.data(), 1, 6);
    }
    push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
    if (cmd == proto::CMD_MOVE_L || cmd == proto::CMD_MOVE_C || cmd == proto::CMD_CART_RUN) {
        if (!cart_err_override) {
            pose = cart_goal(cmd, payload);
        }
        cart_busy_seq = 0;   // 状态帧开始按写死的帧序带 bit10
        push(cart_plan_frame());
    }
}

void FakeTransport::write_frame(uint8_t cmd, const uint8_t* payload, size_t len) {
    const std::vector<uint8_t> p(payload, payload + len);
    if (dfu_gone) {
        // 设备已进 ROM bootloader: 帧发不出去 (不记账 —— 现实里它根本没进 CDC)
        throw TransportError("桩: 设备已进 ROM bootloader, CDC 上的写路径失败");
    }
    {
        std::lock_guard<std::mutex> lk(tx_lock);
        tx_log.emplace_back(cmd, p);
        tx_stamps.push_back(now_s());
    }
    // 注入的"写阻塞"要**先睡再取锁** —— 持着 state_lock 睡会把读线程一起卡住。
    if (cmd == proto::CMD_ZERO_G && !p.empty() && p[0] == 0x01) {
        zg_writes.fetch_add(1);
        if (zg_slow_write_s > 0.0 && zg_writes.load() > 1) {
            std::this_thread::sleep_for(std::chrono::duration<double>(zg_slow_write_s));
        }
    }
    // 本函数会改共享状态 (`resp_` / `q` / `pose` / 各表), 而读线程同时在 read_frame
    // => 必须整体互斥 (见 `state_lock` 的说明)。
    std::lock_guard<std::mutex> sl(state_lock);
    if (unknown_cmds.count(cmd)) {
        // 固件 usb_cmd.c 的 default 分支: 未实现命令只回 ERR{cmd,0x00}, 无任何副作用
        push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x00}));
        return;
    }
    const auto eo = err_override.find(cmd);
    if (eo != err_override.end()) {
        push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, eo->second}));
        return;
    }

    if (cmd == proto::CMD_ZERO_G) {
        if (zg_fail_after >= 0 && zg_writes.load() > zg_fail_after) {
            throw TransportError("桩: CDC 写失败 (注入)");
        }
        push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
    } else if (cmd == proto::CMD_GET_FIRMWARE) {   // 握手
        const std::vector<uint8_t> b(fw.begin(), fw.end());
        push(proto::pack_frame(proto::RSP_FIRMWARE, b));
        auto_status = true;
        push(make_status(q, {}, 0, status_flags(enabled), 0, 0, n));
    } else if (cmd == proto::CMD_GET_STATUS) {
        push(make_status(q, {}, 1, status_flags(enabled), 0, 0, n));
    } else if (cmd == proto::CMD_SET_FF_VEC) {     // 存下来供 0x2B 读回
        if (!p.empty()) ff_vec[int(p[0])] = proto::unpack_f32s(p.data(), 1, n);
        push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
    } else if (cmd == proto::CMD_SET_FF_SCALAR) {  // 存下来供 0x2C 读回
        if (p.size() >= 6) {
            const auto v = proto::unpack_f32s(p.data(), 2, 1);
            ff_scalar[{int(p[0]), int(p[1])}] = v[0];
        }
        push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
    } else if (cmd == proto::CMD_SET_FF_FLAGS) {
        ff_mask = p.size() >= 4 ? proto::read_u32le(p, 0) : 0;
        push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
    } else if (cmd == proto::CMD_GET_FF_VEC) {     // 载荷首字节 = RSP id
        const int item = p.empty() ? 0 : int(p[0]);
        std::vector<double> vals;
        const auto it = ff_vec.find(item);
        if (it != ff_vec.end()) {
            vals = it->second;
        } else {
            vals.assign(size_t(n), 0.0);
        }
        std::vector<uint8_t> body{proto::RSP_FF_VEC, uint8_t(item)};
        const auto f = proto::pack_f32s(vals);
        body.insert(body.end(), f.begin(), f.end());
        push(proto::pack_frame(proto::RSP_FF_VEC, body));
    } else if (cmd == proto::CMD_GET_FF_SCALAR) {  // item 9 = ff_mask (只读)
        const int item = p.size() > 0 ? int(p[0]) : 0;
        const int sub = p.size() > 1 ? int(p[1]) : 0;
        double val = 0.0;
        if (item == 9) {
            val = double(ff_mask);
        } else {
            const auto it = ff_scalar.find({item, sub});
            if (it != ff_scalar.end()) val = it->second;
        }
        std::vector<uint8_t> body{proto::RSP_FF_SCALAR, uint8_t(item), uint8_t(sub)};
        const auto f = proto::pack_f32le(val);
        body.insert(body.end(), f.begin(), f.end());
        push(proto::pack_frame(proto::RSP_FF_SCALAR, body));
    } else if (cmd == proto::CMD_ENABLE) {         // 跟踪使能态 (0x36 门禁用)
        if (!activated) {
            // 照固件: 未激活是 ctrl_enable 的第一条判据 (优先于 EMERGENCY/joint_fault),
            // 重发无用、无旁路。
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x08}));
        } else {
            enabled = true;
            push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
        }
    } else if (cmd == proto::CMD_GET_LICENSE) {    // 0x2F -> RSP_LICENSE(0x4F) 26B
        // 请求不校验长度 (有意放宽); 未激活也回 UID, 而 cust_id/issued/flags 保持 0;
        // 载荷首字节是 state, 不是冗余帧 id。
        int st = 0;
        if (activated) st = (license_flags & 0x1) ? 2 : 1;
        std::vector<uint8_t> body{uint8_t(st), uint8_t(license_ver)};
        body.insert(body.end(), license_uid.begin(), license_uid.end());
        const uint32_t cid = activated ? license_cust_id : 0;
        const uint32_t iss = activated ? license_issued : 0;
        const uint32_t flg = activated ? license_flags : 0;
        const auto a = proto::pack_u32le(cid);
        const auto b = proto::pack_u32le(iss);
        const auto c = proto::pack_u32le(flg);
        body.insert(body.end(), a.begin(), a.end());
        body.insert(body.end(), b.begin(), b.end());
        body.insert(body.end(), c.begin(), c.end());
        push(proto::pack_frame(proto::RSP_LICENSE, body));
    } else if (cmd == proto::CMD_ACTIVATE) {       // 0x3F: 28B -> ACK / ERR
        if (p.size() < 28) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x01}));
        } else if (armed()) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x04}));
        } else if (license_fail_next || activated) {
            // 0x02 是聚合档 —— "已经激活过"与"MAC 不符/写失败"混在一起。
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x02}));
        } else {
            license_cust_id = proto::read_u32le(p, 0);
            license_issued = proto::read_u32le(p, 4);
            license_flags = proto::read_u32le(p, 8);
            activated = true;
            push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
        }
    } else if (cmd == proto::CMD_ENTER_DFU) {
        // 长度优先, 再两道门禁 (使能 / ROM 向量表), 最后只登记。
        // 桩不模拟跳转本身; 它模拟主机看得见的那一半: ACK 之后设备是否还在 CDC 上。
        if (!p.empty()) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x01}));
        } else if (armed()) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x03}));
        } else if (dfu_rom_table_invalid) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x02}));
        } else {
            push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
            for (const auto& extra : dfu_post_ack_frames) {
                push(proto::pack_frame(extra.first, extra.second));
            }
            if (dfu_vanishes) dfu_gone = true;
        }
    } else if (cmd == proto::CMD_DISABLE) {
        enabled = false;
        push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
    } else if (cmd == proto::CMD_EMERGENCY_STOP || cmd == proto::CMD_CLEAR_FAULTS ||
               cmd == proto::CMD_RESET || cmd == proto::CMD_SET_SPEED_PERCENT ||
               cmd == proto::CMD_SET_MOTION_MODE || cmd == proto::CMD_MOVE_JS ||
               cmd == proto::CMD_MOVE_MIT || cmd == proto::CMD_MOVE_MIT_ALL ||
               cmd == proto::CMD_FF_PRESET) {
        push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
    } else if (cmd == proto::CMD_SET_JOINT_PARAM) {   // 0x22: idx + kp,kd,tau_max
        if (p.size() < 13) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x01}));
        } else if (int(p[0]) >= n) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x02}));
        } else {
            const auto v = proto::unpack_f32s(p.data(), 1, 3);
            auto& jp = joint_params[size_t(p[0])];
            jp["kp"] = v[0];
            jp["kd"] = v[1];
            jp["tau_max"] = v[2];
            push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
        }
    } else if (cmd == proto::CMD_SET_JOINT_LIMITS) {  // 0x23: idx + q_min,q_max
        // 分支顺序逐条照固件: 长度(0x01) -> 门禁(0x03) -> idx/空区间(0x02)。
        if (p.size() < 9) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x01}));
        } else {
            const int idx = int(p[0]);
            const auto v = proto::unpack_f32s(p.data(), 1, 2);
            if (armed() && axis_outside(idx, v[0], v[1])) {
                // 武装中该轴的当前参考必须落在新区间内。
                push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x03}));
            } else if (idx >= n || !(v[0] < v[1])) {
                push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x02}));
            } else {
                auto& jp = joint_params[size_t(idx)];
                jp["q_min"] = v[0];
                jp["q_max"] = v[1];
                push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
            }
        }
    } else if (cmd == proto::CMD_GET_JOINT_PARAM) {   // 0x24 -> RSP_JOINT_PARAM(0x49)
        if (p.size() < 1) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x01}));
        } else if (int(p[0]) >= n) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x02}));
        } else {
            const auto& jp = joint_params[size_t(p[0])];
            std::vector<uint8_t> body{uint8_t(p[0])};
            const auto v = proto::pack_f32s(std::vector<double>{
                jp.at("kp"), jp.at("kd"), jp.at("tau_max"), jp.at("q_min"),
                jp.at("q_max")});
            body.insert(body.end(), v.begin(), v.end());
            push(proto::pack_frame(proto::RSP_JOINT_PARAM, body));
        }
    } else if (cmd == proto::CMD_PARAM_SAVE) {       // 0x25: 须失能
        if (armed()) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x04}));
        } else {
            push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
        }
    } else if (cmd == proto::CMD_PARAM_RESET) {      // 0x36: 须失能
        if (armed()) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x04}));
        } else {
            for (auto& jp : joint_params) {
                jp = {{"kp", jp_kp}, {"kd", jp_kd}, {"tau_max", jp_tau_max},
                      {"q_min", jp_q_min}, {"q_max", jp_q_max}};
            }
            push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
        }
    } else if (cmd == proto::CMD_SET_MODEL_PARAM) {  // 0x30: body_idx + f32[10] -> staging
        if (p.size() < 1 + 4 * 10) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x01}));
        } else if (int(p[0]) >= model_nbody) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x02}));
        } else {
            const auto vals = proto::unpack_f32s(p.data(), 1, 10);
            bool bad = false;
            for (double v : vals) {
                if (std::isnan(v) || std::fabs(v) > 1e6) bad = true;
            }
            if (bad) {
                push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x02}));
            } else if (int(p[0]) == model_nbody - 1 && vals[0] != 0.0) {
                push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x02}));
            } else {
                model_stage_body[int(p[0])] = vals;
                model_dirty = true;
                push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
            }
        }
    } else if (cmd == proto::CMD_SET_MODEL_JM) {     // 0x33: f32[7] -> staging
        if (p.size() < 4 * 7) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x01}));
        } else {
            const auto vals = proto::unpack_f32s(p.data(), 0, 7);
            bool bad = false;
            for (double v : vals) {
                if (std::isnan(v) || std::fabs(v) > 1e6) bad = true;
            }
            if (bad) {
                push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x02}));
            } else {
                model_stage_jm = vals;
                model_dirty = true;
                push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
            }
        }
    } else if (cmd == proto::CMD_MODEL_COMMIT) {     // 0x32: u16 mask LE
        if (p.size() < 2) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x01}));
        } else if (armed()) {
            // 与固件同序: 门控优先于掩码检查
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x04}));
        } else {
            const uint16_t mask = proto::read_u16le(p, 0);
            uint16_t staged = 0;
            for (const auto& kv : model_stage_body) staged |= uint16_t(1u << kv.first);
            if (model_stage_jm) staged |= uint16_t(1u << 9);
            if (mask != staged) {
                // 掩码不符 -> 0x07 (与"数值非法"分开的码)
                push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x07}));
            } else if (mask & uint16_t(~0x3FF)) {
                push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x02}));
            } else {
                for (const auto& kv : model_stage_body) {
                    model_body[size_t(kv.first)] = kv.second;
                }
                if (model_stage_jm) model_jm = *model_stage_jm;
                model_override = true;
                model_stage_body.clear();
                model_stage_jm.reset();
                push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
            }
        }
    } else if (cmd == proto::CMD_GET_MODEL_PARAM) {  // 0x34 -> RSP_MODEL_PARAM(0x54)
        if (p.size() < 1) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x01}));
        } else if (int(p[0]) >= model_nbody) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x02}));
        } else {
            const int idx = int(p[0]);
            const int echo = model_body_echo_idx >= 0 ? model_body_echo_idx : idx;
            std::vector<uint8_t> body{proto::RSP_MODEL_PARAM, uint8_t(echo)};
            const auto v = proto::pack_f32s(model_body[size_t(idx)]);
            body.insert(body.end(), v.begin(), v.end());
            push(proto::pack_frame(proto::RSP_MODEL_PARAM, body));
        }
    } else if (cmd == proto::CMD_GET_MODEL_JM) {     // 0x35 -> RSP_MODEL_JM(0x55)
        std::vector<uint8_t> body{proto::RSP_MODEL_JM};
        const auto v = proto::pack_f32s(model_jm);
        body.insert(body.end(), v.begin(), v.end());
        push(proto::pack_frame(proto::RSP_MODEL_JM, body));
    } else if (cmd == proto::CMD_REVERT_MODEL) {     // 0x37: 只回退模型
        if (armed()) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x04}));
        } else {
            model_override = false;
            model_stage_body.clear();
            model_stage_jm.reset();
            model_dirty = true;
            push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
        }
    } else if (cmd == proto::CMD_GET_MODEL_STATUS) {  // 0x38 -> RSP_MODEL_STATUS(0x56)
        uint16_t staged = 0;
        for (const auto& kv : model_stage_body) staged |= uint16_t(1u << kv.first);
        if (model_stage_jm) staged |= uint16_t(1u << 9);
        std::vector<uint8_t> body{proto::RSP_MODEL_STATUS,
                                  uint8_t(model_override ? 1 : 0)};
        const auto m = proto::pack_u16le(staged);
        body.insert(body.end(), m.begin(), m.end());
        body.push_back(uint8_t(model_dirty ? 1 : 0));
        push(proto::pack_frame(proto::RSP_MODEL_STATUS, body));
    } else if (cmd == proto::CMD_GET_GRAVITY) {       // 0x39 -> RSP_GRAVITY(0x57)
        if (p.size() < 4 * 7) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x01}));
        } else {
            // 假固件: 回一个确定性可预测的 G, 只为断言"帧格式对、值能被 SDK 解出来"。
            const auto qv = proto::unpack_f32s(p.data(), 0, 7);
            double msum = 0.0;
            for (size_t i = 1; i < model_body.size(); ++i) msum += model_body[i][0];
            std::vector<double> g(7, 0.0);
            for (int i = 0; i < 7; ++i) g[size_t(i)] = msum * 9.81 * 0.001 * (i + 1);
            g[0] = msum * 9.81 * qv[1] * 0.1;
            std::vector<uint8_t> body{proto::RSP_GRAVITY};
            const auto f = proto::pack_f32s(g);
            body.insert(body.end(), f.begin(), f.end());
            push(proto::pack_frame(proto::RSP_GRAVITY, body));
        }
    } else if (cmd == proto::CMD_KIN_BENCH) {          // 0x49 -> RSP_KIN_BENCH(0x4A)
        // 真机是连续两帧, 不是一帧: 单帧载荷上限 255B 而全文约 306B, 原单帧会在 253B 处
        // 截断, 截断点恰好落在 LINK 行的 loop_k= 之后 => 链路计数全丢。
        for (const auto& part : split_kin_bench(kin_bench_text, kin_bench_one_frame)) {
            const std::vector<uint8_t> b(part.begin(), part.end());
            push(proto::pack_frame(proto::RSP_KIN_BENCH, b));
        }
    } else if (cmd == proto::CMD_LOG_CTRL) {           // 0x2D: u32 n_ticks (0=停/清)
        if (p.size() < 4) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x01}));
        } else {
            const uint32_t nv = proto::read_u32le(p, 0);
            if (int(nv) > log_max) {
                push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x02}));
            } else {
                log_target = int(nv);
                log_active = nv != 0;
                if (log_hz > 0.0 && nv) {
                    log_t0 = now_s();   // 逐拍记录: 此刻还没有数据
                    log_bytes.clear();
                    log_recorded_ticks = 0;
                } else {
                    log_t0 = -1.0;
                    log_bytes = nv ? make_log(int(nv), n) : std::vector<uint8_t>{};
                    log_recorded_ticks = int(nv);
                }
                push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
            }
        }
    } else if (cmd == proto::CMD_LOG_READ) {           // 0x2E: u32 offset -> RSP_LOG_DATA
        if (log_reply_short) {
            push(proto::pack_frame(proto::RSP_LOG_DATA, std::vector<uint8_t>{1, 2}));
        } else if (p.size() < 4) {
            push(proto::pack_frame(proto::RSP_ERR, std::vector<uint8_t>{cmd, 0x01}));
        } else if (log_read_fail_once) {
            log_read_fail_once = false;   // 掉帧: 什么都不回
        } else {
            log_read_calls += 1;
            const uint32_t off = proto::read_u32le(p, 0);
            if (log_cursor_stuck) {
                const uint32_t nxt = off == 0 ? 245u : off;
                const uint32_t cnt = off == 0 ? 245u : 0u;
                std::vector<uint8_t> body = proto::pack_u32le(999999);
                const auto b2 = proto::pack_u32le(nxt);
                body.insert(body.end(), b2.begin(), b2.end());
                body.push_back(uint8_t(cnt));
                body.insert(body.end(), cnt, 0);
                push(proto::pack_frame(proto::RSP_LOG_DATA, body));
                return;
            }
            if (log_hz > 0.0 && log_t0 >= 0.0) {
                // 逐拍记录: 按经过的时间推进 (与本次读回次数无关)。
                const int cur = log_recorded();
                if (cur != log_recorded_ticks) {
                    log_recorded_ticks = cur;
                    log_bytes = make_log(cur, n);
                }
            }
            const uint32_t total = uint32_t(log_bytes.size());
            uint32_t cnt = 0;
            if (off < total) {
                cnt = uint32_t(std::min<size_t>(total - off, 245));
            }
            const uint32_t nxt = (off + cnt) < total ? (off + cnt) : 0u;
            std::vector<uint8_t> body = proto::pack_u32le(total);
            const auto b2 = proto::pack_u32le(nxt);
            body.insert(body.end(), b2.begin(), b2.end());
            body.push_back(uint8_t(cnt));
            body.insert(body.end(), log_bytes.begin() + off, log_bytes.begin() + off + cnt);
            push(proto::pack_frame(proto::RSP_LOG_DATA, body));
        }
    } else if (cmd == proto::CMD_HOME) {               // 固件: 各轴低速回 URDF 零位
        push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
        q.assign(size_t(n), 0.0);
        for (int i = 0; i < 3; ++i) {
            push(make_status(q, std::vector<double>(size_t(n), 1.0), 1, status_flags(enabled), 0, 0, n));
        }
        for (int i = 0; i < 6; ++i) {
            push(make_status(q, std::vector<double>(size_t(n), 0.0), 1, status_flags(enabled), 0, 0, n));
        }
    } else if (cmd == proto::CMD_MOVE_J || cmd == proto::CMD_MOVE_J_SYNC) {
        // 0x07 的载荷与 0x01 逐字节相同, 语义差异只在轨迹形状 (同步 vs 每轴独立 S 曲线),
        // 桩不建模轨迹形状, 故两者走同一条分支。
        // ⚠ Python 侧的桩**没有** 0x07 分支 (落到 else 只回 ACK), 于是 movej_sync 到不了位;
        // 那是它自己注释里记下的"代理失真"局限 —— C++ 侧直接补上, 不是行为分歧。
        push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
        q = proto::unpack_f32s(p.data(), 0, n);
        // 先 3 拍"运动中"(dq=1) 再 6 拍到位(dq=0) —— 检验 Arm 到位等待
        for (int i = 0; i < 3; ++i) {
            push(make_status(q, std::vector<double>(size_t(n), 1.0), 1, status_flags(enabled), 0, 0, n));
        }
        for (int i = 0; i < 6; ++i) {
            push(make_status(q, std::vector<double>(size_t(n), 0.0), 1, status_flags(enabled), 0, 0, n));
        }
    } else if (cmd == proto::CMD_MOVE_P) {
        push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
        pose = proto::unpack_f32s(p.data(), 0, 6);
        push(make_status(q, std::vector<double>(size_t(n), 0.0), 1, status_flags(enabled), 0, 0, n));
    } else if (cmd == proto::CMD_GET_TCP) {
        if (tcp_payload_override) {
            push(proto::pack_frame(proto::RSP_TCP, *tcp_payload_override));
            return;
        }
        std::vector<double> out = pose;
        if (pos_override && pos_override->size() >= 3) {
            for (int i = 0; i < 3; ++i) out[size_t(i)] = (*pos_override)[size_t(i)];
        }
        if (rpy_override && rpy_override->size() >= 3) {
            for (int i = 0; i < 3; ++i) out[size_t(3 + i)] = (*rpy_override)[size_t(i)];
        }
        push(proto::pack_frame(proto::RSP_TCP, proto::pack_f32s(out)));
    } else if (cmd == proto::CMD_GET_IK) {
        const std::vector<double> qs = {0.1, -0.2, 0.3, -0.1, 0.2, -0.05, 0.15};
        auto body = proto::pack_f32s(qs);
        body.push_back(uint8_t(ik_ok ? 0x01 : 0x00));
        push(proto::pack_frame(proto::RSP_IK, body));
    } else if (CART_MIN_LEN.count(cmd) != 0) {
        cart_cmd(cmd, p);
    } else {
        push(proto::pack_frame(proto::RSP_ACK, std::vector<uint8_t>{cmd}));
    }
}

std::optional<proto::Frame> FakeTransport::stamp_status(std::optional<proto::Frame> fr) {
    // 给交付给 SDK 的状态帧盖三样固件属性: 帧序号 seq、enabled (bit9)、CART_BUSY (bit10)。
    //
    // seq 每交付一帧 +1 —— 到位判据的新鲜度闸判的就是它。CART_BUSY 帧序写死: 收到 0x4E
    // 之后的第 2 帧置 1、第 8 帧落 0 (即第 2..7 帧为 1)。enabled (bit9) 跟着自己的
    // self.enabled 走 —— 这一位不是可选的: Arm::enter_dfu 的本地使能态预检读的就是它。
    //
    // 三样都在交付点 (而不是 push) 盖: 这里盖的才是 SDK 真正看到的那一帧。
    if (!fr || fr->cmd != proto::RSP_STATUS) return fr;
    status_seq.store(uint16_t((status_seq.load() + 1) & 0xFFFF));
    auto& payload = fr->payload;
    if (payload.size() < 4) return fr;
    const auto s = proto::pack_u16le(status_seq.load());
    payload[2] = s[0];
    payload[3] = s[1];
    uint16_t flags = proto::read_u16le(payload, 0);
    if (enabled.load()) {
        flags |= uint16_t(1u << proto::FLAG_ENABLED_BIT);
    } else {
        flags &= uint16_t(~(1u << proto::FLAG_ENABLED_BIT));
    }
    if (cart_busy_seq) {
        *cart_busy_seq += 1;
        const int cnt = *cart_busy_seq;
        if (cnt > 8) {
            cart_busy_seq.reset();   // 第 8 帧已落 0 -> 这次规划结束, 停止计数
        } else if (cnt >= 2 && cnt <= 7) {
            flags |= uint16_t(1u << proto::FLAG_CART_BUSY_BIT);
        }
    }
    const auto f = proto::pack_u16le(flags);
    payload[0] = f[0];
    payload[1] = f[1];
    return fr;
}

std::optional<proto::Frame> FakeTransport::read_frame(double timeout_in) {
    (void)timeout_in;
    std::lock_guard<std::mutex> sl(state_lock);
    if (dfu_gone && resp_.empty()) {
        // 队列先投完再抛: 现实里登记 ACK 必须在跳转之前出 USB, 桩若先把 ACK 吞掉, SDK 那条
        // ACK 等待会超时 —— 形状就错了。
        throw TransportError("桩: 设备已进 ROM bootloader (CDC 上的读路径抛错)");
    }
    std::optional<proto::Frame> fr;
    if (!resp_.empty()) {
        fr = proto::unpack_frame(resp_.front());
        resp_.pop_front();
    } else if (auto_status) {
        // 模拟 100Hz 主动状态流 (空闲帧) —— 按 auto_status_period 节流: 不节流的话读线程会
        // 以桩的最高速率空转。
        const double now = now_s();
        if (now - auto_status_last_ < auto_status_period) return std::nullopt;
        auto_status_last_ = now;
        fr = proto::unpack_frame(make_status(q, {}, 1, status_flags(enabled), 0, 0, n));
    }
    return stamp_status(fr);
}

}  // namespace testing
}  // namespace litearm
