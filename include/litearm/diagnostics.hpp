// 固件自检 —— CMD_KIN_BENCH(0x49) -> RSP_KIN_BENCH(0x4A) 文本回执的解析。
//
// 固件侧在后台用 DWT cycle 测各热点单拍开销 (禁中断测量, 最长窗口 5s), 结果拼成多行文本
// 经可靠应答通道回。最有价值的产出是最后一行 LINK —— 它承载了固件里唯一的链路诊断计数
// (CRC 坏帧 / RX 环溢出 / 应答 FIFO 丢弃 / CAN TX 失败分类桶 / 控制拍峰值与超预算拍数 /
// 锁存拍与原因), 这些在固件里原本全部静默。
//
// 0x49 是双 ID: 下行 CMD_KIN_BENCH 与上行 RSP_JOINT_PARAM 同值, 靠收发方向区分。
// 该命令不检查 payload 长度, 发出去必产生一次真实测量 —— 故仅用于显式调用。
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "litearm/msg.hpp"

namespace litearm {

class Arm;

/// 固件 bench 最长窗口 KIN_BENCH_MAX_TICKS = 5s, 留 1.6 倍余量。
inline constexpr double KIN_BENCH_TIMEOUT = 8.0;

/// 固件把 KIN_BENCH 回执拆成连续两帧发 (第 1 帧 = 各项耗时, 第 2 帧 = LINK 诊断行)。
///
/// 拆帧的理由是单帧载荷上限 255B 而全文实测约 306B: 原单帧发出会在 253B 处硬截断,
/// 截断点恰好落在 LINK 行的 loop_k= 之后 => loop_k/ovr/flt/cause/st 一个都读不到。
///
/// **只收一帧必然丢掉全部链路计数, 而丢了它不会报错** —— 五个便捷出口全是
/// link.get(key, 0) 语义, 拿不到就是静默的 0, 与"没有出错"分不开。
inline constexpr int KIN_BENCH_FRAMES = 2;

/// 第 2 帧的等待窗口。新固件它紧跟第 1 帧到; 旧固件根本不发 => 靠超时退出。
/// 代价: 在只发一帧的旧固件上, 每次 kin_bench() 会多等这半个秒。
inline constexpr double KIN_BENCH_MORE_S = 0.5;

/// KIN_BENCH 回执。raw 是固件原文 (逐行), timings/link 是解析结果。
///
/// timings: 块名 -> 数字序列。FK/JAC/SC/G/RNEA/M/LAW 是 (n, avg_cyc, max_cyc),
/// IK 是 (n_try, min_cyc, max_cyc), LOOP1 是 (cycles,)。
/// link: 链路诊断计数 (见模块顶部)。
struct KinBenchResult {
    std::string raw;
    std::map<std::string, std::vector<long long>> timings;
    std::map<std::string, long long> link;

    std::string str() const { return raw; }

    // ---- 常用诊断的便捷出口 (拿不到就是 0, 与"没有出错"分不开, 见 KIN_BENCH_FRAMES) ----
    long long crc_errors() const;        //: 固件累计 CRC 坏帧 (上位机发出的帧被固件丢弃)
    long long reply_dropped() const;     //: 固件应答 FIFO 队满丢弃数
    long long can_tx_fail() const;       //: CAN 发送失败累计
    long long loop_max_kcycle() const;   //: 控制拍峰值耗时 (千 cycle)
    long long loop_overruns() const;     //: 超出周期预算的控制拍数

    //: 固件自己点名的"静默丢反馈"项。键名带数字 (rxl0/rxl1) —— 解析器必须吃得下,
    //: 否则这三个出口恒返回 0, 而"读不到"与"没丢过"长得一模一样。
    long long rx_fifo_lost_motor() const;   //: 电机域 CAN FIFO0 溢出计数
    long long rx_fifo_lost_bridge() const;  //: gs_usb 桥 CAN FIFO1 溢出计数
    long long gsusb_ring_drops() const;     //: gs_usb 桥环满丢帧数
};

/// 解析固件 KIN_BENCH 文本; 空文本视为异常回执 (抛 TransportError)。
KinBenchResult parse_kin_bench(const std::string& text);

/// `arm.diag()` —— 固件自检入口。
class Diagnostics {
public:
    explicit Diagnostics(Arm* arm) : arm_(arm) {}

    /// 触发固件运动学/动力学单拍开销自测, 返回解析后的回执。
    ///
    /// 固件侧最长窗口 5s (期间控制循环心跳监督豁免), 默认超时 8s。
    /// 该命令不检查载荷长度, 发出即产生一次真实测量。
    ///
    /// **回执是连续两帧, 本方法两帧都收** —— 只收第 1 帧会静默丢掉全部链路计数。
    /// 旧固件只发一帧, 靠短窗口超时退出, 行为兼容。
    Msg<KinBenchResult> kin_bench(double timeout = KIN_BENCH_TIMEOUT);

private:
    Arm* arm_;
};

}  // namespace litearm
