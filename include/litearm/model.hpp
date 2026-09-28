// 动力学模型在线导入 —— 固件 0x30/0x32/0x33/0x34/0x35/0x37/0x38/0x39 的封装。
//
// stm32 固件里, 连杆动力学 (m / com / I) 与电机转子惯量 jm 原先都是编译期常量。产线要求
// 「每台臂出厂前上 HYY 控制器辨识一次动力学 -> 把参数写进固件」, 逐台不同 => 必须能在线
// 写入, 否则每台都得重编译烧录。
//
// 固件的模型分三层:
//     bank     模型唯一权威, 永不含 payload; 来源 = 编译期常量 或 导入/flash
//     staging  0x30 / 0x33 写这里, 不参与递推
//     生效层   bank + payload 复合, RNEA 读这一层
// 0x32 commit 把 staging 整组原子拷进 bank => 不存在"半截模型生效"。
//
// 正确用法:
//     arm.model().set_body(1, {...});  // J1 ... J7
//     arm.model().set_jm({...});       // 7 个
//     arm.model().commit(MODEL_MASK_WRITTEN);   // 0x2FE
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "litearm/msg.hpp"

namespace litearm {

class Arm;

/// 模型刚体数 —— 与固件 dyn_model_data.h 的 DYN_MODEL_NBODY 同值。
/// 与 arm.n() (关节数, 台架 1J 时为 1) 无关: 模型恒 9 刚体 (0=base, 1..7=关节,
/// 8=ee 固定体)。照抄 joint param 用 arm.n() 做上界会让台架完全无法验证模型。
inline constexpr int MODEL_NBODY = 9;

/// 单 body 的字段数: m, cmx, cmy, cmz, ixx, ixy, ixz, iyy, iyz, izz
inline constexpr int MODEL_BODY_PARAMS = 10;

/// jm 长度 (关节数)
inline constexpr int MODEL_JM_N = 7;

/// body i 的 expected_mask 位 (i = 0..8)。
inline constexpr uint16_t MODEL_BIT_BODY(int i) { return uint16_t(1u << i); }
inline constexpr uint16_t MODEL_BIT_JM = uint16_t(1u << 9);

/// 工具/产线实际写入的集合: body1..7 + jm (body0=base 与 body8=ee 不写) = 0x2FE。
inline constexpr uint16_t MODEL_MASK_WRITTEN =
    uint16_t((MODEL_BIT_BODY(1) | MODEL_BIT_BODY(2) | MODEL_BIT_BODY(3) |
              MODEL_BIT_BODY(4) | MODEL_BIT_BODY(5) | MODEL_BIT_BODY(6) |
              MODEL_BIT_BODY(7)) |
             MODEL_BIT_JM);
static_assert(MODEL_MASK_WRITTEN == 0x2FE, "MODEL_MASK_WRITTEN 常量漂移");

/// 固件 RSP_MODEL_STATUS(0x56) 的 5B 应答。
struct ModelStatus {
    int override_level = 0;  //: 0 = 编译期常量; 1 = 导入/flash 模型 (不区分二者)
    uint16_t staged_mask = 0;  //: bit0..8 = body0..8, bit9 = jm (0..0x3FF)
    int dirty = 0;             //: 1 = RAM 有未固化改动 (定义 = RAM != flash)

    static ModelStatus decode(const std::vector<uint8_t>& payload);
};

/// `arm.model()` —— 动力学模型的读/写/提交/回退入口。
///
/// 无状态: 每次调用直发一帧, 不做任何本地缓存 —— 固件才是唯一权威, 缓存会与"另一端
/// 可能改了"打架。
class ModelParams {
public:
    explicit ModelParams(Arm* arm) : arm_(arm) {}

    /// 固件是否支持模型在线导入。
    ///
    /// 用 0x34 探测: 旧固件没有这条命令的 case => 落 default => ERR{0x34,0x00} => 捕获
    /// UnsupportedByFirmwareError 并回 false。
    /// 不能用 0x30/0x33 探测 —— 旧固件对它们有显式 case 回 ERR{cmd,0x02} (与"数值非法"
    /// 同码), 无法区分「无此命令」与「参数非法」。
    bool probe();

    /// 读生效 bank 的第 idx 个刚体 (0..8) 的 10 个参数。读的是 bank (无 payload 复合),
    /// 故与 set_body 写入值可直接逐项比对。
    Msg<std::vector<double>> get_body(int idx, double timeout = 1.0);

    /// 写 staging 的第 idx 个刚体。**不生效**, 须 commit()。
    /// 固件拒绝 (ERR{0x30,0x02}) 的情形: 索引越界 / 任一分量非有限或 |v|>1e6 /
    /// idx == 8 (ee 固定体) 而质量非 0。
    void set_body(int idx, const std::vector<double>& vals);

    /// 读生效 bank 的 jm (7 个关节转子惯量)。
    Msg<std::vector<double>> get_jm(double timeout = 1.0);

    /// 写 staging 的 jm[7]。**不生效**, 须 commit()。
    /// ⚠ set_jm 会改写关节映射, 用错会让臂乱摆且没有可靠的回退路径 —— 谨慎使用。
    void set_jm(const std::vector<double>& vals);

    /// staging 整组原子生效。
    ///
    /// expected_mask 必须与固件侧"本次会话实际写过的项"逐位相等, 否则回
    /// ERR{0x32,0x07} (掩码不符 —— 与"数值非法"分开的码, 便于产线定位)。
    /// 零模型哨兵不过回 ERR{0x32,0x02}; 已武装回 ERR{0x32,0x04}。
    void commit(uint16_t expected_mask);

    /// 只回退模型: bank 回编译期常量 (含 jm) + 清 staging。须失能。
    ///
    /// 三条后果 (工具/产线必须知道):
    ///   1. 不动 flash => flash 里的导入模型仍在, 重新上电会复活;
    ///   2. 此后任何一次 save_params() 会把 flash 里的导入模型一并抹掉
    ///      (整扇区擦除, 旧记录不可恢复);
    ///   3. 回退后 status().dirty == 1 (RAM != flash), 但不要据此提示"补固化"。
    void revert();

    /// 读模型状态 (override / staged_mask / dirty)。
    Msg<ModelStatus> status(double timeout = 1.0);

    /// 给定关节角算重力项 G(q) (纯读, 无门控, 不改任何状态)。
    Msg<std::vector<double>> get_gravity(const std::vector<double>& q,
                                         double timeout = 1.0);

private:
    int check_body_idx(int idx) const;
    Arm* arm_;
};

}  // namespace litearm
