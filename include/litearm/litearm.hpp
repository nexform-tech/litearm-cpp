// litearm-cpp —— LiteArm STM32 直连后端 (薄协议 SDK)。
//
// 定位: 固件 (litearm-stm32, 版本约定 Litearm1.5.0+) 已内置 S 曲线/IK/动力学/控制律,
// 本库直连 USB CDC 用高层子集镜像 litearm-python 的常用用法, PC 端不做轨迹/运动学。
//
// 典型用法:
//
//     #include <litearm/litearm.hpp>
//     using namespace litearm;
//
//     Arm arm;                       // 不指定端口则自动发现 (VID:PID 1d50:606f)
//     arm.connect();                 // 校验固件版本约定
//     arm.enable();
//     arm.movej({0.1, 0, 0, 0, 0, 0, 0}, 0.3);
//     auto tcp = arm.get_tcp().value;              // 读值要 .value (返回信封 Msg)
//     auto q = arm.ik({0.30, 0.0, 0.35, 3.14, 0.0, 0.0});
//     arm.move_l(std::array<double,6>{0.30, 0.0, 0.40, 3.14, 0.0, 0.0}, 0.5);
//     arm.disable();
//     arm.close();
//
// **返回信封** Msg<T> { value, hz, timestamp }: 11 个"读一帧"型 getter
// (get_state / get_status_now / get_tcp / get_ff_vec / get_ff_scalar /
//  params().get_joint_param / model().get_body / model().get_jm / model().status /
//  model().get_gravity / diag().kin_bench) 返回它 —— value 是原返回值, hz 是该类帧在本
// 会话里的平均到达频率, timestamp 是最近一帧的本地时刻。move_* / home (动作结果) 与纯本地量
// 不包。
//
// ⚠ 与本库的 Python 版一致: 每条会话都带一条读线程, 所以**必须 close()**。
// C++ 里 Arm 的析构会兜底调用 close(), 但显式 close() 仍是推荐做法 (析构只在对象真的被
// 析构时才跑, 而"还想再用一会儿"的语义会让对象活到作用域结束)。
#pragma once

#include "litearm/ack.hpp"
#include "litearm/arm.hpp"
#include "litearm/cart.hpp"
#include "litearm/clock.hpp"
#include "litearm/diagnostics.hpp"
#include "litearm/errors.hpp"
#include "litearm/log.hpp"
#include "litearm/model.hpp"
#include "litearm/msg.hpp"
#include "litearm/params.hpp"
#include "litearm/protocol.hpp"
#include "litearm/rot.hpp"
#include "litearm/state.hpp"
#include "litearm/transport.hpp"

#define LITEARM_VERSION_MAJOR 2
#define LITEARM_VERSION_MINOR 1
#define LITEARM_VERSION_PATCH 0
#define LITEARM_VERSION_STRING "2.1.0"
#define LITEARM_VERSION_CSTR "2.1.0"

namespace litearm {

inline const char* version() { return LITEARM_VERSION_STRING; }

}  // namespace litearm
