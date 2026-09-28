// _common —— 样例共用样板: 命令行开关 + 建 Arm + 统一错误外壳。
//
// 约定 (安全): 默认**只读** —— 连接后仅能 status/tcp/ik 等查询。
// 任何会 enable / 运动 / 改参 的样例都要显式加 `--go` 才执行, 防误动真机。
#pragma once

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "litearm/litearm.hpp"

namespace example {

struct Args {
    std::string port;        // 空 = 自动发现 (VID:PID 1d50:606f)
    bool go = false;         // 真正 enable/运动/改参 (默认只读连接, 不上力)
    double speed = 0.3;      // move 速度倍率 0..1
    std::vector<double> rest;   // 位置参数 (如 movej 的 7 个目标角)
    /// 本样例私有的开关 (`--dist 0.1` -> extra["dist"] = "0.1")。
    /// 通用解析不认识的 `--x` 都落这里, 免得每个样例各写一遍扫描。
    std::map<std::string, std::string> extra;

    /// 取一个位置参数 (越界返回 fallback)。
    double num(size_t i, double fallback = 0.0) const {
        return i < rest.size() ? rest[i] : fallback;
    }
    double num(const std::string& key, double fallback) const {
        const auto it = extra.find(key);
        return it == extra.end() ? fallback : std::atof(it->second.c_str());
    }
    bool has(const std::string& key) const { return extra.count(key) != 0; }
    std::string str(const std::string& key, const std::string& fallback) const {
        const auto it = extra.find(key);
        return it == extra.end() ? fallback : it->second;
    }
};

/// 解析 argv。`desc` 用于 --help。遇到 --help 时打印并返回 false。
bool parse_args(int argc, char** argv, const std::string& desc, Args* out);

/// 连接并校验固件版本约定 (Litearm<主.次.修>-{7J|1J} >= 1.5.0)。
///
/// 端口优先级: --port > 环境变量 LITEARM_PORT > 自动发现 (1d50:606f)。
/// ⚠ SDK 本身**不读任何环境变量** —— LITEARM_PORT 是脚本/样例这一层的事。
std::unique_ptr<litearm::Arm> make_arm(const Args& args);

/// 样例的统一外壳: 解析 -> 连接 -> 跑 body -> 收尾 -> 统一报错。
///
/// 为什么要它: 每个样例各写一遍 try/catch 时, "**连不上**"这一步 (它发生在 body 之外)
/// 最容易漏 —— 漏了就是一句 `terminate called after throwing ...` 的崩溃, 而不是
/// 一句能读的错误。所有对外入口都挂在这一个外壳上。
///
/// `body` 返回进程退出码 (0 = 成功)。`on_close` 在关闭**之前**跑 (打印收尾提示之类)。
int run_example(int argc, char** argv, const std::string& desc,
                const std::function<int(litearm::Arm&, const Args&)>& body,
                const std::function<void()>& on_close = {});

/// 把一组数打印成 `[a, b, ...]`。
std::string fmt(const std::vector<double>& v, int prec = 3);
std::string fmt(const std::array<double, 6>& v, int prec = 3);

}  // namespace example
