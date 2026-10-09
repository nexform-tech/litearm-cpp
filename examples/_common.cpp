#include "_common.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>

namespace example {

namespace {

bool is_known_switch(const std::string& a) {
    return a == "--port" || a == "--go" || a == "--speed" || a == "-h" || a == "--help";
}

/// 整串能否解析成一个数 —— 负数 (如 `-0.25`) 是**参数**, 不是开关。
/// 只看"开头是 `-`"会把 `./02_movej --go 0.1 0 -0.1 0 0 0 0` (02 头注释里的用法)
/// 整个挡死; `--dist -0.04` 同理 (实测踩过)。
bool parses_as_number(const std::string& a) {
    if (a.empty()) return false;
    char* end = nullptr;
    std::strtod(a.c_str(), &end);
    return end != a.c_str() && *end == '\0';
}

}  // namespace

bool parse_args(int argc, char** argv, const std::string& desc, Args* out) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "-h" || a == "--help") {
            std::cout << desc << "\n\n"
                      << "开关:\n"
                      << "  --port PORT   串口 (默认自动找 1d50:606f, 也可用 "
                         "LITEARM_PORT)\n"
                      << "  --go          真正 enable/运动/改参 (默认只读连接, 不上力)\n"
                      << "  --speed S     move 速度倍率 0~1 (默认 0.3)\n"
                      << "  (其余 --x [值] 开关由各样例自己解释)\n";
            return false;
        }
        if (a == "--port" && i + 1 < argc) {
            out->port = argv[++i];
        } else if (a == "--go") {
            out->go = true;
        } else if (a == "--speed" && i + 1 < argc) {
            out->speed = std::atof(argv[++i]);
        } else if (a.size() > 2 && a.rfind("--", 0) == 0) {
            // 样例私有的开关: `--dist 0.1` 或裸 `--verbose`
            const std::string key = a.substr(2);
            std::string val = "1";
            if (i + 1 < argc) {
                const std::string nxt = argv[i + 1];
                if (!nxt.empty() && (nxt[0] != '-' || parses_as_number(nxt))) val = argv[++i];
            }
            out->extra[key] = val;
        } else if (!a.empty() && a[0] == '-' && !is_known_switch(a) && !parses_as_number(a)) {
            std::cerr << "未知开关: " << a << " (用 --help 看用法)\n";
            std::exit(2);
        } else {
            out->rest.push_back(std::atof(a.c_str()));
        }
    }
    return true;
}

std::unique_ptr<litearm::Arm> make_arm(const Args& args) {
    using namespace litearm;
    std::string port = args.port;
    if (port.empty()) {
        const char* env = std::getenv("LITEARM_PORT");
        if (env != nullptr && *env != '\0') port = env;
    }
    ArmOptions opts;
    if (!port.empty()) opts.port = port;
    auto arm = std::make_unique<Arm>(opts);
    arm->connect();
    return arm;
}

int run_example(int argc, char** argv, const std::string& desc,
                const std::function<int(litearm::Arm&, const Args&)>& body,
                const std::function<void()>& on_close) {
    Args args;
    if (!parse_args(argc, argv, desc, &args)) return 0;
    std::unique_ptr<litearm::Arm> arm;
    try {
        arm = make_arm(args);
        const int rc = body(*arm, args);
        if (on_close) on_close();
        arm->close();
        return rc;
    } catch (const litearm::MotionSupersededError& e) {
        // 接管是**预期内**的, 与"规划失败"分开报 —— 混成故障会让调用方走错误恢复。
        std::printf("被接管 (预期内): %s\n", e.what());
    } catch (const litearm::LiteArmError& e) {
        std::printf("出错: %s\n", e.what());
    } catch (const std::exception& e) {
        std::printf("出错: %s\n", e.what());
    }
    if (arm) {
        if (on_close) on_close();
        try {
            arm->close();
        } catch (...) {
        }
    }
    return 1;
}

std::string fmt(const std::vector<double>& v, int prec) {
    std::string out = "[";
    char buf[64];
    for (size_t i = 0; i < v.size(); ++i) {
        std::snprintf(buf, sizeof(buf), "%.*f", prec, v[i]);
        if (i) out += ", ";
        out += buf;
    }
    return out + "]";
}

std::string fmt(const std::array<double, 6>& v, int prec) {
    return fmt(std::vector<double>(v.begin(), v.end()), prec);
}

}  // namespace example
