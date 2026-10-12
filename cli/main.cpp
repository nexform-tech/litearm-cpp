// litearm-cpp 命令行入口 —— 库的薄壳, 零额外依赖。
//
// 只做两件事: 把 `argv` 转成 `std::vector<std::string>` 交给 `litearm::main_impl`
// (巡检/冒烟, 对应 python -m litearm), 再把 main_impl 抛出的异常统一收敛成
// "打一行到 stderr + 返回 1", 不裸露堆栈。用法见 README "Command line" 一节。
#include <cstdio>

#include <exception>
#include <string>
#include <vector>

#include <litearm/litearm.hpp>

int main(int argc, char** argv) {
    try {
        return litearm::main_impl({argv + 1, argv + argc});
    } catch (const litearm::LiteArmError& e) {
        std::fprintf(stderr, "litearm-cpp: %s\n", e.what());
        return 1;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "litearm-cpp: %s\n", e.what());
        return 1;
    }
}
