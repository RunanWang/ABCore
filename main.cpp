#include "ConstraintGeneration/Solver.h"
#include "ExchangeSearch/Solver.h"
#include "DataStructure/Graph.h"
#include "DataStructure/Subgraph.h"
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using Clock = std::chrono::steady_clock; // 单调墙钟，计时单位为毫秒。

// 解析非负整数；输入 text 为命令行参数，输出限定在 NODE_TYPE 范围内的值。
NODE_TYPE parseUnsigned(const std::string &text) {
    std::size_t used = 0; // 已消费的字符数。
    if (text.empty() || text[0] == '-') throw std::invalid_argument("invalid unsigned integer");
    const auto value = std::stoull(text, &used); // 参数的无符号值。
    if (used != text.size() || value > std::numeric_limits<NODE_TYPE>::max())
        throw std::invalid_argument("integer is outside vertex-ID range");
    return static_cast<NODE_TYPE>(value);
}

// 解析秒数；输入 text 为限时参数，输出有限正数，允许小数秒。
double parseSeconds(const std::string &text) {
    std::size_t used = 0; // 已消费的字符数。
    const double seconds = std::stod(text, &used); // 用户配置的秒数。
    if (used != text.size() || !std::isfinite(seconds) || seconds <= 0)
        throw std::invalid_argument("time limit must be positive and finite");
    return seconds;
}

// 计算最小 anchor；输入 graph、target 和正阈值 k，输出内部度小于 k 的目标顶点。
std::vector<NODE_TYPE> minimumAnchors(const Graph &graph, const Subgraph &target, NODE_TYPE k) {
    std::vector<unsigned char> inside(graph.nodesNum, 0); // 目标成员位图。
    for (const auto v : target.vertices()) inside[v] = 1;
    std::vector<NODE_TYPE> anchors; // 不满足目标内部度阈值的顶点。
    for (const auto v : target.vertices()) {
        NODE_TYPE degree = 0; // v 在目标诱导子图中的度。
        for (auto p = graph.start(v); p < graph.end(v); ++p) degree += inside[graph.edges[p]];
        if (degree < k) anchors.push_back(v);
    }
    return anchors;
}

// 运行 citeseer 实例；输入可选数据目录、子图编号、k 和两种算法限时，输出 CG 下界、ES 上界、最优性状态与耗时。
int main(int argc, char **argv) {
    try {
        std::vector<std::string> positional; // 原有的数据目录、子图和 k 参数。
        constraint_generation::ExactConfig cg_config; // CG 默认独立预算为 300 秒。
        anytime::AnytimeConfig config; // ES 默认独立预算为 300 秒。
        for (int i = 1; i < argc; ++i) {
            const std::string argument = argv[i]; // 当前命令行参数。
            if (argument == "--cg-time-limit" || argument == "--es-time-limit") {
                if (++i == argc) throw std::invalid_argument("missing time limit value");
                const double seconds = parseSeconds(argv[i]); // 该算法的墙钟秒数。
                if (argument == "--cg-time-limit") cg_config.time_limit_seconds = seconds;
                else config.time_limit_seconds = seconds;
            } else if (argument.rfind("--", 0) == 0) {
                throw std::invalid_argument("unknown option: " + argument);
            } else positional.push_back(argument);
        }
        if (positional.size() > 3) throw std::invalid_argument(
            "usage: abcore_case [dataset-directory [subgraph-id [k]]] [--cg-time-limit SECONDS] [--es-time-limit SECONDS]");
        const std::string directory = positional.empty() ? "Dataset/citeseer" : positional[0]; // 数据目录。
        const NODE_TYPE subgraph_id = positional.size() > 1 ? parseUnsigned(positional[1]) : 2; // 零起始子图编号。
        const NODE_TYPE k = positional.size() > 2 ? parseUnsigned(positional[2]) : 6; // 固定 core 阈值。
        if (!k) throw std::invalid_argument("k must be positive");
        Graph graph; // 清洗后的无向 CSR 图。
        graph.readUndirectedGraphFromFile(directory + "/citeseer.graph.txt");
        std::ifstream input(directory + "/citeseer.subgraph.txt"); // 每行一个目标子图。
        if (!input) throw std::runtime_error("cannot open subgraph file");
        std::string line; // 当前子图的原始文本。
        for (NODE_TYPE i = 0; i <= subgraph_id; ++i)
            if (!std::getline(input, line)) throw std::invalid_argument("subgraph ID is outside dataset");
        const Subgraph target = Subgraph::fromText(line); // 所选的完整目标子图。
        target.validate(graph);
        if (target.empty() || target.connectedComponents(graph).size() != 1)
            throw std::invalid_argument("target must be nonempty and connected");
        const auto start = Clock::now(); // anchor 预处理开始。
        const auto anchors = minimumAnchors(graph, target, k); // 最小 anchor 集。
        const auto after_anchor = Clock::now(); // CG 开始。
        std::cout << std::boolalpha << std::defaultfloat << std::setprecision(9)
                  << "CG-time-limit-s=" << cg_config.time_limit_seconds
                  << " ES-time-limit-s=" << config.time_limit_seconds << '\n'
                  << std::fixed << std::setprecision(3)
                  << "subgraph-id=" << subgraph_id << " k=" << k << " target-size=" << target.size() << '\n'
                  << "anchor-size=" << anchors.size() << " preprocessing-ms="
                  << std::chrono::duration<double, std::milli>(after_anchor-start).count() << '\n';
        const auto cg = constraint_generation::Solver{}.solve(graph, target, k, cg_config); // CG 已证明的下界。
        const auto after_cg = Clock::now(); // ES 开始。
        std::cout << "CG LB=" << cg.lower_bound << " time-ms="
                  << std::chrono::duration<double, std::milli>(after_cg-after_anchor).count()
                  << " timed-out=" << cg.timed_out << " optimal=" << cg.proved_optimal << '\n';
        config.max_exchange_size = std::numeric_limits<std::size_t>::max();
        const auto es = anytime::solveExchangeSearch(graph, target, k, config); // ES 当前解提供上界。
        const auto after_es = Clock::now(); // 算法计时结束。
        const std::size_t upper_bound = es.blockers.size(); // ES 当前全局上界。
        const bool optimal = cg.lower_bound == upper_bound; // 上下界相等即认证全局最优。
        std::cout << "ES UB=" << upper_bound << " time-ms="
                  << std::chrono::duration<double, std::milli>(after_es-after_cg).count()
                  << " timed-out=" << es.timed_out << " optimal=" << es.proved_optimal << '\n';
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
