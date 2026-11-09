#include "ConstraintGeneration/Solver.h"
#include "ConstraintGeneration/Preprocessor.h"
#include "ConstraintGeneration/WitnessPipeline.h"
#include "ConstraintGeneration/HittingSetModel.h"
#include <algorithm>
#include <chrono>
#include <stdexcept>
namespace constraint_generation {
SolveResult Solver::solve(const Graph &graph, const Subgraph &target,
                         NODE_TYPE k, ExactConfig config) const {
    config.validate();
    const auto started = std::chrono::steady_clock::now(); // CG 独立预算起点。
    // 返回当前算法剩余秒数；输入为捕获的起点和配置，输出可为负数。
    const auto remaining = [&]() {
        return config.time_limit_seconds - std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started).count();
    };
    // 1. forced blockers 必选，每个 target-related residual 分量至少需要一个 blocker。
    const auto preprocessing = Preprocessor(graph, target, k).run(); // 全局预处理产物。
    SolveResult result; // 全局已证明下界与完成状态。
    result.component_count = preprocessing.components.size();
    result.lower_bound = preprocessing.forced_blockers.size() + result.component_count;
    for (const auto &problem : preprocessing.components) {
        if (remaining() <= 0) { result.timed_out = true; break; }
        std::size_t component_bound = 1; // 当前分量已计入全局下界的部分。
        WitnessPipeline pipeline(problem, config); // Slack + Growth 分离器。
        HittingSetModel master(problem.size()); // 当前分量的 restricted master。
        auto separated = pipeline.initial(); // forced-only residual 的首批约束。
        if (separated.isFeasible()) throw std::logic_error("unexpected feasible residual component");
        // 2. 每轮吸收 master 的已证明下界，包括 HiGHS 限时中断时的 dual bound。
        while (!separated.isFeasible()) {
            if (remaining() <= 0) { result.timed_out = true; break; }
            const auto &batch = separated.constraints(); // 当前候选违反的约束。
            std::size_t added = master.addSlackCuts(batch.slack_cuts); // 去重后新增行数。
            added += master.addWitnesses(batch.slack_fallback_witnesses);
            added += master.addWitnesses(batch.witnesses);
            if (!added) throw std::logic_error("separation made no progress");
            const auto local = master.solve(remaining()); // restricted 最优候选，超时为空。
            ++result.iterations;
            const auto next_bound = std::max(component_bound, master.lowerBound()); // 防止超时。
            result.lower_bound += next_bound - component_bound;
            component_bound = next_bound;
            if (!local || remaining() <= 0) { result.timed_out = true; break; }
            separated = pipeline.separate(*local);
        }
        if (result.timed_out) break;
    }
    // 3. 全部分量候选均通过分离时，下界等于全局最优值；不构造 CG 可行解。
    result.proved_optimal = !result.timed_out;
    return result;
}
}
