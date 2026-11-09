#ifndef ABCORE_CONSTRAINT_GENERATION_TYPES_H
#define ABCORE_CONSTRAINT_GENERATION_TYPES_H

#include "constant.h"

#include <cstddef>
#include <vector>

namespace constraint_generation {

using Witness = std::vector<NODE_TYPE>; // canonical local-ID target witness。

// 一个 target-related residual component 的完整局部问题。
struct ComponentProblem {
    NODE_TYPE index{0};                         // 从 1 开始的 component 编号。
    std::vector<NODE_TYPE> vertices;            // local ID 到 global ID 的映射。
    std::vector<std::size_t> offsets;           // 局部 CSR 行起点。
    std::vector<NODE_TYPE> neighbors;           // 局部 CSR 邻接 local IDs。
    std::vector<NODE_TYPE> thresholds;          // 每点的异质 peeling 阈值 delta。
    std::vector<unsigned char> boundary;        // 每点是否连接目标子图。
    std::vector<NODE_TYPE> baseline_degrees;    // forced-only 基线外部度数。

    // 返回当前局部问题的顶点数。
    NODE_TYPE size() const noexcept {
        return static_cast<NODE_TYPE>(vertices.size());
    }
};

// 对某个完整 blocker 集执行 fixed-point peeling 后的不可变快照。
struct Residual {
    std::vector<unsigned char> active; // local ID 是否仍在 residual 中。
    std::vector<NODE_TYPE> degrees;    // active 诱导子图中的当前度数。
    std::size_t active_vertices{0};    // active 顶点数。
    std::size_t active_boundaries{0};  // active boundary 数。

    // 判断当前 blocker 是否已消除全部 target witnesses。
    bool feasible() const noexcept {
        return active_boundaries == 0;
    }
};

// Witness generation 的统一交换类型，保留 growth/slack 的 root provenance。
struct WitnessCandidate {
    Witness witness;              // canonical、结构稳定的 local-ID 集合。
    std::vector<NODE_TYPE> roots; // 属于 witness 的 active boundary local IDs。
};

// Maximal-slack decomposition 对一个 boundary 产生的加权 cut。
struct SlackCut {
    NODE_TYPE boundary{0}; // 具有加权系数的 boundary local ID。
    NODE_TYPE slack{0};    // maximal slack s；对应 RHS 为 s+1。
    Witness component;     // boundary 所在 maximal-slack component。

    // 为模型去重提供确定性顺序；零层忽略代数上无区别的 boundary。
    // 输入：另一个 slack cut。输出：当前 cut 的 canonical key 是否更小。
    bool operator<(const SlackCut &other) const noexcept;
};

// 交给 master 的稀疏下界行，与具体求解器 API 解耦。
struct MasterRow {
    std::vector<NODE_TYPE> columns; // 升序 local variable IDs。
    std::vector<double> coefficients; // 与 columns 一一对应的正系数。
    double lower_bound{0.0}; // 线性表达式的有限下界。
};

// Pipeline 一次生成或分离的 typed constraints 及阶段计数。
struct ConstraintBatch {
    std::vector<Witness> witnesses;       // 后续 Growth 的普通约束。
    std::vector<Witness> slack_fallback_witnesses; // Slack 零层整块保底约束。
    std::vector<SlackCut> slack_cuts;      // 首轮或 residual separation 的加权 cuts。

    // 判断本 batch 是否尚未携带任何一种约束。
    bool empty() const noexcept {
        return witnesses.empty() && slack_fallback_witnesses.empty() && slack_cuts.empty();
    }

    // 返回本 batch 提交给 master 的 typed constraint 数量。
    std::size_t size() const noexcept {
        return witnesses.size() + slack_fallback_witnesses.size() + slack_cuts.size();
    }
};

// Pipeline 对一次完整 residual 检查的闭合结果，避免用空 batch 猜测可行性。
class PipelineResult {
public:
    // 构造“完整 residual 已可行”的结果。
    // 输入：无。输出：不携带约束的可行结果。
    static PipelineResult feasibleResidual();

    // 构造“完整 residual 不可行”的结果，并强制至少携带一条分离约束。
    // 输入：非空 typed batch。输出：不可行结果；空 batch 时抛出异常。
    static PipelineResult violatedResidual(ConstraintBatch constraints);

    // 返回完整 peeling residual 的显式可行状态。
    bool isFeasible() const noexcept;

    // 返回本次生成的约束；可行结果对应空 batch。
    const ConstraintBatch &constraints() const noexcept;

private:
    // 仅允许两个命名构造函数建立满足不变量的结果。
    // 输入：可行标记与 batch。输出：值语义 pipeline 结果。
    PipelineResult(bool feasible, ConstraintBatch constraints);

    bool feasible_{false};       // 完整 residual 是否已消除全部 boundary。
    ConstraintBatch constraints_; // 不可行时非空的分离约束。
};

// CG 分量可加下界及最优性认证状态。
struct SolveResult {
    std::size_t lower_bound{0}; // forced blockers 与各分量已证明下界之和。
    bool timed_out{false}; // 是否因墙钟预算耗尽而返回当前下界。
    bool proved_optimal{false}; // 是否完成全部分量的最优性认证。
    std::size_t component_count{0}; // 进入约束生成的分量数量。
    std::size_t iterations{0}; // master 求解总轮数。
};

// 预处理输出：forced blockers 与互不相交的局部问题。
struct PreprocessingResult {
    std::vector<NODE_TYPE> forced_blockers; // global IDs。
    std::vector<ComponentProblem> components; // target-related residuals。
};

}  // namespace constraint_generation

#endif  // ABCORE_CONSTRAINT_GENERATION_TYPES_H
