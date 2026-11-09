#ifndef ABCORE_CONSTRAINT_GENERATION_HITTING_SET_MODEL_H
#define ABCORE_CONSTRAINT_GENERATION_HITTING_SET_MODEL_H

#include "ConstraintGeneration/Types.h"

#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

namespace constraint_generation {

class HittingSetModel {
public:
    // 建立一个每个 local vertex 对应一个二元变量的最小 hitting-set 模型。
    // 输入：局部候选点数。输出：尚无约束行的 restricted master。
    explicit HittingSetModel(NODE_TYPE vertex_count);

    // 释放隐藏的规划求解器资源。
    // 输入：无。输出：内部资源随对象销毁。
    ~HittingSetModel();

    HittingSetModel(const HittingSetModel &) = delete;
    HittingSetModel &operator=(const HittingSetModel &) = delete;

    // 批量加入 sum_{v in W} x_v >= 1 的普通 witness 行。
    // 输入：非空、canonical、尚未加入的 witnesses。输出：新增行数。
    std::size_t addWitnesses(const std::vector<Witness> &witnesses);

    // 批量加入 (s+1)x_b + sum_{v!=b}x_v >= s+1 的 slack cuts。
    // 输入：保留 component、boundary、slack 的 cuts。输出：新增代数行数。
    std::size_t addSlackCuts(const std::vector<SlackCut> &cuts);

    // 求解当前 restricted master 的最小基数 blocker 集。
    // 输入：当前全部 witness 行和剩余秒数。输出：升序最优 local IDs；超时为空。
    std::optional<std::vector<NODE_TYPE>> solve(double remaining_seconds);

    // 返回历次 master 求解已证明的整数下界；无输入，未求解时为零。
    std::size_t lowerBound() const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_; // 隐藏具体规划求解器的头文件与模型细节。
};

}  // namespace constraint_generation

#endif  // ABCORE_CONSTRAINT_GENERATION_HITTING_SET_MODEL_H
