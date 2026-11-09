#include "ConstraintGeneration/ConstraintEncoder.h"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace constraint_generation {
namespace {

// 检查一个 local-ID support 是否可安全编码为稀疏 master 行。
// 输入：support、局部变量数和错误上下文。输出：合法时返回，否则抛出。
void validateSupport(const Witness &support,
                     const NODE_TYPE vertex_count,
                     const char *context) {
    if (support.empty() || !std::is_sorted(support.begin(), support.end()) ||
        std::adjacent_find(support.begin(), support.end()) != support.end()) {
        throw std::invalid_argument(
            std::string(context) + " support must be canonical and non-empty");
    }
    for (const NODE_TYPE vertex : support) {
        if (vertex >= vertex_count) {
            throw std::invalid_argument(
                std::string(context) + " support vertex is out of range");
        }
    }
}

}  // namespace

MasterRow encodeWitnessRow(const Witness &witness,
                           const NODE_TYPE vertex_count) {
    validateSupport(witness, vertex_count, "witness row");
    MasterRow row; // 全系数 1 的普通 hitting-set 行。
    row.columns = witness;
    row.coefficients.assign(witness.size(), 1.0);
    row.lower_bound = 1.0;
    return row;
}

MasterRow encodeSlackCut(const SlackCut &cut,
                         const NODE_TYPE vertex_count) {
    validateSupport(cut.component, vertex_count, "slack cut");
    const auto boundary_position =
        std::lower_bound(cut.component.begin(), cut.component.end(),
                         cut.boundary); // boundary 在有序 support 中的位置。
    if (boundary_position == cut.component.end() ||
        *boundary_position != cut.boundary) {
        throw std::invalid_argument(
            "slack cut component omits its boundary");
    }

    MasterRow row; // 保留 boundary 鲁棒性的加权 slack 行。
    row.columns = cut.component;
    row.coefficients.assign(cut.component.size(), 1.0);
    const std::size_t boundary_offset =
        static_cast<std::size_t>(boundary_position -
                                 cut.component.begin()); // boundary 系数下标。
    row.coefficients[boundary_offset] =
        static_cast<double>(cut.slack) + 1.0;
    row.lower_bound = static_cast<double>(cut.slack) + 1.0;
    return row;
}

}  // namespace constraint_generation
