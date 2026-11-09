#include "ConstraintGeneration/Types.h"

#include <cstddef>
#include <stdexcept>
#include <utility>

namespace constraint_generation {

bool SlackCut::operator<(const SlackCut &other) const noexcept {
    if (slack != other.slack) {
        return slack < other.slack;
    }
    if (component != other.component) {
        return component < other.component;
    }
    // s=0 时所有点系数均为 1，同一 component 的 boundary 不改变代数行。
    return slack != 0 && boundary < other.boundary;
}

PipelineResult PipelineResult::feasibleResidual() {
    return PipelineResult(true, {});
}

PipelineResult PipelineResult::violatedResidual(
    ConstraintBatch constraints) {
    if (constraints.empty()) {
        throw std::invalid_argument(
            "infeasible residual requires a non-empty constraint batch");
    }
    return PipelineResult(false, std::move(constraints));
}

bool PipelineResult::isFeasible() const noexcept {
    return feasible_;
}

const ConstraintBatch &PipelineResult::constraints() const noexcept {
    return constraints_;
}

PipelineResult::PipelineResult(const bool feasible,
                               ConstraintBatch constraints)
    : feasible_(feasible), constraints_(std::move(constraints)) {}

}  // namespace constraint_generation
