#include "ConstraintGeneration/ExactConfig.h"
#include <stdexcept>
#include <cmath>
namespace constraint_generation {
void ExactConfig::validate() const {
    if (!std::isfinite(time_limit_seconds) || time_limit_seconds <= 0)
        throw std::invalid_argument("CG time limit must be positive and finite");
    if (!slack.initial_batch_limit || !slack.separation_batch_limit ||
        !growth.generation_limit || !growth.batch_limit) {
        throw std::invalid_argument("constraint generation budgets must be positive");
    }
}
}
