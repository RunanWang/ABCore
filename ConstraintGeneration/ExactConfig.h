#pragma once
#include <cstddef>
namespace constraint_generation {
struct GrowthOptions {
    std::size_t generation_limit{1024}; // 每轮候选生成上限。
    std::size_t batch_limit{128}; // 每轮 packing/coverage 接纳上限。
};
struct SlackOptions {
    std::size_t initial_batch_limit{128}; // 首轮高 depth cut 上限。
    std::size_t separation_batch_limit{128}; // 后续高 depth cut 上限。
};
struct ExactConfig {
    double time_limit_seconds{300.0}; // 包含预处理的独立墙钟预算，单位秒。
    SlackOptions slack{}; // Slack 预算。
    GrowthOptions growth{}; // Growth 预算。
    // 检查时间预算与候选批量、生成预算；无参数，非法时抛出异常。
    void validate() const;
};
}
