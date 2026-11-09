#include "ConstraintGeneration/HittingSetModel.h"

#include "ConstraintGeneration/ConstraintEncoder.h"

#include "highs/Highs.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace constraint_generation {
namespace {

// 把 HiGHS API 状态统一转换为带操作上下文的异常。
// 输入：API 返回值与动作描述。输出：成功时返回；失败时抛出 runtime_error。
void requireHighsOk(const HighsStatus status, const char *operation) {
    if (status != HighsStatus::kOk) {
        throw std::runtime_error(std::string("HiGHS failed to ") + operation);
    }
}

}  // namespace

class HittingSetModel::Impl {
public:
    // 初始化 blocker 规模和 HiGHS 选项。
    // 输入：局部顶点数。输出：空约束池；每轮求解前重建模型。
    explicit Impl(const NODE_TYPE vertex_count) : vertex_count_(vertex_count) {
        requireHighsOk(highs_.setOptionValue("output_flag", false),
                       "disable exact hitting-set output");
        // 整数目标必须求到零 gap，才能把 restricted optimum 作为已证明下界。
        requireHighsOk(highs_.setOptionValue("mip_rel_gap", 0.0), "disable relative MIP gap");
        requireHighsOk(highs_.setOptionValue("mip_abs_gap", 0.0), "disable absolute MIP gap");
        // HiGHS 1.14.0 的 MIP presolve 在带 weighted Slack rows 的真实模型上
        requireHighsOk(
            highs_.setOptionValue("presolve",
                                  "off"),
            "configure exact hitting-set presolve");
    }

    // 去重并批量加入 ordinary witness rows。
    // 输入：候选 witnesses。输出：实际加入 HiGHS 的行数。
    std::size_t addWitnesses(const std::vector<Witness> &witnesses) {
        std::vector<MasterRow> rows; // 本批首次出现的普通代数行。

        // 1. 先编码校验，再与普通行及 s=0 Slack 行共享 support 去重。
        for (const Witness &witness : witnesses) {
            MasterRow row =
                encodeWitnessRow(witness, vertex_count_); // 校验并编码。
            const auto inserted =
                unit_rows_.insert(witness); // 跨类型去重 key。
            if (inserted.second) {
                rows.push_back(std::move(row));
            }
        }

        // 2. 只把本批新行提交给持久 HiGHS 模型。
        addRows(rows);
        return rows.size();
    }

    // 去重并批量加入 maximal-slack rows；零层与普通 witness 共享 key。
    // 输入：逐 boundary slack cuts。输出：实际加入 HiGHS 的代数行数。
    std::size_t addSlackCuts(const std::vector<SlackCut> &cuts) {
        std::vector<MasterRow> rows; // 本批首次出现的 slack 代数行。

        // 1. s=0 按 support 去重；s>0 的 key 保留 boundary 与 slack。
        for (const SlackCut &cut : cuts) {
            MasterRow row =
                encodeSlackCut(cut, vertex_count_); // 校验并编码加权行。
            bool inserted = false; // 当前 cut 是否产生新的代数行。
            if (cut.slack == 0) {
                inserted = unit_rows_.insert(cut.component).second;
            } else {
                inserted = positive_slack_rows_.insert(cut).second;
            }
            if (inserted) {
                rows.push_back(std::move(row));
            }
        }

        // 2. 统一通过求解器无关的 MasterRow 批量追加。
        addRows(rows);
        return rows.size();
    }

    // 把已去重的求解器无关行追加到完整 row pool。
    // 输入：待追加 rows。输出：通过维度检查后持久保存；空输入不变。
    void addRows(const std::vector<MasterRow> &rows) {
        if (rows.empty()) {
            return;
        }

        // 1. 防御性检查 solver-independent row 的稀疏维度。
        for (const MasterRow &row : rows) {
            if (row.columns.size() != row.coefficients.size()) {
                throw std::logic_error(
                    "exact master row has inconsistent sparse dimensions");
            }
        }

        // 2. 保存已校验行，供下一轮规范重建。
        rows_.insert(rows_.end(), rows.begin(), rows.end());
    }

    // 求解当前模型并把数值解恢复成 local blocker IDs。
    // 输入：持久模型与 remaining_seconds 剩余秒数。输出：升序最优二元解；超时为空。
    std::optional<std::vector<NODE_TYPE>> solve(double remaining_seconds) {
        const auto started = std::chrono::steady_clock::now(); // 包含模型重建的本轮起点。
        if (remaining_seconds <= 0) return std::nullopt;
        // 1. 重建完整模型，避开 HiGHS 1.14.0 增量 MIP 的陈旧搜索状态。
        {
            requireHighsOk(highs_.clearModel(),
                           "clear exact hitting-set model");
            addColumnsToHighs();
            addRowsToHighs(rows_);
        }

        // 2. 要求 HiGHS 返回经过证明的最优整数模型状态。
        remaining_seconds -= std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started).count();
        if (remaining_seconds <= 0) return std::nullopt;
        // HiGHS 的计时跨 run 累积，选项使用累计已用时间加本轮剩余预算。
        requireHighsOk(highs_.setOptionValue("time_limit", highs_.getRunTime() + remaining_seconds),
                       "set remaining master time limit");
        const auto status = highs_.run(); // 限时中断允许返回 warning 状态。
        if (status == HighsStatus::kError) requireHighsOk(status, "solve exact hitting-set ILP");
        const auto &info = highs_.getInfo(); // 本轮求解的有效性与 MIP 统计。
        const double dual_bound = info.mip_dual_bound; // 最小化 master 的已证明下界。
        if (info.valid && std::isfinite(dual_bound) && dual_bound >= 0 && dual_bound <= vertex_count_) {
            // blocker 目标为整数；容差避免浮点误差把整数下界错误抬高一位。
            const auto integer_bound = static_cast<std::size_t>(std::ceil(std::max(0.0, dual_bound - 1e-6))); // 整数下界。
            lower_bound_ = std::max(lower_bound_, integer_bound);
        }
        if (highs_.getModelStatus() == HighsModelStatus::kTimeLimit) return std::nullopt;
        requireHighsOk(status, "solve exact hitting-set ILP");
        if (highs_.getModelStatus() != HighsModelStatus::kOptimal) {
            throw std::runtime_error(
                "HiGHS did not solve exact hitting-set ILP optimally");
        }
        const std::vector<double> &values =
            highs_.getSolution().col_value; // HiGHS 返回的列值。
        if (values.size() != static_cast<std::size_t>(vertex_count_)) {
            throw std::logic_error(
                "HiGHS returned an incomplete exact blocker solution");
        }

        // 3. 按变量下标恢复 blocker IDs，并拒绝非整数数值结果。
        std::vector<NODE_TYPE> blockers; // 恢复出的升序 local IDs。
        for (NODE_TYPE vertex = 0; vertex < vertex_count_; ++vertex) {
            if (values[vertex] > 0.5) {
                blockers.push_back(vertex);
            } else if (std::abs(values[vertex]) > 1e-6) {
                throw std::runtime_error(
                    "HiGHS returned a non-integral exact blocker variable");
            }
        }
        const double objective =
            highs_.getObjectiveValue(); // 求解器报告的最优目标值。
        if (std::abs(objective - static_cast<double>(blockers.size())) >
            1e-6) {
            throw std::runtime_error(
                "HiGHS objective disagrees with the recovered blocker count");
        }
        std::vector<unsigned char> selected(
            vertex_count_, 0); // 恢复解的 blocker 位图。
        for (const NODE_TYPE blocker : blockers) {
            selected[blocker] = 1;
        }
        // 4. 用 solver-independent rows 复核 primal feasibility，拒绝错误解。
        for (const MasterRow &row : rows_) {
            double activity = 0.0; // 当前解在该代数行上的精确活动值。
            for (std::size_t position = 0; position < row.columns.size();
                 ++position) {
                if (selected[row.columns[position]]) {
                    activity += row.coefficients[position];
                }
            }
            if (activity + 1e-6 < row.lower_bound) {
                throw std::runtime_error(
                    "HiGHS returned an infeasible exact blocker solution");
            }
        }
        lower_bound_ = std::max(lower_bound_, blockers.size());
        return blockers;
    }

    // 返回当前已证明的整数下界；无输入，不改变模型。
    std::size_t lowerBound() const noexcept { return lower_bound_; }

private:
    // 向空 HiGHS 模型加入全部二元 blocker 列。
    // 输入：固定 vertex_count_。输出：模型获得对应数量的单位目标列。
    void addColumnsToHighs() {
        for (NODE_TYPE vertex = 0; vertex < vertex_count_; ++vertex) {
            requireHighsOk(
                highs_.addCol(1.0, 0.0, 1.0, 0, nullptr, nullptr),
                "build an exact blocker variable");
            requireHighsOk(
                highs_.changeColIntegrality(static_cast<HighsInt>(vertex),
                                            HighsVarType::kInteger),
                "build an exact binary blocker variable");
        }
    }

    // 将完整 row pool 一次性提交给刚重建的 HiGHS 模型。
    // 输入：已校验 rows，按 Slack 类型规范分组。
    // 输出：内部模型增加相同行数。
    void addRowsToHighs(const std::vector<MasterRow> &rows) {
        std::vector<const MasterRow *> ordered_rows; // 与生成时序无关的规范 row 顺序。
        ordered_rows.reserve(rows.size());
        // 1. 先收集正 depth Slack rows，再收集普通行。
        for (const MasterRow &row : rows) {
            if (row.lower_bound > 1.0) ordered_rows.push_back(&row);
        }
        for (const MasterRow &row : rows) {
            if (row.lower_bound <= 1.0) ordered_rows.push_back(&row);
        }

        std::vector<double> lower; // 每行的有限下界。
        std::vector<double> upper(
            ordered_rows.size(), kHighsInf); // 每行的无穷上界。
        std::vector<HighsInt> starts; // row-wise 稀疏矩阵行起点。
        std::vector<HighsInt> columns; // 展平后的 blocker 列。
        std::vector<double> coefficients; // 展平后的行系数。
        lower.reserve(ordered_rows.size());
        starts.reserve(ordered_rows.size() + 1);
        starts.push_back(0);
        // 2. 按选定顺序展平为 HiGHS row-wise 稀疏矩阵。
        for (const MasterRow *row_pointer : ordered_rows) {
            const MasterRow &row = *row_pointer; // 当前只读代数行。
            lower.push_back(row.lower_bound);
            for (std::size_t position = 0; position < row.columns.size();
                 ++position) {
                columns.push_back(
                    static_cast<HighsInt>(row.columns[position]));
                coefficients.push_back(row.coefficients[position]);
            }
            starts.push_back(static_cast<HighsInt>(columns.size()));
        }
        requireHighsOk(
            highs_.addRows(static_cast<HighsInt>(ordered_rows.size()),
                           lower.data(), upper.data(),
                           static_cast<HighsInt>(columns.size()), starts.data(),
                           columns.data(), coefficients.data()),
            "rebuild exact constraint rows");
    }

    std::size_t lower_bound_{0}; // 跨轮单调保留的 master 下界。
    NODE_TYPE vertex_count_{0}; // 二元变量数量。
    Highs highs_;               // 持久 restricted master。
    std::vector<MasterRow> rows_; // 跨轮累积、供完整重建的代数行。
    std::set<Witness> unit_rows_; // 普通 witness 与 s=0 slack 的共享 rows。
    std::set<SlackCut> positive_slack_rows_; // 保留 boundary 的正 slack rows。
};

HittingSetModel::HittingSetModel(const NODE_TYPE vertex_count)
    : impl_(std::make_unique<Impl>(vertex_count)) {}

HittingSetModel::~HittingSetModel() = default;

std::size_t HittingSetModel::addWitnesses(
    const std::vector<Witness> &witnesses) {
    return impl_->addWitnesses(witnesses);
}

std::size_t HittingSetModel::addSlackCuts(
    const std::vector<SlackCut> &cuts) {
    return impl_->addSlackCuts(cuts);
}

std::optional<std::vector<NODE_TYPE>> HittingSetModel::solve(double remaining_seconds) {
    return impl_->solve(remaining_seconds);
}

std::size_t HittingSetModel::lowerBound() const noexcept {
    return impl_->lowerBound();
}

}  // namespace constraint_generation
