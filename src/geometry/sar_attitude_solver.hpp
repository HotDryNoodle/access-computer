#pragma once

#include <array>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "geometry/sar_geometry.hpp"

/**
 * @file sar_attitude_solver.hpp
 * @brief AC-024：SAR 零距离率毫秒时刻和完整执行姿态。
 *
 * 状态坐标为 EarthMJ2000Eq；位置 km、速度 km/s。内部连续时间为相对报告
 * 首节点秒，公共时刻为 ISO-8601 UTC 毫秒。
 */

namespace mp {

/** AC-024 no-result 稳定 warning（勿改字）。 */
inline constexpr const char* kSarAttitudeNoFeasibleWarning =
    "SAR attitude refinement rejected selected window: no millisecond "
    "candidate satisfies range-rate and geometry constraints";

/** @brief SAR AE 显式几何与残差门限。 */
struct SarAttitudeOptions {
    double      step_sec          = 0.0;
    double      working_time_sec  = 0.0;
    double      incidence_min_deg = 0.0;
    double      incidence_max_deg = 0.0;
    std::string allowed_look_side;
    double      roll_max_deg           = 0.0;
    double      center_frequency_hz    = 0.0;
    double      azimuth_beamwidth_deg  = 0.0;
    double      max_abs_squint_deg     = 0.0;
    double      max_abs_range_rate_mps = 0.1;
    std::optional<std::chrono::system_clock::time_point> expected_start;
    std::optional<std::chrono::system_clock::time_point> expected_end;
};

/** @brief 权威 SAR body→EarthMJ2000Eq 姿态及派生欧拉角。 */
struct SarAttitude {
    std::array<double, 4> quaternion_wxyz{1.0, 0.0, 0.0, 0.0};
    double                roll_deg   = 0.0;
    double                pitch_deg  = 0.0;
    double                yaw_deg    = 0.0;
    double                squint_deg = 0.0;
};

/** @brief SAR AE 精化结果。 */
struct SarAttitudeRefineResult {
    bool                     ok = false;
    SarAccessWindow          window;
    SarAttitude              attitude;
    std::vector<std::string> warnings;
};

/**
 * @brief 在某连续时刻 Hermite 插值卫星/目标状态和目标法线。
 * @param nodes 严格 19-token 状态节点。
 * @param t_sec 相对首节点秒。
 * @return 可直接输入 @c compute_sar_geometry 的插值节点。
 * @throws std::runtime_error 节点不足、覆盖外或插值退化。
 */
SarStateNode evaluate_sar_state(const std::vector<SarStateNode>& nodes,
                                double                           t_sec);

/**
 * @brief 精化 task-manager 已选 SAR RSA 窗。
 * @param state_path GMAT 19-token SAR 状态报告。
 * @param selected_window 含 ISO @c start_utc/end_utc/t0_utc 的对象。
 * @param options 报告步长、几何、中心频率、工作时长和残差门限。
 * @return 成功时精确执行窗与完整姿态；无合格毫秒候选时 @c ok=false。
 * @throws std::runtime_error 输入/报告/覆盖/数值算法错误。
 * @note 连续解经绝对 UTC 毫秒网格的 floor/ceil 重算；最终窗口还与
 *       refined geometry window 求交，不允许 clamp 出窗。
 */
SarAttitudeRefineResult refine_sar_attitude(
    const std::filesystem::path& state_path,
    const nlohmann::json&        selected_window,
    const SarAttitudeOptions&    options);

/**
 * @brief 导出 SAR 完整姿态 JSON。
 * @param t0_utc 精确 ISO-8601 UTC 毫秒时刻。
 * @param attitude 权威四元数与派生角。
 * @param geometry 同一时刻重算的 SAR 几何。
 * @return output schema 的 @c attitude 对象。
 */
nlohmann::json sar_attitude_to_json(const std::string& t0_utc,
                                    const SarAttitude& attitude,
                                    const SarGeometry& geometry);

}  // namespace mp
