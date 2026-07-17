#pragma once

#include <array>
#include <chrono>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry/attitude_solver.hpp"

/**
 * @file sar_geometry.hpp
 * @brief AC-010：SAR 状态报告、粗几何门与长期候选窗。
 *
 * 坐标：EarthMJ2000Eq；位置 km、速度 km/s。输出距离率 m/s、斜距 km、
 * 角度 deg、多普勒 Hz。时间报告为 GMAT UTCGregorian，公共窗口为 ISO-8601 UTC。
 */

namespace mp {

/**
 * @brief 单个 SAR 候选的几何或姿态退化。
 * @note 仅此类型可被 RSA/AE 当作候选不可行；报告、插值与算法错误必须上抛。
 */
class SarCandidateInfeasibleError : public std::runtime_error {
  public:
    using std::runtime_error::runtime_error;
};

/** @brief SAR 报告单节点（19-token 合同）。 */
struct SarStateNode {
    double                                t_sec = 0.0;
    std::chrono::system_clock::time_point utc;
    std::string                           utc_gregorian;
    Vec3                                  r_sat;
    Vec3                                  v_sat;
    Vec3                                  r_tgt;
    Vec3                                  v_tgt;
    Vec3                                  r_normal_point;
};

/** @brief 某一时刻的 SAR 几何诊断。 */
struct SarGeometry {
    double      incidence_angle_deg = 0.0;
    std::string look_side;
    double      side_look_angle_deg = 0.0;
    double      squint_deg          = 0.0;
    double      roll_deg            = 0.0;
    double      pitch_deg           = 0.0;
    double      yaw_deg             = 0.0;
    double      slant_range_km      = 0.0;
    double      range_rate_mps      = 0.0;
    double      doppler_centroid_hz = 0.0;
    bool        los_clear           = false;
};

/** @brief 同一状态一次计算得到的 SAR 几何与权威机体姿态。 */
struct SarOrientation {
    SarGeometry           geometry;
    std::array<double, 4> quaternion_wxyz{1.0, 0.0, 0.0, 0.0};
};

/** @brief AC-010 SAR RSA 几何门与粗采样配置。 */
struct SarMergeOptions {
    double      step_sec          = 10.0;
    double      working_time_sec  = 0.0;
    double      incidence_min_deg = 0.0;
    double      incidence_max_deg = 0.0;
    std::string allowed_look_side;
    double      roll_max_deg          = 0.0;
    double      center_frequency_hz   = 0.0;
    double      azimuth_beamwidth_deg = 0.0;
    double      max_abs_squint_deg    = 0.0;
    std::optional<std::chrono::system_clock::time_point> expected_start;
    std::optional<std::chrono::system_clock::time_point> expected_end;
};

/** @brief SAR RSA 输出窗口及粗种子诊断。 */
struct SarAccessWindow {
    std::chrono::system_clock::time_point start;
    std::chrono::system_clock::time_point end;
    std::chrono::system_clock::time_point t0;
    double                                duration_sec = 0.0;
    SarGeometry                           geometry;
};

/** @brief SAR RSA 合并结果。 */
struct SarMergeResult {
    std::vector<SarAccessWindow> windows;
};

/**
 * @brief 严格解析 GMAT SAR 19-token 状态报告。
 * @param path 报告路径。
 * @return 时间严格递增的状态节点。
 * @throws std::runtime_error 文件缺失、token/数值/UTC 非法或节点不足。
 */
std::vector<SarStateNode> load_sar_state_report(
    const std::filesystem::path& path);

/**
 * @brief 计算单节点 SAR 几何。
 * @param node 同一 EarthMJ2000Eq 时刻的卫星、目标和法线辅助点状态。
 * @param center_frequency_hz 雷达中心频率（Hz，>0）。
 * @return 入射角、左右视、LOS 投影角、实际 squint、机械 roll/pitch/yaw、
 *         斜距、距离率和多普勒。
 * @throws std::runtime_error 状态退化、非有限或频率非法。
 */
SarGeometry compute_sar_geometry(const SarStateNode& node,
                                 double              center_frequency_hz);

/**
 * @brief 计算共享 SAR 侧视几何、零-squint 机体系和权威四元数。
 * @param node 同一 EarthMJ2000Eq 时刻的卫星、目标和法线状态。
 * @param center_frequency_hz 雷达中心频率（Hz，>0）。
 * @return 几何诊断及 body→EarthMJ2000Eq、wxyz、w>=0 的单位四元数。
 * @throws SarCandidateInfeasibleError 相对速度投影等候选姿态退化。
 * @throws std::runtime_error 状态、法线、LVLH 或数值非法。
 */
SarOrientation compute_sar_orientation(const SarStateNode& node,
                                       double              center_frequency_hz);

/**
 * @brief 从严格状态报告生成 SAR RSA 长期候选窗。
 * @param state_path 19-token 状态报告。
 * @param options 显式 SAR 几何/beam/squint 约束、精确报告边界与时间模型。
 * @return 候选窗口；每窗 t0 为粗网格 min-|range_rate| 种子。
 * @throws std::runtime_error 报告或参数不满足合同。
 */
SarMergeResult merge_sar_windows(const std::filesystem::path& state_path,
                                 const SarMergeOptions&       options);

/**
 * @brief 将 SAR 窗口导出为 WindowSet JSON。
 * @param windows SAR 窗口。
 * @return ISO-8601 毫秒窗口数组，含 @c sar_geometry。
 */
nlohmann::json sar_windows_to_json(const std::vector<SarAccessWindow>& windows);

/**
 * @brief 将 SAR 几何诊断导出为 JSON。
 * @param geometry 几何诊断。
 * @return 字段和单位与 output schema 一致的对象。
 */
nlohmann::json sar_geometry_to_json(const SarGeometry& geometry);

}  // namespace mp
