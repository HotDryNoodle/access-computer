#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "geometry/window_merger.hpp"

/**
 * @file attitude_solver.hpp
 * @brief AC-008：EarthMJ2000Eq 临期姿态精化（亚秒 t0 + 真 pitch/roll）。
 *
 * 时间：UTCGregorian，内部相对星历首行秒；输出落在 1 ms 网格。
 * 坐标：EarthMJ2000Eq；位置 km；速度 km/s；角度 deg。
 * @warning 禁止 C++ 常角速度/地球自转近似；禁止对 pitch/roll
 * 直接线性插值当最终解。
 */

namespace mp {

/**
 * AC-008 D3：side_roll_only 窗内最优仍 |pitch|≥0.5°（勿改字）。
 * 调用方须置 @c status=no_result、空 @c windows[]、零 summary，不得发出
 * @c pitch_status=computed（见 @c apply_attitude_estimation_result）。
 */
inline constexpr const char* kSideRollPitchInfeasibleWarning =
    "side_roll_only pitch residual at or above 0.5 deg at refined t0; "
    "no feasible side-roll solution";

/** @brief 三维向量（km 或 km/s）。 */
struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/** @brief 有符号姿态角（D1）。 */
struct AttitudeAngles {
    double roll_deg  = 0.0; /**< @brief phi；LOS 朝 +C 为正。 */
    double pitch_deg = 0.0; /**< @brief 目标朝 +T 为正。 */
    double u_dot_d   = 0.0; /**< @brief u·D；须 >0。 */
    double u_dot_t   = 0.0; /**< @brief g=u·T。 */
    double u_dot_c   = 0.0;
};

/** @brief 星历节点（13-token 合同）。 */
struct EphemerisNode {
    double      t_sec = 0.0; /**< @brief 相对首行的秒。 */
    std::string utc;
    Vec3        r_sat;
    Vec3        v_sat;
    Vec3        r_tgt;
};

/**
 * @brief AE 精化结果。
 * @note @c ok=false 仅用于 side_roll
 * 产品门限失败（no_result）；星历/几何错误抛异常。
 */
struct AttitudeRefineResult {
    bool                      ok = true;
    std::vector<AccessWindow> windows;
    std::string               t0_utc;
    double                    phi_deg      = 0.0;
    double                    pitch_deg    = 0.0;
    std::string               pitch_status = "computed";
    std::size_t               best_index   = 0;
    std::vector<std::string>  warnings;
};

/**
 * @brief 由卫星 r/v 与目标 r 计算有符号 roll/pitch（EarthMJ2000Eq）。
 * @param r_sat 卫星位置（km）。
 * @param v_sat 卫星速度（km/s）。
 * @param r_tgt 目标位置（km）。
 * @return 姿态角。
 * @throws std::runtime_error 范数退化或 @c u·D≤0 / 非 finite。
 */
AttitudeAngles compute_attitude_angles(const Vec3& r_sat,
                                       const Vec3& v_sat,
                                       const Vec3& r_tgt);

/**
 * @brief 由 roll/pitch 与 LVLH 基重建 boresight 单位向量。
 * @param roll_deg 有符号 roll（deg）。
 * @param pitch_deg 有符号 pitch（deg）。
 * @param C 交叉航迹单位向量。
 * @param T 沿航迹单位向量。
 * @param D 天底单位向量。
 * @return 重建的单位 LOS。
 * @throws std::runtime_error 重建向量退化。
 */
Vec3 reconstruct_boresight(double      roll_deg,
                           double      pitch_deg,
                           const Vec3& C,
                           const Vec3& T,
                           const Vec3& D);

/**
 * @brief 构造 LVLH 基 (C,T,D)。
 * @param r_sat 卫星位置（km）。
 * @param v_sat 卫星速度（km/s）。
 * @param[out] C 交叉航迹。
 * @param[out] T 沿航迹。
 * @param[out] D 天底。
 * @throws std::runtime_error 范数或切向速度退化。
 */
void build_lvlh_frame(
    const Vec3& r_sat, const Vec3& v_sat, Vec3* C, Vec3* T, Vec3* D);

/**
 * @brief 严格解析 13-token 星历报告。
 * @param path 星历文件路径。
 * @return 节点序列（相对首行秒）。
 * @throws std::runtime_error 前缀 @c attitude solver: …（含行号）。
 */
std::vector<EphemerisNode> load_attitude_ephemeris(
    const std::filesystem::path& path);

/**
 * @brief Hermite 插值卫星状态 + 目标线性插值。
 * @param nodes 星历节点。
 * @param t_sec 相对首行秒。
 * @param[out] r_sat 卫星位置。
 * @param[out] v_sat 卫星速度。
 * @param[out] r_tgt 目标位置。
 * @throws std::runtime_error 覆盖外或节点不足。
 */
void evaluate_ephemeris_state(const std::vector<EphemerisNode>& nodes,
                              double                            t_sec,
                              Vec3*                             r_sat,
                              Vec3*                             v_sat,
                              Vec3*                             r_tgt);

/**
 * @brief 在裁剪后光学窗上精化 AE 姿态。
 * @param ephemeris_path GMAT 13-token 星历。
 * @param windows 非空裁剪后光学窗。
 * @param mode @c side_roll_only 或 @c stare。
 * @return 精化窗（仅改 t0_utc/phi_deg）+ 顶层 attitude；side_roll 超门限时
 *         @c ok=false 且含 @c kSideRollPitchInfeasibleWarning。
 * @throws std::runtime_error 星历/覆盖/几何/无合法毫秒候选（D8/D9）。
 */
AttitudeRefineResult refine_attitude_windows(
    const std::filesystem::path&     ephemeris_path,
    const std::vector<AccessWindow>& windows,
    const std::string&               mode);

/**
 * @brief 解析 GMAT UTCGregorian 为 Unix 秒（含毫秒）。
 * @param text 如 @c 30 Dec 2026 03:39:59.668。
 * @return Unix 秒（双精度）。
 * @throws std::runtime_error 非法日期、尾随 junk、非有限秒、leap :60。
 * @note 委托 @c parse_gmat_utcgregorian（完整消费 + calendar round-trip）。
 */
double parse_utc_gregorian_sec(const std::string& text);

/**
 * @brief 格式化为 UTCGregorian，毫秒三位（输入须已在 1 ms 网格）。
 * @param unix_sec Unix 秒。
 * @return GMAT UTCGregorian 字符串。
 * @throws std::runtime_error 非有限时间。
 */
std::string format_utc_gregorian_ms(double unix_sec);

}  // namespace mp
