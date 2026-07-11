#pragma once

#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

/**
 * @file window_merger.hpp
 * @brief 光学访问窗口合并与 JSON 导出。
 *
 * 时间：@c start_utc / @c end_utc 为 GMAT 轨迹 UTC 字符串；
 * @c duration_sec 为秒。
 * 角度：@c phi_deg 、@c min_off_nadir_deg 、@c max_sun_elevation_deg 为度。
 */

namespace mp {

/** @brief 合并后的访问窗口。 */
struct AccessWindow {
    std::string start_utc;
    std::string end_utc;
    double      duration_sec = 0.0;
    std::string t0_utc;
    double      phi_deg = 0.0;
    std::string pass_type;
    double      min_off_nadir_deg     = 0.0;
    double      max_sun_elevation_deg = 0.0;
};

/** @brief 窗口合并选项。 */
struct MergeOptions {
    /** @brief 采样步长（秒）。 */
    double step_sec         = 10.0;
    bool   exclude_penumbra = false;
    bool   require_sunlit   = true;
};

/**
 * @brief 从 GMAT 轨迹与食带文件合并光学访问窗口。
 * @param trace_path GMAT 采样轨迹文件。
 * @param eclipse_path 食带区间文件（不存在则跳过食带过滤）。
 * @param options 合并选项。
 */
std::vector<AccessWindow> merge_optical_windows(
    const std::filesystem::path& trace_path,
    const std::filesystem::path& eclipse_path,
    const MergeOptions&          options);

/** @brief 将窗口列表转为 JSON 数组。 */
nlohmann::json windows_to_json(const std::vector<AccessWindow>& windows);

}  // namespace mp
