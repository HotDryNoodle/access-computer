#pragma once

#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

/**
 * @file contact_windows.hpp
 * @brief 解析 GMAT ContactLocator 下行接触报告（AC-007）。
 */

namespace mp {

/** @brief 接触报告解析结果。 */
struct ContactParseResult {
    nlohmann::json           windows = nlohmann::json::array();
    std::vector<std::string> warnings;
};

/**
 * @brief 解析 contact_intervals.txt。
 *
 * 支持 SiteViewMaxElevationReport（含 max_elevation_deg）与 Legacy。
 * 峰值须 isfinite 且落在 [0, 90]；否则省略字段并产生 warning。
 */
ContactParseResult parse_contact_windows(const std::filesystem::path& path);

}  // namespace mp
