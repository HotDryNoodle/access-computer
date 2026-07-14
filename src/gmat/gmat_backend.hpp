#pragma once

#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>

/**
 * @file gmat_backend.hpp
 * @brief GMAT 脚本渲染与控制台调用。
 *
 * 时间单位：@c compute_horizon_sec 、@c working_time_sec 、@c step_sec 均为秒；
 * GMAT 快进使用派生 @c delta_prop_sec（start−epoch）；传播总长为
 * (@c delta_prop_sec + @c compute_horizon_sec) / 86400 天。
 * 角度单位：字段后缀 @c _deg 为度。
 */

namespace mp {

/** @brief GMAT 安装路径解析结果。 */
struct GmatPaths {
    std::filesystem::path install_root;
    std::filesystem::path console_binary;
};

/** @brief GMAT 控制台运行结果。 */
struct GmatRunResult {
    int                   exit_code = 0;
    std::filesystem::path script_path;
    std::filesystem::path console_log;
    std::filesystem::path trace_path;
    std::filesystem::path eclipse_path;
    std::filesystem::path ephemeris_csv;
    std::string           stdout_text;
    std::string           stderr_text;
};

/**
 * @brief 从请求 JSON 或环境变量解析 GMAT 路径。
 * @throws std::runtime_error 未设置 @c GMAT_ROOT 且请求无 @c gmat.install_root
 * 。
 */
GmatPaths resolve_gmat_paths(const nlohmann::json& request);

/** @brief 渲染光学访问 GMAT 脚本。 */
std::string render_optical_access_script(const nlohmann::json&        request,
                                         const std::filesystem::path& work_dir);

/** @brief 渲染下行窗口 GMAT 脚本。 */
std::string render_downlink_script(const nlohmann::json&        request,
                                   const std::filesystem::path& work_dir);

/** @brief 调用 GMAT 控制台执行脚本。 */
GmatRunResult run_gmat_console(const GmatPaths&             paths,
                               const std::filesystem::path& script_path,
                               const std::filesystem::path& work_dir);

}  // namespace mp
