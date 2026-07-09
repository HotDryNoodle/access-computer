#pragma once

#include <filesystem>
#include <string>

#include <nlohmann/json.hpp>

namespace mp {

struct GmatPaths {
    std::filesystem::path install_root;
    std::filesystem::path console_binary;
};

struct GmatRunResult {
    int exit_code = 0;
    std::filesystem::path script_path;
    std::filesystem::path console_log;
    std::filesystem::path trace_path;
    std::filesystem::path eclipse_path;
    std::filesystem::path ephemeris_csv;
    std::string stdout_text;
    std::string stderr_text;
};

GmatPaths resolve_gmat_paths(const nlohmann::json& request);
std::string render_optical_access_script(const nlohmann::json& request,
                                         const std::filesystem::path& work_dir);
std::string render_downlink_script(const nlohmann::json& request,
                                   const std::filesystem::path& work_dir);
GmatRunResult run_gmat_console(const GmatPaths& paths,
                               const std::filesystem::path& script_path,
                               const std::filesystem::path& work_dir);

}  // namespace mp
