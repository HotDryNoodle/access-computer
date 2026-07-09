#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace mp {

struct AccessWindow {
    std::string start_utc;
    std::string end_utc;
    double duration_sec = 0.0;
    std::string t0_utc;
    double phi_deg = 0.0;
    std::string pass_type;
    double min_off_nadir_deg = 0.0;
    double max_sun_elevation_deg = 0.0;
};

struct MergeOptions {
    double step_sec = 10.0;
    bool exclude_penumbra = false;
    bool require_sunlit = true;
};

std::vector<AccessWindow> merge_optical_windows(const std::filesystem::path& trace_path,
                                                const std::filesystem::path& eclipse_path,
                                                const MergeOptions& options);

nlohmann::json windows_to_json(const std::vector<AccessWindow>& windows);

}  // namespace mp
