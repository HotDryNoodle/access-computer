#pragma once

#include <filesystem>
#include <string>

#include <nlohmann/json.hpp>

namespace mp {

struct RunContext {
    std::filesystem::path work_dir;
    bool dry_run = false;
    std::string task_id;
    std::optional<std::string> trace_id;
};

nlohmann::json make_manifest();
nlohmann::json run_planner(const nlohmann::json& request, const RunContext& ctx);

}  // namespace mp
