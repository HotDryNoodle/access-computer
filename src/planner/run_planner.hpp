#pragma once

#include <filesystem>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

/**
 * @file run_planner.hpp
 * @brief access-computer 规划入口。
 */

namespace mp {

/** @brief run 子命令上下文。 */
struct RunContext {
    std::filesystem::path      work_dir;
    bool                       dry_run = false;
    std::string                task_id;
    std::optional<std::string> trace_id;
};

/** @brief 返回插件 manifest JSON。 */
nlohmann::json make_manifest();

/**
 * @brief 执行访问窗口规划。
 * @param request 已校验的请求 JSON。
 * @param ctx 运行上下文（含 dry_run）。
 */
nlohmann::json run_planner(const nlohmann::json& request,
                           const RunContext&     ctx);

}  // namespace mp
