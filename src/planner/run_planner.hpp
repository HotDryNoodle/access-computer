#pragma once

#include <filesystem>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "geometry/attitude_solver.hpp"

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
 * @return 结果 JSON（含 @c status / @c windows / @c artifacts）。
 * @throws std::runtime_error validate 失败、GMAT 失败或姿态求解失败。
 */
nlohmann::json run_planner(const nlohmann::json& request,
                           const RunContext&     ctx);

/**
 * @brief 将 AE 精化结果写入 run 输出（AC-003 / AC-008 D3）。
 *
 * @c refined.ok=false（side_roll 超门限）时：@c status=no_result、空
 * @c windows[]、零 summary、删除 @c attitude、保留既有 @c artifacts 并追加
 * warning。成功时回写精化窗与 @c attitude。
 *
 * @param output 已含 artifacts/warnings 的 run 输出（就地修改）。
 * @param refined @c refine_attitude_windows 结果。
 * @param mode 传感器 mode（写入 attitude.mode）。
 */
void apply_attitude_estimation_result(nlohmann::json&             output,
                                      const AttitudeRefineResult& refined,
                                      const std::string&          mode);

/**
 * @brief 将 run 输出 status 映射为 CLI 退出码（与 main 一致）。
 * @param output run_planner 结果。
 * @return @c EXIT_NO_RESULT(4) 若 status=no_result，否则 @c EXIT_OK(0)。
 */
int run_status_exit_code(const nlohmann::json& output);

}  // namespace mp
