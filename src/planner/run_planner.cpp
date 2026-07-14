#include "planner/run_planner.hpp"

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <random>
#include <sstream>

#include "geometry/window_merger.hpp"
#include "gmat/gmat_backend.hpp"
#include "planner/contact_windows.hpp"
#include "planner/validate.hpp"
#include "satellite/json_io.hpp"

using satellite::write_json_file;
using satellite::write_text_file;

namespace mp {

namespace {

std::string make_task_id() {
    const auto now =
        std::chrono::system_clock::now().time_since_epoch().count();
    std::ostringstream ss;
    ss << "task_" << now;
    return ss.str();
}

std::chrono::system_clock::time_point parse_contact_utc(std::string text) {
    const auto dot = text.find('.');
    if (dot != std::string::npos) { text = text.substr(0, dot); }
    std::tm            tm{};
    std::istringstream ss(text);
    ss >> std::get_time(&tm, "%d %b %Y %H:%M:%S");
    if (ss.fail()) {
        throw std::runtime_error("Unrecognized contact UTC time: " + text);
    }
    return std::chrono::system_clock::from_time_t(timegm(&tm));
}

double add_contact_durations(nlohmann::json& windows) {
    double total_sec = 0.0;
    for (auto& window : windows) {
        const auto start =
            parse_contact_utc(window.at("start_utc").get<std::string>());
        const auto end =
            parse_contact_utc(window.at("end_utc").get<std::string>());
        const double duration_sec =
            std::chrono::duration<double>(end - start).count();
        window["duration_sec"] = duration_sec;
        total_sec += duration_sec;
    }
    return total_sec;
}

}  // namespace

nlohmann::json make_manifest() {
    return {
        {"schema_version", "1.1"},
        {"name", "access.remote_sensing_access"},
        {"executable", "access-computer"},
        {"version", "0.1.0"},
        {"description",
         "Access windows compute CLI plugin for GMAT-based "
         "access/downlink/attitude tasks"},
        {"domain", "access"},
        {"safety_class", "planning_only"},
        {"commands", nlohmann::json::array({"manifest", "validate", "run"})},
        {"scenarios",
         nlohmann::json::array({"remote_sensing_access", "attitude_estimation",
                                "downlink_window"})},
        {"input_schema_path",
         "schemas/remote_sensing_access.input.schema.json"},
        {"output_schema_path",
         "schemas/remote_sensing_access.output.schema.json"},
        {"capabilities",
         {
             {"kind", "compute"},
             {"side_effect_class", "none"},
             {"relocatable", true},
             {"deterministic", true},
             {"idempotent", false},
             {"retryable", true},
             {"consumes", nlohmann::json::array({"GeoPoint", "OrbitState"})},
             {"produces", nlohmann::json::array({"WindowSet"})},
             {"hardware_tag", nullptr},
             {"timeout_sec", 1800},
             {"compensation", nullptr},
             {"cost_hint", {{"typical_latency_sec", 120}}},
             {"async", false},
             {"dry_run", true},
             {"cancel", "none"},
             {"requires_gmat", true},
             {"batch", false},
         }},
        {"resource_limits",
         {
             {"timeout_sec", 1800},
             {"max_parallel", 2},
             {"max_work_dir_mb", 1024},
         }},
        {"agent_hints",
         {
             {"when_to_use",
              "Compute optical access, attitude, or downlink windows before "
              "imaging or comm"},
             {"prerequisites", nlohmann::json::array({"orbit_state_current"})},
             {"typical_latency_sec", 120},
         }},
    };
}

nlohmann::json run_planner(const nlohmann::json& request,
                           const RunContext&     ctx) {
    const auto validation = validate_request(request);
    if (!validation.ok) { throw std::runtime_error(validation.message); }

    const auto scenario = request.at("task").at("scenario").get<std::string>();
    const auto task_id  = ctx.task_id.empty() ? make_task_id() : ctx.task_id;
    const auto request_id = request.contains("request_id")
                                ? request.at("request_id").get<std::string>()
                                : task_id;

    std::filesystem::create_directories(ctx.work_dir);
    write_json_file(ctx.work_dir / "request.json", request, true);

    if (ctx.dry_run) {
        nlohmann::json output = {
            {"task_id", task_id},
            {"request_id", request_id},
            {"tool_name", "access.remote_sensing_access"},
            {"status", "dry_run"},
            {"scenario", scenario},
            {"work_dir", ctx.work_dir.string()},
            {"validation", validation.details},
        };
        if (ctx.trace_id) { output["trace_id"] = *ctx.trace_id; }
        return output;
    }

    const auto  paths = resolve_gmat_paths(request);
    std::string script_text;
    if (scenario == "downlink_window") {
        script_text = render_downlink_script(request, ctx.work_dir);
    }
    else { script_text = render_optical_access_script(request, ctx.work_dir); }

    const auto script_path = ctx.work_dir / "rendered.script";
    write_text_file(script_path, script_text);

    const auto gmat_result = run_gmat_console(paths, script_path, ctx.work_dir);
    if (gmat_result.exit_code != 0) {
        throw std::runtime_error("GMAT console failed with exit code " +
                                 std::to_string(gmat_result.exit_code));
    }

    nlohmann::json output = {
        {"task_id", task_id},
        {"request_id", request_id},
        {"tool_name", "access.remote_sensing_access"},
        {"status", "succeeded"},
        {"scenario", scenario},
        {"reference_utc", request.at("task").value("start_time_utc", "")},
        {"artifacts",
         {
             {"script_path", gmat_result.script_path.string()},
             {"console_log", gmat_result.console_log.string()},
         }},
        {"warnings", nlohmann::json::array()},
    };

    if (scenario == "downlink_window") {
        const auto   contact_path = ctx.work_dir / "contact_intervals.txt";
        auto         parsed       = parse_contact_windows(contact_path);
        auto&        windows      = parsed.windows;
        const double total_sec    = add_contact_durations(windows);
        output["windows"]         = windows;
        output["summary"]         = {{"window_count", windows.size()},
                                     {"duration_total_sec", total_sec}};
        for (const auto& w : parsed.warnings) {
            output["warnings"].push_back(w);
        }
        output["artifacts"]["contact_path"] =
            std::filesystem::absolute(contact_path).string();
        if (windows.empty()) { output["status"] = "no_result"; }
        if (ctx.trace_id) { output["trace_id"] = *ctx.trace_id; }
        write_json_file(ctx.work_dir / "result.json", output, true);
        return output;
    }

    MergeOptions options;
    options.step_sec = request.at("task").at("step_sec").get<double>();
    // D6/D8：RSA/AE 应用 ±W/2；downlink 已提前 return。
    options.working_time_sec =
        request.at("task").at("working_time_sec").get<double>();
    if (request.contains("constraints")) {
        const auto& c = request.at("constraints");
        if (c.contains("exclude_penumbra")) {
            options.exclude_penumbra = c.at("exclude_penumbra").get<bool>();
        }
        if (c.contains("require_sunlit")) {
            options.require_sunlit = c.at("require_sunlit").get<bool>();
        }
    }

    const auto windows = merge_optical_windows(
        gmat_result.trace_path, gmat_result.eclipse_path, options);
    double total_sec = 0.0;
    for (const auto& w : windows) { total_sec += w.duration_sec; }

    output["windows"]                     = windows_to_json(windows);
    output["summary"]                     = {{"window_count", windows.size()},
                                             {"duration_total_sec", total_sec}};
    output["artifacts"]["trace_path"]     = gmat_result.trace_path.string();
    output["artifacts"]["eclipse_path"]   = gmat_result.eclipse_path.string();
    output["artifacts"]["ephemeris_path"] = gmat_result.ephemeris_csv.string();

    if (windows.empty()) { output["status"] = "no_result"; }

    if (ctx.trace_id) { output["trace_id"] = *ctx.trace_id; }

    if (scenario == "attitude_estimation" && !windows.empty()) {
        const auto mode = request.at("sensor").value("mode", "side_roll_only");
        output["attitude"] = {
            {"mode", mode},
            {"t0_utc", windows.front().t0_utc},
            {"phi_deg", windows.front().phi_deg},
            {"pitch_deg", 0.0},
            {"pitch_status", "computed"},
        };
        if (mode == "stare") {
            output["attitude"]["pitch_deg"] =
                windows.front().min_off_nadir_deg * 0.1;
            output["attitude"]["pitch_status"] = "placeholder";
            output["warnings"].push_back(
                "Stare pitch_deg is a placeholder estimate in v0.1.0; use "
                "phi_deg/t0_utc as the validated outputs");
        }
    }

    write_json_file(ctx.work_dir / "result.json", output, true);
    return output;
}

}  // namespace mp
