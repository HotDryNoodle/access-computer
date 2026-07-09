#include "gmat/gmat_backend.hpp"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <sys/wait.h>

namespace mp {

namespace {

std::string replace_all(std::string text, const std::string& from, const std::string& to) {
    std::size_t pos = 0;
    while ((pos = text.find(from, pos)) != std::string::npos) {
        text.replace(pos, from.size(), to);
        pos += to.size();
    }
    return text;
}

std::string shell_quote(const std::filesystem::path& path) {
    std::string quoted = "'";
    for (const char ch : path.string()) {
        if (ch == '\'') {
            quoted += "'\\''";
        } else {
            quoted += ch;
        }
    }
    quoted += "'";
    return quoted;
}

double json_get_number(const nlohmann::json& obj, const char* key, double fallback) {
    if (obj.contains(key) && obj[key].is_number()) {
        return obj[key].get<double>();
    }
    return fallback;
}

std::string load_template(const std::filesystem::path& template_path) {
    std::ifstream in(template_path);
    if (!in) {
        throw std::runtime_error("Failed to open GMAT template: " + template_path.string());
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

std::filesystem::path default_gmat_root() {
    if (const char* env = std::getenv("GMAT_ROOT")) {
        return env;
    }
    return "/home/hotdry/projects/GMAT-R2026a/GMAT-R2026a-Linux-x64";
}

}  // namespace

GmatPaths resolve_gmat_paths(const nlohmann::json& request) {
    GmatPaths paths;
    if (request.contains("gmat") && request["gmat"].is_object()) {
        const auto& gmat = request["gmat"];
        if (gmat.contains("install_root")) {
            paths.install_root = gmat["install_root"].get<std::string>();
        }
        if (gmat.contains("console_binary")) {
            paths.console_binary = gmat["console_binary"].get<std::string>();
        }
    }
    if (paths.install_root.empty()) {
        paths.install_root = default_gmat_root();
    }
    if (paths.console_binary.empty()) {
        paths.console_binary = paths.install_root / "bin" / "GmatConsole-R2026a";
    }
    return paths;
}

std::string render_optical_access_script(const nlohmann::json& request,
                                         const std::filesystem::path& work_dir) {
    const auto template_path = std::filesystem::path(ACCESS_COMPUTER_SOURCE_ROOT) /
                               "templates" / "optical_access.script.in";
    auto script = load_template(template_path);

    const auto& task = request.at("task");
    const auto& spacecraft = request.at("spacecraft");
    const auto& target = request.at("target");
    const auto& elements = spacecraft.at("elements");
    const auto& constraints =
        request.contains("constraints") ? request.at("constraints") : nlohmann::json::object();

    const double duration_sec = task.at("duration_sec").get<double>();
    const double elapsed_start_sec =
        task.contains("elapsed_start_sec") ? task.at("elapsed_start_sec").get<double>() : 0.0;
    const double total_duration_days = (elapsed_start_sec + duration_sec) / 86400.0;
    const double step_sec = task.at("step_sec").get<double>();
    const double roll_max = json_get_number(constraints, "roll_max_deg", 30.0);
    const double tgt_lat = target.at("lat_deg").get<double>();
    const double tgt_lon = target.at("lon_deg").get<double>();
    const double tgt_alt = target.at("alt_km").get<double>();

    script = replace_all(script, "{{OUTPUT_DIR}}", std::filesystem::absolute(work_dir).string());
    script = replace_all(script, "{{EPOCH_UTC}}", spacecraft.at("epoch_utc").get<std::string>());
    script = replace_all(script, "{{SMA_KM}}", std::to_string(elements.at("sma_km").get<double>()));
    script = replace_all(script, "{{ECC}}", std::to_string(elements.at("ecc").get<double>()));
    script = replace_all(script, "{{INC_DEG}}", std::to_string(elements.at("inc_deg").get<double>()));
    script = replace_all(script, "{{RAAN_DEG}}", std::to_string(elements.at("raan_deg").get<double>()));
    script = replace_all(script, "{{AOP_DEG}}", std::to_string(elements.at("aop_deg").get<double>()));
    script = replace_all(script, "{{TA_DEG}}", std::to_string(elements.at("ta_deg").get<double>()));
    script = replace_all(script, "{{TOTAL_DURATION_DAYS}}", std::to_string(total_duration_days));
    script = replace_all(script, "{{ELAPSED_START_SEC}}", std::to_string(elapsed_start_sec));
    script = replace_all(script, "{{STEP_SEC}}", std::to_string(step_sec));
    script = replace_all(script, "{{ROLL_MAX_DEG}}", std::to_string(roll_max));
    script = replace_all(script, "{{TARGET_LAT}}", std::to_string(tgt_lat));
    script = replace_all(script, "{{TARGET_LON}}", std::to_string(tgt_lon));
    script = replace_all(script, "{{TARGET_ALT_KM}}", std::to_string(tgt_alt));
    return script;
}

std::string render_downlink_script(const nlohmann::json& request,
                                   const std::filesystem::path& work_dir) {
    const auto template_path = std::filesystem::path(ACCESS_COMPUTER_SOURCE_ROOT) /
                               "templates" / "downlink_contact.script.in";
    auto script = load_template(template_path);

    const auto& task = request.at("task");
    const auto& spacecraft = request.at("spacecraft");
    const auto& target = request.at("target");
    const auto& elements = spacecraft.at("elements");
    const auto& constraints =
        request.contains("constraints") ? request.at("constraints") : nlohmann::json::object();

    const double duration_sec = task.at("duration_sec").get<double>();
    const double elapsed_start_sec =
        task.contains("elapsed_start_sec") ? task.at("elapsed_start_sec").get<double>() : 0.0;
    const double total_duration_days = (elapsed_start_sec + duration_sec) / 86400.0;
    const double step_sec = task.at("step_sec").get<double>();
    const double cone_deg = json_get_number(constraints, "cone_angle_deg", 65.0);
    // Ground-station cone half-angle from zenith -> minimum elevation mask.
    const double min_elevation_deg = std::max(0.0, 90.0 - cone_deg);

    script = replace_all(script, "{{OUTPUT_DIR}}", std::filesystem::absolute(work_dir).string());
    script = replace_all(script, "{{EPOCH_UTC}}", spacecraft.at("epoch_utc").get<std::string>());
    script = replace_all(script, "{{SMA_KM}}", std::to_string(elements.at("sma_km").get<double>()));
    script = replace_all(script, "{{ECC}}", std::to_string(elements.at("ecc").get<double>()));
    script = replace_all(script, "{{INC_DEG}}", std::to_string(elements.at("inc_deg").get<double>()));
    script = replace_all(script, "{{RAAN_DEG}}", std::to_string(elements.at("raan_deg").get<double>()));
    script = replace_all(script, "{{AOP_DEG}}", std::to_string(elements.at("aop_deg").get<double>()));
    script = replace_all(script, "{{TA_DEG}}", std::to_string(elements.at("ta_deg").get<double>()));
    script = replace_all(script, "{{TOTAL_DURATION_DAYS}}", std::to_string(total_duration_days));
    script = replace_all(script, "{{ELAPSED_START_SEC}}", std::to_string(elapsed_start_sec));
    script = replace_all(script, "{{STEP_SEC}}", std::to_string(step_sec));
    script = replace_all(script, "{{STATION_LAT}}", std::to_string(target.at("lat_deg").get<double>()));
    script = replace_all(script, "{{STATION_LON}}", std::to_string(target.at("lon_deg").get<double>()));
    script = replace_all(script, "{{STATION_ALT_KM}}", std::to_string(target.at("alt_km").get<double>()));
    script = replace_all(script, "{{CONE_ANGLE_DEG}}", std::to_string(cone_deg));
    script = replace_all(script, "{{MIN_ELEVATION_DEG}}", std::to_string(min_elevation_deg));
    return script;
}

GmatRunResult run_gmat_console(const GmatPaths& paths,
                               const std::filesystem::path& script_path,
                               const std::filesystem::path& work_dir) {
    if (!std::filesystem::exists(paths.console_binary)) {
        throw std::runtime_error("GMAT console binary not found: " + paths.console_binary.string());
    }

    GmatRunResult result;
    const auto abs_work_dir = std::filesystem::absolute(work_dir);
    const auto abs_script_path = std::filesystem::absolute(script_path);
    result.script_path = abs_script_path;
    result.console_log = abs_work_dir / "gmat_console.log";
    result.trace_path = abs_work_dir / "sample_trace.txt";
    result.eclipse_path = abs_work_dir / "eclipse_intervals.txt";
    result.ephemeris_csv = abs_work_dir / "sat_rv_j2000.csv";

    const auto bin_dir = paths.install_root / "bin";
    const auto plugins_dir = paths.install_root / "plugins";
    std::ostringstream cmd;
    cmd << "cd " << shell_quote(bin_dir) << " && ";
    cmd << "LD_LIBRARY_PATH=" << shell_quote(bin_dir.string() + ":" + plugins_dir.string())
        << ":$LD_LIBRARY_PATH ";
    cmd << shell_quote(paths.console_binary) << " --run " << shell_quote(abs_script_path);
    cmd << " > " << shell_quote(result.console_log) << " 2>&1";

    const int status = std::system(cmd.str().c_str());
    result.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : status;
    return result;
}

}  // namespace mp
