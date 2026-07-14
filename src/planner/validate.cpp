#include "planner/validate.hpp"

#include <algorithm>
#include <cmath>

#include "planner/time_model.hpp"

namespace mp {

namespace {

constexpr double kAccessDefaultHorizonSec     = 172800.0;
constexpr double kAccessDefaultWorkingTimeSec = 200.0;
constexpr double kAccessDefaultStepSec        = 10.0;
constexpr double kAccessDefaultRollMaxDeg     = 30.0;
constexpr double kAttitudeMinHorizonSec       = 300.0;
constexpr double kAttitudeMaxHorizonSec       = 1800.0;
constexpr double kAttitudeDefaultStepSec      = 1.0;
constexpr double kDownlinkMinHorizonSec       = 3600.0;
constexpr double kDownlinkMaxHorizonSec       = 7200.0;
constexpr double kDownlinkDefaultStepSec      = 5.0;
/** AC-007：默认 cone 80° → MinimumElevationAngle 10°。 */
constexpr double kDownlinkDefaultConeDeg = 80.0;

bool has_number(const nlohmann::json& obj, const char* key) {
    return obj.contains(key) && obj[key].is_number();
}

bool has_string(const nlohmann::json& obj, const char* key) {
    return obj.contains(key) && obj[key].is_string() &&
           !obj[key].get<std::string>().empty();
}

bool has_bool(const nlohmann::json& obj, const char* key) {
    return obj.contains(key) && obj[key].is_boolean();
}

double get_number_or(const nlohmann::json& obj,
                     const char*           key,
                     double                fallback) {
    if (obj.contains(key) && obj[key].is_number()) {
        return obj[key].get<double>();
    }
    return fallback;
}

bool experimental_allows_sar(const nlohmann::json& request) {
    return request.contains("experimental") &&
           request["experimental"].is_object() &&
           request["experimental"].value("allow_sar", false);
}

}  // namespace

ValidationResult validate_request(const nlohmann::json& request) {
    ValidationResult result;
    result.details = nlohmann::json::object();

    const auto fail = [&](const std::string& msg) {
        result.ok                = false;
        result.message           = msg;
        result.details["errors"] = nlohmann::json::array({msg});
        return result;
    };

    if (!request.is_object()) {
        return fail("Request root must be a JSON object");
    }
    if (!request.contains("task") || !request["task"].is_object()) {
        return fail("Missing object field: task");
    }
    if (!request.contains("spacecraft") || !request["spacecraft"].is_object()) {
        return fail("Missing object field: spacecraft");
    }
    if (!request.contains("target") || !request["target"].is_object()) {
        return fail("Missing object field: target");
    }
    if (!request.contains("constraints") ||
        !request["constraints"].is_object()) {
        return fail("Missing object field: constraints");
    }

    const auto& task = request["task"];
    if (task.contains("duration_sec") || task.contains("elapsed_start_sec")) {
        return fail(
            "Obsolete task fields duration_sec/elapsed_start_sec removed "
            "(AC-004); use compute_horizon_sec and working_time_sec");
    }
    if (request["constraints"].contains("task_duration_sec")) {
        return fail(
            "Obsolete constraints.task_duration_sec removed (AC-004); use "
            "task.working_time_sec");
    }

    if (!has_string(task, "scenario")) {
        return fail("task.scenario is required");
    }
    const auto scenario = task["scenario"].get<std::string>();
    if (scenario != "remote_sensing_access" &&
        scenario != "attitude_estimation" && scenario != "downlink_window") {
        return fail("Unsupported task.scenario: " + scenario);
    }
    if (!has_string(task, "start_time_utc")) {
        return fail("task.start_time_utc is required");
    }
    if (!has_number(task, "compute_horizon_sec") ||
        task["compute_horizon_sec"].get<double>() <= 0) {
        return fail("task.compute_horizon_sec must be > 0");
    }
    if (!has_number(task, "working_time_sec") ||
        task["working_time_sec"].get<double>() <= 0) {
        return fail("task.working_time_sec must be > 0");
    }
    if (!has_number(task, "step_sec") || task["step_sec"].get<double>() <= 0) {
        return fail("task.step_sec must be > 0");
    }

    const double horizon_sec = task["compute_horizon_sec"].get<double>();
    const double working_sec = task["working_time_sec"].get<double>();
    const double step_sec    = task["step_sec"].get<double>();

    if (working_sec > horizon_sec) {
        return fail(
            "task.working_time_sec must be <= task.compute_horizon_sec "
            "(working window cannot exceed compute horizon)");
    }

    const auto& spacecraft = request["spacecraft"];
    if (!has_string(spacecraft, "sat_id")) {
        return fail("spacecraft.sat_id is required");
    }
    if (!has_string(spacecraft, "epoch_utc")) {
        return fail("spacecraft.epoch_utc is required");
    }
    if (!has_string(spacecraft, "state_type") ||
        spacecraft["state_type"].get<std::string>() != "keplerian") {
        return fail("spacecraft.state_type must be keplerian in v0.1.0");
    }
    if (!spacecraft.contains("elements") ||
        !spacecraft["elements"].is_object()) {
        return fail("spacecraft.elements is required");
    }

    const auto start_utc = task["start_time_utc"].get<std::string>();
    const auto epoch_utc = spacecraft["epoch_utc"].get<std::string>();
    if (!parse_iso8601_utc(start_utc)) {
        return fail(
            "task.start_time_utc must be ISO-8601 UTC ending with Z "
            "(e.g. 2026-12-30T03:37:00Z)");
    }
    if (!parse_gmat_utcgregorian(epoch_utc)) {
        return fail(
            "spacecraft.epoch_utc must be GMAT UTCGregorian "
            "(e.g. 30 Dec 2026 00:00:00.000)");
    }
    const auto delta_opt = delta_prop_sec(start_utc, epoch_utc);
    if (!delta_opt) {
        return fail(
            "Failed to derive delta_prop_sec from start_time_utc and "
            "epoch_utc");
    }
    const double delta_prop = *delta_opt;
    if (delta_prop < -kTimeToleranceSec) {
        return fail(
            "task.start_time_utc is before spacecraft.epoch_utc "
            "(delta_prop_sec=" +
            std::to_string(delta_prop) + "); v1.0 requires start >= epoch");
    }

    const auto profile =
        spacecraft.value("propagation_profile", "sso_j2_default");
    if (profile != "sso_j2_default" && profile != "segmented_drag_profile") {
        return fail("Unsupported spacecraft.propagation_profile: " + profile);
    }
    if (profile == "segmented_drag_profile") {
        return fail(
            "spacecraft.propagation_profile segmented_drag_profile is reserved "
            "for future segmented drag/attitude models");
    }

    const auto& target = request["target"];
    if (!has_number(target, "lon_deg") ||
        target["lon_deg"].get<double>() < -180 ||
        target["lon_deg"].get<double>() > 180) {
        return fail("target.lon_deg must be in [-180, 180]");
    }
    if (!has_number(target, "lat_deg") ||
        target["lat_deg"].get<double>() < -90 ||
        target["lat_deg"].get<double>() > 90) {
        return fail("target.lat_deg must be in [-90, 90]");
    }
    if (!has_string(target, "type")) { return fail("target.type is required"); }

    const auto& constraints = request["constraints"];

    if (scenario == "downlink_window") {
        if (target["type"].get<std::string>() != "ground_station") {
            return fail(
                "downlink_window requires target.type = ground_station");
        }
        if (horizon_sec < kDownlinkMinHorizonSec ||
            horizon_sec > kDownlinkMaxHorizonSec) {
            return fail(
                "downlink_window task.compute_horizon_sec must be within "
                "[3600, 7200] seconds (1-2 hours)");
        }
        if (step_sec < 0.1 || step_sec > 60.0) {
            return fail(
                "downlink_window task.step_sec is expected around 5 seconds");
        }
        const auto cone = get_number_or(constraints, "cone_angle_deg",
                                        kDownlinkDefaultConeDeg);
        if (cone <= 0 || cone > 90) {
            return fail(
                "constraints.cone_angle_deg must be in (0, 90] for "
                "downlink_window");
        }
        result.details["downlink"] = {
            {"cone_angle_deg", cone},
            {"min_elevation_deg", 90.0 - cone},
        };
        if (request.contains("sensor")) {
            if (!request["sensor"].is_object()) {
                return fail("sensor must be an object when provided");
            }
            const auto sensor_type = request["sensor"].value("type", "");
            if (!sensor_type.empty() && sensor_type != "downlink_cone") {
                return fail(
                    "downlink_window sensor.type must be downlink_cone when "
                    "provided");
            }
        }
    }
    else {
        if (!request.contains("sensor") || !request["sensor"].is_object()) {
            return fail("Missing object field: sensor");
        }
        const auto& sensor = request["sensor"];
        if (!has_string(sensor, "type")) {
            return fail("sensor.type is required");
        }
        const auto sensor_type = sensor["type"].get<std::string>();

        if (sensor_type == "sar") {
            if (!experimental_allows_sar(request)) {
                return fail(
                    "sensor.type=sar is not implemented in v0.1.0; set "
                    "experimental.allow_sar=true only for contract testing");
            }
        }
        else if (sensor_type == "optical_linescan") {
            const auto mode = sensor.value("mode", "side_roll_only");
            if (mode != "side_roll_only") {
                return fail(
                    "optical_linescan currently supports "
                    "sensor.mode=side_roll_only");
            }
        }
        else if (sensor_type == "optical_area_array") {
            const auto mode = sensor.value("mode", "stare");
            if (mode != "stare" && mode != "side_roll_only") {
                return fail(
                    "optical_area_array supports sensor.mode in {stare, "
                    "side_roll_only}");
            }
        }
        else { return fail("Unsupported sensor.type: " + sensor_type); }

        if (target["type"].get<std::string>() != "ground_point") {
            return fail(scenario + " requires target.type = ground_point");
        }

        if (constraints.contains("roll_max_deg") &&
            constraints["roll_max_deg"].get<double>() <= 0) {
            return fail("constraints.roll_max_deg must be > 0");
        }

        if (scenario == "remote_sensing_access") {
            if (horizon_sec < 60.0 || horizon_sec > 604800.0) {
                return fail(
                    "remote_sensing_access task.compute_horizon_sec is "
                    "expected around 172800 seconds (2 days)");
            }
            if (step_sec < 0.1 || step_sec > 120.0) {
                return fail(
                    "remote_sensing_access task.step_sec is expected around 10 "
                    "seconds");
            }
            if (sensor_type == "optical_linescan" ||
                sensor_type == "optical_area_array") {
                if (!has_bool(constraints, "require_sunlit")) {
                    result.details["warnings"] = nlohmann::json::array(
                        {"Optical sensors should set "
                         "constraints.require_sunlit=true"});
                }
            }
        }

        if (scenario == "attitude_estimation") {
            if (horizon_sec < kAttitudeMinHorizonSec ||
                horizon_sec > kAttitudeMaxHorizonSec) {
                return fail(
                    "attitude_estimation task.compute_horizon_sec must be "
                    "within [300, 1800] seconds (5-30 minutes)");
            }
            if (step_sec < 0.1 || step_sec > 10.0) {
                return fail(
                    "attitude_estimation task.step_sec is expected around 1 "
                    "second");
            }
            if (sensor_type != "optical_area_array" &&
                sensor_type != "optical_linescan") {
                return fail(
                    "attitude_estimation currently supports optical sensors "
                    "only");
            }
        }
    }

    result.ok              = true;
    result.message         = "ok";
    nlohmann::json details = {
        {"scenario", scenario},
        {"estimated_samples",
         static_cast<int>(std::ceil(horizon_sec / step_sec))},
        {"requires_gmat", true},
        {"time_model",
         {
             {"epoch_utc", epoch_utc},
             {"start_time_utc", start_utc},
             {"compute_horizon_sec", horizon_sec},
             {"working_time_sec", working_sec},
             {"delta_prop_sec", delta_prop},
             {"tolerance_sec", kTimeToleranceSec},
         }},
        {"defaults",
         {
             {"remote_sensing_access",
              {{"compute_horizon_sec", kAccessDefaultHorizonSec},
               {"working_time_sec", kAccessDefaultWorkingTimeSec},
               {"step_sec", kAccessDefaultStepSec},
               {"roll_max_deg", kAccessDefaultRollMaxDeg}}},
             {"attitude_estimation",
              {{"compute_horizon_sec", kAttitudeMaxHorizonSec},
               {"working_time_sec", kAttitudeMaxHorizonSec},
               {"step_sec", kAttitudeDefaultStepSec}}},
             {"downlink_window",
              {{"compute_horizon_sec", kDownlinkMaxHorizonSec},
               {"working_time_sec", kDownlinkMaxHorizonSec},
               {"step_sec", kDownlinkDefaultStepSec},
               {"cone_angle_deg", kDownlinkDefaultConeDeg}}},
         }},
    };
    if (result.details.contains("warnings")) {
        details["warnings"] = result.details["warnings"];
    }
    if (result.details.contains("downlink")) {
        details["downlink"] = result.details["downlink"];
    }
    result.details = std::move(details);
    if (spacecraft.contains("propagation_profile")) {
        result.details["propagation_profile"] =
            spacecraft["propagation_profile"];
    }
    return result;
}

}  // namespace mp
