#include "planner/validate.hpp"

#include <algorithm>
#include <cmath>

namespace mp {

namespace {

constexpr double kAccessDefaultDurationSec = 172800.0;
constexpr double kAccessDefaultStepSec = 10.0;
constexpr double kAccessDefaultRollMaxDeg = 30.0;
constexpr double kAttitudeMinDurationSec = 300.0;
constexpr double kAttitudeMaxDurationSec = 1800.0;
constexpr double kAttitudeDefaultStepSec = 1.0;
constexpr double kDownlinkMinDurationSec = 3600.0;
constexpr double kDownlinkMaxDurationSec = 7200.0;
constexpr double kDownlinkDefaultStepSec = 5.0;
constexpr double kDownlinkDefaultConeDeg = 65.0;

bool has_number(const nlohmann::json& obj, const char* key) {
    return obj.contains(key) && obj[key].is_number();
}

bool has_string(const nlohmann::json& obj, const char* key) {
    return obj.contains(key) && obj[key].is_string() && !obj[key].get<std::string>().empty();
}

bool has_bool(const nlohmann::json& obj, const char* key) {
    return obj.contains(key) && obj[key].is_boolean();
}

double get_number_or(const nlohmann::json& obj, const char* key, double fallback) {
    if (obj.contains(key) && obj[key].is_number()) {
        return obj[key].get<double>();
    }
    return fallback;
}

bool experimental_allows_sar(const nlohmann::json& request) {
    return request.contains("experimental") && request["experimental"].is_object() &&
           request["experimental"].value("allow_sar", false);
}

}  // namespace

ValidationResult validate_request(const nlohmann::json& request) {
    ValidationResult result;
    result.details = nlohmann::json::object();

    const auto fail = [&](const std::string& msg) {
        result.ok = false;
        result.message = msg;
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
    if (!request.contains("constraints") || !request["constraints"].is_object()) {
        return fail("Missing object field: constraints");
    }

    const auto& task = request["task"];
    if (!has_string(task, "scenario")) {
        return fail("task.scenario is required");
    }
    const auto scenario = task["scenario"].get<std::string>();
    if (scenario != "remote_sensing_access" && scenario != "attitude_estimation" &&
        scenario != "downlink_window") {
        return fail("Unsupported task.scenario: " + scenario);
    }
    if (!has_string(task, "start_time_utc")) {
        return fail("task.start_time_utc is required");
    }
    if (!has_number(task, "duration_sec") || task["duration_sec"].get<double>() <= 0) {
        return fail("task.duration_sec must be > 0");
    }
    if (!has_number(task, "step_sec") || task["step_sec"].get<double>() <= 0) {
        return fail("task.step_sec must be > 0");
    }

    const double duration_sec = task["duration_sec"].get<double>();
    const double step_sec = task["step_sec"].get<double>();

    const auto& spacecraft = request["spacecraft"];
    if (!has_string(spacecraft, "sat_id")) {
        return fail("spacecraft.sat_id is required");
    }
    if (!has_string(spacecraft, "epoch_utc")) {
        return fail("spacecraft.epoch_utc is required");
    }
    if (!has_string(spacecraft, "state_type") || spacecraft["state_type"].get<std::string>() != "keplerian") {
        return fail("spacecraft.state_type must be keplerian in v0.1.0");
    }
    if (!spacecraft.contains("elements") || !spacecraft["elements"].is_object()) {
        return fail("spacecraft.elements is required");
    }

  const auto profile = spacecraft.value("propagation_profile", "sso_j2_default");
    if (profile != "sso_j2_default" && profile != "segmented_drag_profile") {
        return fail("Unsupported spacecraft.propagation_profile: " + profile);
    }
    if (profile == "segmented_drag_profile") {
        return fail("spacecraft.propagation_profile segmented_drag_profile is reserved for future segmented drag/attitude models");
    }

    const auto& target = request["target"];
    if (!has_number(target, "lon_deg") || target["lon_deg"].get<double>() < -180 ||
        target["lon_deg"].get<double>() > 180) {
        return fail("target.lon_deg must be in [-180, 180]");
    }
    if (!has_number(target, "lat_deg") || target["lat_deg"].get<double>() < -90 ||
        target["lat_deg"].get<double>() > 90) {
        return fail("target.lat_deg must be in [-90, 90]");
    }
    if (!has_string(target, "type")) {
        return fail("target.type is required");
    }

    const auto& constraints = request["constraints"];

    if (scenario == "downlink_window") {
        if (target["type"].get<std::string>() != "ground_station") {
            return fail("downlink_window requires target.type = ground_station");
        }
        if (duration_sec < kDownlinkMinDurationSec || duration_sec > kDownlinkMaxDurationSec) {
            return fail("downlink_window task.duration_sec must be within [3600, 7200] seconds (1-2 hours)");
        }
        if (step_sec < 0.1 || step_sec > 60.0) {
            return fail("downlink_window task.step_sec is expected around 5 seconds");
        }
        const auto cone = get_number_or(constraints, "cone_angle_deg", kDownlinkDefaultConeDeg);
        if (cone <= 0 || cone > 90) {
            return fail("constraints.cone_angle_deg must be in (0, 90] for downlink_window");
        }
        if (request.contains("sensor")) {
            if (!request["sensor"].is_object()) {
                return fail("sensor must be an object when provided");
            }
            const auto sensor_type = request["sensor"].value("type", "");
            if (!sensor_type.empty() && sensor_type != "downlink_cone") {
                return fail("downlink_window sensor.type must be downlink_cone when provided");
            }
        }
    } else {
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
                return fail("sensor.type=sar is not implemented in v0.1.0; set experimental.allow_sar=true only for contract testing");
            }
        } else if (sensor_type == "optical_linescan") {
            const auto mode = sensor.value("mode", "side_roll_only");
            if (mode != "side_roll_only") {
                return fail("optical_linescan currently supports sensor.mode=side_roll_only");
            }
        } else if (sensor_type == "optical_area_array") {
            const auto mode = sensor.value("mode", "stare");
            if (mode != "stare" && mode != "side_roll_only") {
                return fail("optical_area_array supports sensor.mode in {stare, side_roll_only}");
            }
        } else {
            return fail("Unsupported sensor.type: " + sensor_type);
        }

        if (target["type"].get<std::string>() != "ground_point") {
            return fail(scenario + " requires target.type = ground_point");
        }

        if (constraints.contains("roll_max_deg") && constraints["roll_max_deg"].get<double>() <= 0) {
            return fail("constraints.roll_max_deg must be > 0");
        }

        if (scenario == "remote_sensing_access") {
            if (duration_sec < 60.0 || duration_sec > 604800.0) {
                return fail("remote_sensing_access task.duration_sec is expected around 172800 seconds (2 days)");
            }
            if (step_sec < 0.1 || step_sec > 120.0) {
                return fail("remote_sensing_access task.step_sec is expected around 10 seconds");
            }
            if (sensor_type == "optical_linescan" || sensor_type == "optical_area_array") {
                if (!has_bool(constraints, "require_sunlit")) {
                    result.details["warnings"] = nlohmann::json::array(
                        {"Optical sensors should set constraints.require_sunlit=true"});
                }
            }
        }

        if (scenario == "attitude_estimation") {
            if (duration_sec < kAttitudeMinDurationSec || duration_sec > kAttitudeMaxDurationSec) {
                return fail("attitude_estimation task.duration_sec must be within [300, 1800] seconds (5-30 minutes)");
            }
            if (step_sec < 0.1 || step_sec > 10.0) {
                return fail("attitude_estimation task.step_sec is expected around 1 second");
            }
            if (sensor_type != "optical_area_array" && sensor_type != "optical_linescan") {
                return fail("attitude_estimation currently supports optical sensors only");
            }
        }
    }

    result.ok = true;
    result.message = "ok";
    result.details = {
        {"scenario", scenario},
        {"estimated_samples", static_cast<int>(std::ceil(duration_sec / step_sec))},
        {"requires_gmat", true},
        {"defaults",
         {
             {"remote_sensing_access",
              {{"duration_sec", kAccessDefaultDurationSec},
               {"step_sec", kAccessDefaultStepSec},
               {"roll_max_deg", kAccessDefaultRollMaxDeg}}},
             {"attitude_estimation",
              {{"duration_sec", kAttitudeMaxDurationSec},
               {"step_sec", kAttitudeDefaultStepSec}}},
             {"downlink_window",
              {{"duration_sec", kDownlinkMaxDurationSec},
               {"step_sec", kDownlinkDefaultStepSec},
               {"cone_angle_deg", kDownlinkDefaultConeDeg}}},
         }},
    };
    if (spacecraft.contains("propagation_profile")) {
        result.details["propagation_profile"] = spacecraft["propagation_profile"];
    }
    return result;
}

}  // namespace mp
