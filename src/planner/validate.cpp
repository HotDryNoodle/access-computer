#include "planner/validate.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>

#include "planner/time_model.hpp"

namespace mp {

namespace {

constexpr double kAccessDefaultHorizonSec      = 172800.0;
constexpr double kAccessDefaultWorkingTimeSec  = 200.0;
constexpr double kAccessDefaultStepSec         = 10.0;
constexpr double kAccessDefaultRollMaxDeg      = 30.0;
constexpr double kAttitudeMinHorizonSec        = 300.0;
constexpr double kAttitudeMaxHorizonSec        = 1800.0;
constexpr double kAttitudeDefaultStepSec       = 1.0;
constexpr double kSarDefaultMaxAbsRangeRateMps = 0.1;
constexpr double kDownlinkMinHorizonSec        = 3600.0;
constexpr double kDownlinkMaxHorizonSec        = 7200.0;
constexpr double kDownlinkDefaultStepSec       = 5.0;
/** AC-007：默认 cone 80° → MinimumElevationAngle 10°。 */
constexpr double kDownlinkDefaultConeDeg = 80.0;

bool has_number(const nlohmann::json& obj, const char* key) {
    return obj.contains(key) && obj[key].is_number() &&
           std::isfinite(obj[key].get<double>());
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

std::string first_unknown_field(
    const nlohmann::json&              object,
    std::initializer_list<const char*> allowed_fields) {
    for (auto field = object.begin(); field != object.end(); ++field) {
        const bool known = std::any_of(
            allowed_fields.begin(), allowed_fields.end(),
            [&](const char* allowed) { return field.key() == allowed; });
        if (!known) { return field.key(); }
    }
    return {};
}

}  // namespace

bool has_illumination_constraint_flags(const nlohmann::json& constraints) {
    return constraints.contains("require_sunlit") ||
           constraints.contains("exclude_umbra") ||
           constraints.contains("exclude_penumbra");
}

OpticalIlluminationResolved resolve_optical_illumination(
    const nlohmann::json& request) {
    OpticalIlluminationResolved out;
    const bool                  is_sar = request.contains("sensor") &&
                        request["sensor"].is_object() &&
                        request["sensor"].value("type", "") == "sar";

    const nlohmann::json* constraints = nullptr;
    if (request.contains("constraints") && request["constraints"].is_object()) {
        constraints = &request["constraints"];
    }

    if (is_sar) {
        // D8：SAR 有效三旗全部关闭，不读取请求值。
        out.require_sunlit   = false;
        out.exclude_umbra    = false;
        out.exclude_penumbra = false;
        if (constraints != nullptr &&
            has_illumination_constraint_flags(*constraints)) {
            out.warnings.push_back(kIlluminationFlagsIgnoredSar);
        }
        return out;
    }

    if (constraints == nullptr) { return out; }
    const auto& c = *constraints;
    if (c.contains("exclude_umbra") && c["exclude_umbra"].is_boolean()) {
        out.exclude_umbra = c["exclude_umbra"].get<bool>();
    }
    if (c.contains("exclude_penumbra") && c["exclude_penumbra"].is_boolean()) {
        out.exclude_penumbra = c["exclude_penumbra"].get<bool>();
    }
    if (c.contains("require_sunlit") && c["require_sunlit"].is_boolean()) {
        out.require_sunlit = c["require_sunlit"].get<bool>();
    }
    return out;
}

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
        if (has_illumination_constraint_flags(constraints)) {
            result.details["warnings"] =
                nlohmann::json::array({kIlluminationFlagsIgnoredDownlink});
        }
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
            if (sensor.value("mode", "") != "stripmap") {
                return fail("SAR MVP requires sensor.mode=stripmap");
            }
            if (!has_number(sensor, "center_frequency_hz") ||
                sensor["center_frequency_hz"].get<double>() <= 0.0) {
                return fail(
                    "SAR MVP requires finite sensor.center_frequency_hz > 0");
            }
            if (!has_number(sensor, "azimuth_beamwidth_deg")) {
                return fail(
                    "SAR MVP requires finite sensor.azimuth_beamwidth_deg");
            }
            const double beamwidth_deg =
                sensor["azimuth_beamwidth_deg"].get<double>();
            if (!(beamwidth_deg > 0.0 && beamwidth_deg < 180.0)) {
                return fail(
                    "sensor.azimuth_beamwidth_deg must satisfy 0 < width < "
                    "180 deg");
            }
            if (!has_number(constraints, "incidence_min_deg") ||
                !has_number(constraints, "incidence_max_deg")) {
                return fail(
                    "SAR MVP requires finite constraints.incidence_min_deg "
                    "and constraints.incidence_max_deg");
            }
            const double incidence_min =
                constraints["incidence_min_deg"].get<double>();
            const double incidence_max =
                constraints["incidence_max_deg"].get<double>();
            if (!(incidence_min > 0.0 && incidence_min < incidence_max &&
                  incidence_max < 90.0)) {
                return fail(
                    "SAR incidence constraints must satisfy 0 < min < max < "
                    "90 deg");
            }
            if (!has_string(constraints, "allowed_look_side")) {
                return fail("SAR MVP requires constraints.allowed_look_side");
            }
            const auto allowed_side =
                constraints["allowed_look_side"].get<std::string>();
            if (allowed_side != "left" && allowed_side != "right" &&
                allowed_side != "either") {
                return fail(
                    "constraints.allowed_look_side must be left, right, or "
                    "either");
            }
            if (!has_number(constraints, "roll_max_deg") ||
                constraints["roll_max_deg"].get<double>() <= 0.0 ||
                constraints["roll_max_deg"].get<double>() > 90.0) {
                return fail(
                    "SAR MVP requires finite constraints.roll_max_deg in "
                    "(0, 90]");
            }
            double max_abs_squint_deg = beamwidth_deg / 2.0;
            if (constraints.contains("max_abs_squint_deg")) {
                if (!has_number(constraints, "max_abs_squint_deg")) {
                    return fail(
                        "constraints.max_abs_squint_deg must be finite when "
                        "provided");
                }
                max_abs_squint_deg =
                    constraints["max_abs_squint_deg"].get<double>();
                if (!(max_abs_squint_deg > 0.0 &&
                      max_abs_squint_deg <= beamwidth_deg / 2.0)) {
                    return fail(
                        "constraints.max_abs_squint_deg must satisfy 0 < "
                        "value <= sensor.azimuth_beamwidth_deg/2");
                }
            }
            result.details["sar"] = {
                {"mode", "stripmap"},
                {"center_frequency_hz",
                 sensor["center_frequency_hz"].get<double>()},
                {"incidence_min_deg", incidence_min},
                {"incidence_max_deg", incidence_max},
                {"allowed_look_side", allowed_side},
                {"roll_max_deg", constraints["roll_max_deg"].get<double>()},
                {"azimuth_beamwidth_deg", beamwidth_deg},
                {"max_abs_squint_deg", max_abs_squint_deg},
            };
            if (has_illumination_constraint_flags(constraints)) {
                result.details["warnings"] =
                    nlohmann::json::array({kIlluminationFlagsIgnoredSar});
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
            (!has_number(constraints, "roll_max_deg") ||
             constraints["roll_max_deg"].get<double>() <= 0)) {
            return fail("constraints.roll_max_deg must be finite and > 0");
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
            if (sensor_type == "sar") {
                if (!request.contains("selected_window") ||
                    !request["selected_window"].is_object()) {
                    return fail(
                        "SAR attitude_estimation requires selected_window");
                }
                const auto& selected         = request["selected_window"];
                const auto  unknown_selected = first_unknown_field(
                    selected, {"start_utc", "end_utc", "t0_utc", "duration_sec",
                                "phi_deg", "pass_type", "min_off_nadir_deg",
                                "max_sun_elevation_deg", "max_elevation_deg",
                                "node_id", "sar_geometry"});
                if (!unknown_selected.empty()) {
                    return fail("selected_window contains unknown field: " +
                                unknown_selected);
                }
                if (selected.contains("sar_geometry")) {
                    if (!selected["sar_geometry"].is_object()) {
                        return fail(
                            "selected_window.sar_geometry must be an "
                            "object when provided");
                    }
                    const auto unknown_geometry = first_unknown_field(
                        selected["sar_geometry"],
                        {"incidence_angle_deg", "look_side",
                         "side_look_angle_deg", "squint_deg", "roll_deg",
                         "pitch_deg", "yaw_deg", "slant_range_km",
                         "range_rate_mps", "doppler_centroid_hz", "los_clear"});
                    if (!unknown_geometry.empty()) {
                        return fail(
                            "selected_window.sar_geometry contains unknown "
                            "field: " +
                            unknown_geometry);
                    }
                }
                if (!has_string(selected, "start_utc") ||
                    !has_string(selected, "end_utc") ||
                    !has_string(selected, "t0_utc")) {
                    return fail(
                        "selected_window requires start_utc, end_utc, and "
                        "t0_utc");
                }
                const auto selected_start =
                    parse_iso8601_utc(selected["start_utc"].get<std::string>());
                const auto selected_end =
                    parse_iso8601_utc(selected["end_utc"].get<std::string>());
                const auto selected_seed =
                    parse_iso8601_utc(selected["t0_utc"].get<std::string>());
                if (!selected_start || !selected_end || !selected_seed) {
                    return fail(
                        "selected_window times must be canonical ISO-8601 "
                        "UTC ending with Z");
                }
                if (!(*selected_start < *selected_end) ||
                    *selected_seed < *selected_start ||
                    *selected_seed > *selected_end) {
                    return fail(
                        "selected_window must satisfy start < end and start "
                        "<= t0 <= end");
                }
                const auto task_start = parse_iso8601_utc(start_utc);
                const auto task_end =
                    *task_start +
                    std::chrono::duration_cast<
                        std::chrono::system_clock::duration>(
                        std::chrono::duration<double>(horizon_sec));
                if (*selected_start < *task_start || *selected_end > task_end) {
                    return fail(
                        "selected_window must be contained in the task "
                        "compute horizon");
                }
                if (constraints.contains("max_abs_range_rate_mps") &&
                    !has_number(constraints, "max_abs_range_rate_mps")) {
                    return fail(
                        "constraints.max_abs_range_rate_mps must be finite "
                        "when provided");
                }
                const double max_rate =
                    get_number_or(constraints, "max_abs_range_rate_mps",
                                  kSarDefaultMaxAbsRangeRateMps);
                if (!(max_rate > 0.0) || !std::isfinite(max_rate)) {
                    return fail(
                        "constraints.max_abs_range_rate_mps must be finite "
                        "and > 0");
                }
                result.details["sar"]["max_abs_range_rate_mps"] = max_rate;
                result.details["sar"]["selected_window"]        = selected;
            }
            else if (sensor_type != "optical_area_array" &&
                     sensor_type != "optical_linescan") {
                return fail("Unsupported attitude_estimation sensor.type: " +
                            sensor_type);
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
    if (result.details.contains("sar")) {
        details["sar"] = result.details["sar"];
    }
    result.details = std::move(details);
    if (spacecraft.contains("propagation_profile")) {
        result.details["propagation_profile"] =
            spacecraft["propagation_profile"];
    }
    return result;
}

}  // namespace mp
