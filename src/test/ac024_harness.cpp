#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "geometry/sar_attitude_solver.hpp"
#include "planner/run_planner.hpp"
#include "planner/time_model.hpp"
#include "planner/validate.hpp"

namespace {

int failures = 0;

void expect(bool condition, const std::string& label) {
    if (condition) { std::cout << "  OK  " << label << '\n'; }
    else {
        std::cerr << "  FAIL " << label << '\n';
        ++failures;
    }
}

std::filesystem::path temp_dir() {
    const auto path = std::filesystem::path("/tmp") / "ac024_harness";
    std::filesystem::create_directories(path);
    return path;
}

std::filesystem::path write_file(const std::string& name,
                                 const std::string& body) {
    const auto    path = temp_dir() / name;
    std::ofstream output(path);
    output << body;
    return path;
}

std::string state_line(double second,
                       double sat_x,
                       double target_x,
                       double target_vx,
                       double normal_x_offset = 0.0,
                       double normal_z_offset = 1.0) {
    char utc[32];
    std::snprintf(utc, sizeof(utc), "00:00:%09.6f", second);
    std::ostringstream stream;
    stream << "30 Dec 2026 " << utc << ' ' << sat_x << " 0 7000 7.5 0 0 "
           << target_x << " 300 6378 " << target_vx << " 0 0 "
           << target_x + normal_x_offset << " 300 " << 6378.0 + normal_z_offset
           << '\n';
    return stream.str();
}

std::filesystem::path moving_target_report(
    const std::string& name,
    double             root_sec,
    double             target_velocity_x       = 0.0,
    bool               report_target_velocity  = true,
    double             report_epoch_offset_sec = 0.0) {
    constexpr double satellite_velocity_x = 7.5;
    const double relative_velocity = satellite_velocity_x - target_velocity_x;
    const double target_x0         = relative_velocity * root_sec;
    std::string  body;
    for (int second = 0; second <= 3; ++second) {
        const double target_x = target_x0 + target_velocity_x * second;
        body += state_line(second + report_epoch_offset_sec,
                           satellite_velocity_x * second, target_x,
                           report_target_velocity ? target_velocity_x : 0.0);
    }
    return write_file(name, body);
}

std::filesystem::path transient_geometry_island_report(
    const std::string& name) {
    const std::vector<double> epochs = {0.0, 1.0, 1.16, 1.18, 1.20, 2.0, 3.0};
    std::string               body;
    for (const double second : epochs) {
        const bool bad_normal = std::fabs(second - 1.18) < 1.0e-12;
        body += state_line(second, 7.5 * second, 7.5, 0.0,
                           bad_normal ? 1.0 : 0.0, bad_normal ? 0.0 : 1.0);
    }
    return write_file(name, body);
}

std::filesystem::path orientation_degenerate_report(const std::string& name,
                                                    bool bad_normal = false) {
    std::string body;
    for (int second = 0; second <= 3; ++second) {
        char utc[32];
        std::snprintf(utc, sizeof(utc), "00:00:%09.6f",
                      static_cast<double>(second));
        std::ostringstream stream;
        stream << "30 Dec 2026 " << utc << " 0 0 7000 1 0 0 100 0 7000 0 0 0 "
               << (bad_normal ? 100 : 101) << " 0 7000\n";
        body += stream.str();
    }
    return write_file(name, body);
}

nlohmann::json selected_window(double seed_sec = 0.2, double end_sec = 3.0) {
    char seed[40];
    char end[40];
    std::snprintf(seed, sizeof(seed), "2026-12-30T00:00:%06.3fZ", seed_sec);
    std::snprintf(end, sizeof(end), "2026-12-30T00:00:%06.3fZ", end_sec);
    return {{"start_utc", "2026-12-30T00:00:00.000Z"},
            {"end_utc", end},
            {"t0_utc", seed}};
}

mp::SarAttitudeOptions options() {
    mp::SarAttitudeOptions options;
    options.step_sec               = 1.0;
    options.working_time_sec       = 2.0;
    options.incidence_min_deg      = 20.0;
    options.incidence_max_deg      = 40.0;
    options.allowed_look_side      = "left";
    options.roll_max_deg           = 40.0;
    options.center_frequency_hz    = 5.405e9;
    options.azimuth_beamwidth_deg  = 10.0;
    options.max_abs_squint_deg     = 5.0;
    options.max_abs_range_rate_mps = 0.1;
    return options;
}

nlohmann::json base_request() {
    return {
        {"task",
         {{"scenario", "attitude_estimation"},
          {"start_time_utc", "2026-12-30T00:00:00Z"},
          {"compute_horizon_sec", 300.0},
          {"working_time_sec", 2.0},
          {"step_sec", 1.0}}},
        {"selected_window", selected_window()},
        {"spacecraft",
         {{"sat_id", "sat"},
          {"epoch_utc", "30 Dec 2026 00:00:00.000"},
          {"state_type", "keplerian"},
          {"elements",
           {{"sma_km", 7000.0},
            {"ecc", 0.0},
            {"inc_deg", 98.0},
            {"raan_deg", 0.0},
            {"aop_deg", 0.0},
            {"ta_deg", 0.0}}}}},
        {"target",
         {{"type", "ground_point"},
          {"lon_deg", 114.0},
          {"lat_deg", 30.0},
          {"alt_km", 0.0}}},
        {"sensor",
         {{"type", "sar"},
          {"mode", "stripmap"},
          {"center_frequency_hz", 5.405e9},
          {"azimuth_beamwidth_deg", 10.0}}},
        {"constraints",
         {{"incidence_min_deg", 20.0},
          {"incidence_max_deg", 40.0},
          {"allowed_look_side", "left"},
          {"roll_max_deg", 40.0}}},
    };
}

double iso_relative_sec(const std::string& iso) {
    const auto parsed = mp::parse_iso8601_utc(iso);
    const auto origin = mp::parse_iso8601_utc("2026-12-30T00:00:00.000Z");
    return std::chrono::duration<double>(*parsed - *origin).count();
}

double quaternion_norm(const std::array<double, 4>& q) {
    double sum = 0.0;
    for (const double value : q) { sum += value * value; }
    return std::sqrt(sum);
}

mp::Vec3 quaternion_z_axis(const std::array<double, 4>& q) {
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];
    return {2.0 * (x * z + w * y), 2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y)};
}

mp::Vec3 quaternion_x_axis(const std::array<double, 4>& q) {
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];
    return {1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + w * z),
            2.0 * (x * z - w * y)};
}

double dot3(const mp::Vec3& a, const mp::Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

mp::Vec3 normalized(const mp::Vec3& value) {
    const double norm = std::sqrt(dot3(value, value));
    return {value.x / norm, value.y / norm, value.z / norm};
}

mp::Vec3 subtract(const mp::Vec3& a, const mp::Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

mp::Vec3 scaled(const mp::Vec3& value, double scale) {
    return {value.x * scale, value.y * scale, value.z * scale};
}

mp::Vec3 cross(const mp::Vec3& a, const mp::Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

struct IndependentSarGeometry {
    double      incidence_deg = 0.0;
    std::string look_side;
    double      squint_deg = 0.0;
    double      roll_deg   = 0.0;
    bool        los_clear  = false;
};

IndependentSarGeometry independent_sar_geometry(const mp::SarStateNode& state) {
    constexpr double rad_to_deg = 57.2957795130823208768;
    const auto       radial     = normalized(state.r_sat);
    const auto       D          = scaled(radial, -1.0);
    const auto       tangential =
        subtract(state.v_sat, scaled(radial, dot3(state.v_sat, radial)));
    const auto T      = normalized(tangential);
    const auto C      = normalized(cross(T, D));
    const auto u      = normalized(subtract(state.r_tgt, state.r_sat));
    const auto normal = normalized(subtract(state.r_normal_point, state.r_tgt));
    const auto platform_velocity  = subtract(state.v_sat, state.v_tgt);
    const auto platform_direction = normalized(platform_velocity);
    const auto x_body             = normalized(
        subtract(platform_velocity, scaled(u, dot3(platform_velocity, u))));
    const auto   y_body     = normalized(cross(u, x_body));
    const double side_dot   = dot3(u, C);
    const double ground_los = dot3(normal, scaled(u, -1.0));

    IndependentSarGeometry geometry;
    geometry.incidence_deg =
        std::acos(std::clamp(ground_los, -1.0, 1.0)) * rad_to_deg;
    geometry.look_side = std::fabs(side_dot) <= 1.0e-12
                             ? "degenerate"
                             : (side_dot > 0.0 ? "left" : "right");
    geometry.squint_deg =
        std::asin(std::clamp(dot3(u, platform_direction), -1.0, 1.0)) *
        rad_to_deg;
    geometry.roll_deg  = std::atan2(dot3(D, y_body), dot3(D, u)) * rad_to_deg;
    geometry.los_clear = ground_los > 0.0;
    return geometry;
}

bool independent_geometry_allowed(const IndependentSarGeometry& geometry,
                                  const nlohmann::json&         request) {
    const auto&  constraints  = request.at("constraints");
    const auto&  sensor       = request.at("sensor");
    const double squint_limit = constraints.value(
        "max_abs_squint_deg",
        sensor.at("azimuth_beamwidth_deg").get<double>() / 2.0);
    const auto allowed_side =
        constraints.at("allowed_look_side").get<std::string>();
    const bool resolved =
        geometry.look_side == "left" || geometry.look_side == "right";
    const bool side_ok = resolved && (allowed_side == "either" ||
                                      geometry.look_side == allowed_side);
    return geometry.los_clear && side_ok &&
           geometry.incidence_deg >=
               constraints.at("incidence_min_deg").get<double>() &&
           geometry.incidence_deg <=
               constraints.at("incidence_max_deg").get<double>() &&
           std::fabs(geometry.squint_deg) <= squint_limit &&
           std::fabs(geometry.roll_deg) <=
               constraints.at("roll_max_deg").get<double>();
}

void replay_real_result(const std::filesystem::path& state_path,
                        const std::filesystem::path& result_path,
                        const std::filesystem::path& request_path) {
    using namespace mp;
    std::ifstream  input(result_path);
    nlohmann::json result;
    input >> result;
    std::ifstream  request_input(request_path);
    nlohmann::json request;
    request_input >> request;
    const auto nodes = load_sar_state_report(state_path);
    const auto t0    = parse_iso8601_utc(
        result.at("attitude").at("t0_utc").get<std::string>());
    const double t_sec =
        std::chrono::duration<double>(*t0 - nodes.front().utc).count();
    const auto   state            = evaluate_sar_state(nodes, t_sec);
    const auto   q_los            = subtract(state.r_tgt, state.r_sat);
    const auto   u                = normalized(q_los);
    const auto   v_rel            = subtract(state.v_tgt, state.v_sat);
    const double independent_rate = dot3(u, v_rel) * 1000.0;
    const auto normal = normalized(subtract(state.r_normal_point, state.r_tgt));
    constexpr double rad_to_deg = 57.2957795130823208768;
    const double     independent_incidence =
        std::acos(std::clamp(dot3(normal, scaled(u, -1.0)), -1.0, 1.0)) *
        rad_to_deg;

    const auto& attitude_json = result.at("attitude");
    const auto  values =
        attitude_json.at("quaternion_body_to_reference").at("values");
    std::array<double, 4> quaternion{};
    for (std::size_t i = 0; i < quaternion.size(); ++i) {
        quaternion[i] = values[i].get<double>();
    }
    const auto relative_platform = subtract(state.v_sat, state.v_tgt);
    const auto expected_x        = normalized(
        subtract(relative_platform, scaled(u, dot3(relative_platform, u))));
    const auto independent_geometry = independent_sar_geometry(state);

    expect(std::fabs(independent_rate -
                     attitude_json.at("range_rate_mps").get<double>()) < 1e-8,
           "G1 real independent range-rate replay");
    expect(
        std::fabs(independent_incidence -
                  attitude_json.at("incidence_angle_deg").get<double>()) < 1e-8,
        "G1 real independent incidence replay");
    expect(dot3(quaternion_z_axis(quaternion), u) > 1.0 - 1e-12,
           "G1 real quaternion boresight replay");
    expect(dot3(quaternion_x_axis(quaternion), expected_x) > 1.0 - 1e-12,
           "G1 real quaternion zero-squint axis replay");
    expect(std::fabs(independent_geometry.squint_deg -
                     attitude_json.at("squint_deg").get<double>()) < 1e-8,
           "G1 real squint uses independent vector formula");
    expect(std::fabs(independent_geometry.roll_deg -
                     attitude_json.at("roll_deg").get<double>()) < 1e-8,
           "G1 real mechanical roll uses independent body-to-LVLH formula");
    expect(std::fabs(result.at("windows")[0].at("phi_deg").get<double>() -
                     std::fabs(attitude_json.at("roll_deg").get<double>())) <
               1e-12,
           "G1 real phi=|attitude.roll| contract");

    const auto window_start = parse_iso8601_utc(
        result.at("windows")[0].at("start_utc").get<std::string>());
    const auto window_end = parse_iso8601_utc(
        result.at("windows")[0].at("end_utc").get<std::string>());
    const auto geometry_at =
        [&](const std::chrono::system_clock::time_point& tp) {
            const double relative =
                std::chrono::duration<double>(tp - nodes.front().utc).count();
            return independent_sar_geometry(
                evaluate_sar_state(nodes, relative));
        };
    expect(independent_geometry_allowed(geometry_at(*window_start), request) &&
               independent_geometry_allowed(geometry_at(*window_end), request),
           "G1 real refined-window endpoints satisfy independent geometry");
    const auto selected_start = parse_iso8601_utc(
        request.at("selected_window").at("start_utc").get<std::string>());
    const auto selected_end = parse_iso8601_utc(
        request.at("selected_window").at("end_utc").get<std::string>());
    const auto half_working =
        std::chrono::duration_cast<std::chrono::system_clock::duration>(
            std::chrono::duration<double>(
                request.at("task").at("working_time_sec").get<double>() / 2.0));
    const auto work_start = *t0 - half_working;
    const auto work_end   = *t0 + half_working;
    const auto final_allowed =
        [&](const std::chrono::system_clock::time_point& tp) {
            return tp >= *selected_start && tp <= *selected_end &&
                   tp >= work_start && tp <= work_end &&
                   independent_geometry_allowed(geometry_at(tp), request);
        };
    const auto one_ms = std::chrono::milliseconds(1);
    expect(!final_allowed(*window_start - one_ms) &&
               !final_allowed(*window_end + one_ms),
           "G1 real adjacent exterior milliseconds fail final intersection");

    const auto two_ms                      = std::chrono::milliseconds(2);
    int        geometry_boundaries_checked = 0;
    bool       geometry_boundary_ok        = true;
    if (*window_start > std::max(*selected_start, work_start) + two_ms) {
        ++geometry_boundaries_checked;
        geometry_boundary_ok =
            geometry_boundary_ok &&
            !independent_geometry_allowed(geometry_at(*window_start - two_ms),
                                          request);
    }
    if (*window_end < std::min(*selected_end, work_end) - two_ms) {
        ++geometry_boundaries_checked;
        geometry_boundary_ok = geometry_boundary_ok &&
                               !independent_geometry_allowed(
                                   geometry_at(*window_end + two_ms), request);
    }
    expect(geometry_boundaries_checked > 0 && geometry_boundary_ok,
           "G1 real binding refined-geometry boundary independently bracketed");
}

}  // namespace

int main(int argc, char** argv) {
    using namespace mp;

    std::cout << "==> V selected-window contract\n";
    {
        auto       request    = base_request();
        const auto validation = validate_request(request);
        expect(validation.ok &&
                   validation.details["sar"]["max_abs_range_rate_mps"] == 0.1,
               "V1 SAR AE valid and default residual=0.1");

        auto missing = request;
        missing.erase("selected_window");
        expect(!validate_request(missing).ok, "V1 selected_window required");

        auto out_of_horizon = request;
        out_of_horizon["selected_window"]["end_utc"] =
            "2026-12-30T00:06:00.000Z";
        expect(!validate_request(out_of_horizon).ok,
               "V1 selected_window contained in horizon");

        auto unknown_window                             = request;
        unknown_window["selected_window"]["unexpected"] = true;
        expect(!validate_request(unknown_window).ok,
               "V1 selected_window unknown field rejected");

        auto unknown_geometry                               = request;
        unknown_geometry["selected_window"]["sar_geometry"] = {
            {"unexpected", true}};
        expect(!validate_request(unknown_geometry).ok,
               "V1 selected_window.sar_geometry unknown field rejected");

        auto bad_residual                                     = request;
        bad_residual["constraints"]["max_abs_range_rate_mps"] = 0.0;
        expect(!validate_request(bad_residual).ok,
               "V2 non-positive residual rejected");

        auto typed_residual                                     = request;
        typed_residual["constraints"]["max_abs_range_rate_mps"] = "0.1";
        expect(!validate_request(typed_residual).ok,
               "V2 explicit non-numeric residual rejected");
    }

    std::cout << "==> A zero-Doppler / millisecond / attitude\n";
    {
        constexpr double truth  = 1.2344;
        const auto       report = moving_target_report("root.txt", truth);
        const auto       refined =
            refine_sar_attitude(report, selected_window(), options());
        expect(refined.ok, "A1 known zero-rate root succeeds");
        const auto t0_iso = format_iso8601_utc_ms(refined.window.t0);
        const auto t0_sec = iso_relative_sec(t0_iso);
        expect(std::fabs(t0_sec - truth) <= 0.0005 + 1e-9,
               "A1 t0 nearest millisecond");
        expect(std::fabs(refined.window.geometry.range_rate_mps) <= 0.1,
               "A1 quantized residual <=0.1m/s");
        expect(std::fabs(t0_sec * 1000.0 - std::round(t0_sec * 1000.0)) < 1e-9,
               "A4 t0 exactly on millisecond grid");
        expect(std::fabs(refined.window.duration_sec - 2.0) < 1e-9,
               "A12 exact-t0 working window clip");

        const auto& q = refined.attitude.quaternion_wxyz;
        expect(std::fabs(quaternion_norm(q) - 1.0) < 1e-12 && q[0] >= 0.0,
               "A7 normalized canonical wxyz quaternion");
        const auto nodes = load_sar_state_report(report);
        const auto state = evaluate_sar_state(nodes, t0_sec);
        const auto los   = normalized({state.r_tgt.x - state.r_sat.x,
                                       state.r_tgt.y - state.r_sat.y,
                                       state.r_tgt.z - state.r_sat.z});
        expect(dot3(quaternion_z_axis(q), los) > 1.0 - 1e-12,
               "A7 quaternion +Z reconstructs boresight");
        expect(std::fabs(refined.attitude.squint_deg -
                         refined.window.geometry.squint_deg) < 1e-12 &&
                   std::fabs(refined.attitude.squint_deg) <=
                       options().max_abs_squint_deg,
               "A7 actual squint recomputed and within beam gate");
        constexpr double c_mps      = 299792458.0;
        const double     wavelength = c_mps / options().center_frequency_hz;
        expect(std::fabs(refined.window.geometry.doppler_centroid_hz +
                         2.0 * refined.window.geometry.range_rate_mps /
                             wavelength) < 1e-9,
               "A7 Doppler has opposite sign to range rate");

        auto boundary         = options();
        boundary.roll_max_deg = std::fabs(refined.window.geometry.roll_deg);
        expect(refine_sar_attitude(report, selected_window(), boundary).ok,
               "A5 roll equality accepted");
        boundary.roll_max_deg -= 1e-6;
        expect(!refine_sar_attitude(report, selected_window(), boundary).ok,
               "A5 roll above limit rejected");

        auto incidence_boundary = options();
        incidence_boundary.incidence_min_deg =
            refined.window.geometry.incidence_angle_deg;
        incidence_boundary.incidence_max_deg =
            refined.window.geometry.incidence_angle_deg + 1.0;
        expect(
            refine_sar_attitude(report, selected_window(), incidence_boundary)
                .ok,
            "A5 incidence equality accepted");
        incidence_boundary.incidence_min_deg += 1e-6;
        expect(
            !refine_sar_attitude(report, selected_window(), incidence_boundary)
                 .ok,
            "A5 incidence above limit rejected");

        auto wrong_side              = options();
        wrong_side.allowed_look_side = "right";
        expect(!refine_sar_attitude(report, selected_window(), wrong_side).ok,
               "A5 wrong look side rejected");

        auto residual_boundary = options();
        residual_boundary.max_abs_range_rate_mps =
            std::fabs(refined.window.geometry.range_rate_mps);
        expect(refine_sar_attitude(report, selected_window(), residual_boundary)
                   .ok,
               "A8 residual equality accepted");
        residual_boundary.max_abs_range_rate_mps -= 1e-9;
        expect(
            !refine_sar_attitude(report, selected_window(), residual_boundary)
                 .ok,
            "A8 residual above limit rejected");

        const auto repeated =
            refine_sar_attitude(report, selected_window(), options());
        expect(repeated.ok && repeated.window.t0 == refined.window.t0 &&
                   repeated.attitude.quaternion_wxyz ==
                       refined.attitude.quaternion_wxyz,
               "G2 repeated refinement is deterministic");

        auto passthrough_request = base_request();
        passthrough_request["selected_window"] =
            sar_windows_to_json({refined.window})[0];
        expect(validate_request(passthrough_request).ok,
               "V1 complete AC-010 window passes through to SAR AE");
    }

    std::cout << "==> A2 target velocity / A3 no sign / A6 seed\n";
    {
        constexpr double truth = 1.234;
        const auto       moving =
            moving_target_report("moving.txt", truth, 0.5, true);
        const auto wrong =
            moving_target_report("moving_zero_v.txt", truth, 0.5, false);
        const auto correct_result =
            refine_sar_attitude(moving, selected_window(0.1), options());
        const auto wrong_result =
            refine_sar_attitude(wrong, selected_window(0.1), options());
        const double correct_t =
            iso_relative_sec(format_iso8601_utc_ms(correct_result.window.t0));
        const double wrong_t =
            iso_relative_sec(format_iso8601_utc_ms(wrong_result.window.t0));
        expect(std::fabs(correct_t - truth) <= 0.001 &&
                   std::fabs(correct_t - wrong_t) > 0.005,
               "A2 v_t changes zero-Doppler solution");

        auto loose                   = options();
        loose.max_abs_range_rate_mps = 1000.0;
        const auto no_sign =
            refine_sar_attitude(moving_target_report("outside.txt", 2.0),
                                selected_window(0.2, 1.0), loose);
        expect(no_sign.ok && no_sign.window.t0 <=
                                 *parse_iso8601_utc("2026-12-30T00:00:01.000Z"),
               "A3 no-sign bounded minimum stays in window");

        const auto far_seed = selected_window(0.01);
        const auto seeded   = refine_sar_attitude(
            moving_target_report("far_seed.txt", truth), far_seed, options());
        expect(seeded.ok && std::fabs(iso_relative_sec(format_iso8601_utc_ms(
                                          seeded.window.t0)) -
                                      truth) <= 0.001,
               "A6 coarse seed does not discard in-window subsecond root");
    }

    std::cout << "==> A4 absolute UTC millisecond grid\n";
    {
        constexpr double report_offset = 0.0004;
        constexpr double relative_root = 1.2344;
        const auto       report        = moving_target_report(
            "absolute_ms.txt", relative_root, 0.0, true, report_offset);
        const nlohmann::json window = {
            {"start_utc", "2026-12-30T00:00:00.001Z"},
            {"end_utc", "2026-12-30T00:00:03.000Z"},
            {"t0_utc", "2026-12-30T00:00:00.200Z"},
        };
        const auto refined = refine_sar_attitude(report, window, options());
        const auto t0_ms =
            std::chrono::time_point_cast<std::chrono::milliseconds>(
                refined.window.t0);
        expect(refined.ok && refined.window.t0 == t0_ms,
               "A4 t0 lies on absolute Unix millisecond grid");
        const double absolute_t =
            iso_relative_sec(format_iso8601_utc_ms(refined.window.t0));
        expect(std::fabs(absolute_t - (report_offset + relative_root)) <=
                   0.0005 + 1e-9,
               "A4 absolute-grid candidate is nearest millisecond");
    }

    std::cout << "==> A5 refined geometry window / orientation failures\n";
    {
        constexpr double truth = 1.2344;
        const auto report = moving_target_report("geometry_window.txt", truth);
        auto       gated  = options();
        gated.working_time_sec       = 3.0;
        gated.max_abs_squint_deg     = 0.2;
        gated.max_abs_range_rate_mps = 0.1;
        const auto refined =
            refine_sar_attitude(report, selected_window(), gated);
        expect(refined.ok && refined.window.duration_sec > 0.0 &&
                   refined.window.duration_sec < 1.0,
               "A5 refined geometry window narrows working-time window");
        if (refined.ok) {
            const auto   nodes = load_sar_state_report(report);
            const double left_sec =
                std::chrono::duration<double>(refined.window.start -
                                              nodes.front().utc)
                    .count();
            const double right_sec = std::chrono::duration<double>(
                                         refined.window.end - nodes.front().utc)
                                         .count();
            const auto left = compute_sar_orientation(
                evaluate_sar_state(nodes, left_sec), gated.center_frequency_hz);
            const auto right =
                compute_sar_orientation(evaluate_sar_state(nodes, right_sec),
                                        gated.center_frequency_hz);
            expect(std::fabs(left.geometry.squint_deg) <=
                           gated.max_abs_squint_deg &&
                       std::fabs(right.geometry.squint_deg) <=
                           gated.max_abs_squint_deg,
                   "A5 refined endpoints are inside squint boundary");
            const auto before = compute_sar_orientation(
                evaluate_sar_state(nodes, left_sec - 0.003),
                gated.center_frequency_hz);
            const auto after = compute_sar_orientation(
                evaluate_sar_state(nodes, right_sec + 0.003),
                gated.center_frequency_hz);
            expect(std::fabs(before.geometry.squint_deg) >
                           gated.max_abs_squint_deg &&
                       std::fabs(after.geometry.squint_deg) >
                           gated.max_abs_squint_deg,
                   "A5 millisecond grid scan excludes 3ms exterior");

            const auto off_zero =
                compute_sar_orientation(evaluate_sar_state(nodes, truth + 0.5),
                                        gated.center_frequency_hz);
            expect(std::fabs(off_zero.geometry.roll_deg -
                             off_zero.geometry.side_look_angle_deg) > 1.0e-6,
                   "A5 mechanical roll is distinct from LOS side-look angle");
        }

        auto degenerate                   = options();
        degenerate.allowed_look_side      = "either";
        degenerate.max_abs_range_rate_mps = 2000.0;
        const auto no_result              = refine_sar_attitude(
            orientation_degenerate_report("orientation_degenerate.txt"),
            selected_window(), degenerate);
        expect(!no_result.ok && no_result.warnings.size() == 1 &&
                   no_result.warnings[0] == kSarAttitudeNoFeasibleWarning,
               "A5 explicit candidate orientation degeneracy is infeasible");

        bool threw = false;
        try {
            refine_sar_attitude(
                orientation_degenerate_report("bad_normal.txt", true),
                selected_window(), degenerate);
        } catch (const std::exception& error) {
            threw = std::string(error.what()).find("degenerate first normal") !=
                    std::string::npos;
        }
        expect(threw, "A5 interpolation/report algorithm failure propagates");
    }

    std::cout << "==> A5b transient geometry island\n";
    {
        auto island_options             = options();
        island_options.working_time_sec = 3.0;
        const auto report =
            transient_geometry_island_report("geometry_island.txt");
        const auto nodes       = load_sar_state_report(report);
        const auto request     = base_request();
        const auto geometry_at = [&](double second) {
            return independent_sar_geometry(evaluate_sar_state(nodes, second));
        };
        expect(independent_geometry_allowed(geometry_at(1.127), request) &&
                   !independent_geometry_allowed(geometry_at(1.180), request) &&
                   independent_geometry_allowed(geometry_at(1.255), request),
               "A5b fixture has feasible probes around a narrow infeasible "
               "island");

        const auto refined =
            refine_sar_attitude(report, selected_window(), island_options);
        const double end_sec = std::chrono::duration<double>(
                                   refined.window.end - nodes.front().utc)
                                   .count();
        expect(refined.ok && end_sec >= 1.160 && end_sec < 1.180,
               "A5b refined window stops before first infeasible island");
        if (refined.ok) {
            bool every_grid_point_allowed = true;
            for (auto time = refined.window.start; time <= refined.window.end;
                 time += std::chrono::milliseconds(1)) {
                const double second =
                    std::chrono::duration<double>(time - nodes.front().utc)
                        .count();
                every_grid_point_allowed =
                    every_grid_point_allowed &&
                    independent_geometry_allowed(geometry_at(second), request);
            }
            expect(every_grid_point_allowed,
                   "A5b every returned millisecond satisfies geometry");
        }
    }

    std::cout << "==> A5c final interval quantizes inward\n";
    {
        auto broad                           = options();
        broad.working_time_sec               = 10.0;
        broad.incidence_min_deg              = 1.0;
        broad.incidence_max_deg              = 89.0;
        broad.allowed_look_side              = "either";
        broad.roll_max_deg                   = 90.0;
        broad.azimuth_beamwidth_deg          = 179.0;
        broad.max_abs_squint_deg             = 89.5;
        nlohmann::json submillisecond_window = {
            {"start_utc", "2026-12-30T00:00:00.0004Z"},
            {"end_utc", "2026-12-30T00:00:02.9996Z"},
            {"t0_utc", "2026-12-30T00:00:01.000Z"},
        };
        const auto refined =
            refine_sar_attitude(moving_target_report("inward-ms.txt", 1.2344),
                                submillisecond_window, broad);
        expect(refined.ok &&
                   format_iso8601_utc_ms(refined.window.start) ==
                       "2026-12-30T00:00:00.001Z" &&
                   format_iso8601_utc_ms(refined.window.end) ==
                       "2026-12-30T00:00:02.999Z",
               "A5c selected/working/refined intersection uses inward ms grid");
    }

    std::cout << "==> A8/A10 residual no-result contract\n";
    {
        auto strict                   = options();
        strict.max_abs_range_rate_mps = 0.001;
        const auto rejected =
            refine_sar_attitude(moving_target_report("strict.txt", 1.2344),
                                selected_window(), strict);
        expect(!rejected.ok && rejected.warnings.size() == 1 &&
                   rejected.warnings[0] == kSarAttitudeNoFeasibleWarning,
               "A8 residual exceed returns stable no-result warning");

        nlohmann::json output = {
            {"status", "succeeded"},
            {"windows", nlohmann::json::array({{{"old", true}}})},
            {"summary", {{"window_count", 1}, {"duration_total_sec", 1.0}}},
            {"attitude", {{"attitude_status", "computed"}}},
            {"artifacts", {{"sar_state_path", "kept"}}},
            {"warnings", nlohmann::json::array()},
        };
        apply_sar_attitude_result(output, rejected);
        expect(output["status"] == "no_result" && output["windows"].empty() &&
                   output["summary"]["window_count"] == 0 &&
                   !output.contains("attitude") &&
                   output["artifacts"]["sar_state_path"] == "kept",
               "A10 final JSON no_result contract");
        expect(run_status_exit_code(output) == 4, "A10 no_result CLI exit 4");
    }

    std::cout << "==> A11 strict report coverage\n";
    {
        const auto gap =
            write_file("gap.txt", state_line(0, 0.0, 9.0, 0.0) +
                                      state_line(3, 22.5, 9.0, 0.0));
        bool threw = false;
        try {
            refine_sar_attitude(gap, selected_window(), options());
        } catch (const std::exception& error) {
            threw =
                std::string(error.what()).find("coverage gap before line 2") !=
                std::string::npos;
        }
        expect(threw, "A11 state report coverage gap fails explicitly");

        threw = false;
        try {
            load_sar_state_report(
                write_file("blank.txt", state_line(0, 0.0, 9.0, 0.0) + "\n" +
                                            state_line(1, 7.5, 9.0, 0.0)));
        } catch (const std::exception& error) {
            threw =
                std::string(error.what()).find("line 2") != std::string::npos;
        }
        expect(threw, "A11 blank state row fails strict 19-token contract");
    }

    if (argc == 4) {
        std::cout << "==> G real GMAT independent replay\n";
        replay_real_result(argv[1], argv[2], argv[3]);
    }

    std::cout << "==> ac024-harness failures=" << failures << '\n';
    return failures == 0 ? 0 : 1;
}
