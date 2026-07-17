#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>

#include "geometry/sar_geometry.hpp"
#include "gmat/gmat_backend.hpp"
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

struct IndependentGeometry {
    double      incidence_deg = 0.0;
    std::string look_side;
    double      squint_deg     = 0.0;
    double      roll_deg       = 0.0;
    double      range_rate_mps = 0.0;
    bool        los_clear      = false;
};

double independent_dot(const mp::Vec3& a, const mp::Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

mp::Vec3 independent_subtract(const mp::Vec3& a, const mp::Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

mp::Vec3 independent_scale(const mp::Vec3& value, double scale) {
    return {value.x * scale, value.y * scale, value.z * scale};
}

mp::Vec3 independent_cross(const mp::Vec3& a, const mp::Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

mp::Vec3 independent_normalize(const mp::Vec3& value) {
    const double norm = std::sqrt(independent_dot(value, value));
    if (!(norm > 1.0e-12) || !std::isfinite(norm)) {
        throw std::runtime_error("independent replay degenerate vector");
    }
    return independent_scale(value, 1.0 / norm);
}

IndependentGeometry independent_geometry(const mp::SarStateNode& node) {
    constexpr double rad_to_deg = 57.2957795130823208768;
    const auto       radial     = independent_normalize(node.r_sat);
    const auto       D          = independent_scale(radial, -1.0);
    const auto       tangential = independent_subtract(
        node.v_sat,
        independent_scale(radial, independent_dot(node.v_sat, radial)));
    const auto T = independent_normalize(tangential);
    const auto C = independent_normalize(independent_cross(T, D));
    const auto u =
        independent_normalize(independent_subtract(node.r_tgt, node.r_sat));
    const auto normal = independent_normalize(
        independent_subtract(node.r_normal_point, node.r_tgt));
    const auto platform_velocity = independent_subtract(node.v_sat, node.v_tgt);
    const auto platform_direction = independent_normalize(platform_velocity);
    const auto x_body             = independent_normalize(independent_subtract(
        platform_velocity,
        independent_scale(u, independent_dot(platform_velocity, u))));
    const auto y_body = independent_normalize(independent_cross(u, x_body));
    const double cross_track = independent_dot(u, C);
    const double ground_los =
        independent_dot(normal, independent_scale(u, -1.0));

    IndependentGeometry geometry;
    geometry.incidence_deg =
        std::acos(std::clamp(ground_los, -1.0, 1.0)) * rad_to_deg;
    geometry.look_side = std::fabs(cross_track) <= 1.0e-12
                             ? "degenerate"
                             : (cross_track > 0.0 ? "left" : "right");
    geometry.squint_deg =
        std::asin(
            std::clamp(independent_dot(u, platform_direction), -1.0, 1.0)) *
        rad_to_deg;
    geometry.roll_deg =
        std::atan2(independent_dot(D, y_body), independent_dot(D, u)) *
        rad_to_deg;
    geometry.range_rate_mps =
        independent_dot(u, independent_subtract(node.v_tgt, node.v_sat)) *
        1000.0;
    geometry.los_clear = ground_los > 0.0;
    return geometry;
}

bool independent_eligible(const IndependentGeometry& geometry) {
    return geometry.los_clear &&
           (geometry.look_side == "left" || geometry.look_side == "right") &&
           geometry.incidence_deg >= 20.0 && geometry.incidence_deg <= 80.0 &&
           std::fabs(geometry.squint_deg) <= 5.0 &&
           std::fabs(geometry.roll_deg) <= 70.0;
}

std::filesystem::path temp_dir() {
    const auto path = std::filesystem::path("/tmp") / "ac010_harness";
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

nlohmann::json base_request() {
    return {
        {"task",
         {{"scenario", "remote_sensing_access"},
          {"start_time_utc", "2026-12-30T00:00:00Z"},
          {"compute_horizon_sec", 172800.0},
          {"working_time_sec", 2.0},
          {"step_sec", 1.0}}},
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

std::string state_line(double second, double sat_x, double target_x, double y) {
    char utc[32];
    std::snprintf(utc, sizeof(utc), "00:00:%09.6f", second);
    std::ostringstream stream;
    stream << "30 Dec 2026 " << utc << ' ' << sat_x << " 0 7000 7.5 0 0 "
           << target_x << ' ' << y << " 6378 0 0 0 " << target_x << ' ' << y
           << " 6379\n";
    return stream.str();
}

std::filesystem::path synthetic_report(const std::string& name,
                                       double             y = 300.0) {
    std::string  body;
    const double target_x = 7.5;
    for (int second = 0; second <= 4; ++second) {
        body += state_line(second, 7.5 * second, target_x, y);
    }
    return write_file(name, body);
}

mp::SarMergeOptions merge_options() {
    mp::SarMergeOptions options;
    options.step_sec              = 1.0;
    options.working_time_sec      = 2.0;
    options.incidence_min_deg     = 20.0;
    options.incidence_max_deg     = 40.0;
    options.allowed_look_side     = "left";
    options.roll_max_deg          = 40.0;
    options.center_frequency_hz   = 5.405e9;
    options.azimuth_beamwidth_deg = 10.0;
    options.max_abs_squint_deg    = 5.0;
    return options;
}

void print_real_report_stats(const std::filesystem::path& path) {
    const auto  nodes         = mp::load_sar_state_report(path);
    double      min_incidence = std::numeric_limits<double>::infinity();
    double      max_incidence = -std::numeric_limits<double>::infinity();
    double      min_roll      = std::numeric_limits<double>::infinity();
    double      min_rate      = std::numeric_limits<double>::infinity();
    std::size_t visible       = 0;
    for (const auto& node : nodes) {
        const auto geometry = mp::compute_sar_geometry(node, 5.405e9);
        min_incidence = std::min(min_incidence, geometry.incidence_angle_deg);
        max_incidence = std::max(max_incidence, geometry.incidence_angle_deg);
        min_roll      = std::min(min_roll, std::fabs(geometry.roll_deg));
        min_rate      = std::min(min_rate, std::fabs(geometry.range_rate_mps));
        if (geometry.los_clear && geometry.incidence_angle_deg >= 15.0 &&
            geometry.incidence_angle_deg <= 60.0 &&
            std::fabs(geometry.roll_deg) <= 70.0) {
            ++visible;
        }
    }
    std::cout << "real-report nodes=" << nodes.size() << " visible=" << visible
              << " incidence=[" << min_incidence << ',' << max_incidence
              << "] min|roll|=" << min_roll << " min|range_rate|=" << min_rate
              << '\n';
}

void replay_real_result(const std::filesystem::path& state_path,
                        const std::filesystem::path& result_path) {
    using namespace mp;
    std::ifstream  input(result_path);
    nlohmann::json result;
    input >> result;
    const auto nodes = load_sar_state_report(state_path);
    expect(
        result.at("status") == "succeeded" && result.at("windows").size() >= 1,
        "G1 real RSA result has candidate windows");
    for (const auto& window : result.at("windows")) {
        const auto start =
            parse_iso8601_utc(window.at("start_utc").get<std::string>());
        const auto end =
            parse_iso8601_utc(window.at("end_utc").get<std::string>());
        const auto t0 =
            parse_iso8601_utc(window.at("t0_utc").get<std::string>());
        const SarStateNode* seed_node = nullptr;
        double best_abs_rate          = std::numeric_limits<double>::infinity();
        for (const auto& node : nodes) {
            if (node.utc < *start || node.utc > *end) { continue; }
            const auto geometry = independent_geometry(node);
            if (independent_eligible(geometry)) {
                best_abs_rate =
                    std::min(best_abs_rate, std::fabs(geometry.range_rate_mps));
            }
            if (std::chrono::duration<double>(node.utc - *t0).count() == 0.0) {
                seed_node = &node;
            }
        }
        expect(seed_node != nullptr, "G1 real RSA t0 lies on report grid");
        if (seed_node == nullptr) { continue; }
        const auto  geometry        = independent_geometry(*seed_node);
        const auto& output_geometry = window.at("sar_geometry");
        expect(
            std::fabs(geometry.range_rate_mps -
                      output_geometry.at("range_rate_mps").get<double>()) <
                    1e-9 &&
                std::fabs(
                    geometry.incidence_deg -
                    output_geometry.at("incidence_angle_deg").get<double>()) <
                    1e-9 &&
                std::fabs(geometry.squint_deg -
                          output_geometry.at("squint_deg").get<double>()) <
                    1e-9 &&
                std::fabs(geometry.roll_deg -
                          output_geometry.at("roll_deg").get<double>()) <
                    1e-9 &&
                geometry.look_side ==
                    output_geometry.at("look_side").get<std::string>(),
            "G1 real RSA diagnostics use independent vector formulas");
        expect(std::fabs(geometry.range_rate_mps) <= best_abs_rate + 1e-9,
               "G1 real RSA seed is clipped-window min-|range_rate|");
        expect(std::fabs(window.at("phi_deg").get<double>() -
                         std::fabs(geometry.roll_deg)) < 1e-12,
               "G1 real RSA phi=|geometry.roll|");
    }
}

}  // namespace

int main(int argc, char** argv) {
    setenv("ACCESS_COMPUTER_DEV_SOURCE_ROOT", ACCESS_COMPUTER_SOURCE_ROOT, 1);
    using namespace mp;

    std::cout << "==> V validation / renderer\n";
    {
        auto request = base_request();
        expect(validate_request(request).ok, "V1 explicit SAR request valid");

        auto missing = request;
        missing["constraints"].erase("incidence_min_deg");
        expect(!validate_request(missing).ok, "V1 missing incidence rejected");

        auto bad_range                                = request;
        bad_range["constraints"]["incidence_min_deg"] = 40.0;
        expect(!validate_request(bad_range).ok,
               "V1 incidence min>=max rejected");

        auto bad_mode              = request;
        bad_mode["sensor"]["mode"] = "stare";
        expect(!validate_request(bad_mode).ok, "V2 non-stripmap rejected");

        auto missing_beam = request;
        missing_beam["sensor"].erase("azimuth_beamwidth_deg");
        expect(!validate_request(missing_beam).ok,
               "V1 azimuth beamwidth required");

        auto bad_squint                                 = request;
        bad_squint["constraints"]["max_abs_squint_deg"] = 5.001;
        expect(!validate_request(bad_squint).ok,
               "V1 explicit squint may not exceed beam half-width");

        auto explicit_squint                                 = request;
        explicit_squint["constraints"]["max_abs_squint_deg"] = 4.0;
        const auto explicit_validated = validate_request(explicit_squint);
        expect(
            explicit_validated.ok &&
                explicit_validated.details["sar"]["max_abs_squint_deg"] == 4.0,
            "V1 explicit squint gate accepted and echoed");

        auto legacy_false            = request;
        legacy_false["experimental"] = {{"allow_sar", false}};
        auto legacy_true             = request;
        legacy_true["experimental"]  = {{"allow_sar", true}};
        expect(validate_request(legacy_false).ok &&
                   validate_request(legacy_true).ok,
               "V3 deprecated allow_sar does not gate feature");

        auto flags                            = request;
        flags["constraints"]["exclude_umbra"] = true;
        const auto validated                  = validate_request(flags);
        expect(validated.ok && validated.details.contains("warnings") &&
                   validated.details["warnings"][0] ==
                       kIlluminationFlagsIgnoredSar,
               "R5 illumination flags produce stable warning");

        const auto script = render_sar_access_script(request, temp_dir());
        expect(script.find("TargetA.EarthMJ2000Eq.VX") != std::string::npos &&
                   script.find("TargetNormalPoint.EarthMJ2000Eq.X") !=
                       std::string::npos,
               "R6 renderer reports target velocity and normal point");
        expect(script.find("EclipseLocator") == std::string::npos &&
                   script.find("Sun.") == std::string::npos,
               "R5 renderer has no illumination path");
        expect(
            script.find("TargetA.HorizonReference = Ellipsoid") !=
                    std::string::npos &&
                script.find("TargetNormalPoint.HorizonReference = Ellipsoid") !=
                    std::string::npos,
            "R6 renderer locks both ground points to ellipsoid horizon");
        expect(
            script.find("REGULAR_STEP_COUNT") == std::string::npos &&
                script.find("TOTAL_HORIZON_SEC") == std::string::npos &&
                script.find("stepIndex < 172799") != std::string::npos &&
                script.find("stopEpoch = Sat.A1ModJulian + 172800 / 86400") !=
                    std::string::npos &&
                script.find("Sat.A1ModJulian = stopEpoch") != std::string::npos,
            "R6 renderer reserves exact terminal correction step");
        auto non_integer_horizon                           = request;
        non_integer_horizon["task"]["compute_horizon_sec"] = 300.5;
        const auto tail_script =
            render_sar_access_script(non_integer_horizon, temp_dir());
        expect(tail_script.find("stepIndex < 300") != std::string::npos &&
                   tail_script.find(
                       "stopEpoch = Sat.A1ModJulian + 300.5 / 86400") !=
                       std::string::npos,
               "R6 renderer emits exact non-integer terminal correction");
    }

    std::cout << "==> R geometry / merge\n";
    {
        const auto path  = synthetic_report("valid.txt");
        const auto nodes = load_sar_state_report(path);
        expect(nodes.size() == 5, "R6 strict 19-token parse");
        const auto geometry = compute_sar_geometry(nodes[1], 5.405e9);
        expect(geometry.look_side == "left" && geometry.los_clear,
               "R2 left look and LOS");
        expect(geometry.incidence_angle_deg > 20.0 &&
                   geometry.incidence_angle_deg < 40.0,
               "R1 incidence inside range");

        const auto merged = merge_sar_windows(path, merge_options());
        expect(merged.windows.size() == 1, "R1 eligible samples merge");
        expect(merged.windows[0].duration_sec <= 2.0 + 1e-9,
               "R7 AC-004 working-time clip");
        expect(std::fabs(merged.windows[0].geometry.range_rate_mps) < 5.0,
               "R6 coarse min-|range_rate| seed");
        const auto json = sar_windows_to_json(merged.windows);
        expect(json.size() == 1 && json[0].contains("sar_geometry") &&
                   json[0]["phi_deg"] ==
                       std::fabs(
                           json[0]["sar_geometry"]["roll_deg"].get<double>()),
               "R6 SAR WindowSet diagnostics");

        auto right              = merge_options();
        right.allowed_look_side = "right";
        expect(merge_sar_windows(path, right).windows.empty(),
               "R2 wrong look side rejected");

        const auto degenerate_side =
            synthetic_report("degenerate-side.txt", 0.0);
        auto either              = merge_options();
        either.allowed_look_side = "either";
        either.incidence_min_deg = 1.0;
        either.incidence_max_deg = 89.0;
        expect(merge_sar_windows(degenerate_side, either).windows.empty(),
               "R2 either rejects unresolved degenerate look side");

        auto narrow_squint               = merge_options();
        narrow_squint.max_abs_squint_deg = 0.1;
        const auto zero_doppler_only = merge_sar_windows(path, narrow_squint);
        expect(zero_doppler_only.windows.size() == 1 &&
                   format_iso8601_utc_ms(zero_doppler_only.windows[0].start) ==
                       "2026-12-30T00:00:01.000Z",
               "R2 forward/aft samples excluded by squint gate");

        auto roll         = merge_options();
        roll.roll_max_deg = 1.0;
        expect(merge_sar_windows(path, roll).windows.empty(),
               "R3 roll above limit rejected");

        const auto boundary_path =
            write_file("boundary.txt", state_line(0, 0.0, 7.5, 300.0) +
                                           state_line(1, 0.0, 7.5, 300.0));
        const auto boundary_geometry = compute_sar_geometry(
            load_sar_state_report(boundary_path)[0], 5.405e9);
        auto incidence_min_boundary = merge_options();
        incidence_min_boundary.incidence_min_deg =
            boundary_geometry.incidence_angle_deg;
        incidence_min_boundary.incidence_max_deg =
            boundary_geometry.incidence_angle_deg + 1.0;
        expect(!merge_sar_windows(boundary_path, incidence_min_boundary)
                    .windows.empty(),
               "R1 incidence lower equality accepted");

        auto incidence_max_boundary = merge_options();
        incidence_max_boundary.incidence_min_deg =
            boundary_geometry.incidence_angle_deg - 1.0;
        incidence_max_boundary.incidence_max_deg =
            boundary_geometry.incidence_angle_deg;
        expect(!merge_sar_windows(boundary_path, incidence_max_boundary)
                    .windows.empty(),
               "R1 incidence upper equality accepted");

        auto roll_boundary         = merge_options();
        roll_boundary.roll_max_deg = std::fabs(boundary_geometry.roll_deg);
        expect(!merge_sar_windows(boundary_path, roll_boundary).windows.empty(),
               "R3 roll equality accepted");

        const auto below_horizon = synthetic_report("below.txt", 5000.0);
        expect(
            merge_sar_windows(below_horizon, merge_options()).windows.empty(),
            "R4 horizon/incidence rejection");

        const auto tail_path =
            write_file("tail.txt", state_line(0.0, 0.0, 7.5, 300.0) +
                                       state_line(1.0, 7.5, 7.5, 300.0) +
                                       state_line(2.0, 15.0, 7.5, 300.0) +
                                       state_line(2.5, 18.75, 7.5, 300.0));
        auto tail_options             = merge_options();
        tail_options.working_time_sec = 10.0;
        tail_options.expected_start =
            *parse_iso8601_utc("2026-12-30T00:00:00.000Z");
        tail_options.expected_end =
            *parse_iso8601_utc("2026-12-30T00:00:02.500Z");
        const auto tail = merge_sar_windows(tail_path, tail_options);
        expect(!tail.windows.empty() &&
                   format_iso8601_utc_ms(tail.windows.back().end) ==
                       "2026-12-30T00:00:02.500Z",
               "R7 non-integer-step tail ends at real report endpoint");

        std::string terminal_body;
        for (int second = 0; second <= 4; ++second) {
            terminal_body += state_line(second, 7.5 * second, 30.0, 300.0);
        }
        auto terminal_options             = merge_options();
        terminal_options.working_time_sec = 10.0;
        const auto terminal               = merge_sar_windows(
            write_file("terminal-seed.txt", terminal_body), terminal_options);
        expect(
            terminal.windows.size() == 1 &&
                format_iso8601_utc_ms(terminal.windows[0].t0) ==
                    "2026-12-30T00:00:04.000Z" &&
                std::fabs(terminal.windows[0].geometry.range_rate_mps) < 1.0e-9,
            "R6 exact report endpoint participates in coarse seed choice");
    }

    std::cout << "==> P strict failures\n";
    {
        bool threw = false;
        try {
            load_sar_state_report(
                write_file("short.txt", "30 Dec 2026 00:00:00.000 1 2 3\n"));
        } catch (const std::exception& error) {
            threw =
                std::string(error.what()).find("line 1") != std::string::npos;
        }
        expect(threw, "R8 short line rejected with line number");

        threw = false;
        try {
            std::string body =
                state_line(1, 0, 7.5, 300.0) + state_line(0, 7.5, 7.5, 300.0);
            load_sar_state_report(write_file("reverse.txt", body));
        } catch (const std::exception& error) {
            threw =
                std::string(error.what()).find("line 2") != std::string::npos;
        }
        expect(threw, "R8 reverse UTC rejected");

        threw = false;
        try {
            std::string body = state_line(0, 0, 7.5, 300.0);
            const auto  pos  = body.find(" 7.5 ");
            body.replace(pos, 5, " nan ");
            body += state_line(1, 7.5, 7.5, 300.0);
            load_sar_state_report(write_file("nan.txt", body));
        } catch (const std::exception& error) {
            threw =
                std::string(error.what()).find("line 1") != std::string::npos;
        }
        expect(threw, "R8 non-finite rejected");
    }

    if (argc == 2) { print_real_report_stats(argv[1]); }
    if (argc == 3) {
        std::cout << "==> G real GMAT RSA replay\n";
        replay_real_result(argv[1], argv[2]);
    }

    std::cout << "==> ac010-harness failures=" << failures << '\n';
    return failures == 0 ? 0 : 1;
}
