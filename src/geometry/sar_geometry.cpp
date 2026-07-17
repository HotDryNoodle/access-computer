#include "geometry/sar_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "planner/time_model.hpp"

namespace mp {

namespace {

constexpr double kPi              = 3.14159265358979323846;
constexpr double kRad2Deg         = 180.0 / kPi;
constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kNormEps         = 1.0e-12;
constexpr double kLookSideEps     = 1.0e-12;

[[noreturn]] void fail(const std::string& message) {
    throw std::runtime_error("SAR geometry: " + message);
}

double dot3(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 sub3(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 cross3(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

Vec3 scale3(const Vec3& v, double scale) {
    return {v.x * scale, v.y * scale, v.z * scale};
}

double norm3(const Vec3& v) { return std::sqrt(dot3(v, v)); }

Vec3 normalize3(const Vec3& v, const std::string& label) {
    const double norm = norm3(v);
    if (!(norm > kNormEps) || !std::isfinite(norm)) {
        fail("degenerate " + label);
    }
    return scale3(v, 1.0 / norm);
}

double strict_number(const std::string& token, std::size_t line_no) {
    std::size_t consumed = 0;
    double      value    = 0.0;
    try {
        value = std::stod(token, &consumed);
    } catch (...) {
        fail("line " + std::to_string(line_no) + ": invalid number '" + token +
             "'");
    }
    if (consumed != token.size() || !std::isfinite(value)) {
        fail("line " + std::to_string(line_no) + ": invalid finite number '" +
             token + "'");
    }
    return value;
}

double unix_seconds(const std::chrono::system_clock::time_point& tp) {
    return std::chrono::duration<double>(tp.time_since_epoch()).count();
}

bool geometry_eligible(const SarGeometry&     geometry,
                       const SarMergeOptions& options) {
    const bool resolved_side =
        geometry.look_side == "left" || geometry.look_side == "right";
    const bool side_ok =
        resolved_side && (options.allowed_look_side == "either" ||
                          geometry.look_side == options.allowed_look_side);
    return geometry.los_clear && side_ok &&
           geometry.incidence_angle_deg >= options.incidence_min_deg &&
           geometry.incidence_angle_deg <= options.incidence_max_deg &&
           std::fabs(geometry.squint_deg) <= options.max_abs_squint_deg &&
           std::fabs(geometry.roll_deg) <= options.roll_max_deg;
}

struct Matrix3 {
    double m[3][3]{};
};

std::array<double, 4> matrix_to_quaternion(const Matrix3& matrix) {
    std::array<double, 4> q{};
    const double trace = matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2];
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q              = {0.25 * s, (matrix.m[2][1] - matrix.m[1][2]) / s,
                          (matrix.m[0][2] - matrix.m[2][0]) / s,
                          (matrix.m[1][0] - matrix.m[0][1]) / s};
    }
    else if (matrix.m[0][0] > matrix.m[1][1] &&
             matrix.m[0][0] > matrix.m[2][2]) {
        const double s =
            std::sqrt(1.0 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2]) *
            2.0;
        q = {(matrix.m[2][1] - matrix.m[1][2]) / s, 0.25 * s,
             (matrix.m[0][1] + matrix.m[1][0]) / s,
             (matrix.m[0][2] + matrix.m[2][0]) / s};
    }
    else if (matrix.m[1][1] > matrix.m[2][2]) {
        const double s =
            std::sqrt(1.0 + matrix.m[1][1] - matrix.m[0][0] - matrix.m[2][2]) *
            2.0;
        q = {(matrix.m[0][2] - matrix.m[2][0]) / s,
             (matrix.m[0][1] + matrix.m[1][0]) / s, 0.25 * s,
             (matrix.m[1][2] + matrix.m[2][1]) / s};
    }
    else {
        const double s =
            std::sqrt(1.0 + matrix.m[2][2] - matrix.m[0][0] - matrix.m[1][1]) *
            2.0;
        q = {(matrix.m[1][0] - matrix.m[0][1]) / s,
             (matrix.m[0][2] + matrix.m[2][0]) / s,
             (matrix.m[1][2] + matrix.m[2][1]) / s, 0.25 * s};
    }
    double q_norm = 0.0;
    for (const double value : q) { q_norm += value * value; }
    q_norm = std::sqrt(q_norm);
    if (!(q_norm > kNormEps) || !std::isfinite(q_norm)) {
        fail("degenerate quaternion");
    }
    for (double& value : q) { value /= q_norm; }
    if (q[0] < 0.0) {
        for (double& value : q) { value = -value; }
    }
    return q;
}

void validate_report_coverage(const std::vector<SarStateNode>& nodes,
                              const SarMergeOptions&           options) {
    constexpr double tolerance_sec = 1.0e-6;
    if (options.expected_start &&
        std::fabs(std::chrono::duration<double>(nodes.front().utc -
                                                *options.expected_start)
                      .count()) > tolerance_sec) {
        fail("state report does not start at task_start");
    }
    if (options.expected_end &&
        std::fabs(std::chrono::duration<double>(nodes.back().utc -
                                                *options.expected_end)
                      .count()) > tolerance_sec) {
        fail("state report does not end at task_end");
    }
}

bool better_seed(const SarGeometry&                           candidate,
                 const std::chrono::system_clock::time_point& candidate_time,
                 const SarGeometry&                           current,
                 const std::chrono::system_clock::time_point& current_time) {
    const double candidate_rate = std::fabs(candidate.range_rate_mps);
    const double current_rate   = std::fabs(current.range_rate_mps);
    if (candidate_rate < current_rate - 1.0e-12) { return true; }
    if (current_rate < candidate_rate - 1.0e-12) { return false; }
    const double candidate_roll = std::fabs(candidate.roll_deg);
    const double current_roll   = std::fabs(current.roll_deg);
    if (candidate_roll < current_roll - 1.0e-12) { return true; }
    if (current_roll < candidate_roll - 1.0e-12) { return false; }
    return candidate_time < current_time;
}

void finish_window(SarAccessWindow* window) {
    window->duration_sec =
        std::chrono::duration<double>(window->end - window->start).count();
}

std::vector<SarAccessWindow> clip_windows(
    const std::vector<SarAccessWindow>& windows, double working_time_sec) {
    if (!(working_time_sec > 0.0)) { return windows; }
    const auto half =
        std::chrono::duration_cast<std::chrono::system_clock::duration>(
            std::chrono::duration<double>(working_time_sec / 2.0));
    std::vector<SarAccessWindow> clipped;
    for (auto window : windows) {
        window.start = std::max(window.start, window.t0 - half);
        window.end   = std::min(window.end, window.t0 + half);
        if (window.end <= window.start) { continue; }
        finish_window(&window);
        clipped.push_back(window);
    }
    return clipped;
}

nlohmann::json geometry_to_json(const SarGeometry& geometry) {
    return {
        {"incidence_angle_deg", geometry.incidence_angle_deg},
        {"look_side", geometry.look_side},
        {"side_look_angle_deg", geometry.side_look_angle_deg},
        {"squint_deg", geometry.squint_deg},
        {"roll_deg", geometry.roll_deg},
        {"pitch_deg", geometry.pitch_deg},
        {"yaw_deg", geometry.yaw_deg},
        {"slant_range_km", geometry.slant_range_km},
        {"range_rate_mps", geometry.range_rate_mps},
        {"doppler_centroid_hz", geometry.doppler_centroid_hz},
        {"los_clear", geometry.los_clear},
    };
}

}  // namespace

std::vector<SarStateNode> load_sar_state_report(
    const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) { fail("state report unavailable: " + path.string()); }

    std::vector<SarStateNode> nodes;
    std::string               line;
    std::size_t               line_no = 0;
    double                    unix0   = 0.0;
    while (std::getline(input, line)) {
        ++line_no;
        std::istringstream       stream(line);
        std::vector<std::string> parts;
        std::string              token;
        while (stream >> token) { parts.push_back(token); }
        if (parts.size() != 19) {
            fail("line " + std::to_string(line_no) +
                 ": expected 19 tokens, got " + std::to_string(parts.size()));
        }
        const std::string utc_text =
            parts[0] + " " + parts[1] + " " + parts[2] + " " + parts[3];
        const auto utc = parse_gmat_utcgregorian(utc_text);
        if (!utc) {
            fail("line " + std::to_string(line_no) +
                 ": invalid UTCGregorian '" + utc_text + "'");
        }
        SarStateNode node;
        node.utc            = *utc;
        node.utc_gregorian  = utc_text;
        node.r_sat          = {strict_number(parts[4], line_no),
                               strict_number(parts[5], line_no),
                               strict_number(parts[6], line_no)};
        node.v_sat          = {strict_number(parts[7], line_no),
                               strict_number(parts[8], line_no),
                               strict_number(parts[9], line_no)};
        node.r_tgt          = {strict_number(parts[10], line_no),
                               strict_number(parts[11], line_no),
                               strict_number(parts[12], line_no)};
        node.v_tgt          = {strict_number(parts[13], line_no),
                               strict_number(parts[14], line_no),
                               strict_number(parts[15], line_no)};
        node.r_normal_point = {strict_number(parts[16], line_no),
                               strict_number(parts[17], line_no),
                               strict_number(parts[18], line_no)};
        const double unix   = unix_seconds(*utc);
        if (nodes.empty()) {
            unix0      = unix;
            node.t_sec = 0.0;
        }
        else {
            node.t_sec = unix - unix0;
            if (!(node.t_sec > nodes.back().t_sec + 1.0e-12)) {
                fail("line " + std::to_string(line_no) +
                     ": UTC not strictly increasing");
            }
        }
        nodes.push_back(node);
    }
    if (nodes.size() < 2) { fail("state report needs at least 2 rows"); }
    return nodes;
}

SarOrientation compute_sar_orientation(const SarStateNode& node,
                                       double center_frequency_hz) {
    if (!(center_frequency_hz > 0.0) || !std::isfinite(center_frequency_hz)) {
        fail("center_frequency_hz must be finite and > 0");
    }
    const Vec3 q = sub3(node.r_tgt, node.r_sat);
    const Vec3 u = normalize3(q, "satellite-to-target LOS");
    const Vec3 normal =
        normalize3(sub3(node.r_normal_point, node.r_tgt), "target normal");
    Vec3 C{}, T{}, D{};
    build_lvlh_frame(node.r_sat, node.v_sat, &C, &T, &D);

    const double ground_los = dot3(normal, scale3(u, -1.0));
    const double incidence =
        std::acos(std::clamp(ground_los, -1.0, 1.0)) * kRad2Deg;
    const double cross             = dot3(u, C);
    const Vec3   relative_velocity = sub3(node.v_tgt, node.v_sat);
    const Vec3   platform_velocity = sub3(node.v_sat, node.v_tgt);
    const Vec3   platform_direction =
        normalize3(platform_velocity, "platform-relative-target velocity");
    const double range_rate_mps = dot3(u, relative_velocity) * 1000.0;
    const double wavelength_m   = kSpeedOfLightMps / center_frequency_hz;

    const double squint =
        std::asin(std::clamp(dot3(u, platform_direction), -1.0, 1.0)) *
        kRad2Deg;
    const Vec3 projected_velocity =
        sub3(platform_velocity, scale3(u, dot3(platform_velocity, u)));
    const double projected_norm = norm3(projected_velocity);
    if (!(projected_norm > kNormEps) || !std::isfinite(projected_norm)) {
        throw SarCandidateInfeasibleError(
            "SAR orientation: degenerate zero-squint azimuth axis");
    }
    const Vec3 x_body = scale3(projected_velocity, 1.0 / projected_norm);
    const Vec3 z_body = u;
    const Vec3 y_body = normalize3(cross3(z_body, x_body), "body Y axis");

    Matrix3    body_to_reference{};
    const Vec3 body_axes[3] = {x_body, y_body, z_body};
    for (int column = 0; column < 3; ++column) {
        body_to_reference.m[0][column] = body_axes[column].x;
        body_to_reference.m[1][column] = body_axes[column].y;
        body_to_reference.m[2][column] = body_axes[column].z;
    }

    const Vec3 local_axes[3] = {T, scale3(C, -1.0), D};
    Matrix3    body_to_local{};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            body_to_local.m[row][column] =
                dot3(local_axes[row], body_axes[column]);
        }
    }
    const double pitch =
        std::asin(std::clamp(-body_to_local.m[2][0], -1.0, 1.0));
    const double roll =
        std::atan2(body_to_local.m[2][1], body_to_local.m[2][2]);
    const double yaw = std::atan2(body_to_local.m[1][0], body_to_local.m[0][0]);

    SarOrientation orientation;
    auto&          geometry      = orientation.geometry;
    geometry.incidence_angle_deg = incidence;
    geometry.look_side           = std::fabs(cross) <= kLookSideEps
                                       ? "degenerate"
                                       : (cross > 0.0 ? "left" : "right");
    geometry.side_look_angle_deg = std::atan2(cross, dot3(u, D)) * kRad2Deg;
    geometry.squint_deg          = squint;
    geometry.roll_deg            = roll * kRad2Deg;
    geometry.pitch_deg           = pitch * kRad2Deg;
    geometry.yaw_deg             = yaw * kRad2Deg;
    geometry.slant_range_km      = norm3(q);
    geometry.range_rate_mps      = range_rate_mps;
    geometry.doppler_centroid_hz = -2.0 * range_rate_mps / wavelength_m;
    geometry.los_clear           = ground_los > 0.0;
    if (!std::isfinite(geometry.incidence_angle_deg) ||
        !std::isfinite(geometry.side_look_angle_deg) ||
        !std::isfinite(geometry.squint_deg) ||
        !std::isfinite(geometry.roll_deg) ||
        !std::isfinite(geometry.pitch_deg) ||
        !std::isfinite(geometry.yaw_deg) ||
        !std::isfinite(geometry.slant_range_km) ||
        !std::isfinite(geometry.range_rate_mps) ||
        !std::isfinite(geometry.doppler_centroid_hz)) {
        fail("non-finite geometry at " + node.utc_gregorian);
    }
    orientation.quaternion_wxyz = matrix_to_quaternion(body_to_reference);
    return orientation;
}

SarGeometry compute_sar_geometry(const SarStateNode& node,
                                 double              center_frequency_hz) {
    return compute_sar_orientation(node, center_frequency_hz).geometry;
}

SarMergeResult merge_sar_windows(const std::filesystem::path& state_path,
                                 const SarMergeOptions&       options) {
    if (!(options.step_sec > 0.0) || !(options.working_time_sec > 0.0) ||
        !(options.incidence_min_deg > 0.0) ||
        !(options.incidence_max_deg > options.incidence_min_deg) ||
        !(options.incidence_max_deg < 90.0) || !(options.roll_max_deg > 0.0) ||
        !(options.center_frequency_hz > 0.0) ||
        !(options.azimuth_beamwidth_deg > 0.0) ||
        !(options.azimuth_beamwidth_deg < 180.0) ||
        !(options.max_abs_squint_deg > 0.0) ||
        !(options.max_abs_squint_deg <= options.azimuth_beamwidth_deg / 2.0) ||
        (options.allowed_look_side != "left" &&
         options.allowed_look_side != "right" &&
         options.allowed_look_side != "either")) {
        fail("invalid merge options");
    }

    const auto nodes = load_sar_state_report(state_path);
    validate_report_coverage(nodes, options);
    for (std::size_t i = 1; i < nodes.size(); ++i) {
        const double delta = nodes[i].t_sec - nodes[i - 1].t_sec;
        if (delta > options.step_sec * 1.5 + 1.0e-9) {
            fail("state report coverage gap before line " +
                 std::to_string(i + 1));
        }
    }

    std::vector<SarAccessWindow> windows;
    SarAccessWindow*             active = nullptr;
    for (std::size_t i = 0; i + 1 < nodes.size(); ++i) {
        SarGeometry geometry;
        bool        eligible = false;
        try {
            geometry =
                compute_sar_orientation(nodes[i], options.center_frequency_hz)
                    .geometry;
            eligible = geometry_eligible(geometry, options);
        } catch (const SarCandidateInfeasibleError&) { eligible = false; }
        const auto row_end = nodes[i + 1].utc;
        if (eligible) {
            if (active == nullptr) {
                windows.push_back({});
                active           = &windows.back();
                active->start    = nodes[i].utc;
                active->end      = row_end;
                active->t0       = nodes[i].utc;
                active->geometry = geometry;
            }
            else {
                active->end = row_end;
                if (better_seed(geometry, nodes[i].utc, active->geometry,
                                active->t0)) {
                    active->t0       = nodes[i].utc;
                    active->geometry = geometry;
                }
            }
        }
        else if (active != nullptr) {
            finish_window(active);
            active = nullptr;
        }
    }
    if (active != nullptr) {
        try {
            const auto terminal_geometry =
                compute_sar_orientation(nodes.back(),
                                        options.center_frequency_hz)
                    .geometry;
            if (geometry_eligible(terminal_geometry, options) &&
                better_seed(terminal_geometry, nodes.back().utc,
                            active->geometry, active->t0)) {
                active->t0       = nodes.back().utc;
                active->geometry = terminal_geometry;
            }
        } catch (const SarCandidateInfeasibleError&) {
            // 终点姿态退化不改变由真实相邻节点形成的已有窗口。
        }
        finish_window(active);
    }

    SarMergeResult result;
    result.windows = clip_windows(windows, options.working_time_sec);
    return result;
}

nlohmann::json sar_windows_to_json(
    const std::vector<SarAccessWindow>& windows) {
    const char*       node_env = std::getenv("SATELLITE_NODE_ID");
    const std::string node_id =
        node_env != nullptr && node_env[0] != '\0' ? node_env : "local";
    nlohmann::json output = nlohmann::json::array();
    for (const auto& window : windows) {
        output.push_back({
            {"start_utc", format_iso8601_utc_ms(window.start)},
            {"end_utc", format_iso8601_utc_ms(window.end)},
            {"duration_sec", window.duration_sec},
            {"t0_utc", format_iso8601_utc_ms(window.t0)},
            {"phi_deg", std::fabs(window.geometry.roll_deg)},
            {"pass_type", window.geometry.look_side},
            {"sar_geometry", sar_geometry_to_json(window.geometry)},
            {"node_id", node_id},
        });
    }
    return output;
}

nlohmann::json sar_geometry_to_json(const SarGeometry& geometry) {
    return geometry_to_json(geometry);
}

}  // namespace mp
