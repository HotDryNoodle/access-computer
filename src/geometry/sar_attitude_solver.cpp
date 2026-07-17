#include "geometry/sar_attitude_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include "planner/time_model.hpp"

namespace mp {

namespace {

constexpr double kNormEps = 1.0e-12;
using Milliseconds        = std::chrono::milliseconds;
using TimePoint           = std::chrono::system_clock::time_point;

[[noreturn]] void fail(const std::string& message) {
    throw std::runtime_error("SAR attitude solver: " + message);
}

double dot3(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 add3(const Vec3& a, const Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 sub3(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
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

void hermite(const Vec3& r0,
             const Vec3& v0,
             const Vec3& r1,
             const Vec3& v1,
             double      segment_sec,
             double      fraction,
             Vec3*       position,
             Vec3*       velocity) {
    const double s   = std::clamp(fraction, 0.0, 1.0);
    const double s2  = s * s;
    const double s3  = s2 * s;
    const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
    const double h10 = s3 - 2.0 * s2 + s;
    const double h01 = -2.0 * s3 + 3.0 * s2;
    const double h11 = s3 - s2;
    *position = add3(add3(scale3(r0, h00), scale3(v0, h10 * segment_sec)),
                     add3(scale3(r1, h01), scale3(v1, h11 * segment_sec)));

    const double dh00 = 6.0 * s2 - 6.0 * s;
    const double dh10 = 3.0 * s2 - 4.0 * s + 1.0;
    const double dh01 = -6.0 * s2 + 6.0 * s;
    const double dh11 = 3.0 * s2 - 2.0 * s;
    *velocity = add3(add3(scale3(r0, dh00 / segment_sec), scale3(v0, dh10)),
                     add3(scale3(r1, dh01 / segment_sec), scale3(v1, dh11)));
}

double relative_seconds(const std::chrono::system_clock::time_point& time,
                        const std::chrono::system_clock::time_point& origin) {
    return std::chrono::duration<double>(time - origin).count();
}

double range_rate_at(const std::vector<SarStateNode>& nodes,
                     double                           t_sec,
                     double /*center_frequency_hz*/) {
    const auto state = evaluate_sar_state(nodes, t_sec);
    const auto los   = normalize3(sub3(state.r_tgt, state.r_sat), "boresight");
    return dot3(los, sub3(state.v_tgt, state.v_sat)) * 1000.0;
}

/** Brent-Dekker 括区间求根。 */
double brent_root(const std::vector<SarStateNode>& nodes,
                  double                           lower,
                  double                           upper,
                  double                           center_frequency_hz) {
    double a  = lower;
    double b  = upper;
    double fa = range_rate_at(nodes, a, center_frequency_hz);
    double fb = range_rate_at(nodes, b, center_frequency_hz);
    if (fa == 0.0) { return a; }
    if (fb == 0.0) { return b; }
    if (fa * fb > 0.0) { fail("internal unbracketed Brent root"); }
    if (std::fabs(fa) < std::fabs(fb)) {
        std::swap(a, b);
        std::swap(fa, fb);
    }

    double c      = a;
    double fc     = fa;
    double d      = c;
    bool   bisect = true;
    for (int iteration = 0; iteration < 100; ++iteration) {
        double s = 0.0;
        if (fa != fc && fb != fc) {
            s = a * fb * fc / ((fa - fb) * (fa - fc)) +
                b * fa * fc / ((fb - fa) * (fb - fc)) +
                c * fa * fb / ((fc - fa) * (fc - fb));
        }
        else { s = b - fb * (b - a) / (fb - fa); }

        const double min_ab     = std::min((3.0 * a + b) / 4.0, b);
        const double max_ab     = std::max((3.0 * a + b) / 4.0, b);
        const bool   condition1 = s < min_ab || s > max_ab;
        const bool   condition2 =
            bisect && std::fabs(s - b) >= 0.5 * std::fabs(b - c);
        const bool condition3 =
            !bisect && std::fabs(s - b) >= 0.5 * std::fabs(c - d);
        const bool condition4 = bisect && std::fabs(b - c) < 1.0e-10;
        const bool condition5 = !bisect && std::fabs(c - d) < 1.0e-10;
        if (condition1 || condition2 || condition3 || condition4 ||
            condition5 || !std::isfinite(s)) {
            s      = 0.5 * (a + b);
            bisect = true;
        }
        else { bisect = false; }

        const double fs = range_rate_at(nodes, s, center_frequency_hz);
        d               = c;
        c               = b;
        fc              = fb;
        if (fa * fs < 0.0) {
            b  = s;
            fb = fs;
        }
        else {
            a  = s;
            fa = fs;
        }
        if (std::fabs(fa) < std::fabs(fb)) {
            std::swap(a, b);
            std::swap(fa, fb);
        }
        if (fb == 0.0 || std::fabs(b - a) <= 1.0e-10) { return b; }
    }
    return b;
}

/** Brent 1973 有界最小化（距离率平方）。 */
double brent_minimize_rate2(const std::vector<SarStateNode>& nodes,
                            double                           lower,
                            double                           upper,
                            double center_frequency_hz) {
    const auto objective = [&](double t) {
        const double rate = range_rate_at(nodes, t, center_frequency_hz);
        return rate * rate;
    };
    constexpr double golden = 0.3819660112501051;
    double           a      = lower;
    double           b      = upper;
    double           x      = a + golden * (b - a);
    double           w      = x;
    double           v      = x;
    double           fx     = objective(x);
    double           fw     = fx;
    double           fv     = fx;
    double           d      = 0.0;
    double           e      = 0.0;
    for (int iteration = 0; iteration < 100; ++iteration) {
        const double midpoint  = 0.5 * (a + b);
        const double tolerance = 1.0e-12 * std::fabs(x) + 1.0e-11;
        if (std::fabs(x - midpoint) <= 2.0 * tolerance - 0.5 * (b - a)) {
            return x;
        }
        bool parabolic = false;
        if (std::fabs(e) > tolerance) {
            const double r = (x - w) * (fx - fv);
            double       q = (x - v) * (fx - fw);
            double       p = (x - v) * q - (x - w) * r;
            q              = 2.0 * (q - r);
            if (q > 0.0) { p = -p; }
            q                       = std::fabs(q);
            const double previous_e = e;
            e                       = d;
            if (std::fabs(p) < std::fabs(0.5 * q * previous_e) &&
                p > q * (a - x) && p < q * (b - x)) {
                d         = p / q;
                parabolic = true;
            }
        }
        if (!parabolic) {
            e = x < midpoint ? b - x : a - x;
            d = golden * e;
        }
        const double u  = x + (std::fabs(d) >= tolerance
                                   ? d
                                   : (d > 0.0 ? tolerance : -tolerance));
        const double fu = objective(u);
        if (fu <= fx) {
            if (u < x) { b = x; }
            else { a = x; }
            v  = w;
            fv = fw;
            w  = x;
            fw = fx;
            x  = u;
            fx = fu;
        }
        else {
            if (u < x) { a = u; }
            else { b = u; }
            if (fu <= fw || w == x) {
                v  = w;
                fv = fw;
                w  = u;
                fw = fu;
            }
            else if (fu <= fv || v == x || v == w) {
                v  = u;
                fv = fu;
            }
        }
    }
    return x;
}

bool geometry_allowed(const SarGeometry&        geometry,
                      const SarAttitudeOptions& options) {
    const bool resolved_side =
        geometry.look_side == "left" || geometry.look_side == "right";
    const bool side_ok =
        resolved_side && (options.allowed_look_side == "either" ||
                          geometry.look_side == options.allowed_look_side);
    return geometry.los_clear && side_ok &&
           geometry.incidence_angle_deg >= options.incidence_min_deg &&
           geometry.incidence_angle_deg <= options.incidence_max_deg &&
           std::fabs(geometry.squint_deg) <= options.max_abs_squint_deg &&
           std::fabs(geometry.roll_deg) <= options.roll_max_deg &&
           std::fabs(geometry.range_rate_mps) <= options.max_abs_range_rate_mps;
}

bool geometry_window_allowed(const SarGeometry&        geometry,
                             const SarAttitudeOptions& options) {
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

struct Candidate {
    double       t_sec = 0.0;
    SarStateNode state;
    SarGeometry  geometry;
    SarAttitude  attitude;
};

bool better_candidate(const Candidate& candidate, const Candidate& current) {
    const double candidate_rate = std::fabs(candidate.geometry.range_rate_mps);
    const double current_rate   = std::fabs(current.geometry.range_rate_mps);
    if (candidate_rate < current_rate - 1.0e-12) { return true; }
    if (current_rate < candidate_rate - 1.0e-12) { return false; }
    const double candidate_roll = std::fabs(candidate.geometry.roll_deg);
    const double current_roll   = std::fabs(current.geometry.roll_deg);
    if (candidate_roll < current_roll - 1.0e-12) { return true; }
    if (current_roll < candidate_roll - 1.0e-12) { return false; }
    return candidate.t_sec < current.t_sec;
}

bool geometry_feasible_at(const std::vector<SarStateNode>& nodes,
                          double                           t_sec,
                          const SarAttitudeOptions&        options) {
    try {
        const auto state = evaluate_sar_state(nodes, t_sec);
        const auto orientation =
            compute_sar_orientation(state, options.center_frequency_hz);
        return geometry_window_allowed(orientation.geometry, options);
    } catch (const SarCandidateInfeasibleError&) { return false; }
}

/**
 * @brief 在绝对 UTC 毫秒网格上寻找首个不可行点前的可执行端点。
 *
 * @note t0 必须已在绝对毫秒网格且几何可行。逐点扫描保证不会跨过任何
 * 公共 1 ms 执行网格上的短暂不可行区；非毫秒 limit 仅用于限制扫描范围。
 */
TimePoint find_geometry_boundary(const std::vector<SarStateNode>& nodes,
                                 const TimePoint&                 origin,
                                 TimePoint                        t0,
                                 TimePoint                        limit,
                                 int                              direction,
                                 const SarAttitudeOptions&        options) {
    if (direction != -1 && direction != 1) {
        fail("geometry boundary direction must be -1 or 1");
    }
    TimePoint  current = t0;
    const auto stride  = Milliseconds(direction);
    while ((direction < 0 && current > limit) ||
           (direction > 0 && current < limit)) {
        const auto probe = current + stride;
        if ((direction < 0 && probe < limit) ||
            (direction > 0 && probe > limit)) {
            return limit;
        }
        const double probe_sec = relative_seconds(probe, origin);
        if (!geometry_feasible_at(nodes, probe_sec, options)) {
            return current;
        }
        current = probe;
    }
    return limit;
}

}  // namespace

SarStateNode evaluate_sar_state(const std::vector<SarStateNode>& nodes,
                                double                           t_sec) {
    if (nodes.size() < 2) { fail("state report needs at least 2 rows"); }
    if (t_sec < nodes.front().t_sec - 1.0e-12 ||
        t_sec > nodes.back().t_sec + 1.0e-12) {
        fail("time outside state report coverage");
    }
    const auto upper =
        std::upper_bound(nodes.begin(), nodes.end(), t_sec,
                         [](double time, const SarStateNode& node) {
                             return time < node.t_sec;
                         });
    std::size_t index = 0;
    if (upper == nodes.end()) { index = nodes.size() - 2; }
    else if (upper != nodes.begin()) {
        index = static_cast<std::size_t>(upper - nodes.begin() - 1);
    }
    const auto&  first       = nodes[index];
    const auto&  second      = nodes[index + 1];
    const double segment_sec = second.t_sec - first.t_sec;
    if (!(segment_sec > 0.0)) { fail("non-positive state segment"); }
    const double fraction = (t_sec - first.t_sec) / segment_sec;

    SarStateNode state;
    state.t_sec = t_sec;
    hermite(first.r_sat, first.v_sat, second.r_sat, second.v_sat, segment_sec,
            fraction, &state.r_sat, &state.v_sat);
    hermite(first.r_tgt, first.v_tgt, second.r_tgt, second.v_tgt, segment_sec,
            fraction, &state.r_tgt, &state.v_tgt);
    const Vec3 first_normal =
        normalize3(sub3(first.r_normal_point, first.r_tgt), "first normal");
    const Vec3 second_normal =
        normalize3(sub3(second.r_normal_point, second.r_tgt), "second normal");
    const Vec3 normal    = normalize3(add3(scale3(first_normal, 1.0 - fraction),
                                           scale3(second_normal, fraction)),
                                      "interpolated normal");
    state.r_normal_point = add3(state.r_tgt, normal);
    state.utc            = nodes.front().utc +
                std::chrono::duration_cast<std::chrono::system_clock::duration>(
                    std::chrono::duration<double>(t_sec));
    state.utc_gregorian = format_gmat_utcgregorian(state.utc);
    return state;
}

SarAttitudeRefineResult refine_sar_attitude(
    const std::filesystem::path& state_path,
    const nlohmann::json&        selected_window,
    const SarAttitudeOptions&    options) {
    if (!(options.step_sec > 0.0) || !(options.working_time_sec > 0.0) ||
        !(options.incidence_min_deg > 0.0) ||
        !(options.incidence_max_deg > options.incidence_min_deg) ||
        !(options.incidence_max_deg < 90.0) || !(options.roll_max_deg > 0.0) ||
        !(options.center_frequency_hz > 0.0) ||
        !(options.azimuth_beamwidth_deg > 0.0) ||
        !(options.azimuth_beamwidth_deg < 180.0) ||
        !(options.max_abs_squint_deg > 0.0) ||
        !(options.max_abs_squint_deg <= options.azimuth_beamwidth_deg / 2.0) ||
        !(options.max_abs_range_rate_mps > 0.0)) {
        fail("invalid solver options");
    }
    const auto       nodes                  = load_sar_state_report(state_path);
    constexpr double coverage_tolerance_sec = 1.0e-6;
    if (options.expected_start &&
        std::fabs(std::chrono::duration<double>(nodes.front().utc -
                                                *options.expected_start)
                      .count()) > coverage_tolerance_sec) {
        fail("state report does not start at task_start");
    }
    if (options.expected_end &&
        std::fabs(std::chrono::duration<double>(nodes.back().utc -
                                                *options.expected_end)
                      .count()) > coverage_tolerance_sec) {
        fail("state report does not end at task_end");
    }
    for (std::size_t i = 1; i < nodes.size(); ++i) {
        const double delta = nodes[i].t_sec - nodes[i - 1].t_sec;
        if (delta > options.step_sec * 1.5 + 1.0e-9) {
            fail("state report coverage gap before line " +
                 std::to_string(i + 1));
        }
    }
    const auto start =
        parse_iso8601_utc(selected_window.at("start_utc").get<std::string>());
    const auto end =
        parse_iso8601_utc(selected_window.at("end_utc").get<std::string>());
    if (!start || !end || !(*start < *end)) {
        fail("invalid selected_window UTC");
    }
    const double lower = relative_seconds(*start, nodes.front().utc);
    const double upper = relative_seconds(*end, nodes.front().utc);
    if (lower < nodes.front().t_sec - 1.0e-9 ||
        upper > nodes.back().t_sec + 1.0e-9) {
        fail("selected_window outside state report coverage");
    }

    std::vector<double> continuous_candidates{lower, upper};
    for (std::size_t i = 0; i + 1 < nodes.size(); ++i) {
        const double segment_lower = std::max(lower, nodes[i].t_sec);
        const double segment_upper = std::min(upper, nodes[i + 1].t_sec);
        if (!(segment_lower < segment_upper)) { continue; }
        const double rate_lower =
            range_rate_at(nodes, segment_lower, options.center_frequency_hz);
        const double rate_upper =
            range_rate_at(nodes, segment_upper, options.center_frequency_hz);
        if (rate_lower == 0.0 || rate_upper == 0.0 ||
            rate_lower * rate_upper < 0.0) {
            continuous_candidates.push_back(
                brent_root(nodes, segment_lower, segment_upper,
                           options.center_frequency_hz));
        }
        else {
            continuous_candidates.push_back(
                brent_minimize_rate2(nodes, segment_lower, segment_upper,
                                     options.center_frequency_hz));
        }
    }

    std::vector<TimePoint> millisecond_candidates;
    for (const double continuous : continuous_candidates) {
        const auto continuous_time =
            nodes.front().utc +
            std::chrono::duration_cast<std::chrono::system_clock::duration>(
                std::chrono::duration<double>(continuous));
        const auto floor_ms = std::chrono::floor<Milliseconds>(continuous_time);
        const auto ceil_ms  = std::chrono::ceil<Milliseconds>(continuous_time);
        if (floor_ms >= *start && floor_ms <= *end) {
            millisecond_candidates.push_back(floor_ms);
        }
        if (ceil_ms >= *start && ceil_ms <= *end) {
            millisecond_candidates.push_back(ceil_ms);
        }
    }
    std::sort(millisecond_candidates.begin(), millisecond_candidates.end());
    millisecond_candidates.erase(std::unique(millisecond_candidates.begin(),
                                             millisecond_candidates.end()),
                                 millisecond_candidates.end());

    std::vector<Candidate> feasible;
    for (const auto& candidate_time : millisecond_candidates) {
        try {
            Candidate    candidate;
            const double t_sec =
                relative_seconds(candidate_time, nodes.front().utc);
            candidate.t_sec     = t_sec;
            candidate.state     = evaluate_sar_state(nodes, t_sec);
            candidate.state.utc = candidate_time;
            candidate.state.utc_gregorian =
                format_gmat_utcgregorian(candidate_time);
            const auto orientation = compute_sar_orientation(
                candidate.state, options.center_frequency_hz);
            candidate.geometry = orientation.geometry;
            if (!geometry_allowed(candidate.geometry, options)) { continue; }
            candidate.attitude.quaternion_wxyz = orientation.quaternion_wxyz;
            candidate.attitude.roll_deg        = orientation.geometry.roll_deg;
            candidate.attitude.pitch_deg       = orientation.geometry.pitch_deg;
            candidate.attitude.yaw_deg         = orientation.geometry.yaw_deg;
            candidate.attitude.squint_deg = orientation.geometry.squint_deg;
            feasible.push_back(candidate);
        } catch (const SarCandidateInfeasibleError&) {
            // 仅明确的候选姿态退化可降级为不可行；其余异常必须上抛。
        }
    }

    SarAttitudeRefineResult result;
    if (feasible.empty()) {
        result.warnings.push_back(kSarAttitudeNoFeasibleWarning);
        return result;
    }
    Candidate best = feasible.front();
    for (std::size_t i = 1; i < feasible.size(); ++i) {
        if (better_candidate(feasible[i], best)) { best = feasible[i]; }
    }

    const auto half =
        std::chrono::duration_cast<std::chrono::system_clock::duration>(
            std::chrono::duration<double>(options.working_time_sec / 2.0));
    const auto geometry_search_start = std::max(*start, best.state.utc - half);
    const auto geometry_search_end   = std::min(*end, best.state.utc + half);
    const auto refined_window_start =
        find_geometry_boundary(nodes, nodes.front().utc, best.state.utc,
                               geometry_search_start, -1, options);
    const auto refined_window_end =
        find_geometry_boundary(nodes, nodes.front().utc, best.state.utc,
                               geometry_search_end, 1, options);
    const auto quantized_window_start =
        std::chrono::ceil<std::chrono::milliseconds>(refined_window_start);
    const auto quantized_window_end =
        std::chrono::floor<std::chrono::milliseconds>(refined_window_end);

    result.ok              = true;
    result.attitude        = best.attitude;
    result.window.start    = quantized_window_start;
    result.window.end      = quantized_window_end;
    result.window.t0       = best.state.utc;
    result.window.geometry = best.geometry;
    result.window.duration_sec =
        std::chrono::duration<double>(result.window.end - result.window.start)
            .count();
    if (!(result.window.duration_sec > 0.0)) {
        result = {};
        result.warnings.push_back(kSarAttitudeNoFeasibleWarning);
    }
    return result;
}

nlohmann::json sar_attitude_to_json(const std::string& t0_utc,
                                    const SarAttitude& attitude,
                                    const SarGeometry& geometry) {
    return {
        {"mode", "stripmap"},
        {"attitude_status", "computed"},
        {"t0_utc", t0_utc},
        {"reference_frame", "EarthMJ2000Eq"},
        {"quaternion_body_to_reference",
         {{"order", "wxyz"}, {"values", attitude.quaternion_wxyz}}},
        {"roll_deg", attitude.roll_deg},
        {"pitch_deg", attitude.pitch_deg},
        {"yaw_deg", attitude.yaw_deg},
        {"squint_deg", attitude.squint_deg},
        {"incidence_angle_deg", geometry.incidence_angle_deg},
        {"look_side", geometry.look_side},
        {"side_look_angle_deg", geometry.side_look_angle_deg},
        {"slant_range_km", geometry.slant_range_km},
        {"range_rate_mps", geometry.range_rate_mps},
        {"doppler_centroid_hz", geometry.doppler_centroid_hz},
    };
}

}  // namespace mp
