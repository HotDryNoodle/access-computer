#include "geometry/attitude_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "planner/time_model.hpp"

namespace mp {

namespace {

constexpr double kPi              = 3.14159265358979323846;
constexpr double kRad2Deg         = 180.0 / kPi;
constexpr double kDeg2Rad         = kPi / 180.0;
constexpr double kMs              = 1.0e-3;
constexpr double kPitchRootTolRad = 1.0e-8;
constexpr double kNormEps         = 1.0e-12;
constexpr double kSideRollMaxDeg  = 0.5;

[[noreturn]] void fail(const std::string& msg) {
    throw std::runtime_error("attitude solver: " + msg);
}

double norm3(const Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vec3 scale3(const Vec3& v, double s) { return {v.x * s, v.y * s, v.z * s}; }

Vec3 add3(const Vec3& a, const Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 sub3(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

double dot3(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross3(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

Vec3 normalize3(const Vec3& v, const char* what) {
    const double n = norm3(v);
    if (!(n > kNormEps) || !std::isfinite(n)) {
        fail(std::string("degenerate ") + what);
    }
    return scale3(v, 1.0 / n);
}

bool all_finite(const Vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

double parse_strict_double(const std::string& s, std::size_t line_no) {
    if (s.empty()) {
        fail("line " + std::to_string(line_no) + ": empty numeric token");
    }
    std::size_t idx = 0;
    double      v   = 0.0;
    try {
        v = std::stod(s, &idx);
    } catch (...) {
        fail("line " + std::to_string(line_no) + ": invalid number '" + s +
             "'");
    }
    if (idx != s.size()) {
        fail("line " + std::to_string(line_no) + ": trailing junk in number '" +
             s + "'");
    }
    if (!std::isfinite(v)) {
        fail("line " + std::to_string(line_no) + ": non-finite number '" + s +
             "'");
    }
    return v;
}

const char* month_abbr(int mon) {
    static const char* kMonths[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    if (mon < 1 || mon > 12) { return "???"; }
    return kMonths[mon - 1];
}

void hermite_state(const EphemerisNode& a,
                   const EphemerisNode& b,
                   double               t,
                   Vec3*                r_sat,
                   Vec3*                v_sat,
                   Vec3*                r_tgt) {
    const double h = b.t_sec - a.t_sec;
    if (!(h > 0.0)) { fail("non-positive ephemeris segment"); }
    const double s = (t - a.t_sec) / h;
    if (s < -1.0e-12 || s > 1.0 + 1.0e-12) {
        fail("Hermite evaluation outside segment");
    }
    const double sc = std::min(1.0, std::max(0.0, s));
    const double s2 = sc * sc;
    const double s3 = s2 * sc;

    const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
    const double h10 = s3 - 2.0 * s2 + sc;
    const double h01 = -2.0 * s3 + 3.0 * s2;
    const double h11 = s3 - s2;

    *r_sat = add3(add3(scale3(a.r_sat, h00), scale3(a.v_sat, h10 * h)),
                  add3(scale3(b.r_sat, h01), scale3(b.v_sat, h11 * h)));

    const double dh00 = 6.0 * s2 - 6.0 * sc;
    const double dh10 = 3.0 * s2 - 4.0 * sc + 1.0;
    const double dh01 = -6.0 * s2 + 6.0 * sc;
    const double dh11 = 3.0 * s2 - 2.0 * sc;
    *v_sat = add3(add3(scale3(a.r_sat, dh00 / h), scale3(a.v_sat, dh10)),
                  add3(scale3(b.r_sat, dh01 / h), scale3(b.v_sat, dh11)));

    *r_tgt = add3(a.r_tgt, scale3(sub3(b.r_tgt, a.r_tgt), sc));
}

struct EvalPack {
    AttitudeAngles ang;
    Vec3           C{};
    Vec3           T{};
    Vec3           D{};
    Vec3           u{};
};

EvalPack eval_at(const std::vector<EphemerisNode>& nodes, double t_sec) {
    Vec3 r_sat{}, v_sat{}, r_tgt{};
    evaluate_ephemeris_state(nodes, t_sec, &r_sat, &v_sat, &r_tgt);
    Vec3 C{}, T{}, D{};
    build_lvlh_frame(r_sat, v_sat, &C, &T, &D);
    const auto ang = compute_attitude_angles(r_sat, v_sat, r_tgt);
    const Vec3 u   = normalize3(sub3(r_tgt, r_sat), "LOS");
    return {ang, C, T, D, u};
}

struct Candidate {
    double      t_sec     = 0.0;
    double      pitch_deg = 0.0;
    double      roll_deg  = 0.0;
    std::size_t win_index = 0;
};

bool better_candidate(const Candidate& a, const Candidate& b) {
    const double ap = std::fabs(a.pitch_deg);
    const double bp = std::fabs(b.pitch_deg);
    if (ap < bp - 1.0e-15) { return true; }
    if (bp < ap - 1.0e-15) { return false; }
    const double ar = std::fabs(a.roll_deg);
    const double br = std::fabs(b.roll_deg);
    if (ar < br - 1.0e-15) { return true; }
    if (br < ar - 1.0e-15) { return false; }
    return a.t_sec < b.t_sec;
}

double find_root_bracket(const std::vector<EphemerisNode>& nodes,
                         double                            t_lo,
                         double                            t_hi,
                         double                            g_lo,
                         double                            g_hi) {
    double a  = t_lo;
    double b  = t_hi;
    double fa = g_lo;
    double fb = g_hi;
    double t  = 0.5 * (a + b);
    for (int i = 0; i < 64; ++i) {
        if (std::fabs(fb - fa) > 1.0e-18) {
            t = b - fb * (b - a) / (fb - fa);
            if (t <= a || t >= b) { t = 0.5 * (a + b); }
        }
        else { t = 0.5 * (a + b); }

        const auto   pack      = eval_at(nodes, t);
        const double g         = pack.ang.u_dot_t;
        const double pitch_rad = pack.ang.pitch_deg * kDeg2Rad;
        if (std::fabs(pitch_rad) <= kPitchRootTolRad) { return t; }
        if (std::fabs(b - a) <= 1.0e-6 &&
            std::fabs(pitch_rad) <= 1.0e-4 * kDeg2Rad) {
            return t;
        }
        if (std::fabs(b - a) <= 1.0e-9) { return t; }
        if (fa * g <= 0.0) {
            b  = t;
            fb = g;
        }
        else {
            a  = t;
            fa = g;
        }
    }
    return t;
}

/** Brent 1973 有界最小化（抛物线插值 + 黄金分割回退）。 */
double brent_minimize_pitch2(const std::vector<EphemerisNode>& nodes,
                             double                            t_lo,
                             double                            t_hi) {
    auto f = [&](double t) {
        const double p = eval_at(nodes, t).ang.pitch_deg;
        return p * p;
    };

    const double gold = 0.5 * (3.0 - std::sqrt(5.0));
    const double tol  = 1.0e-12;

    double a  = t_lo;
    double b  = t_hi;
    double x  = a + gold * (b - a);
    double w  = x;
    double v  = x;
    double fx = f(x);
    double fw = fx;
    double fv = fx;
    double d  = 0.0;
    double e  = 0.0;

    for (int iter = 0; iter < 100; ++iter) {
        const double m    = 0.5 * (a + b);
        const double tol1 = tol * std::fabs(x) + 1.0e-14;
        const double tol2 = 2.0 * tol1;
        if (std::fabs(x - m) <= tol2 - 0.5 * (b - a) || (b - a) <= kMs) {
            return x;
        }

        double u;
        double fu;
        bool   used_parab = false;
        if (std::fabs(e) > tol1) {
            // Parabolic fit through x, w, v
            const double r = (x - w) * (fx - fv);
            double       q = (x - v) * (fx - fw);
            double       p = (x - v) * q - (x - w) * r;
            q              = 2.0 * (q - r);
            if (q > 0.0) { p = -p; }
            q                   = std::fabs(q);
            const double e_prev = e;
            e                   = d;
            if (std::fabs(p) < std::fabs(0.5 * q * e_prev) && p > q * (a - x) &&
                p < q * (b - x)) {
                d = p / q;
                u = x + d;
                if (u - a < tol2 || b - u < tol2) {
                    d = (m > x) ? tol1 : -tol1;
                }
                used_parab = true;
            }
        }
        if (!used_parab) {
            e = (x >= m) ? a - x : b - x;
            d = gold * e;
        }
        u  = x + ((std::fabs(d) >= tol1) ? d : ((d > 0) ? tol1 : -tol1));
        fu = f(u);

        if (fu <= fx) {
            if (u >= x) { a = x; }
            else { b = x; }
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

Candidate quantize_ms_in_window(const std::vector<EphemerisNode>& nodes,
                                double                            t_cont,
                                double                            rel_lo,
                                double                            rel_hi,
                                std::size_t                       win_index) {
    const double t_floor = std::floor(t_cont / kMs + 1.0e-15) * kMs;
    const double t_ceil  = t_floor + kMs;

    std::vector<Candidate> ms_cands;
    auto                   try_add = [&](double t) {
        if (t < rel_lo - 1.0e-12 || t > rel_hi + 1.0e-12) { return; }
        if (std::fabs(t - rel_lo) <= 1.0e-12) { t = rel_lo; }
        if (std::fabs(t - rel_hi) <= 1.0e-12) { t = rel_hi; }
        // Snap to exact ms grid relative to 0 for interior points
        if (t > rel_lo + 1.0e-12 && t < rel_hi - 1.0e-12) {
            t = std::round(t / kMs) * kMs;
        }
        const auto ang = eval_at(nodes, t).ang;
        ms_cands.push_back({t, ang.pitch_deg, ang.roll_deg, win_index});
    };
    try_add(t_floor);
    if (std::fabs(t_ceil - t_floor) > 1.0e-15) { try_add(t_ceil); }

    if (ms_cands.empty()) {
        fail("window[" + std::to_string(win_index) +
             "]: no feasible millisecond candidate inside window");
    }

    Candidate best = ms_cands.front();
    for (std::size_t i = 1; i < ms_cands.size(); ++i) {
        if (better_candidate(ms_cands[i], best)) { best = ms_cands[i]; }
    }

    // Closure at chosen ms
    Vec3 r_sat{}, v_sat{}, r_tgt{};
    evaluate_ephemeris_state(nodes, best.t_sec, &r_sat, &v_sat, &r_tgt);
    Vec3 C{}, T{}, D{};
    build_lvlh_frame(r_sat, v_sat, &C, &T, &D);
    const auto ang = compute_attitude_angles(r_sat, v_sat, r_tgt);
    const Vec3 u   = normalize3(sub3(r_tgt, r_sat), "LOS");
    const Vec3 ur = reconstruct_boresight(ang.roll_deg, ang.pitch_deg, C, T, D);
    const double clo =
        std::acos(std::max(-1.0, std::min(1.0, dot3(ur, u)))) * kRad2Deg;
    if (!(clo < 1.0e-6)) {
        fail("window[" + std::to_string(win_index) + "] boresight closure " +
             std::to_string(clo) + " deg");
    }
    best.pitch_deg = ang.pitch_deg;
    best.roll_deg  = ang.roll_deg;
    return best;
}

Candidate refine_one_window(const std::vector<EphemerisNode>& nodes,
                            const AccessWindow&               win,
                            std::size_t                       win_index) {
    const double t_start = parse_utc_gregorian_sec(win.start_utc);
    const double t_end   = parse_utc_gregorian_sec(win.end_utc);
    if (!(t_end > t_start)) {
        fail("window[" + std::to_string(win_index) + "] empty or inverted");
    }

    const double unix0  = parse_utc_gregorian_sec(nodes.front().utc);
    const double rel_lo = t_start - unix0;
    const double rel_hi = t_end - unix0;
    if (rel_lo < nodes.front().t_sec - 1.0e-9 ||
        rel_hi > nodes.back().t_sec + 1.0e-9) {
        fail("window[" + std::to_string(win_index) +
             "] not covered by ephemeris");
    }

    std::vector<double> sample_t;
    sample_t.push_back(rel_lo);
    for (const auto& n : nodes) {
        if (n.t_sec > rel_lo + 1.0e-12 && n.t_sec < rel_hi - 1.0e-12) {
            sample_t.push_back(n.t_sec);
        }
    }
    sample_t.push_back(rel_hi);
    std::sort(sample_t.begin(), sample_t.end());
    sample_t.erase(std::unique(sample_t.begin(), sample_t.end()),
                   sample_t.end());
    if (sample_t.size() < 2) {
        fail("window[" + std::to_string(win_index) +
             "] lacks interpolation segment");
    }

    std::vector<double> g_vals;
    g_vals.reserve(sample_t.size());
    for (double t : sample_t) {
        g_vals.push_back(eval_at(nodes, t).ang.u_dot_t);
    }

    std::vector<Candidate> cands;
    for (std::size_t i = 0; i < sample_t.size(); ++i) {
        if (std::fabs(g_vals[i]) <= 1.0e-15) {
            const auto ang = eval_at(nodes, sample_t[i]).ang;
            cands.push_back(
                {sample_t[i], ang.pitch_deg, ang.roll_deg, win_index});
        }
    }
    for (std::size_t i = 0; i + 1 < sample_t.size(); ++i) {
        if (g_vals[i] * g_vals[i + 1] < 0.0) {
            const double tr = find_root_bracket(
                nodes, sample_t[i], sample_t[i + 1], g_vals[i], g_vals[i + 1]);
            const auto ang = eval_at(nodes, tr).ang;
            cands.push_back({tr, ang.pitch_deg, ang.roll_deg, win_index});
        }
    }

    if (cands.empty()) {
        Candidate best{};
        bool      have     = false;
        auto      consider = [&](double t) {
            const auto ang = eval_at(nodes, t).ang;
            Candidate  c{t, ang.pitch_deg, ang.roll_deg, win_index};
            if (!have || better_candidate(c, best)) {
                best = c;
                have = true;
            }
        };
        consider(rel_lo);
        consider(rel_hi);
        for (std::size_t i = 0; i + 1 < sample_t.size(); ++i) {
            consider(
                brent_minimize_pitch2(nodes, sample_t[i], sample_t[i + 1]));
        }
        if (!have) {
            fail("window[" + std::to_string(win_index) +
                 "] minimization failed");
        }
        cands.push_back(best);
    }

    Candidate best = cands.front();
    for (std::size_t i = 1; i < cands.size(); ++i) {
        if (better_candidate(cands[i], best)) { best = cands[i]; }
    }

    return quantize_ms_in_window(nodes, best.t_sec, rel_lo, rel_hi, win_index);
}

}  // namespace

void build_lvlh_frame(
    const Vec3& r_sat, const Vec3& v_sat, Vec3* C, Vec3* T, Vec3* D) {
    if (!all_finite(r_sat) || !all_finite(v_sat)) {
        fail("non-finite satellite state");
    }
    const Vec3 R       = normalize3(r_sat, "|r_sat|");
    *D                 = scale3(R, -1.0);
    const double v_r   = dot3(v_sat, R);
    const Vec3   t_raw = sub3(v_sat, scale3(R, v_r));
    *T                 = normalize3(t_raw, "along-track T");
    *C                 = normalize3(cross3(*T, *D), "cross-track C");
}

AttitudeAngles compute_attitude_angles(const Vec3& r_sat,
                                       const Vec3& v_sat,
                                       const Vec3& r_tgt) {
    if (!all_finite(r_tgt)) { fail("non-finite target position"); }
    Vec3 C{}, T{}, D{};
    build_lvlh_frame(r_sat, v_sat, &C, &T, &D);
    const Vec3   q = sub3(r_tgt, r_sat);
    const Vec3   u = normalize3(q, "LOS");
    const double c = dot3(u, C);
    const double a = dot3(u, T);
    const double d = dot3(u, D);
    if (!(d > 0.0) || !std::isfinite(d)) {
        fail("u·D <= 0 (LOS not in nadir hemisphere)");
    }
    AttitudeAngles out;
    out.u_dot_c   = c;
    out.u_dot_t   = a;
    out.u_dot_d   = d;
    out.roll_deg  = std::atan2(c, d) * kRad2Deg;
    out.pitch_deg = std::atan2(a, std::hypot(c, d)) * kRad2Deg;
    if (!std::isfinite(out.roll_deg) || !std::isfinite(out.pitch_deg)) {
        fail("non-finite attitude angles");
    }
    return out;
}

Vec3 reconstruct_boresight(double      roll_deg,
                           double      pitch_deg,
                           const Vec3& C,
                           const Vec3& T,
                           const Vec3& D) {
    const double roll  = roll_deg * kDeg2Rad;
    const double pitch = pitch_deg * kDeg2Rad;
    const double cp    = std::cos(pitch);
    const double sp    = std::sin(pitch);
    const double cr    = std::cos(roll);
    const double sr    = std::sin(roll);
    return normalize3(
        add3(add3(scale3(C, cp * sr), scale3(T, sp)), scale3(D, cp * cr)),
        "reconstructed boresight");
}

double parse_utc_gregorian_sec(const std::string& text) {
    // Reuse AC-004 parse_gmat_utcgregorian：完整消费 + calendar round-trip。
    const auto tp = parse_gmat_utcgregorian(text);
    if (!tp) { fail("invalid UTCGregorian: " + text); }
    const double unix =
        std::chrono::duration<double>(tp->time_since_epoch()).count();
    if (!std::isfinite(unix)) { fail("non-finite UTCGregorian: " + text); }
    return unix;
}

std::string format_utc_gregorian_ms(double unix_sec) {
    if (!std::isfinite(unix_sec)) { fail("non-finite unix time"); }
    // Assume already on 1 ms grid; avoid re-rounding drift.
    const double snapped = std::round(unix_sec / kMs) * kMs;
    const auto sec_i = static_cast<std::time_t>(std::floor(snapped + 1.0e-12));
    int        ms    = static_cast<int>(
        std::llround((snapped - static_cast<double>(sec_i)) * 1000.0));
    if (ms < 0) { ms = 0; }
    if (ms >= 1000) { ms = 999; }
    std::tm tm{};
    gmtime_r(&sec_i, &tm);
    std::ostringstream ss;
    ss << tm.tm_mday << ' ' << month_abbr(tm.tm_mon + 1) << ' '
       << (tm.tm_year + 1900) << ' ' << std::setfill('0') << std::setw(2)
       << tm.tm_hour << ':' << std::setw(2) << tm.tm_min << ':' << std::setw(2)
       << tm.tm_sec << '.' << std::setw(3) << ms;
    return ss.str();
}

std::vector<EphemerisNode> load_attitude_ephemeris(
    const std::filesystem::path& path) {
    if (!std::filesystem::exists(path)) {
        fail("ephemeris unavailable: " + path.string());
    }
    std::ifstream in(path);
    if (!in) { fail("ephemeris unavailable: cannot open " + path.string()); }

    std::vector<EphemerisNode> nodes;
    std::string                line;
    std::size_t                line_no = 0;
    double                     unix0   = 0.0;
    bool                       have0   = false;

    while (std::getline(in, line)) {
        ++line_no;
        if (line.empty()) { continue; }
        std::istringstream       ls(line);
        std::vector<std::string> parts;
        std::string              tok;
        while (ls >> tok) { parts.push_back(tok); }
        if (parts.size() != 13) {
            fail("line " + std::to_string(line_no) +
                 ": expected 13 tokens, got " + std::to_string(parts.size()));
        }
        try {
            const std::string utc =
                parts[0] + " " + parts[1] + " " + parts[2] + " " + parts[3];
            if (parts[3].find(":60") != std::string::npos) {
                fail("line " + std::to_string(line_no) +
                     ": leap second not supported");
            }
            const double  unix = parse_utc_gregorian_sec(utc);
            EphemerisNode n;
            n.utc   = utc;
            n.r_sat = {parse_strict_double(parts[4], line_no),
                       parse_strict_double(parts[5], line_no),
                       parse_strict_double(parts[6], line_no)};
            n.v_sat = {parse_strict_double(parts[7], line_no),
                       parse_strict_double(parts[8], line_no),
                       parse_strict_double(parts[9], line_no)};
            n.r_tgt = {parse_strict_double(parts[10], line_no),
                       parse_strict_double(parts[11], line_no),
                       parse_strict_double(parts[12], line_no)};
            if (!have0) {
                unix0   = unix;
                have0   = true;
                n.t_sec = 0.0;
            }
            else {
                n.t_sec = unix - unix0;
                if (!(n.t_sec > nodes.back().t_sec + 1.0e-12)) {
                    fail("line " + std::to_string(line_no) +
                         ": UTC not strictly increasing");
                }
            }
            nodes.push_back(n);
        } catch (const std::exception& ex) {
            const std::string msg = ex.what();
            if (msg.rfind("attitude solver:", 0) == 0) { throw; }
            fail("line " + std::to_string(line_no) + ": " + msg);
        }
    }
    if (nodes.size() < 2) { fail("ephemeris needs at least 2 rows"); }
    return nodes;
}

void evaluate_ephemeris_state(const std::vector<EphemerisNode>& nodes,
                              double                            t_sec,
                              Vec3*                             r_sat,
                              Vec3*                             v_sat,
                              Vec3*                             r_tgt) {
    if (nodes.size() < 2) { fail("ephemeris empty"); }
    if (t_sec < nodes.front().t_sec - 1.0e-12 ||
        t_sec > nodes.back().t_sec + 1.0e-12) {
        fail("time outside ephemeris coverage");
    }
    std::size_t i = 0;
    while (i + 1 < nodes.size() && nodes[i + 1].t_sec < t_sec) { ++i; }
    if (i + 1 >= nodes.size()) { i = nodes.size() - 2; }
    hermite_state(nodes[i], nodes[i + 1], t_sec, r_sat, v_sat, r_tgt);
}

AttitudeRefineResult refine_attitude_windows(
    const std::filesystem::path&     ephemeris_path,
    const std::vector<AccessWindow>& windows,
    const std::string&               mode) {
    if (windows.empty()) { fail("internal: refine called with empty windows"); }
    const auto   nodes = load_attitude_ephemeris(ephemeris_path);
    const double unix0 = parse_utc_gregorian_sec(nodes.front().utc);

    AttitudeRefineResult result;
    result.windows      = windows;
    result.pitch_status = "computed";
    result.ok           = true;

    std::vector<Candidate> all_best;
    all_best.reserve(windows.size());
    for (std::size_t i = 0; i < windows.size(); ++i) {
        const double    preserved_min_off = windows[i].min_off_nadir_deg;
        const Candidate c = refine_one_window(nodes, windows[i], i);
        all_best.push_back(c);
        result.windows[i].t0_utc  = format_utc_gregorian_ms(unix0 + c.t_sec);
        result.windows[i].phi_deg = c.roll_deg;
        result.windows[i].min_off_nadir_deg = preserved_min_off;
    }

    Candidate best = all_best.front();
    for (std::size_t i = 1; i < all_best.size(); ++i) {
        if (better_candidate(all_best[i], best)) { best = all_best[i]; }
    }
    result.best_index = best.win_index;
    result.t0_utc     = result.windows[best.win_index].t0_utc;
    result.phi_deg    = best.roll_deg;
    result.pitch_deg  = best.pitch_deg;

    if (mode == "side_roll_only" &&
        !(std::fabs(best.pitch_deg) < kSideRollMaxDeg)) {
        result.ok           = false;
        result.pitch_status = "";
        result.warnings.push_back(kSideRollPitchInfeasibleWarning);
    }
    return result;
}

}  // namespace mp
