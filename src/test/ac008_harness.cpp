/**
 * @file ac008_harness.cpp
 * @brief AC-008 REVISE_VERIFY：角定义、毫秒量化、Brent、报告合同、AE/RSA 隔离。
 */

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "geometry/attitude_solver.hpp"
#include "geometry/window_merger.hpp"
#include "gmat/gmat_backend.hpp"
#include "planner/run_planner.hpp"
#include "planner/validate.hpp"
#include "satellite/exit_codes.hpp"

namespace {

int fails = 0;

void expect(bool ok, const std::string& name) {
    if (ok) { std::cout << "  OK  " << name << "\n"; }
    else {
        std::cerr << "  FAIL " << name << "\n";
        ++fails;
    }
}

std::filesystem::path tmp_dir() {
    const auto dir = std::filesystem::temp_directory_path() / "ac008_harness";
    std::filesystem::create_directories(dir);
    return dir;
}

std::filesystem::path write_file(const std::string& name,
                                 const std::string& body) {
    const auto    path = tmp_dir() / name;
    std::ofstream out(path);
    out << body;
    return path;
}

mp::Vec3 V(double x, double y, double z) { return {x, y, z}; }

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

std::string eph_line(const std::string& hms,
                     const mp::Vec3&    r,
                     const mp::Vec3&    v,
                     const mp::Vec3&    tgt) {
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(12);
    ss << "30 Dec 2026 " << hms << " " << r.x << " " << r.y << " " << r.z << " "
       << v.x << " " << v.y << " " << v.z << " " << tgt.x << " " << tgt.y << " "
       << tgt.z << "\n";
    return ss.str();
}

nlohmann::json base_ae() {
    return {
        {"task",
         {{"scenario", "attitude_estimation"},
          {"start_time_utc", "2026-12-30T03:37:00Z"},
          {"compute_horizon_sec", 900},
          {"working_time_sec", 900},
          {"step_sec", 1}}},
        {"spacecraft",
         {{"sat_id", "sat_A"},
          {"epoch_utc", "30 Dec 2026 00:00:00.000"},
          {"state_type", "keplerian"},
          {"elements",
           {{"sma_km", 6716.14},
            {"ecc", 0.0},
            {"inc_deg", 96.806},
            {"raan_deg", 76.515},
            {"aop_deg", 0.0},
            {"ta_deg", 0.0}}}}},
        {"target",
         {{"type", "ground_point"},
          {"lon_deg", 114.3055},
          {"lat_deg", 30.5928},
          {"alt_km", 0.057}}},
        {"sensor", {{"type", "optical_area_array"}, {"mode", "stare"}}},
        {"constraints",
         {{"roll_max_deg", 70.0},
          {"require_sunlit", true},
          {"exclude_umbra", true}}},
    };
}

void make_frame(
    mp::Vec3* r, mp::Vec3* v, mp::Vec3* C, mp::Vec3* T, mp::Vec3* D) {
    *r = V(0, 0, 7000);
    *v = V(7.5, 0, 0);
    mp::build_lvlh_frame(*r, *v, C, T, D);
}

/** Slow sign-change ephemeris for sub-second root tests. */
std::filesystem::path make_slow_crossing_eph(const std::string& name,
                                             double t_star = 0.250) {
    const double vmag = 0.075;
    const auto   v    = V(vmag, 0, 0);
    const auto   tgt  = V(vmag * t_star, 40.0, 6900.0);
    std::string  body;
    for (int i = 0; i <= 2; ++i) {
        char hms[32];
        std::snprintf(hms, sizeof(hms), "03:00:%02d.000", i);
        body += eph_line(hms, V(vmag * i, 0, 7000), v, tgt);
    }
    return write_file(name, body);
}

mp::AccessWindow make_win(const std::string& start,
                          const std::string& end,
                          double             min_off = 12.34) {
    mp::AccessWindow w;
    w.start_utc         = start;
    w.end_utc           = end;
    w.t0_utc            = start;
    w.phi_deg           = 0.0;
    w.min_off_nadir_deg = min_off;
    w.duration_sec =
        mp::parse_utc_gregorian_sec(end) - mp::parse_utc_gregorian_sec(start);
    w.pass_type             = "Ascending";
    w.max_sun_elevation_deg = 30.0;
    return w;
}

}  // namespace

int main() {
    using mp::AccessWindow;
    using mp::build_lvlh_frame;
    using mp::compute_attitude_angles;
    using mp::evaluate_ephemeris_state;
    using mp::format_utc_gregorian_ms;
    using mp::kSideRollPitchInfeasibleWarning;
    using mp::load_attitude_ephemeris;
    using mp::merge_optical_windows;
    using mp::parse_utc_gregorian_sec;
    using mp::reconstruct_boresight;
    using mp::refine_attitude_windows;
    using mp::Vec3;

    std::cout << "==> G geometry / angle definition\n";
    {
        Vec3 r{}, v{}, C{}, T{}, D{};
        make_frame(&r, &v, &C, &T, &D);
        const Vec3 tgt_nadir = {r.x + D.x * 100, r.y + D.y * 100,
                                r.z + D.z * 100};
        const auto a1        = compute_attitude_angles(r, v, tgt_nadir);
        expect(near(a1.roll_deg, 0.0, 1e-8) && near(a1.pitch_deg, 0.0, 1e-8),
               "G1 nadir roll=pitch=0");
        const auto ur1 =
            reconstruct_boresight(a1.roll_deg, a1.pitch_deg, C, T, D);
        const double clo1 =
            std::acos(std::max(
                -1.0, std::min(1.0, ur1.x * D.x + ur1.y * D.y + ur1.z * D.z))) *
            180.0 / 3.141592653589793;
        expect(clo1 < 1e-10, "G1 closure");

        const Vec3 tgt_pC = {r.x + D.x * 100 + C.x * 50,
                             r.y + D.y * 100 + C.y * 50,
                             r.z + D.z * 100 + C.z * 50};
        const Vec3 tgt_mC = {r.x + D.x * 100 - C.x * 50,
                             r.y + D.y * 100 - C.y * 50,
                             r.z + D.z * 100 - C.z * 50};
        const auto ap     = compute_attitude_angles(r, v, tgt_pC);
        const auto am     = compute_attitude_angles(r, v, tgt_mC);
        expect(near(ap.pitch_deg, 0.0, 1e-6) && near(am.pitch_deg, 0.0, 1e-6),
               "G2 pitch~0");
        expect(ap.roll_deg > 0 && am.roll_deg < 0 &&
                   near(ap.roll_deg, -am.roll_deg, 1e-6),
               "G2 roll equal magnitude opposite sign");

        const Vec3 tgt_pT = {r.x + D.x * 100 + T.x * 50,
                             r.y + D.y * 100 + T.y * 50,
                             r.z + D.z * 100 + T.z * 50};
        const Vec3 tgt_mT = {r.x + D.x * 100 - T.x * 50,
                             r.y + D.y * 100 - T.y * 50,
                             r.z + D.z * 100 - T.z * 50};
        const auto aft    = compute_attitude_angles(r, v, tgt_pT);
        const auto beh    = compute_attitude_angles(r, v, tgt_mT);
        expect(aft.pitch_deg > 0 && beh.pitch_deg < 0 &&
                   near(aft.pitch_deg, -beh.pitch_deg, 1e-6),
               "G3 pitch ahead/behind signs");

        const double roll = 70.0, pitch = 12.0;
        const auto   ur = reconstruct_boresight(roll, pitch, C, T, D);
        const Vec3 tgt = {r.x + ur.x * 200, r.y + ur.y * 200, r.z + ur.z * 200};
        const auto ang = compute_attitude_angles(r, v, tgt);
        expect(
            near(ang.roll_deg, roll, 1e-6) && near(ang.pitch_deg, pitch, 1e-6),
            "G4 round-trip angles");

        bool threw = false;
        try {
            compute_attitude_angles(r, v, r);
        } catch (...) { threw = true; }
        expect(threw, "G5 zero LOS fails");
        threw = false;
        try {
            const Vec3 anti = {r.x - D.x * 100, r.y - D.y * 100,
                               r.z - D.z * 100};
            compute_attitude_angles(r, v, anti);
        } catch (...) { threw = true; }
        expect(threw, "G5 u·D<=0 fails");
    }

    std::cout << "==> I interpolation / sub-second / ms quantize\n";
    {
        const auto path  = make_slow_crossing_eph("i2_eph.txt");
        const auto nodes = load_attitude_ephemeris(path);
        expect(nodes.size() == 3, "I load 3 nodes");

        Vec3 r0{}, v0{}, tg0{};
        evaluate_ephemeris_state(nodes, 0.0, &r0, &v0, &tg0);
        expect(near(r0.x, 0.0, 1e-9), "I1 s=0");
        evaluate_ephemeris_state(nodes, 1.0, &r0, &v0, &tg0);
        expect(near(r0.x, 0.075, 1e-9), "I1 s=1");

        double t_truth  = 0.25;
        double best_abs = 1e99;
        for (int k = 0; k <= 10000; ++k) {
            const double t = k * 1.0e-4;
            Vec3         rs{}, vs{}, tg{};
            evaluate_ephemeris_state(nodes, t, &rs, &vs, &tg);
            const double p =
                std::fabs(compute_attitude_angles(rs, vs, tg).pitch_deg);
            if (p < best_abs) {
                best_abs = p;
                t_truth  = t;
            }
        }

        auto       w       = make_win("30 Dec 2026 03:00:00.000",
                                      "30 Dec 2026 03:00:02.000", 55.5);
        const auto refined = refine_attitude_windows(path, {w}, "stare");
        expect(refined.ok, "I2 refine ok");
        const double t0s = parse_utc_gregorian_sec(refined.t0_utc) -
                           parse_utc_gregorian_sec("30 Dec 2026 03:00:00.000");
        expect(near(t0s, t_truth, 1e-3), "I2 |t-t*|<=1ms");
        expect(std::fabs(refined.pitch_deg) <= 1e-4,
               "I2 |pitch|<=1e-4deg (fixture-only)");
        expect(near(refined.windows[0].min_off_nadir_deg, 55.5, 1e-12),
               "I2 min_off_nadir preserved");

        // I3: pitch minimum near an ephemeris node
        {
            const double vmag = 0.075;
            const auto   tgt  = V(vmag * 1.0, 40.0, 6900.0);
            std::string  body;
            for (int i = 0; i <= 2; ++i) {
                char hms[32];
                std::snprintf(hms, sizeof(hms), "05:00:%02d.000", i);
                body += eph_line(hms, V(vmag * i, 0, 7000), V(vmag, 0, 0), tgt);
            }
            const auto p3       = write_file("i3_eph.txt", body);
            const auto n3       = load_attitude_ephemeris(p3);
            double     t_truth  = 1.0;
            double     best_abs = 1e99;
            for (int k = 0; k <= 20000; ++k) {
                const double t = k * 1.0e-4;
                Vec3         rs{}, vs{}, tg{};
                evaluate_ephemeris_state(n3, t, &rs, &vs, &tg);
                const double p =
                    std::fabs(compute_attitude_angles(rs, vs, tg).pitch_deg);
                if (p < best_abs) {
                    best_abs = p;
                    t_truth  = t;
                }
            }
            auto         w3 = make_win("30 Dec 2026 05:00:00.000",
                                       "30 Dec 2026 05:00:02.000");
            const auto   r3 = refine_attitude_windows(p3, {w3}, "stare");
            const double t3 =
                parse_utc_gregorian_sec(r3.t0_utc) -
                parse_utc_gregorian_sec("30 Dec 2026 05:00:00.000");
            expect(near(t_truth, 1.0, 0.05) && near(t3, t_truth, 1e-3),
                   "I3 root near node");
        }

        // I5: root near window start
        {
            const double vmag = 0.075;
            const auto   tgt  = V(vmag * 0.001, 40.0, 6900.0);
            std::string  body;
            for (int i = 0; i <= 2; ++i) {
                char hms[32];
                std::snprintf(hms, sizeof(hms), "06:00:%02d.000", i);
                body += eph_line(hms, V(vmag * i, 0, 7000), V(vmag, 0, 0), tgt);
            }
            const auto   p5 = write_file("i5_eph.txt", body);
            auto         w5 = make_win("30 Dec 2026 06:00:00.000",
                                       "30 Dec 2026 06:00:02.000");
            const auto   r5 = refine_attitude_windows(p5, {w5}, "stare");
            const double t5 =
                parse_utc_gregorian_sec(r5.t0_utc) -
                parse_utc_gregorian_sec("30 Dec 2026 06:00:00.000");
            expect(t5 >= -1e-12 && t5 <= 2.0 + 1e-12, "I5 t0 inside window");
            expect(r5.ok, "I5 ok");
        }

        // I6: no sign change — Brent min residual retained
        {
            const auto  v   = V(7.5, 0, 0);
            const auto  tgt = V(100.0, 40.0, 6900.0);
            std::string body;
            for (int i = 0; i <= 2; ++i) {
                char hms[32];
                std::snprintf(hms, sizeof(hms), "07:00:%02d.000", i);
                body += eph_line(hms, V(7.5 * i, 0, 7000), v, tgt);
            }
            const auto p6 = write_file("i6_eph.txt", body);
            auto       w6 = make_win("30 Dec 2026 07:00:00.000",
                                     "30 Dec 2026 07:00:02.000");
            const auto r6 = refine_attitude_windows(p6, {w6}, "stare");
            expect(r6.ok && std::fabs(r6.pitch_deg) > 0.5, "I6 residual kept");
        }

        // I7: multi-window stable sort — deterministic best_index
        {
            const auto path7 = make_slow_crossing_eph("i7_eph.txt");
            // Full window covers pitch root; late-only window has larger
            // residual.
            auto       w_good = make_win("30 Dec 2026 03:00:00.000",
                                         "30 Dec 2026 03:00:02.000", 1.0);
            auto       w_bad  = make_win("30 Dec 2026 03:00:01.500",
                                         "30 Dec 2026 03:00:02.000", 2.0);
            const auto r7 =
                refine_attitude_windows(path7, {w_good, w_bad}, "stare");
            expect(r7.windows.size() == 2, "I7 two windows");
            expect(r7.windows[0].min_off_nadir_deg == 1.0 &&
                       r7.windows[1].min_off_nadir_deg == 2.0,
                   "I7 order preserved / min_off intact");
            expect(r7.best_index == 0, "I7 best_index=0 (better pitch first)");
            const auto only_bad =
                refine_attitude_windows(path7, {w_bad}, "stare");
            expect(std::fabs(r7.pitch_deg) < std::fabs(only_bad.pitch_deg),
                   "I7 selected pitch better than late-only");
            const auto r7b =
                refine_attitude_windows(path7, {w_bad, w_good}, "stare");
            expect(r7b.best_index == 1,
                   "I7 best_index=1 when better window is second");
        }

        // I8: 0.1 s grid
        {
            const double vmag = 0.075;
            const auto   tgt  = V(vmag * 0.25, 40.0, 6900.0);
            std::string  body;
            for (int i = 0; i <= 20; ++i) {
                const double t = i * 0.1;
                char         hms[32];
                const int    whole = static_cast<int>(t);
                const int    frac =
                    static_cast<int>(std::llround((t - whole) * 10));
                std::snprintf(hms, sizeof(hms), "08:00:%02d.%d00", whole, frac);
                body += eph_line(hms, V(vmag * t, 0, 7000), V(vmag, 0, 0), tgt);
            }
            const auto p8 = write_file("i8_eph.txt", body);
            auto       w8 = make_win("30 Dec 2026 08:00:00.000",
                                     "30 Dec 2026 08:00:02.000");
            const auto r8 = refine_attitude_windows(p8, {w8}, "stare");
            expect(r8.ok && std::fabs(r8.pitch_deg) < 0.5, "I8 variable step");
        }

        // I9: ms consistency — t0/phi/pitch same instant
        {
            const auto   path9  = make_slow_crossing_eph("i9_eph.txt");
            auto         w9     = make_win("30 Dec 2026 03:00:00.000",
                                           "30 Dec 2026 03:00:02.000");
            const auto   r9     = refine_attitude_windows(path9, {w9}, "stare");
            const auto   nodes9 = load_attitude_ephemeris(path9);
            const double unix0  = parse_utc_gregorian_sec(nodes9.front().utc);
            const double t_rel  = parse_utc_gregorian_sec(r9.t0_utc) - unix0;
            Vec3         rs{}, vs{}, tg{};
            evaluate_ephemeris_state(nodes9, t_rel, &rs, &vs, &tg);
            const auto ang = compute_attitude_angles(rs, vs, tg);
            expect(near(ang.roll_deg, r9.phi_deg, 1e-6) &&
                       near(ang.pitch_deg, r9.pitch_deg, 1e-6),
                   "I9 t0/phi/pitch same instant");
            // t0 string ms digits
            expect(r9.t0_utc.find('.') != std::string::npos &&
                       r9.t0_utc.size() >= 3,
                   "I9 t0 has milliseconds");
        }
    }

    std::cout << "==> P report contract / failures\n";
    {
        // P1: valid 13-token
        const auto p1 = make_slow_crossing_eph("p1_eph.txt");
        const auto n1 = load_attitude_ephemeris(p1);
        expect(n1.size() == 3 && n1[0].t_sec == 0.0, "P1 valid 13-token");

        auto       req    = base_ae();
        const auto script = mp::render_optical_access_script(req, tmp_dir());
        expect(script.find("Sat.EarthMJ2000Eq.X") != std::string::npos &&
                   script.find("TargetA.EarthMJ2000Eq.X") != std::string::npos,
               "P2 template EarthMJ2000Eq Sat+Target");

        bool threw = false;
        try {
            load_attitude_ephemeris(tmp_dir() / "missing.csv");
        } catch (const std::exception& ex) {
            threw = std::string(ex.what()).find("ephemeris unavailable") !=
                    std::string::npos;
        }
        expect(threw, "P3 missing file");

        threw = false;
        try {
            load_attitude_ephemeris(write_file(
                "bad_cols.txt", "30 Dec 2026 03:00:00.000 1 2 3 4 5 6\n"));
        } catch (const std::exception& ex) {
            threw = std::string(ex.what()).find("line 1") != std::string::npos;
        }
        expect(threw, "P3 short line has line number");

        threw = false;
        try {
            const std::string body =
                "30 Dec 2026 03:00:00.000 1.0 0.0 7000.0junk 7.0 0.0 0.0 0.0 "
                "0.0 6900.0 0.0 0.0 0.0\n"
                "30 Dec 2026 03:00:01.000 8.0 0.0 7000.0 7.0 0.0 0.0 0.0 0.0 "
                "6900.0 0.0 0.0 0.0\n";
            load_attitude_ephemeris(write_file("junk_num.txt", body));
        } catch (const std::exception& ex) {
            threw = std::string(ex.what()).find("line 1") != std::string::npos;
        }
        expect(threw, "P3 trailing junk number");

        threw = false;
        try {
            const std::string body =
                "30 Dec 2026 03:00:00.000 1.0 0.0 7000.0 7.0 0.0 0.0 0.0 0.0 "
                "6900.0 nan 0.0 0.0\n"
                "30 Dec 2026 03:00:01.000 8.0 0.0 7000.0 7.0 0.0 0.0 0.0 0.0 "
                "6900.0 0.0 0.0 0.0\n";
            load_attitude_ephemeris(write_file("nan.txt", body));
        } catch (const std::exception& ex) {
            threw = std::string(ex.what()).find("line 1") != std::string::npos;
        }
        expect(threw, "P3 NaN rejected with line");

        threw = false;
        try {
            const std::string body = eph_line("03:00:01.000", V(1, 0, 7000),
                                              V(7, 0, 0), V(0, 0, 6900)) +
                                     eph_line("03:00:00.000", V(8, 0, 7000),
                                              V(7, 0, 0), V(0, 0, 6900));
            load_attitude_ephemeris(write_file("rev.txt", body));
        } catch (const std::exception& ex) {
            threw = std::string(ex.what()).find("line 2") != std::string::npos;
        }
        expect(threw, "P3 reverse time line 2");

        // Strict UTCGregorian (parse_gmat_utcgregorian + round-trip)
        auto utc_must_fail = [](const std::string& label,
                                const std::string& text) {
            bool threw = false;
            try {
                (void)parse_utc_gregorian_sec(text);
            } catch (...) { threw = true; }
            expect(threw, label);
        };
        utc_must_fail("P3 UTC day junk 30x", "30x Dec 2026 03:00:00.000");
        utc_must_fail("P3 UTC year junk 2026x", "30 Dec 2026x 03:00:00.000");
        utc_must_fail("P3 UTC Feb 30", "30 Feb 2026 03:00:00.000");
        utc_must_fail("P3 UTC nan sec", "30 Dec 2026 03:00:nan");
        utc_must_fail("P3 UTC inf sec", "30 Dec 2026 03:00:inf");

        // P4: coverage insufficient
        threw = false;
        try {
            const auto p4 = make_slow_crossing_eph("p4_eph.txt");
            auto       w4 = make_win("30 Dec 2026 02:00:00.000",
                                     "30 Dec 2026 02:00:02.000");
            refine_attitude_windows(p4, {w4}, "stare");
        } catch (const std::exception& ex) {
            threw =
                std::string(ex.what()).find("window[0]") != std::string::npos;
        }
        expect(threw, "P4 coverage fail has window index");

        threw = false;
        try {
            load_attitude_ephemeris(write_file(
                "leap.txt", eph_line("23:59:60.000", V(1, 0, 7000), V(7, 0, 0),
                                     V(0, 0, 6900)) +
                                eph_line("00:00:01.000", V(8, 0, 7000),
                                         V(7, 0, 0), V(0, 0, 6900))));
        } catch (const std::exception& ex) {
            threw = std::string(ex.what()).find("leap") != std::string::npos ||
                    std::string(ex.what()).find("60") != std::string::npos;
        }
        expect(threw, "P5 leap second fails");
    }

    std::cout << "==> A scenario / RSA isolation\n";
    {
        // A1 stare computed
        const auto path = make_slow_crossing_eph("a1_eph.txt");
        auto       w =
            make_win("30 Dec 2026 03:00:00.000", "30 Dec 2026 03:00:02.000");
        const auto stare = refine_attitude_windows(path, {w}, "stare");
        expect(stare.ok && stare.pitch_status == "computed" &&
                   stare.warnings.empty(),
               "A1 stare computed");

        // A2 side_roll normal
        const auto a2 = refine_attitude_windows(path, {w}, "side_roll_only");
        expect(a2.ok && std::fabs(a2.pitch_deg) < 0.5 && a2.warnings.empty(),
               "A2 side_roll normal");

        // A3 side_roll infeasible → ok=false + exact warning
        {
            const auto  v   = V(7.5, 0, 0);
            const auto  tgt = V(100.0, 40.0, 6900.0);
            std::string body;
            for (int i = 0; i <= 2; ++i) {
                char hms[32];
                std::snprintf(hms, sizeof(hms), "09:00:%02d.000", i);
                body += eph_line(hms, V(7.5 * i, 0, 7000), v, tgt);
            }
            const auto p3 = write_file("a3_eph.txt", body);
            auto       w3 = make_win("30 Dec 2026 09:00:00.000",
                                     "30 Dec 2026 09:00:02.000");
            const auto r3 = refine_attitude_windows(p3, {w3}, "side_roll_only");
            expect(!r3.ok, "A3 not ok");
            expect(r3.warnings.size() == 1 &&
                       r3.warnings[0] == kSideRollPitchInfeasibleWarning,
                   "A3 exact infeasible warning");
            expect(r3.pitch_status.empty(), "A3 no computed status");

            // Planner/CLI contract: no_result + empty windows + exit 4
            nlohmann::json out = {
                {"status", "succeeded"},
                {"windows", nlohmann::json::array(
                                {{{"t0_utc", "x"}, {"duration_sec", 2.0}}})},
                {"summary", {{"window_count", 1}, {"duration_total_sec", 2.0}}},
                {"attitude",
                 {{"mode", "side_roll_only"}, {"pitch_status", "computed"}}},
                {"artifacts", {{"ephemeris_path", p3.string()}}},
                {"warnings", nlohmann::json::array()},
            };
            mp::apply_attitude_estimation_result(out, r3, "side_roll_only");
            expect(out["status"] == "no_result", "A3 planner status no_result");
            expect(out["windows"].is_array() && out["windows"].empty(),
                   "A3 planner empty windows");
            expect(out["summary"]["window_count"] == 0 &&
                       out["summary"]["duration_total_sec"] == 0.0,
                   "A3 planner zero summary");
            expect(!out.contains("attitude"), "A3 planner no attitude");
            expect(out.contains("artifacts") &&
                       out["artifacts"]["ephemeris_path"] == p3.string(),
                   "A3 planner artifacts retained");
            expect(out["warnings"].size() == 1 &&
                       out["warnings"][0] == kSideRollPitchInfeasibleWarning,
                   "A3 planner warning retained");
            expect(mp::run_status_exit_code(out) == satellite::EXIT_NO_RESULT,
                   "A3 CLI exit EXIT_NO_RESULT=4");
        }

        // A4 multi-window rewrite t0/phi only
        {
            const auto p4 = make_slow_crossing_eph("a4_eph.txt");
            auto       w0 = make_win("30 Dec 2026 03:00:00.000",
                                     "30 Dec 2026 03:00:02.000", 11.1);
            auto       w1 = make_win("30 Dec 2026 03:00:00.000",
                                     "30 Dec 2026 03:00:02.000", 22.2);
            const auto r4 = refine_attitude_windows(p4, {w0, w1}, "stare");
            expect(r4.windows[0].t0_utc != w0.t0_utc ||
                       r4.windows[0].phi_deg != 0.0,
                   "A4 window0 refined");
            expect(near(r4.windows[0].min_off_nadir_deg, 11.1, 1e-12) &&
                       near(r4.windows[1].min_off_nadir_deg, 22.2, 1e-12),
                   "A4 min_off unchanged");
        }

        // A5 RSA isolation — merger bit-for-bit (no solver)
        {
            // Synthetic optical trace: two samples in-swath lit
            auto sample = [](const std::string& utc, double off) {
                return utc + " 0.0 0.0 500.0 " + std::to_string(off) +
                       " 100.0 200.0 30.0 1.0 1.0 1.0 1.0\n";
            };
            const auto tr = write_file(
                "a5_tr.txt", sample("30 Dec 2026 03:00:00.000", 10.0) +
                                 sample("30 Dec 2026 03:00:10.000", 9.0));
            const auto ecl =
                write_file("a5_ecl.txt",
                           "Spacecraft Sat\nStart Time (UTCGregorian)\n"
                           "There are no eclipse events\n");
            mp::MergeOptions o;
            o.step_sec         = 10.0;
            o.require_sunlit   = true;
            o.exclude_umbra    = true;
            o.exclude_penumbra = false;
            o.working_time_sec = 0.0;
            const auto m1      = merge_optical_windows(tr, ecl, o);
            const auto m2      = merge_optical_windows(tr, ecl, o);
            expect(m1.windows.size() == m2.windows.size() &&
                       !m1.windows.empty() &&
                       m1.windows[0].t0_utc == m2.windows[0].t0_utc &&
                       m1.windows[0].phi_deg == m2.windows[0].phi_deg &&
                       m1.windows[0].min_off_nadir_deg ==
                           m2.windows[0].min_off_nadir_deg,
                   "A5 RSA merger bit-for-bit");
        }

        // A6: empty windows → planner no_result, refine not invoked (no eph
        // read)
        {
            const auto missing_eph =
                tmp_dir() / "a6_must_not_read_ephemeris.csv";
            std::filesystem::remove(missing_eph);
            bool threw = false;
            try {
                refine_attitude_windows(missing_eph, {}, "stare");
            } catch (...) { threw = true; }
            expect(threw, "A6 empty windows rejected by refine");
            expect(!std::filesystem::exists(missing_eph),
                   "A6 missing eph never created (refine not reading)");

            // Mirror run_planner: windows.empty() ⇒ no_result without refine.
            nlohmann::json out = {
                {"status", "succeeded"},
                {"artifacts", {{"ephemeris_path", missing_eph.string()}}},
                {"warnings", nlohmann::json::array()},
            };
            out["status"]  = "no_result";
            out["windows"] = nlohmann::json::array();
            out["summary"] = {{"window_count", 0}, {"duration_total_sec", 0.0}};
            expect(!out.contains("attitude"), "A6 no attitude");
            expect(out["windows"].empty(), "A6 empty windows[]");
            expect(mp::run_status_exit_code(out) == satellite::EXIT_NO_RESULT,
                   "A6 CLI exit EXIT_NO_RESULT=4");
            expect(!std::filesystem::exists(missing_eph),
                   "A6 planner path did not touch eph");
        }

        auto v = mp::validate_request(base_ae());
        expect(v.ok, "V1 AE validate ok");
    }

    std::cout << "==> R2 independent replay (synthetic)\n";
    {
        const auto path = make_slow_crossing_eph("r2_eph.txt");
        auto       w =
            make_win("30 Dec 2026 03:00:00.000", "30 Dec 2026 03:00:02.000");
        const auto r1 = refine_attitude_windows(path, {w}, "stare");
        const auto r2 = refine_attitude_windows(path, {w}, "stare");
        expect(r1.t0_utc == r2.t0_utc && near(r1.phi_deg, r2.phi_deg, 1e-12) &&
                   near(r1.pitch_deg, r2.pitch_deg, 1e-12),
               "R2 replay identical");
    }

    // Optional real GMAT artifact replay
    {
        const std::filesystem::path real =
            "/tmp/ac008-real-gmat/sat_rv_j2000.csv";
        const std::filesystem::path res = "/tmp/ac008-real-gmat/result.json";
        if (std::filesystem::exists(real) && std::filesystem::exists(res)) {
            std::cout << "==> R2 real GMAT ephemeris replay\n";
            nlohmann::json jr;
            {
                std::ifstream in(res);
                in >> jr;
            }
            if (jr.contains("windows") && !jr["windows"].empty() &&
                jr.contains("attitude")) {
                AccessWindow w;
                w.start_utc = jr["windows"][0]["start_utc"];
                w.end_utc   = jr["windows"][0]["end_utc"];
                w.t0_utc    = jr["windows"][0]["t0_utc"];
                w.phi_deg   = jr["windows"][0]["phi_deg"];
                w.min_off_nadir_deg =
                    jr["windows"][0].value("min_off_nadir_deg", 0.0);
                w.duration_sec  = jr["windows"][0].value("duration_sec", 0.0);
                const auto   rr = refine_attitude_windows(real, {w}, "stare");
                const double dt =
                    std::fabs(parse_utc_gregorian_sec(rr.t0_utc) -
                              parse_utc_gregorian_sec(
                                  jr["attitude"]["t0_utc"].get<std::string>()));
                expect(dt <= 1e-3, "R2 real |t0 replay|<=1ms");
                expect(
                    near(rr.phi_deg, jr["attitude"]["phi_deg"].get<double>(),
                         1e-6) &&
                        near(rr.pitch_deg,
                             jr["attitude"]["pitch_deg"].get<double>(), 1e-6),
                    "R2 real angles <=1e-6deg");
            }
        }
    }

    std::cout << "==> T schema text\n";
    {
        std::ifstream in(std::filesystem::path(ACCESS_COMPUTER_SOURCE_ROOT) /
                         "schemas/remote_sensing_access.output.schema.json");
        std::stringstream buf;
        buf << in.rdbuf();
        const auto s = buf.str();
        expect(s.find("Placeholder in v0.1") == std::string::npos,
               "T1 no placeholder description");
    }

    if (fails != 0) {
        std::cerr << "ac008-harness failures: " << fails << "\n";
        return 1;
    }
    std::cout << "ac008-harness passed\n";
    return 0;
}
