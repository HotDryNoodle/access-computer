/**
 * @file ac006_harness.cpp
 * @brief AC-006：光照/本影/半影三旗与 D9 best-effort（合成 fixture）。
 */

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "geometry/window_merger.hpp"
#include "gmat/gmat_backend.hpp"
#include "planner/validate.hpp"

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
    const auto dir = std::filesystem::temp_directory_path() / "ac006_harness";
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

/** Trace columns: UTC(4) lat lon alt offNadir gndRange swathHalf alpha inSwath
 * lit geom vn */
std::string sample(const std::string& utc,
                   double             in_swath,
                   double             lit,
                   double             off_nadir = 1.0) {
    // geomVisible unused by merger (uses in_swath); keep = in_swath
    return utc + " 0.0 0.0 500.0 " + std::to_string(off_nadir) +
           " 100.0 200.0 30.0 " + std::to_string(in_swath) + " " +
           std::to_string(lit) + " " + std::to_string(in_swath) + " 1.0\n";
}

std::string two_sample_trace(double in_swath, double lit) {
    return sample("30 Dec 2026 03:00:00.000", in_swath, lit) +
           sample("30 Dec 2026 03:00:10.000", in_swath, lit);
}

std::string eclipse_line(const std::string& kind) {
    return "30 Dec 2026 02:59:00.000 30 Dec 2026 03:01:00.000 " + kind + "\n";
}

mp::MergeOptions base_opts() {
    mp::MergeOptions o;
    o.step_sec         = 10.0;
    o.require_sunlit   = true;
    o.exclude_umbra    = true;
    o.exclude_penumbra = false;
    o.working_time_sec = 0.0;  // no clip
    return o;
}

nlohmann::json base_rsa() {
    return {
        {"task",
         {{"scenario", "remote_sensing_access"},
          {"start_time_utc", "2026-12-30T00:00:00Z"},
          {"compute_horizon_sec", 600},
          {"working_time_sec", 200},
          {"step_sec", 10}}},
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
          {"lon_deg", 114.3},
          {"lat_deg", 30.5},
          {"alt_km", 0.0}}},
        {"sensor", {{"type", "optical_linescan"}, {"mode", "side_roll_only"}}},
        {"constraints", {{"roll_max_deg", 30.0}}},
    };
}

}  // namespace

int main() {
    setenv("ACCESS_COMPUTER_DEV_SOURCE_ROOT", ACCESS_COMPUTER_SOURCE_ROOT, 1);
    using mp::kEclipseFilterUnavailableWarning;
    using mp::merge_optical_windows;
    using mp::validate_request;

    const auto empty_ecl =
        write_file("empty_ecl.txt",
                   "Spacecraft Sat\nStart Time (UTCGregorian)\n"
                   "There are no eclipse events\n");

    std::cout << "==> H1 baseline sunlit non-umbra\n";
    {
        auto       o  = base_opts();
        const auto tr = write_file("h1_tr.txt", two_sample_trace(1.0, 1.0));
        const auto r  = merge_optical_windows(tr, empty_ecl, o);
        expect(r.windows.size() == 1, "H1 one window");
        expect(r.warnings.empty(), "H1 no warnings");
    }

    std::cout << "==> H2 require_sunlit=false keeps night\n";
    {
        const auto tr    = write_file("h2_tr.txt", two_sample_trace(1.0, 0.0));
        auto       o     = base_opts();
        o.require_sunlit = false;
        const auto keep  = merge_optical_windows(tr, empty_ecl, o);
        expect(keep.windows.size() == 1, "H2 night kept when sunlit=false");
        o.require_sunlit = true;
        const auto drop  = merge_optical_windows(tr, empty_ecl, o);
        expect(drop.windows.empty(), "H2 night dropped when sunlit=true");
    }

    std::cout << "==> H3 exclude_umbra=false keeps umbra\n";
    {
        const auto tr   = write_file("h3_tr.txt", two_sample_trace(1.0, 1.0));
        const auto ecl  = write_file("h3_ecl.txt", eclipse_line("Umbra"));
        auto       o    = base_opts();
        o.exclude_umbra = false;
        const auto keep = merge_optical_windows(tr, ecl, o);
        expect(keep.windows.size() == 1, "H3 umbra kept");
        o.exclude_umbra = true;
        const auto drop = merge_optical_windows(tr, ecl, o);
        expect(drop.windows.empty(), "H3 umbra dropped when exclude=true");
    }

    std::cout << "==> H4/H5 penumbra flag\n";
    {
        const auto tr   = write_file("h45_tr.txt", two_sample_trace(1.0, 1.0));
        const auto ecl  = write_file("h45_ecl.txt", eclipse_line("Penumbra"));
        auto       o    = base_opts();
        o.exclude_umbra = true;
        o.exclude_penumbra = true;
        expect(merge_optical_windows(tr, ecl, o).windows.empty(),
               "H4 penumbra dropped");
        o.exclude_penumbra = false;
        expect(merge_optical_windows(tr, ecl, o).windows.size() == 1,
               "H5 penumbra kept");
    }

    std::cout << "==> H6 umbra open penumbra closed\n";
    {
        const auto tr   = write_file("h6_tr.txt", two_sample_trace(1.0, 1.0));
        auto       o    = base_opts();
        o.exclude_umbra = true;
        o.exclude_penumbra = false;
        const auto pen     = write_file("h6_pen.txt", eclipse_line("Penumbra"));
        expect(merge_optical_windows(tr, pen, o).windows.size() == 1,
               "H6 penumbra survives");
        const auto umb = write_file("h6_umb.txt", eclipse_line("Umbra"));
        expect(merge_optical_windows(tr, umb, o).windows.empty(),
               "H6 umbra filtered");
    }

    std::cout << "==> H7 Antumbra maps to exclude_penumbra\n";
    {
        const auto tr   = write_file("h7_tr.txt", two_sample_trace(1.0, 1.0));
        const auto ecl  = write_file("h7_ecl.txt", eclipse_line("Antumbra"));
        auto       o    = base_opts();
        o.exclude_umbra = true;
        o.exclude_penumbra = true;
        expect(merge_optical_windows(tr, ecl, o).windows.empty(),
               "H7 Antumbra dropped when penumbra exclude");
        o.exclude_penumbra = false;
        expect(merge_optical_windows(tr, ecl, o).windows.size() == 1,
               "H7 Antumbra kept when penumbra allow");

        // 真实 GMAT 模板须请求 Antumbra（否则报告不会含环食行）。
        const auto script =
            mp::render_optical_access_script(base_rsa(), tmp_dir());
        expect(script.find("'Antumbra'") != std::string::npos ||
                   script.find("\"Antumbra\"") != std::string::npos,
               "H7 template requests Antumbra");
        expect(script.find("EclipseTypes") != std::string::npos &&
                   script.find("Umbra") != std::string::npos &&
                   script.find("Penumbra") != std::string::npos &&
                   script.find("Antumbra") != std::string::npos,
               "H7 EclipseTypes has Umbra/Penumbra/Antumbra");
    }

    std::cout << "==> H8 missing/unreadable eclipse report best-effort\n";
    {
        const auto tr = write_file("h8_tr.txt", two_sample_trace(1.0, 1.0));
        const auto missing = tmp_dir() / "does_not_exist.txt";
        auto       o       = base_opts();
        o.exclude_umbra    = true;

        const auto r = merge_optical_windows(tr, missing, o);
        expect(r.windows.size() == 1, "H8 missing keeps window");
        expect(r.warnings.size() == 1 &&
                   r.warnings[0] == kEclipseFilterUnavailableWarning,
               "H8 missing exact warning");

        o.exclude_umbra    = false;
        o.exclude_penumbra = false;
        const auto r2      = merge_optical_windows(tr, missing, o);
        expect(r2.windows.size() == 1, "H8 both-false keeps window");
        expect(r2.warnings.empty(), "H8 both-false no warning");

        o.exclude_umbra = true;
        const auto r3   = merge_optical_windows(tr, empty_ecl, o);
        expect(r3.windows.size() == 1, "H8 empty-valid keeps window");
        expect(r3.warnings.empty(), "H8 empty-valid no warning");

        const auto blank = write_file("h8_blank.txt", "");
        const auto r4    = merge_optical_windows(tr, blank, o);
        expect(r4.windows.size() == 1, "H8 blank keeps window");
        expect(r4.warnings.size() == 1 &&
                   r4.warnings[0] == kEclipseFilterUnavailableWarning,
               "H8 blank exact warning");

        const auto trunc =
            write_file("h8_trunc.txt", "30 Dec 2026 02:59:00.000 Umbra\n");
        const auto r5 = merge_optical_windows(tr, trunc, o);
        expect(r5.windows.size() == 1, "H8 truncated keeps window");
        expect(r5.warnings.size() == 1 &&
                   r5.warnings[0] == kEclipseFilterUnavailableWarning,
               "H8 truncated exact warning");

        const auto junk =
            write_file("h8_junk.txt",
                       "Spacecraft Sat\nStart Time (UTCGregorian)\n"
                       "not-a-valid-eclipse-row\n");
        const auto r6 = merge_optical_windows(tr, junk, o);
        expect(r6.windows.size() == 1, "H8 malformed keeps window");
        expect(r6.warnings.size() == 1 &&
                   r6.warnings[0] == kEclipseFilterUnavailableWarning,
               "H8 malformed exact warning");

        const auto headers_only = write_file(
            "h8_headers.txt", "Spacecraft Sat\nStart Time (UTCGregorian)\n");
        const auto r7 = merge_optical_windows(tr, headers_only, o);
        expect(r7.windows.size() == 1, "H8 headers-only keeps window");
        expect(r7.warnings.size() == 1 &&
                   r7.warnings[0] == kEclipseFilterUnavailableWarning,
               "H8 headers-only exact warning");

        // 有效事件 + 畸形行 → 整份不可读（不得因 saw_event 吞掉畸形）
        const auto mixed =
            write_file("h8_mixed.txt", eclipse_line("Umbra") +
                                           "30 Dec 2026 02:59:00.000 Umbra\n");
        const auto r8 = merge_optical_windows(tr, mixed, o);
        expect(r8.windows.size() == 1, "H8 valid+malformed keeps window");
        expect(r8.warnings.size() == 1 &&
                   r8.warnings[0] == kEclipseFilterUnavailableWarning,
               "H8 valid+malformed exact warning");

        // 未知 kind 不得默认 Umbra
        const auto unk =
            write_file("h8_unk.txt",
                       "30 Dec 2026 02:59:00.000 30 Dec 2026 03:01:00.000 "
                       "UnknownKind\n");
        const auto r9 = merge_optical_windows(tr, unk, o);
        expect(r9.windows.size() == 1, "H8 unknown-kind keeps window");
        expect(r9.warnings.size() == 1 &&
                   r9.warnings[0] == kEclipseFilterUnavailableWarning,
               "H8 unknown-kind exact warning");
    }

    std::cout << "==> V1 RSA missing require_sunlit warning\n";
    {
        auto       req = base_rsa();
        const auto v   = validate_request(req);
        expect(v.ok, "V1 validate ok");
        expect(v.details.contains("warnings"), "V1 has warnings");
    }

    std::cout << "==> V2 DL/SAR illumination flags ignored\n";
    {
        using mp::kIlluminationFlagsIgnoredDownlink;
        using mp::kIlluminationFlagsIgnoredSar;
        using mp::resolve_optical_illumination;

        nlohmann::json dl = {
            {"task",
             {{"scenario", "downlink_window"},
              {"start_time_utc", "2026-12-30T03:00:00Z"},
              {"compute_horizon_sec", 7200},
              {"working_time_sec", 7200},
              {"step_sec", 5}}},
            {"spacecraft",
             {{"sat_id", "sat_A"},
              {"epoch_utc", "30 Dec 2026 00:00:00.000"},
              {"state_type", "keplerian"},
              {"elements",
               {{"sma_km", 6716.14},
                {"ecc", 0.0},
                {"inc_deg", 96.8},
                {"raan_deg", 76.5},
                {"aop_deg", 0.0},
                {"ta_deg", 0.0}}}}},
            {"target",
             {{"type", "ground_station"},
              {"lon_deg", 114.3},
              {"lat_deg", 30.5},
              {"alt_km", 0.0}}},
            {"sensor", {{"type", "downlink_cone"}}},
            {"constraints",
             {{"require_sunlit", true},
              {"exclude_umbra", true},
              {"exclude_penumbra", false}}},
        };
        const auto v_dl = validate_request(dl);
        expect(v_dl.ok, "V2 DL validate ok");
        expect(v_dl.details.contains("warnings") &&
                   v_dl.details["warnings"].is_array() &&
                   !v_dl.details["warnings"].empty() &&
                   v_dl.details["warnings"][0].get<std::string>() ==
                       kIlluminationFlagsIgnoredDownlink,
               "V2 DL ignored warning exact");

        auto sar            = base_rsa();
        sar["sensor"]       = {{"type", "sar"},
                               {"mode", "stripmap"},
                               {"center_frequency_hz", 5.405e9},
                               {"azimuth_beamwidth_deg", 10.0}};
        sar["experimental"] = {{"allow_sar", true}};
        sar["constraints"]  = {
            {"incidence_min_deg", 20.0},     {"incidence_max_deg", 80.0},
            {"allowed_look_side", "either"}, {"roll_max_deg", 70.0},
            {"require_sunlit", true},        {"exclude_umbra", true},
            {"exclude_penumbra", true}};
        const auto v_sar = validate_request(sar);
        expect(v_sar.ok, "V2 SAR validate ok");
        expect(v_sar.details.contains("warnings") &&
                   v_sar.details["warnings"].is_array() &&
                   !v_sar.details["warnings"].empty() &&
                   v_sar.details["warnings"][0].get<std::string>() ==
                       kIlluminationFlagsIgnoredSar,
               "V2 SAR validate ignored warning exact");

        // 真实 run 与 validate 共用 resolve_optical_illumination：强制关旗 +
        // warning。
        const auto resolved = resolve_optical_illumination(sar);
        expect(!resolved.require_sunlit && !resolved.exclude_umbra &&
                   !resolved.exclude_penumbra,
               "V2 SAR run forces all flags off");
        expect(resolved.warnings.size() == 1 &&
                   resolved.warnings[0] == kIlluminationFlagsIgnoredSar,
               "V2 SAR run ignored warning exact");

        // 请求写 exclude_umbra=true 但 SAR 有效关旗：本影/夜侧几何仍可留。
        const auto tr = write_file("v2_sar_tr.txt", two_sample_trace(1.0, 0.0));
        const auto ecl = write_file("v2_sar_ecl.txt", eclipse_line("Umbra"));
        mp::MergeOptions o;
        o.step_sec         = 10.0;
        o.working_time_sec = 0.0;
        o.require_sunlit   = resolved.require_sunlit;
        o.exclude_umbra    = resolved.exclude_umbra;
        o.exclude_penumbra = resolved.exclude_penumbra;
        const auto kept    = merge_optical_windows(tr, ecl, o);
        expect(kept.windows.size() == 1, "V2 SAR merge ignores umbra/sunlit");
        expect(kept.warnings.empty(), "V2 SAR merge no D9 warning with report");
    }

    if (fails != 0) {
        std::cerr << "ac006-harness failures: " << fails << "\n";
        return 1;
    }
    std::cout << "ac006-harness passed\n";
    return 0;
}
