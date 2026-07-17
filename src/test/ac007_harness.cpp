/**
 * @file ac007_harness.cpp
 * @brief AC-007：接触报告峰值仰角解析、D7 合法域、默认 cone→10° 门限。
 */

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

#include "gmat/gmat_backend.hpp"
#include "planner/contact_windows.hpp"
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

bool warning_contains(const mp::ContactParseResult& r, const std::string& key) {
    for (const auto& w : r.warnings) {
        if (w.find(key) != std::string::npos) { return true; }
    }
    return false;
}

std::filesystem::path write_tmp(const std::string& name,
                                const std::string& body) {
    const auto dir = std::filesystem::temp_directory_path() / "ac007_harness";
    std::filesystem::create_directories(dir);
    const auto    path = dir / name;
    std::ofstream out(path);
    out << body;
    return path;
}

nlohmann::json base_dl() {
    return {
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
            {"inc_deg", 96.806},
            {"raan_deg", 76.515},
            {"aop_deg", 0.0},
            {"ta_deg", 0.0}}}}},
        {"target",
         {{"type", "ground_station"},
          {"lon_deg", 114.3055},
          {"lat_deg", 30.5928},
          {"alt_km", 0.057}}},
        {"sensor", {{"type", "downlink_cone"}}},
        {"constraints", nlohmann::json::object()},
    };
}

}  // namespace

int main() {
    setenv("ACCESS_COMPUTER_DEV_SOURCE_ROOT", ACCESS_COMPUTER_SOURCE_ROOT, 1);
    using mp::parse_contact_windows;
    using mp::render_downlink_script;
    using mp::validate_request;

    std::cout << "==> H1 MaxElev fixture\n";
    {
        const auto path = write_tmp(
            "maxelev.txt",
            "Observer Start Time (UTCGregorian) Stop Time (UTCGregorian) "
            "Duration Maximum Elevation (deg) Max Elevation Time "
            "(UTCGregorian)\n"
            "Station 01 Jan 2027 12:00:00.000 01 Jan 2027 12:10:00.000 "
            "600.000 42.500 01 Jan 2027 12:05:00.000\n");
        const auto r = parse_contact_windows(path);
        expect(r.windows.size() == 1, "H1 one window");
        expect(r.windows[0].contains("max_elevation_deg") &&
                   std::abs(r.windows[0]["max_elevation_deg"].get<double>() -
                            42.5) < 1e-9,
               "H1 max_elevation_deg=42.5");
        expect(r.windows[0]["start_utc"] == "01 Jan 2027 12:00:00.000",
               "H1 start_utc");
        expect(r.warnings.empty(), "H1 no warnings");
    }

    std::cout << "==> H2 Legacy fixture\n";
    {
        const auto path = write_tmp(
            "legacy.txt",
            "Start Time (UTCGregorian) Stop Time (UTCGregorian) Duration "
            "(s)\n"
            "01 Jan 2027 12:00:00.000 01 Jan 2027 12:10:00.000 600.000\n");
        const auto r = parse_contact_windows(path);
        expect(r.windows.size() == 1, "H2 one window");
        expect(!r.windows[0].contains("max_elevation_deg"),
               "H2 no max_elevation_deg");
        expect(warning_contains(r, "unavailable"), "H2 unavailable warning");
    }

    std::cout << "==> H3 mixed incomplete\n";
    {
        const auto path = write_tmp(
            "mixed.txt",
            "Observer Start Time (UTCGregorian) Stop Time (UTCGregorian) "
            "Duration Maximum Elevation (deg)\n"
            "Station 01 Jan 2027 12:00:00.000 01 Jan 2027 12:10:00.000 "
            "600.000 42.500\n"
            "Station 01 Jan 2027 13:00:00.000 01 Jan 2027 13:10:00.000 "
            "600.000 not_a_number\n");
        const auto r = parse_contact_windows(path);
        expect(r.windows.size() == 2, "H3 two windows");
        expect(r.windows[0].contains("max_elevation_deg"), "H3 first has elev");
        expect(!r.windows[1].contains("max_elevation_deg"),
               "H3 second missing elev");
        expect(warning_contains(r, "incomplete"), "H3 incomplete warning");
    }

    std::cout << "==> H4 empty contact\n";
    {
        const auto path = write_tmp(
            "empty.txt", "There are no contact events located for Station\n");
        const auto r = parse_contact_windows(path);
        expect(r.windows.empty(), "H4 empty windows");
        expect(r.warnings.empty(), "H4 no elev warning");
    }

    std::cout << "==> H5 D7 illegal elev\n";
    {
        const auto path = write_tmp(
            "bad_elev.txt",
            "Observer Start Time (UTCGregorian) Stop Time (UTCGregorian) "
            "Duration Maximum Elevation (deg)\n"
            "Station 01 Jan 2027 12:00:00.000 01 Jan 2027 12:10:00.000 "
            "600.000 NaN\n"
            "Station 01 Jan 2027 13:00:00.000 01 Jan 2027 13:10:00.000 "
            "600.000 91.0\n"
            "Station 01 Jan 2027 14:00:00.000 01 Jan 2027 14:10:00.000 "
            "600.000 -1.0\n"
            "Station 01 Jan 2027 15:00:00.000 01 Jan 2027 15:10:00.000 "
            "600.000 inf\n");
        const auto r = parse_contact_windows(path);
        expect(r.windows.size() == 4, "H5 four windows");
        for (const auto& w : r.windows) {
            expect(!w.contains("max_elevation_deg"),
                   "H5 omit illegal max_elevation_deg");
        }
        expect(warning_contains(r, "unavailable") ||
                   warning_contains(r, "incomplete"),
               "H5 warning present");
    }

    std::cout << "==> H6 default cone 80 -> min elev 10\n";
    {
        auto       req = base_dl();
        const auto v   = validate_request(req);
        expect(v.ok, "H6 validate ok");
        expect(
            v.details.contains("downlink") &&
                std::abs(v.details["downlink"]["cone_angle_deg"].get<double>() -
                         80.0) < 1e-9,
            "H6 details.cone=80");
        expect(
            std::abs(v.details["downlink"]["min_elevation_deg"].get<double>() -
                     10.0) < 1e-9,
            "H6 details.min_elev=10");
        const auto script =
            render_downlink_script(req, std::filesystem::temp_directory_path());
        expect(script.find("MinimumElevationAngle = 10") != std::string::npos ||
                   script.find("MinimumElevationAngle = 10.000000") !=
                       std::string::npos,
               "H6 script MinimumElevationAngle~10");
        // std::to_string(10.0) is typically "10.000000"
        expect(script.find("SiteViewMaxElevationReport") != std::string::npos,
               "H6 ReportFormat SiteViewMaxElevationReport");
    }

    std::cout << "==> H7 explicit cone 65 -> min elev 25\n";
    {
        auto req                             = base_dl();
        req["constraints"]["cone_angle_deg"] = 65.0;
        const auto v                         = validate_request(req);
        expect(v.ok, "H7 validate ok");
        expect(
            std::abs(v.details["downlink"]["min_elevation_deg"].get<double>() -
                     25.0) < 1e-9,
            "H7 details.min_elev=25");
        const auto script =
            render_downlink_script(req, std::filesystem::temp_directory_path());
        expect(script.find("MinimumElevationAngle = 25") != std::string::npos ||
                   script.find("MinimumElevationAngle = 25.000000") !=
                       std::string::npos,
               "H7 script MinimumElevationAngle~25");
    }

    if (fails != 0) {
        std::cerr << "ac007-harness failures: " << fails << "\n";
        return 1;
    }
    std::cout << "ac007-harness passed\n";
    return 0;
}
