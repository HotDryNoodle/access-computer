/**
 * @file ac004_harness.cpp
 * @brief AC-004 确定性自检：时间解析严格性、W≤H、±W/2 裁剪、DL 不裁剪。
 */

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

#include "geometry/window_merger.hpp"
#include "planner/time_model.hpp"
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
            {"ta_deg", 0.0}}},
          {"propagation_profile", "sso_j2_default"}}},
        {"target",
         {{"type", "ground_point"},
          {"lon_deg", 114.3},
          {"lat_deg", 30.5},
          {"alt_km", 0.0}}},
        {"sensor", {{"type", "optical_linescan"}, {"mode", "side_roll_only"}}},
        {"constraints", {{"roll_max_deg", 30.0}, {"require_sunlit", true}}},
    };
}

}  // namespace

int main() {
    using mp::AccessWindow;
    using mp::clip_windows_to_working_time;
    using mp::delta_prop_sec;
    using mp::parse_gmat_utcgregorian;
    using mp::parse_iso8601_utc;
    using mp::validate_request;

    std::cout << "==> time parse strictness\n";
    expect(!parse_iso8601_utc("2026-12-30T00:00:00.123junkZ").has_value(),
           "ISO reject trailing junk before Z");
    expect(!parse_gmat_utcgregorian("30 Dec 2026 00:00:00.000junk").has_value(),
           "GMAT reject trailing junk");
    expect(parse_iso8601_utc("2026-12-30T00:00:00.123Z").has_value(),
           "ISO accept fractional");
    expect(parse_gmat_utcgregorian("30 Dec 2026 00:00:00.500").has_value(),
           "GMAT accept .500");
    expect(!parse_iso8601_utc("2027-02-30T00:00:00Z").has_value(),
           "ISO reject invalid calendar date Feb 30");
    {
        auto req                      = base_rsa();
        req["task"]["start_time_utc"] = "2027-02-30T00:00:00Z";
        const auto r                  = validate_request(req);
        expect(!r.ok, "validate rejects Feb 30 start_time_utc");
    }

    const auto dp =
        delta_prop_sec("2026-12-30T00:00:00.000Z", "30 Dec 2026 00:00:00.500");
    expect(dp.has_value() && std::abs(*dp - (-0.5)) < 1e-6,
           "delta_prop respects epoch fractional .500");

    std::cout << "==> W <= H strict\n";
    {
        auto req                           = base_rsa();
        req["task"]["compute_horizon_sec"] = 200.0;
        req["task"]["working_time_sec"]    = 200.5;
        const auto r                       = validate_request(req);
        expect(!r.ok, "W=H+0.5 rejected");
    }
    {
        auto req                           = base_rsa();
        req["task"]["compute_horizon_sec"] = 200.0;
        req["task"]["working_time_sec"]    = 200.0;
        const auto r                       = validate_request(req);
        expect(r.ok, "W=H accepted");
    }

    std::cout << "==> ±W/2 clip (RSA/AE path)\n";
    AccessWindow wide;
    wide.start_utc    = "30 Dec 2026 03:00:00.000";
    wide.end_utc      = "30 Dec 2026 04:00:00.000";
    wide.t0_utc       = "30 Dec 2026 03:30:00.000";
    wide.duration_sec = 3600.0;
    wide.phi_deg      = 1.0;
    wide.pass_type    = "Ascending";

    const auto clipped =
        clip_windows_to_working_time(std::vector<AccessWindow>{wide}, 200.0);
    expect(clipped.size() == 1, "clip keeps one window");
    if (!clipped.empty()) {
        expect(clipped.front().start_utc == "30 Dec 2026 03:28:20.000",
               "clip start = t0-100s");
        expect(clipped.front().end_utc == "30 Dec 2026 03:31:40.000",
               "clip end = t0+100s");
        expect(std::abs(clipped.front().duration_sec - 200.0) < 1e-6,
               "clip duration = 200");
    }

    std::cout << "==> clip parse errors propagate\n";
    AccessWindow bad = wide;
    bad.t0_utc       = "not-a-timestamp";
    bool threw       = false;
    try {
        (void)clip_windows_to_working_time(std::vector<AccessWindow>{bad},
                                           200.0);
    } catch (const std::exception&) { threw = true; }
    expect(threw, "clip throws on unparseable UTC (no silent drop)");

    std::cout << "==> DL no-clip (W<=0 sentinel)\n";
    const auto unchanged =
        clip_windows_to_working_time(std::vector<AccessWindow>{wide}, 0.0);
    expect(unchanged.size() == 1 &&
               unchanged.front().start_utc == wide.start_utc &&
               unchanged.front().end_utc == wide.end_utc,
           "W=0 leaves contact window untouched");

    if (fails != 0) {
        std::cerr << "ac004 harness failures: " << fails << "\n";
        return 1;
    }
    std::cout << "ac004 harness passed\n";
    return 0;
}
