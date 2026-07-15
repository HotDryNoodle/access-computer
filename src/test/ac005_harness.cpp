/**
 * @file ac005_harness.cpp
 * @brief AC-005：工作窗裁剪边角 — 多窗 / 短几何窗 / 空交集（verify-only）。
 */

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "geometry/window_merger.hpp"

namespace {

int fails = 0;

void expect(bool ok, const std::string& name) {
    if (ok) { std::cout << "  OK  " << name << "\n"; }
    else {
        std::cerr << "  FAIL " << name << "\n";
        ++fails;
    }
}

mp::AccessWindow make_window(const std::string& start,
                             const std::string& end,
                             const std::string& t0) {
    mp::AccessWindow w;
    w.start_utc = start;
    w.end_utc   = end;
    w.t0_utc    = t0;
    w.phi_deg   = 1.0;
    w.pass_type = "Ascending";
    return w;
}

}  // namespace

int main() {
    using mp::clip_windows_to_working_time;

    std::cout << "==> H1 dual windows independent t0 clip\n";
    {
        // W=200 → ±100s around each t0
        const auto a =
            make_window("30 Dec 2026 03:00:00.000", "30 Dec 2026 04:00:00.000",
                        "30 Dec 2026 03:30:00.000");
        const auto b =
            make_window("30 Dec 2026 05:00:00.000", "30 Dec 2026 06:00:00.000",
                        "30 Dec 2026 05:20:00.000");
        const auto clipped = clip_windows_to_working_time(
            std::vector<mp::AccessWindow>{a, b}, 200.0);
        expect(clipped.size() == 2, "H1 keeps two windows");
        if (clipped.size() == 2) {
            expect(clipped[0].start_utc == "30 Dec 2026 03:28:20.000",
                   "H1 win0 start = t0-100s");
            expect(clipped[0].end_utc == "30 Dec 2026 03:31:40.000",
                   "H1 win0 end = t0+100s");
            expect(std::abs(clipped[0].duration_sec - 200.0) < 1e-6,
                   "H1 win0 duration=200");
            expect(clipped[1].start_utc == "30 Dec 2026 05:18:20.000",
                   "H1 win1 start = t0-100s");
            expect(clipped[1].end_utc == "30 Dec 2026 05:21:40.000",
                   "H1 win1 end = t0+100s");
            expect(std::abs(clipped[1].duration_sec - 200.0) < 1e-6,
                   "H1 win1 duration=200");
        }
    }

    std::cout << "==> H2 short geometric window inside [t0±W/2]\n";
    {
        // Geometry 60s centered on t0; W=200 → keep 60s intersection
        const auto short_w =
            make_window("30 Dec 2026 03:29:30.000", "30 Dec 2026 03:30:30.000",
                        "30 Dec 2026 03:30:00.000");
        const auto clipped = clip_windows_to_working_time(
            std::vector<mp::AccessWindow>{short_w}, 200.0);
        expect(clipped.size() == 1, "H2 keeps short window");
        if (!clipped.empty()) {
            expect(clipped.front().start_utc == "30 Dec 2026 03:29:30.000",
                   "H2 start unchanged (geometry binds)");
            expect(clipped.front().end_utc == "30 Dec 2026 03:30:30.000",
                   "H2 end unchanged (geometry binds)");
            expect(std::abs(clipped.front().duration_sec - 60.0) < 1e-6,
                   "H2 duration=60 ≤ W");
            expect(clipped.front().duration_sec <= 200.0 + 1e-9,
                   "H2 duration ≤ W");
        }
    }

    std::cout << "==> H3 empty intersection dropped\n";
    {
        // Geometry entirely before [t0-100, t0+100]
        const auto far =
            make_window("30 Dec 2026 01:00:00.000", "30 Dec 2026 01:10:00.000",
                        "30 Dec 2026 03:30:00.000");
        const auto keep =
            make_window("30 Dec 2026 03:00:00.000", "30 Dec 2026 04:00:00.000",
                        "30 Dec 2026 03:30:00.000");
        const auto clipped = clip_windows_to_working_time(
            std::vector<mp::AccessWindow>{far, keep}, 200.0);
        expect(clipped.size() == 1, "H3 drops empty-intersection window");
        if (!clipped.empty()) {
            expect(clipped.front().start_utc == "30 Dec 2026 03:28:20.000",
                   "H3 remaining window clipped to W");
            expect(std::abs(clipped.front().duration_sec - 200.0) < 1e-6,
                   "H3 remaining duration=200");
        }
    }

    if (fails != 0) {
        std::cerr << "ac005-harness failures: " << fails << "\n";
        return 1;
    }
    std::cout << "ac005-harness passed\n";
    return 0;
}
