#include "geometry/window_merger.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace mp {

namespace {

std::chrono::system_clock::time_point parse_utc(const std::string& text) {
    std::istringstream ss(text);
    std::tm tm{};
    ss >> std::get_time(&tm, "%d %b %Y %H:%M:%S");
    if (ss.fail()) {
        std::istringstream ss2(text);
        ss2 >> std::get_time(&tm, "%d %b %Y %H:%M:%S");
        if (ss2.fail()) {
            throw std::runtime_error("Unrecognized UTC time: " + text);
        }
    }
    const auto tt = timegm(&tm);
    return std::chrono::system_clock::from_time_t(tt);
}

std::string format_utc(const std::chrono::system_clock::time_point& tp) {
    const auto tt = std::chrono::system_clock::to_time_t(tp);
    std::tm tm{};
    gmtime_r(&tt, &tm);
    std::ostringstream ss;
    ss << std::put_time(&tm, "%d %b %Y %H:%M:%S.000");
    return ss.str();
}

struct TraceRow {
    std::chrono::system_clock::time_point utc;
    std::chrono::system_clock::time_point end_utc;
    double off_nadir = 0.0;
    double alpha = 0.0;
    double vn = 0.0;
    bool geom_visible = false;
};

struct EclipseInterval {
    std::chrono::system_clock::time_point start;
    std::chrono::system_clock::time_point end;
    std::string kind;
};

bool in_umbra(const std::chrono::system_clock::time_point& t,
              const std::vector<EclipseInterval>& eclipses) {
    for (const auto& e : eclipses) {
        if (e.kind == "Umbra" && t >= e.start && t < e.end) {
            return true;
        }
    }
    return false;
}

}  // namespace

std::vector<AccessWindow> merge_optical_windows(const std::filesystem::path& trace_path,
                                                const std::filesystem::path& eclipse_path,
                                                const MergeOptions& options) {
    std::vector<EclipseInterval> eclipses;
    if (std::filesystem::exists(eclipse_path)) {
        std::ifstream in(eclipse_path);
        std::string line;
        while (std::getline(in, line)) {
            if (line.empty() || line.rfind("Spacecraft", 0) == 0 || line.rfind("Start Time", 0) == 0) {
                continue;
            }
            std::istringstream ls(line);
            std::vector<std::string> parts;
            std::string token;
            while (ls >> token) {
                parts.push_back(token);
            }
            if (parts.size() < 9) {
                continue;
            }
            try {
                EclipseInterval e;
                e.start = parse_utc(parts[0] + " " + parts[1] + " " + parts[2] + " " + parts[3]);
                e.end = parse_utc(parts[4] + " " + parts[5] + " " + parts[6] + " " + parts[7]);
                e.kind = "Umbra";
                for (std::size_t i = 8; i < parts.size(); ++i) {
                    if (parts[i] == "Umbra" || parts[i] == "Penumbra" || parts[i] == "Antumbra") {
                        e.kind = parts[i];
                        break;
                    }
                }
                if (options.exclude_penumbra && e.kind == "Penumbra") {
                    continue;
                }
                eclipses.push_back(e);
            } catch (...) {
                continue;
            }
        }
    }

    std::vector<TraceRow> rows;
    {
        std::ifstream in(trace_path);
        std::string line;
        while (std::getline(in, line)) {
            if (line.empty()) {
                continue;
            }
            std::istringstream ls(line);
            std::vector<std::string> parts;
            std::string token;
            while (ls >> token) {
                parts.push_back(token);
            }
            if (parts.size() < 15) {
                continue;
            }
            try {
                TraceRow row;
                row.utc = parse_utc(parts[0] + " " + parts[1] + " " + parts[2] + " " + parts[3]);
                row.off_nadir = std::stod(parts[7]);
                row.alpha = std::stod(parts[10]);
                const bool in_swath = std::stod(parts[11]) > 0.5;
                const bool lit = std::stod(parts[12]) > 0.5;
                const bool geom = std::stod(parts[13]) > 0.5;
                row.geom_visible = geom && in_swath && (!options.require_sunlit || lit);
                row.vn = std::stod(parts[14]);
                rows.push_back(row);
            } catch (...) {
                continue;
            }
        }
    }

    for (std::size_t i = 0; i + 1 < rows.size(); ++i) {
        rows[i].end_utc = rows[i + 1].utc;
    }
    if (!rows.empty()) {
        rows.back().end_utc = rows.back().utc + std::chrono::milliseconds(static_cast<int>(options.step_sec * 1000));
    }

    std::vector<AccessWindow> windows;
    AccessWindow* active = nullptr;
    for (const auto& row : rows) {
        const bool visible = row.geom_visible && !in_umbra(row.utc, eclipses) &&
                             !in_umbra(row.end_utc, eclipses);
        if (visible) {
            if (!active) {
                windows.push_back({});
                active = &windows.back();
                active->start_utc = format_utc(row.utc);
                active->end_utc = format_utc(row.end_utc);
                active->pass_type = row.vn > 0 ? "Ascending" : "Descending";
                active->max_sun_elevation_deg = row.alpha;
                active->min_off_nadir_deg = row.off_nadir;
                active->t0_utc = format_utc(row.utc);
                active->phi_deg = row.off_nadir;
            } else {
                active->end_utc = format_utc(row.end_utc);
                active->max_sun_elevation_deg = std::max(active->max_sun_elevation_deg, row.alpha);
                if (row.off_nadir <= active->min_off_nadir_deg) {
                    active->min_off_nadir_deg = row.off_nadir;
                    active->t0_utc = format_utc(row.utc);
                    active->phi_deg = row.off_nadir;
                }
            }
        } else if (active) {
            const auto start = parse_utc(active->start_utc);
            const auto end = parse_utc(active->end_utc);
            active->duration_sec =
                std::chrono::duration<double>(end - start).count();
            active = nullptr;
        }
    }
    if (active) {
        const auto start = parse_utc(active->start_utc);
        const auto end = parse_utc(active->end_utc);
        active->duration_sec = std::chrono::duration<double>(end - start).count();
    }
    return windows;
}

nlohmann::json windows_to_json(const std::vector<AccessWindow>& windows) {
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& w : windows) {
        arr.push_back({
            {"start_utc", w.start_utc},
            {"end_utc", w.end_utc},
            {"duration_sec", w.duration_sec},
            {"t0_utc", w.t0_utc},
            {"phi_deg", w.phi_deg},
            {"pass_type", w.pass_type},
            {"min_off_nadir_deg", w.min_off_nadir_deg},
            {"max_sun_elevation_deg", w.max_sun_elevation_deg},
        });
    }
    return arr;
}

}  // namespace mp
