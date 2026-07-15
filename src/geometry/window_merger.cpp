#include "geometry/window_merger.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace mp {

namespace {

std::chrono::system_clock::time_point parse_utc(const std::string& text) {
    std::istringstream ss(text);
    std::tm            tm{};
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
    std::tm    tm{};
    gmtime_r(&tt, &tm);
    std::ostringstream ss;
    ss << std::put_time(&tm, "%d %b %Y %H:%M:%S.000");
    return ss.str();
}

struct TraceRow {
    std::chrono::system_clock::time_point utc;
    std::chrono::system_clock::time_point end_utc;
    double                                off_nadir = 0.0;
    double                                alpha     = 0.0;
    double                                vn        = 0.0;
    bool                                  in_swath  = false;
    bool                                  lit       = false;
};

struct EclipseInterval {
    std::chrono::system_clock::time_point start;
    std::chrono::system_clock::time_point end;
    std::string                           kind;
};

bool in_kind(const std::chrono::system_clock::time_point& t,
             const std::vector<EclipseInterval>&          eclipses,
             const std::string&                           kind) {
    for (const auto& e : eclipses) {
        if (e.kind == kind && t >= e.start && t < e.end) { return true; }
    }
    return false;
}

bool blocked_by_eclipse(const std::chrono::system_clock::time_point& t,
                        const std::vector<EclipseInterval>&          eclipses,
                        const MergeOptions&                          options) {
    if (options.exclude_umbra && in_kind(t, eclipses, "Umbra")) { return true; }
    if (options.exclude_penumbra && (in_kind(t, eclipses, "Penumbra") ||
                                     in_kind(t, eclipses, "Antumbra"))) {
        return true;
    }
    return false;
}

enum class EclipseReportStatus {
    MissingOrUnreadable,
    EmptyValid,
    HasEvents,
};

EclipseReportStatus load_eclipses(const std::filesystem::path&  path,
                                  std::vector<EclipseInterval>* out) {
    if (!std::filesystem::exists(path)) {
        return EclipseReportStatus::MissingOrUnreadable;
    }
    std::ifstream in(path);
    if (!in) { return EclipseReportStatus::MissingOrUnreadable; }

    bool        saw_event           = false;
    bool        saw_no_event_marker = false;
    bool        saw_unreadable      = false;
    bool        any_nonempty_line   = false;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) { continue; }
        any_nonempty_line = true;
        if (line.rfind("Spacecraft", 0) == 0 ||
            line.rfind("Start Time", 0) == 0) {
            continue;
        }
        // 合法 GMAT 无事件标记（仅此类可 EmptyValid）。
        if (line.find("There are no") != std::string::npos) {
            saw_no_event_marker = true;
            continue;
        }
        if (line.find("Number of events") != std::string::npos) {
            std::istringstream ns(line);
            std::string        tok;
            int                n_events = -1;
            while (ns >> tok) {
                try {
                    n_events = std::stoi(tok);
                } catch (...) {
                    // keep scanning for a trailing integer
                }
            }
            if (n_events == 0) {
                saw_no_event_marker = true;
                continue;
            }
            // 非 0 或无法解析数字：若最终无事件行则视为不可读
            if (n_events < 0) { saw_unreadable = true; }
            continue;
        }

        std::istringstream       ls(line);
        std::vector<std::string> parts;
        std::string              token;
        while (ls >> token) { parts.push_back(token); }
        // 事件行至少两段 UTC（各 4 token）+ kind → ≥9
        if (parts.size() < 9) {
            saw_unreadable = true;
            continue;
        }
        try {
            EclipseInterval e;
            e.start = parse_utc(parts[0] + " " + parts[1] + " " + parts[2] +
                                " " + parts[3]);
            e.end = parse_utc(parts[4] + " " + parts[5] + " " + parts[6] + " " +
                              parts[7]);
            std::string kind;
            for (std::size_t i = 8; i < parts.size(); ++i) {
                if (parts[i] == "Umbra" || parts[i] == "Penumbra" ||
                    parts[i] == "Antumbra") {
                    kind = parts[i];
                    break;
                }
            }
            // 未知 kind 不得默认 Umbra → 整份报告不可读
            if (kind.empty()) {
                saw_unreadable = true;
                continue;
            }
            e.kind = kind;
            out->push_back(e);
            saw_event = true;
        } catch (...) { saw_unreadable = true; }
    }

    // 任一畸形/截断/未知 kind 优先于 HasEvents（丢弃已解析事件）
    if (saw_unreadable) {
        out->clear();
        return EclipseReportStatus::MissingOrUnreadable;
    }
    if (saw_event) { return EclipseReportStatus::HasEvents; }
    // 空文件、仅表头、无合法无事件标记 → unavailable
    if (!any_nonempty_line || !saw_no_event_marker) {
        return EclipseReportStatus::MissingOrUnreadable;
    }
    return EclipseReportStatus::EmptyValid;
}

}  // namespace

OpticalMergeResult merge_optical_windows(
    const std::filesystem::path& trace_path,
    const std::filesystem::path& eclipse_path,
    const MergeOptions&          options) {
    OpticalMergeResult result;

    std::vector<EclipseInterval> eclipses;
    const auto report_status = load_eclipses(eclipse_path, &eclipses);
    const bool need_eclipse = options.exclude_umbra || options.exclude_penumbra;
    bool       apply_eclipse = true;
    if (report_status == EclipseReportStatus::MissingOrUnreadable) {
        if (need_eclipse) {
            result.warnings.push_back(kEclipseFilterUnavailableWarning);
            apply_eclipse = false;
        }
        else { apply_eclipse = false; }
    }

    std::vector<TraceRow> rows;
    {
        std::ifstream in(trace_path);
        std::string   line;
        while (std::getline(in, line)) {
            if (line.empty()) { continue; }
            std::istringstream       ls(line);
            std::vector<std::string> parts;
            std::string              token;
            while (ls >> token) { parts.push_back(token); }
            if (parts.size() < 15) { continue; }
            try {
                TraceRow row;
                row.utc = parse_utc(parts[0] + " " + parts[1] + " " + parts[2] +
                                    " " + parts[3]);
                row.off_nadir = std::stod(parts[7]);
                row.alpha     = std::stod(parts[10]);
                row.in_swath  = std::stod(parts[11]) > 0.5;
                row.lit       = std::stod(parts[12]) > 0.5;
                row.vn        = std::stod(parts[14]);
                rows.push_back(row);
            } catch (...) { continue; }
        }
    }

    for (std::size_t i = 0; i + 1 < rows.size(); ++i) {
        rows[i].end_utc = rows[i + 1].utc;
    }
    if (!rows.empty()) {
        rows.back().end_utc =
            rows.back().utc + std::chrono::milliseconds(
                                  static_cast<int>(options.step_sec * 1000));
    }

    std::vector<AccessWindow> windows;
    AccessWindow*             active = nullptr;
    for (const auto& row : rows) {
        const bool ok_lit = !options.require_sunlit || row.lit;
        bool       ok_ecl = true;
        if (apply_eclipse) {
            ok_ecl = !blocked_by_eclipse(row.utc, eclipses, options) &&
                     !blocked_by_eclipse(row.end_utc, eclipses, options);
        }
        const bool visible = row.in_swath && ok_lit && ok_ecl;
        if (visible) {
            if (!active) {
                windows.push_back({});
                active            = &windows.back();
                active->start_utc = format_utc(row.utc);
                active->end_utc   = format_utc(row.end_utc);
                active->pass_type = row.vn > 0 ? "Ascending" : "Descending";
                active->max_sun_elevation_deg = row.alpha;
                active->min_off_nadir_deg     = row.off_nadir;
                active->t0_utc                = format_utc(row.utc);
                active->phi_deg               = row.off_nadir;
            }
            else {
                active->end_utc = format_utc(row.end_utc);
                active->max_sun_elevation_deg =
                    std::max(active->max_sun_elevation_deg, row.alpha);
                if (row.off_nadir <= active->min_off_nadir_deg) {
                    active->min_off_nadir_deg = row.off_nadir;
                    active->t0_utc            = format_utc(row.utc);
                    active->phi_deg           = row.off_nadir;
                }
            }
        }
        else if (active) {
            const auto start = parse_utc(active->start_utc);
            const auto end   = parse_utc(active->end_utc);
            active->duration_sec =
                std::chrono::duration<double>(end - start).count();
            active = nullptr;
        }
    }
    if (active) {
        const auto start = parse_utc(active->start_utc);
        const auto end   = parse_utc(active->end_utc);
        active->duration_sec =
            std::chrono::duration<double>(end - start).count();
    }

    result.windows =
        clip_windows_to_working_time(windows, options.working_time_sec);
    return result;
}

std::vector<AccessWindow> clip_windows_to_working_time(
    const std::vector<AccessWindow>& windows, double working_time_sec) {
    if (working_time_sec <= 0.0) { return windows; }

    const double              half = working_time_sec / 2.0;
    std::vector<AccessWindow> clipped;
    clipped.reserve(windows.size());
    for (std::size_t i = 0; i < windows.size(); ++i) {
        auto                                  w = windows[i];
        std::chrono::system_clock::time_point t0;
        std::chrono::system_clock::time_point start;
        std::chrono::system_clock::time_point end;
        try {
            t0    = parse_utc(w.t0_utc);
            start = parse_utc(w.start_utc);
            end   = parse_utc(w.end_utc);
        } catch (const std::exception& ex) {
            throw std::runtime_error(
                "clip_windows_to_working_time: failed to parse window[" +
                std::to_string(i) + "] UTC fields: " + ex.what());
        }
        const auto half_ms =
            std::chrono::milliseconds(static_cast<std::int64_t>(half * 1000.0));
        const auto clip_lo   = t0 - half_ms;
        const auto clip_hi   = t0 + half_ms;
        const auto new_start = std::max(start, clip_lo);
        const auto new_end   = std::min(end, clip_hi);
        if (new_end <= new_start) { continue; }
        w.start_utc = format_utc(new_start);
        w.end_utc   = format_utc(new_end);
        w.duration_sec =
            std::chrono::duration<double>(new_end - new_start).count();
        clipped.push_back(std::move(w));
    }
    return clipped;
}

nlohmann::json windows_to_json(const std::vector<AccessWindow>& windows) {
    const char*       node_env = std::getenv("SATELLITE_NODE_ID");
    const std::string node_id =
        (node_env != nullptr && node_env[0] != '\0') ? node_env : "local";
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
            {"node_id", node_id},
        });
    }
    return arr;
}

}  // namespace mp
