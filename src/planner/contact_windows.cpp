#include "planner/contact_windows.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace mp {

namespace {

std::string local_node_id() {
    if (const char* env = std::getenv("SATELLITE_NODE_ID");
        env != nullptr && env[0] != '\0') {
        return env;
    }
    return "local";
}

bool is_skip_line(const std::string& line) {
    if (line.empty()) { return true; }
    if (line.rfind("Spacecraft", 0) == 0) { return true; }
    if (line.rfind("Start Time", 0) == 0) { return true; }
    if (line.rfind("Stop Time", 0) == 0) { return true; }
    if (line.rfind("Observer", 0) == 0) { return true; }
    if (line.rfind("Duration", 0) == 0) { return true; }
    if (line.rfind("Maximum Elevation", 0) == 0) { return true; }
    if (line.rfind("Max Elevation", 0) == 0) { return true; }
    if (line.find("There are no contact") != std::string::npos) { return true; }
    if (line.find("Number of events") != std::string::npos) { return true; }
    return false;
}

std::vector<std::string> tokenize(const std::string& line) {
    std::istringstream       ls(line);
    std::vector<std::string> parts;
    std::string              token;
    while (ls >> token) { parts.push_back(token); }
    return parts;
}

std::string join_utc_tokens(const std::vector<std::string>& parts,
                            std::size_t                     begin) {
    return parts[begin] + " " + parts[begin + 1] + " " + parts[begin + 2] +
           " " + parts[begin + 3];
}

bool try_parse_elevation(const std::string& text, double* out) {
    try {
        std::size_t  idx = 0;
        const double v   = std::stod(text, &idx);
        if (idx != text.size()) { return false; }
        if (!std::isfinite(v) || v < 0.0 || v > 90.0) { return false; }
        *out = v;
        return true;
    } catch (...) { return false; }
}

nlohmann::json make_window(const std::string&           start,
                           const std::string&           end,
                           const std::optional<double>& elev) {
    nlohmann::json w = {
        {"start_utc", start},
        {"end_utc", end},
        {"node_id", local_node_id()},
    };
    if (elev.has_value()) { w["max_elevation_deg"] = *elev; }
    return w;
}

}  // namespace

ContactParseResult parse_contact_windows(const std::filesystem::path& path) {
    ContactParseResult result;
    if (!std::filesystem::exists(path)) { return result; }

    std::ifstream in(path);
    if (!in) { return result; }

    std::vector<std::string> lines;
    std::string              line;
    bool                     maxelev_header = false;
    bool                     no_contact     = false;
    while (std::getline(in, line)) {
        if (line.find("Maximum Elevation") != std::string::npos) {
            maxelev_header = true;
        }
        if (line.find("There are no contact") != std::string::npos) {
            no_contact = true;
        }
        lines.push_back(std::move(line));
    }

    std::size_t missing_elev = 0;
    std::size_t accepted     = 0;

    for (const auto& raw : lines) {
        if (is_skip_line(raw)) { continue; }
        const auto parts = tokenize(raw);
        if (maxelev_header) {
            // Observer + start(4) + end(4) + duration + max_elev [+ time...]
            if (parts.size() < 11) { continue; }
            const auto            start = join_utc_tokens(parts, 1);
            const auto            end   = join_utc_tokens(parts, 5);
            double                elev  = 0.0;
            std::optional<double> elev_opt;
            if (try_parse_elevation(parts[10], &elev)) { elev_opt = elev; }
            else { ++missing_elev; }
            result.windows.push_back(make_window(start, end, elev_opt));
            ++accepted;
        }
        else {
            if (parts.size() < 8) { continue; }
            const auto start = join_utc_tokens(parts, 0);
            const auto end   = join_utc_tokens(parts, 4);
            result.windows.push_back(make_window(start, end, std::nullopt));
            ++missing_elev;
            ++accepted;
        }
    }

    const auto total = result.windows.size();
    if (total == 0) {
        if (maxelev_header && !no_contact) {
            result.warnings.push_back(
                "max_elevation_deg parse failed: SiteViewMaxElevationReport "
                "present but no rows accepted");
        }
        return result;
    }

    if (missing_elev == total) {
        result.warnings.push_back(
            "max_elevation_deg unavailable: contact report missing Maximum "
            "Elevation column (Legacy or parse failure)");
    }
    else if (missing_elev > 0) {
        result.warnings.push_back(
            "max_elevation_deg incomplete: " + std::to_string(missing_elev) +
            " of " + std::to_string(total) + " windows missing peak elevation");
    }
    return result;
}

}  // namespace mp
