#include "planner/time_model.hpp"

#include <cctype>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace mp {

namespace {

bool parse_fractional_seconds(std::istringstream& ss, double& frac_out) {
    if (ss.peek() != '.') {
        frac_out = 0.0;
        return true;
    }
    ss.get();  // '.'
    if (!std::isdigit(static_cast<unsigned char>(ss.peek()))) { return false; }
    std::string digits;
    while (std::isdigit(static_cast<unsigned char>(ss.peek()))) {
        digits.push_back(static_cast<char>(ss.get()));
    }
    if (digits.empty() || digits.size() > 9) { return false; }
    double value = 0.0;
    for (char ch : digits) {
        value = value * 10.0 + static_cast<double>(ch - '0');
    }
    double denom = 1.0;
    for (std::size_t i = 0; i < digits.size(); ++i) { denom *= 10.0; }
    frac_out = value / denom;
    return true;
}

bool stream_exhausted(std::istringstream& ss) {
    ss >> std::ws;
    return ss.eof();
}

std::optional<std::chrono::system_clock::time_point> finish_time_point(
    std::tm& tm, double frac_sec) {
    // Snapshot civil fields before timegm (which may normalize invalid dates).
    const int year = tm.tm_year;
    const int mon  = tm.tm_mon;
    const int mday = tm.tm_mday;
    const int hour = tm.tm_hour;
    const int min  = tm.tm_min;
    const int sec  = tm.tm_sec;

    const auto tt = timegm(&tm);
    if (tt == static_cast<std::time_t>(-1)) { return std::nullopt; }

    std::tm back{};
    if (gmtime_r(&tt, &back) == nullptr) { return std::nullopt; }
    if (back.tm_year != year || back.tm_mon != mon || back.tm_mday != mday ||
        back.tm_hour != hour || back.tm_min != min || back.tm_sec != sec) {
        return std::nullopt;  // e.g. 2027-02-30 normalized away
    }

    auto tp = std::chrono::system_clock::from_time_t(tt);
    if (frac_sec < 0.0 || frac_sec >= 1.0) { return std::nullopt; }
    if (frac_sec > 0.0) {
        tp += std::chrono::duration_cast<std::chrono::system_clock::duration>(
            std::chrono::duration<double>(frac_sec));
    }
    return tp;
}

}  // namespace

std::optional<std::chrono::system_clock::time_point> parse_iso8601_utc(
    const std::string& text) {
    if (text.size() < 2 || text.back() != 'Z') { return std::nullopt; }
    const std::string body = text.substr(0, text.size() - 1);

    std::istringstream ss(body);
    std::tm            tm{};
    ss >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%S");
    if (ss.fail()) { return std::nullopt; }

    double frac = 0.0;
    if (!parse_fractional_seconds(ss, frac)) { return std::nullopt; }
    if (!stream_exhausted(ss)) { return std::nullopt; }
    return finish_time_point(tm, frac);
}

std::optional<std::chrono::system_clock::time_point> parse_gmat_utcgregorian(
    const std::string& text) {
    std::istringstream ss(text);
    std::tm            tm{};
    ss >> std::get_time(&tm, "%d %b %Y %H:%M:%S");
    if (ss.fail()) { return std::nullopt; }

    double frac = 0.0;
    if (!parse_fractional_seconds(ss, frac)) { return std::nullopt; }
    if (!stream_exhausted(ss)) { return std::nullopt; }
    return finish_time_point(tm, frac);
}

std::optional<double> delta_prop_sec(const std::string& start_time_utc,
                                     const std::string& epoch_utc) {
    const auto start = parse_iso8601_utc(start_time_utc);
    const auto epoch = parse_gmat_utcgregorian(epoch_utc);
    if (!start || !epoch) { return std::nullopt; }
    return std::chrono::duration<double>(*start - *epoch).count();
}

std::string format_gmat_utcgregorian(
    const std::chrono::system_clock::time_point& tp) {
    const auto sec_tp = std::chrono::time_point_cast<std::chrono::seconds>(tp);
    const auto tt     = std::chrono::system_clock::to_time_t(sec_tp);
    std::tm    tm{};
    gmtime_r(&tt, &tm);
    const auto   whole  = std::chrono::system_clock::from_time_t(tt);
    const double frac   = std::chrono::duration<double>(tp - whole).count();
    int          millis = static_cast<int>(std::llround(frac * 1000.0));
    if (millis < 0) { millis = 0; }
    if (millis > 999) { millis = 999; }

    std::ostringstream ss;
    ss << std::put_time(&tm, "%d %b %Y %H:%M:%S") << '.' << std::setw(3)
       << std::setfill('0') << millis;
    return ss.str();
}

}  // namespace mp
