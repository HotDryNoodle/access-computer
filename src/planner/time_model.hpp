#pragma once

#include <chrono>
#include <optional>
#include <string>

/**
 * @file time_model.hpp
 * @brief AC-004 时间模型：ISO/GMAT 解析与 Δ_prop 派生。
 */

namespace mp {

/** @brief 时间解析容差（秒）。 */
constexpr double kTimeToleranceSec = 1.0;

/**
 * @brief 解析 ISO-8601 UTC（要求尾缀 Z），例如 2026-12-30T03:37:00Z。
 * @return 失败时 nullopt。
 */
std::optional<std::chrono::system_clock::time_point> parse_iso8601_utc(
    const std::string& text);

/**
 * @brief 解析 GMAT UTCGregorian，例如 30 Dec 2026 00:00:00.000。
 * @return 失败时 nullopt。
 */
std::optional<std::chrono::system_clock::time_point> parse_gmat_utcgregorian(
    const std::string& text);

/**
 * @brief Δ_prop = start_time_utc − epoch_utc（秒）。
 * @return 失败时 nullopt。
 */
std::optional<double> delta_prop_sec(const std::string& start_time_utc,
                                     const std::string& epoch_utc);

/** @brief 将 time_point 格式化为 GMAT UTCGregorian（毫秒 .000）。 */
std::string format_gmat_utcgregorian(
    const std::chrono::system_clock::time_point& tp);

/**
 * @brief 将 time_point 格式化为规范 ISO-8601 UTC，固定毫秒三位。
 * @param tp UTC 时间点。
 * @return 例如 @c 2026-12-30T03:39:59.668Z。
 */
std::string format_iso8601_utc_ms(
    const std::chrono::system_clock::time_point& tp);

}  // namespace mp
