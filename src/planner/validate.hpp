#pragma once

#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

/**
 * @file validate.hpp
 * @brief access-computer 请求校验。
 *
 * @note 合同文档化的默认值仅出现在校验成功时的 @c details.defaults 中；
 *       必填字段缺失将显式失败。
 */

namespace mp {

/** AC-006 D8：DL 携带三旗时的稳定 warning（勿改字）。 */
inline constexpr const char* kIlluminationFlagsIgnoredDownlink =
    "illumination flags ignored for downlink_window: not an "
    "energy constraint; SAR/DL does not apply "
    "require_sunlit/exclude_umbra/exclude_penumbra";

/** AC-006 D8：SAR 携带三旗时的稳定 warning（勿改字）。 */
inline constexpr const char* kIlluminationFlagsIgnoredSar =
    "illumination flags ignored for remote_sensing_access: "
    "not an energy constraint; SAR/DL does not apply "
    "require_sunlit/exclude_umbra/exclude_penumbra";

/** @brief 校验结果。 */
struct ValidationResult {
    bool           ok = false;
    nlohmann::json details;
    std::string    message;
};

/**
 * @brief constraints 是否含任一光照/食影旗键。
 * @param constraints 请求 constraints 对象。
 * @return 若含 @c require_sunlit / @c exclude_umbra / @c exclude_penumbra
 *         任一键则为 true，否则 false。
 */
bool has_illumination_constraint_flags(const nlohmann::json& constraints);

/**
 * @brief AC-006 光学合并三旗解析结果。
 */
struct OpticalIlluminationResolved {
    bool                     require_sunlit   = true;
    bool                     exclude_umbra    = true;
    bool                     exclude_penumbra = false;
    std::vector<std::string> warnings;
};

/**
 * @brief 解析光学合并用三旗（供 validate 与真实 run 共用）。
 * @param request 规划请求 JSON。
 * @return 有效三旗与可选 ignored warning。
 * @note SAR（@c sensor.type=sar）强制三旗全关且不读请求值；若请求携带三旗则
 *       写入 @c kIlluminationFlagsIgnoredSar。光学读请求值（缺省
 *       sunlit/umbra=true、penumbra=false）。
 */
OpticalIlluminationResolved resolve_optical_illumination(
    const nlohmann::json& request);

/**
 * @brief 校验规划请求 JSON。
 * @param request 输入请求对象。
 * @return 校验结果；失败时 @c ok=false 且 @c message 说明原因。
 */
ValidationResult validate_request(const nlohmann::json& request);

}  // namespace mp
