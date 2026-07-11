#pragma once

#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>

/**
 * @file validate.hpp
 * @brief access-computer 请求校验。
 *
 * @note 合同文档化的默认值仅出现在校验成功时的 @c details.defaults 中；
 *       必填字段缺失将显式失败。
 */

namespace mp {

/** @brief 校验结果。 */
struct ValidationResult {
    bool           ok = false;
    nlohmann::json details;
    std::string    message;
};

/**
 * @brief 校验规划请求 JSON。
 * @param request 输入请求对象。
 * @return 校验结果；失败时 @c ok=false 且 @c message 说明原因。
 */
ValidationResult validate_request(const nlohmann::json& request);

}  // namespace mp
