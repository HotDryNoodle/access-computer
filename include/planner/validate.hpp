#pragma once

#include <filesystem>
#include <string>

#include <nlohmann/json.hpp>

namespace mp {

struct ValidationResult {
    bool ok = false;
    nlohmann::json details;
    std::string message;
};

ValidationResult validate_request(const nlohmann::json& request);

}  // namespace mp
