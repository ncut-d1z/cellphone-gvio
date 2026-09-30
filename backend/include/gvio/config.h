#pragma once

#include "gvio/types.h"
#include <nlohmann/json.hpp>

namespace gvio {

using Json = nlohmann::json;

struct Config {
    FilterConfig filter;
    bool valid = false;

    static Config fromJson(const std::string& jsonStr);
    std::string toJson() const;
};

}  // namespace gvio
