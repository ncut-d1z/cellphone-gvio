#pragma once

#include "gvio/server.h"
#include <cstdint>
#include <string>
#include <vector>

namespace gvio {

struct RouteResult {
    int status = 404;
    std::string contentType = "text/plain";
    std::vector<uint8_t> body;
    bool sse = false;
};

// 纯函数: 解析请求并生成响应(不涉及网络)
RouteResult routeRequest(const std::string& method, const std::string& path,
                         const std::vector<uint8_t>& body,
                         const std::string& webRoot,
                         const std::string& statusJson,
                         const Http2Server::ControlFn& controlFn);

}  // namespace gvio
