#include "gvio/handler.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <fstream>
#include <sstream>

namespace gvio {

namespace {

std::string contentTypeFor(const std::string& path) {
    auto ext = path.substr(path.find_last_of('.') == std::string::npos ? 0
                                                                       : path.find_last_of('.'));
    if (ext == ".html" || ext == ".htm") return "text/html; charset=utf-8";
    if (ext == ".js") return "application/javascript; charset=utf-8";
    if (ext == ".css") return "text/css; charset=utf-8";
    if (ext == ".json") return "application/json; charset=utf-8";
    if (ext == ".png") return "image/png";
    if (ext == ".jpg" || ext == ".jpeg") return "image/jpeg";
    if (ext == ".svg") return "image/svg+xml";
    if (ext == ".woff2") return "font/woff2";
    if (ext == ".wasm") return "application/wasm";
    return "application/octet-stream";
}

bool readFile(const std::string& path, std::vector<uint8_t>& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    f.seekg(0, std::ios::end);
    std::streamoff n = f.tellg();
    f.seekg(0, std::ios::beg);
    out.resize(static_cast<size_t>(n));
    if (n > 0) f.read(reinterpret_cast<char*>(out.data()), n);
    return true;
}

}  // namespace

RouteResult routeRequest(const std::string& method, const std::string& path,
                         const std::vector<uint8_t>& body,
                         const std::string& webRoot,
                         const std::string& statusJson,
                         const Http2Server::ControlFn& controlFn) {
    RouteResult r;

    std::string p = path;
    if (p.empty() || p[0] != '/') p = "/" + p;
    size_t q = p.find('?');
    if (q != std::string::npos) p = p.substr(0, q);
    if (p == "/") p = "/index.html";

    if (method == "POST" && p == "/control") {
        r.status = 200;
        r.contentType = "application/json; charset=utf-8";
        std::string bs(body.begin(), body.end());
        std::string resp = controlFn ? controlFn(bs) : "{\"ok\":false,\"msg\":\"no control\"}";
        r.body.assign(resp.begin(), resp.end());
        return r;
    }

    if (method != "GET") {
        r.status = 405;
        r.body = {'n', 'o'};
        return r;
    }

    if (p == "/state") {
        // SSE 流: 头部由服务器特殊处理(见 server.cpp)
        r.sse = true;
        r.status = 200;
        r.contentType = "text/event-stream; charset=utf-8";
        return r;
    }

    // 静态文件(路径净化: 禁止 ..
    std::string rel = p.substr(1);
    if (rel.find("..") != std::string::npos) {
        r.status = 403;
        return r;
    }
    std::string full = webRoot + "/" + rel;
    std::vector<uint8_t> data;
    if (!readFile(full, data)) {
        r.status = 404;
        r.contentType = "text/plain; charset=utf-8";
        std::string s = "404 not found: " + p;
        r.body.assign(s.begin(), s.end());
        return r;
    }
    r.status = 200;
    r.contentType = contentTypeFor(p);
    r.body = std::move(data);
    return r;
}

}  // namespace gvio
