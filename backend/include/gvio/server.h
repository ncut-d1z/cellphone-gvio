#pragma once

#include <functional>
#include <memory>
#include <string>

namespace gvio {

// 内嵌 HTTP/2 服务器(h2 over TLS, 自签名证书)。
// 路由:
//   GET  /            -> webRoot/index.html
//   GET  /state       -> SSE 流(text/event-stream, 周期推送融合状态)
//   GET  /<path>      -> webRoot 下静态文件
//   POST /control     -> JSON {cmd: start|stop|reset|magcal}
class Http2Server {
public:
    using StatusFn = std::function<std::string()>;
    using ControlFn = std::function<std::string(const std::string& body)>;

    Http2Server(const std::string& host, int port, const std::string& certPath,
                const std::string& keyPath, const std::string& webRoot);
    ~Http2Server();

    bool start();
    void stop();
    void setStatusFn(StatusFn fn) { statusFn_ = std::move(fn); }
    void setControlFn(ControlFn fn) { controlFn_ = std::move(fn); }

    StatusFn statusFn() const { return statusFn_; }
    ControlFn controlFn() const { return controlFn_; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    StatusFn statusFn_;
    ControlFn controlFn_;
    std::string host_;
    int port_;
    std::string certPath_, keyPath_, webRoot_;
};

}  // namespace gvio
