#include "gvio/server.h"

#include "gvio/handler.h"

#include <nghttp2/nghttp2.h>

#include <mbedtls/ctr_drbg.h>
#include <mbedtls/entropy.h>
#include <mbedtls/error.h>
#include <mbedtls/net_sockets.h>
#include <mbedtls/pk.h>
#include <mbedtls/ssl.h>
#include <mbedtls/x509_crt.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace gvio {

namespace {

constexpr int kPort = 8443;
constexpr int kReadBuf = 64 * 1024;
constexpr int kSendBuf = 32 * 1024;
constexpr uint64_t kIdleMs = 60000;
constexpr uint64_t kSseIntervalMs = 200;

#ifdef _WIN32
void setNonBlock(int fd) {
    u_long one = 1;
    ioctlsocket(static_cast<SOCKET>(fd), FIONBIO, &one);
}
void closeFd(int fd) { closesocket(static_cast<SOCKET>(fd)); }
int errNo() { return WSAGetLastError(); }
bool isWouldBlock(int e) { return e == WSAEWOULDBLOCK || e == WSAEINPROGRESS; }
#else
void setNonBlock(int fd) {
    int fl = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, fl | O_NONBLOCK);
}
void closeFd(int fd) { ::close(fd); }
int errNo() { return errno; }
bool isWouldBlock(int e) { return e == EWOULDBLOCK || e == EAGAIN; }
#endif

std::string errStr(int code) {
#ifdef MBEDTLS_ERROR_C
    char buf[128];
    mbedtls_strerror(code, buf, sizeof buf);
    return buf;
#else
    return "tls error " + std::to_string(code);
#endif
}

struct BodyRef {
    std::shared_ptr<std::vector<uint8_t>> data;
    size_t off = 0;
    size_t avail() const { return data ? data->size() - off : 0; }
};

struct StreamState {
    std::string method, path;
    std::vector<uint8_t> body;
    bool endStreamSeen = false;
    bool responded = false;
    bool sse = false;
    BodyRef resp;
};

struct Conn {
    int fd = -1;
    mbedtls_net_context net;
    mbedtls_ssl_context ssl;
    bool handshaked = false;
    nghttp2_session* session = nullptr;
    std::map<int32_t, std::unique_ptr<StreamState>> streams;
    std::vector<uint8_t> outbuf;
    BodyRef sseBody;
    uint64_t lastActivity = 0;
    bool closeAfterFlush = false;
};

int onBeginHeaders(nghttp2_session*, const nghttp2_frame* frame, void* ud) {
    auto* c = static_cast<Conn*>(ud);
    if (frame->hd.type == NGHTTP2_HEADERS &&
        frame->headers.cat == NGHTTP2_HCAT_REQUEST) {
        auto st = std::make_unique<StreamState>();
        c->streams[frame->hd.stream_id] = std::move(st);
    }
    return 0;
}

int onHeader(nghttp2_session*, const nghttp2_frame* frame, const uint8_t* name,
             size_t namelen, const uint8_t* value, size_t valuelen, uint8_t,
             void* ud) {
    auto* c = static_cast<Conn*>(ud);
    auto it = c->streams.find(frame->hd.stream_id);
    if (it == c->streams.end()) return 0;
    auto& st = *it->second;
    std::string k(name, name + namelen);
    std::string v(value, value + valuelen);
    if (k == ":method")
        st.method = v;
    else if (k == ":path")
        st.path = v;
    return 0;
}

int onDataChunk(nghttp2_session*, uint8_t, int32_t stream_id, const uint8_t* data,
                size_t len, void* ud) {
    auto* c = static_cast<Conn*>(ud);
    auto it = c->streams.find(stream_id);
    if (it != c->streams.end()) {
        auto& body = it->second->body;
        body.insert(body.end(), data, data + len);
    }
    return 0;
}

int onStreamClose(nghttp2_session*, int32_t stream_id, uint32_t, void* ud) {
    auto* c = static_cast<Conn*>(ud);
    c->streams.erase(stream_id);
    return 0;
}

ssize_t readCallback(nghttp2_session*, int32_t, uint8_t* buf, size_t len,
                     uint32_t* flags, nghttp2_data_source* src, void*) {
    auto* b = static_cast<BodyRef*>(src->ptr);
    size_t avail = b ? b->avail() : 0;
    if (avail == 0) {
        *flags |= NGHTTP2_DATA_FLAG_EOF;
        return 0;
    }
    size_t n = std::min(avail, len);
    if (n) {
        std::memcpy(buf, b->data->data() + b->off, n);
        b->off += n;
    }
    return static_cast<ssize_t>(n);
}

nghttp2_nv makeNv(const char* name, const std::string& value) {
    nghttp2_nv nv{};
    nv.name = reinterpret_cast<uint8_t*>(const_cast<char*>(name));
    nv.value = reinterpret_cast<uint8_t*>(const_cast<char*>(value.data()));
    nv.namelen = std::strlen(name);
    nv.valuelen = value.size();
    nv.flags = NGHTTP2_NV_FLAG_NONE;
    return nv;
}

}  // namespace

struct Http2Server::Impl {
    Http2Server& owner;
    Impl(Http2Server& o) : owner(o) {}

    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr;
    mbedtls_x509_crt crt;
    mbedtls_pk_context pkey;
    bool tlsReady = false;

    int listenFd = -1;
    std::atomic<bool> stop{false};
    std::thread thread;
    std::vector<std::unique_ptr<Conn>> conns;

    nghttp2_session_callbacks* callbacks = nullptr;
    nghttp2_option* opt = nullptr;

    uint64_t lastSseMs = 0;
    std::string lastSse;

    bool setupTls() {
        mbedtls_entropy_init(&entropy);
        mbedtls_ctr_drbg_init(&ctr);
        mbedtls_x509_crt_init(&crt);
        mbedtls_pk_init(&pkey);
        if (mbedtls_ctr_drbg_seed(&ctr, mbedtls_entropy_func, &entropy, nullptr, 0) != 0)
            return false;
        if (mbedtls_x509_crt_parse_file(&crt, owner.certPath_.c_str()) != 0)
            return false;
        if (mbedtls_pk_parse_keyfile(&pkey, owner.keyPath_.c_str(), nullptr, nullptr,
                                     nullptr) != 0)
            return false;
        tlsReady = true;
        return true;
    }

    bool setupListener() {
#ifdef _WIN32
        static bool wsInit = []() {
            WSADATA wsa;
            return WSAStartup(MAKEWORD(2, 2), &wsa) == 0;
        }();
        (void)wsInit;
#endif
        listenFd = static_cast<int>(socket(AF_INET, SOCK_STREAM, 0));
        if (listenFd < 0) return false;
        int one = 1;
        setsockopt(listenFd, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one),
                   sizeof one);
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(owner.port_));
        addr.sin_addr.s_addr = owner.host_ == "0.0.0.0" ? htonl(INADDR_ANY)
                                                        : inet_addr(owner.host_.c_str());
        if (bind(listenFd, reinterpret_cast<sockaddr*>(&addr), sizeof addr) != 0) return false;
        if (listen(listenFd, 8) != 0) return false;
        setNonBlock(listenFd);
        return true;
    }

    bool acceptOne() {
        sockaddr_in peer{};
#ifdef _WIN32
        int plen = sizeof peer;
#else
        socklen_t plen = sizeof peer;
#endif
        int fd = static_cast<int>(accept(listenFd, reinterpret_cast<sockaddr*>(&peer), &plen));
        if (fd < 0) return false;
        setNonBlock(fd);
        auto c = std::make_unique<Conn>();
        c->fd = fd;
        c->net.fd = fd;
        mbedtls_ssl_init(&c->ssl);
        mbedtls_ssl_config conf;
        mbedtls_ssl_config_init(&conf);
        mbedtls_ssl_config_defaults(&conf, MBEDTLS_SSL_IS_SERVER,
                                    MBEDTLS_SSL_TRANSPORT_STREAM, MBEDTLS_SSL_PRESET_DEFAULT);
        mbedtls_ssl_conf_authmode(&conf, MBEDTLS_SSL_VERIFY_NONE);
        mbedtls_ssl_conf_rng(&conf, mbedtls_ctr_drbg_random, &ctr);
        mbedtls_ssl_conf_own_cert(&conf, &crt, &pkey);
        static const char* alpn[] = {"h2", nullptr};
        mbedtls_ssl_conf_alpn_protocols(&conf, alpn);
        if (mbedtls_ssl_setup(&c->ssl, &conf) != 0) {
            mbedtls_ssl_free(&c->ssl);
            closeFd(fd);
            return false;
        }
        mbedtls_ssl_set_bio(&c->ssl, &c->net, mbedtls_net_send, mbedtls_net_recv, nullptr);
        c->lastActivity = nowMs();
        conns.push_back(std::move(c));
        return true;
    }

    static uint64_t nowMs() {
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
    }

    void initNg() {
        nghttp2_session_callbacks_new(&callbacks);
        nghttp2_session_callbacks_set_on_begin_headers_callback(callbacks, onBeginHeaders);
        nghttp2_session_callbacks_set_on_header_callback(callbacks, onHeader);
        nghttp2_session_callbacks_set_on_data_chunk_recv_callback(callbacks, onDataChunk);
        nghttp2_session_callbacks_set_on_stream_close_callback(callbacks, onStreamClose);
        nghttp2_option_new(&opt);
    }

    void respond(Conn& c, StreamState& st) {
        if (st.responded) return;
        st.responded = true;

        int32_t sid = 0;
        for (auto& kv : c.streams)
            if (kv.second.get() == &st) sid = kv.first;
        if (sid == 0) return;

        auto route = routeRequest(st.method, st.path, st.body, owner.webRoot_,
                                  owner.statusFn_ ? owner.statusFn_() : "",
                                  owner.controlFn_);

        const std::string ct = "content-type";
        const std::string cc = "cache-control";
        const std::string noStore = "no-store";

        if (route.sse) {
            nghttp2_nv nv[3];
            nv[0] = makeNv(":status", std::string("200"));
            nv[1] = makeNv("content-type", route.contentType);
            nv[2] = makeNv("cache-control", noStore);
            nghttp2_submit_response(c.session, sid, nv, 3, nullptr);
            st.sse = true;
            return;
        }

        std::string status = std::to_string(route.status);
        std::string len = std::to_string(route.body.size());
        nghttp2_nv nv[4];
        nv[0] = makeNv(":status", status);
        nv[1] = makeNv("content-type", route.contentType);
        nv[2] = makeNv("content-length", len);
        nv[3] = makeNv("cache-control", noStore);

        auto body = std::make_shared<std::vector<uint8_t>>(std::move(route.body));
        st.resp.data = body;
        st.resp.off = 0;
        nghttp2_data_provider prd{};
        prd.source.ptr = &st.resp;
        prd.read_callback = readCallback;
        nghttp2_submit_response(c.session, sid, nv, 4, &prd);
    }

    void processStreams(Conn& c) {
        for (auto& kv : c.streams) {
            auto& st = *kv.second;
            if (st.endStreamSeen && !st.responded) respond(c, st);
        }
    }

    void pumpSend(Conn& c) {
        for (int i = 0; i < 64; ++i) {
            const uint8_t* data = nullptr;
            ssize_t n = nghttp2_session_mem_send(c.session, &data);
            if (n < 0) {
                c.closeAfterFlush = true;
                return;
            }
            if (n == 0) break;
            c.outbuf.insert(c.outbuf.end(), data, data + n);
        }
    }

    void flush(Conn& c) {
        size_t off = 0;
        while (off < c.outbuf.size()) {
#ifdef _WIN32
            int n = ::send(c.fd, reinterpret_cast<const char*>(c.outbuf.data() + off),
                           static_cast<int>(c.outbuf.size() - off), 0);
#else
            ssize_t n = ::send(c.fd, c.outbuf.data() + off, c.outbuf.size() - off, 0);
#endif
            if (n > 0) {
                off += static_cast<size_t>(n);
                continue;
            }
            if (n < 0 && isWouldBlock(errNo())) break;
            c.closeAfterFlush = true;
            break;
        }
        if (off > 0) c.outbuf.erase(c.outbuf.begin(), c.outbuf.begin() + off);
    }

    void readProcess(Conn& c) {
        if (!c.handshaked) {
            int r = mbedtls_ssl_handshake(&c.ssl);
            if (r == 0) {
                const char* alpn = mbedtls_ssl_get_alpn_protocol(&c.ssl);
                if (!alpn || std::strcmp(alpn, "h2") != 0) {
                    c.closeAfterFlush = true;
                    return;
                }
                if (nghttp2_session_server_new2(&c.session, callbacks, &c, opt) != 0) {
                    c.closeAfterFlush = true;
                    return;
                }
                c.handshaked = true;
                c.lastActivity = nowMs();
            } else if (r == MBEDTLS_ERR_SSL_WANT_READ || r == MBEDTLS_ERR_SSL_WANT_WRITE) {
                return;
            } else {
                c.closeAfterFlush = true;
                return;
            }
        }
        c.lastActivity = nowMs();
        for (int i = 0; i < 8; ++i) {
            uint8_t buf[kReadBuf];
            int n = mbedtls_ssl_read(&c.ssl, buf, sizeof buf);
            if (n > 0) {
                ssize_t consumed = nghttp2_session_mem_recv(c.session, buf, n);
                if (consumed < 0) {
                    c.closeAfterFlush = true;
                    return;
                }
                continue;
            }
            if (n == MBEDTLS_ERR_SSL_WANT_READ || n == MBEDTLS_ERR_SSL_WANT_WRITE) break;
            c.closeAfterFlush = true;
            break;
        }
        processStreams(c);
        pumpSend(c);
    }

    void broadcastSse(uint64_t now) {
        if (now - lastSseMs < kSseIntervalMs) return;
        lastSseMs = now;
        bool any = false;
        for (auto& cu : conns)
            for (auto& kv : cu->streams)
                if (kv.second->sse) any = true;
        if (!any) return;
        std::string s = owner.statusFn_ ? owner.statusFn_() : "";
        if (s.empty() || s == lastSse) return;
        lastSse = s;
        for (auto& cu : conns) {
            bool needPump = false;
            for (auto& kv : cu->streams) {
                if (!kv.second->sse) continue;
                auto v = std::make_shared<std::vector<uint8_t>>(s.begin(), s.end());
                cu->sseBody.data = v;
                cu->sseBody.off = 0;
                nghttp2_data_provider prd{};
                prd.source.ptr = &cu->sseBody;
                prd.read_callback = readCallback;
                nghttp2_submit_data(cu->session, NGHTTP2_FLAG_NONE, kv.first, &prd);
                needPump = true;
            }
            if (needPump) {
                pumpSend(*cu);
                flush(*cu);
            }
        }
    }

    void run() {
        initNg();
        uint64_t start = nowMs();
        lastSseMs = start;
        while (!stop) {
            fd_set rf, wf;
            FD_ZERO(&rf);
            FD_ZERO(&wf);
            int maxfd = listenFd;
            FD_SET(listenFd, &rf);
            for (auto& c : conns) {
                if (c->fd < 0) continue;
                FD_SET(c->fd, &rf);
                if (!c->outbuf.empty()) FD_SET(c->fd, &wf);
                if (c->fd > maxfd) maxfd = c->fd;
            }
            timeval tv{0, 50 * 1000};
            int sel = select(maxfd + 1, &rf, &wf, nullptr, &tv);
            if (sel < 0) break;
            if (FD_ISSET(listenFd, &rf)) {
                while (acceptOne()) {
                }
            }
            for (auto& c : conns) {
                if (c->fd < 0) continue;
                bool readable = FD_ISSET(c->fd, &rf) != 0;
                bool writable = FD_ISSET(c->fd, &wf) != 0;
                if (readable) readProcess(*c);
                if (writable || readable) flush(*c);
                if (c->closeAfterFlush && c->outbuf.empty()) c->fd = -1;
            }
            conns.erase(std::remove_if(conns.begin(), conns.end(),
                                       [](const std::unique_ptr<Conn>& c) {
                                           if (c->fd < 0) {
                                               if (c->session) nghttp2_session_del(c->session);
                                               mbedtls_ssl_free(&c->ssl);
                                               return true;
                                           }
                                           return false;
                                       }),
                        conns.end());
            broadcastSse(nowMs());
            uint64_t now = nowMs();
            for (auto& c : conns)
                if (now - c->lastActivity > kIdleMs) c->closeAfterFlush = true;
        }
        for (auto& c : conns) {
            if (c->fd >= 0) closeFd(c->fd);
            if (c->session) nghttp2_session_del(c->session);
            mbedtls_ssl_free(&c->ssl);
        }
        conns.clear();
        if (listenFd >= 0) closeFd(listenFd);
        nghttp2_option_del(opt);
        nghttp2_session_callbacks_del(callbacks);
    }

    ~Impl() {
        mbedtls_x509_crt_free(&crt);
        mbedtls_pk_free(&pkey);
        mbedtls_ctr_drbg_free(&ctr);
        mbedtls_entropy_free(&entropy);
    }
};

Http2Server::Http2Server(const std::string& host, int port, const std::string& certPath,
                         const std::string& keyPath, const std::string& webRoot)
    : impl_(std::make_unique<Impl>(*this)),
      host_(host),
      port_(port > 0 ? port : kPort),
      certPath_(certPath),
      keyPath_(keyPath),
      webRoot_(webRoot) {}

Http2Server::~Http2Server() {
    stop();
}

bool Http2Server::start() {
    if (!impl_->setupTls()) return false;
    if (!impl_->setupListener()) return false;
    impl_->stop = false;
    impl_->thread = std::thread([this]() { impl_->run(); });
    return true;
}

void Http2Server::stop() {
    impl_->stop = true;
    if (impl_->thread.joinable()) impl_->thread.join();
}

}  // namespace gvio
