#include "gvio/server.h"
#ifndef GVIO_WITH_SERVER
#define GVIO_WITH_SERVER 0
#endif
#if GVIO_WITH_SERVER
#include "gvio/handler.h"
#include <nghttp2/nghttp2.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/entropy.h>
#include <mbedtls/net_sockets.h>
#include <mbedtls/pk.h>
#include <mbedtls/ssl.h>
#include <mbedtls/version.h>
#include <mbedtls/x509_crt.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <map>
#include <thread>
#include <vector>

namespace gvio {
namespace {
constexpr size_t kMaxConnections=16,kMaxStreams=32,kMaxBody=65536,kMaxHeaders=16384;
uint64_t nowMs() {
    return uint64_t(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
}
bool retry(int e) { return e==MBEDTLS_ERR_SSL_WANT_READ || e==MBEDTLS_ERR_SSL_WANT_WRITE; }
struct Stream {
    std::string method,path;
    size_t headerBytes=0;
    std::vector<uint8_t> body;
    bool complete=false,responded=false,sse=false;
    std::vector<uint8_t> data;
    size_t offset=0;
    uint64_t nextEvent=0;
};
struct Connection {
    mbedtls_net_context net;
    mbedtls_ssl_context ssl;
    nghttp2_session* h2=nullptr;
    bool handshaked=false,dead=false;
    std::map<int32_t,std::unique_ptr<Stream>> streams;
    std::vector<uint8_t> output;
    size_t offset=0;
    uint64_t activity=nowMs();
    Connection() { mbedtls_net_init(&net); mbedtls_ssl_init(&ssl); }
    ~Connection() {
        if (h2) nghttp2_session_del(h2);
        mbedtls_ssl_free(&ssl); mbedtls_net_free(&net);
    }
};
int beginHeaders(nghttp2_session*,const nghttp2_frame* f,void* ptr) {
    try {
        auto& c=*static_cast<Connection*>(ptr);
        if (f->hd.type==NGHTTP2_HEADERS && f->headers.cat==NGHTTP2_HCAT_REQUEST) {
            if (c.streams.size()>=kMaxStreams) return NGHTTP2_ERR_CALLBACK_FAILURE;
            c.streams.emplace(f->hd.stream_id,std::make_unique<Stream>());
        }
        return 0;
    } catch (...) { return NGHTTP2_ERR_CALLBACK_FAILURE; }
}
int header(nghttp2_session*,const nghttp2_frame* f,const uint8_t* name,size_t n,
           const uint8_t* value,size_t size,uint8_t,void* ptr) {
    try {
        auto& c=*static_cast<Connection*>(ptr); auto it=c.streams.find(f->hd.stream_id);
        if (it==c.streams.end()) return 0;
        auto& s=*it->second;
        if (n>kMaxHeaders || size>kMaxHeaders || s.headerBytes+n+size>kMaxHeaders) return NGHTTP2_ERR_CALLBACK_FAILURE;
        s.headerBytes+=n+size;
        std::string key(reinterpret_cast<const char*>(name),n),v(reinterpret_cast<const char*>(value),size);
        if (key==":method") s.method=std::move(v);
        else if (key==":path") s.path=std::move(v);
        return 0;
    } catch (...) { return NGHTTP2_ERR_CALLBACK_FAILURE; }
}
int dataChunk(nghttp2_session*,uint8_t,int32_t id,const uint8_t* bytes,size_t n,void* ptr) {
    try {
        auto& c=*static_cast<Connection*>(ptr); auto it=c.streams.find(id);
        if (it==c.streams.end()) return 0;
        auto& b=it->second->body;
        if (n>kMaxBody || b.size()>kMaxBody-n) return NGHTTP2_ERR_TEMPORAL_CALLBACK_FAILURE;
        b.insert(b.end(),bytes,bytes+n); return 0;
    } catch (...) { return NGHTTP2_ERR_CALLBACK_FAILURE; }
}
int frameReceived(nghttp2_session*,const nghttp2_frame* f,void* ptr) {
    auto& c=*static_cast<Connection*>(ptr); auto it=c.streams.find(f->hd.stream_id);
    if (it!=c.streams.end() && (f->hd.type==NGHTTP2_HEADERS || f->hd.type==NGHTTP2_DATA) &&
        (f->hd.flags&NGHTTP2_FLAG_END_STREAM)) it->second->complete=true;
    return 0;
}
int streamClosed(nghttp2_session*,int32_t id,uint32_t,void* ptr) {
    static_cast<Connection*>(ptr)->streams.erase(id); return 0;
}
ssize_t readBody(nghttp2_session*,int32_t,uint8_t* out,size_t n,uint32_t* flags,nghttp2_data_source* source,void*) {
    auto& s=*static_cast<Stream*>(source->ptr);
    size_t available=s.data.size()-s.offset;
    if (!available && s.sse) return NGHTTP2_ERR_DEFERRED;
    size_t count=std::min(n,available);
    if (count) std::memcpy(out,s.data.data()+s.offset,count);
    s.offset+=count;
    if (!s.sse && s.offset==s.data.size()) *flags|=NGHTTP2_DATA_FLAG_EOF;
    return static_cast<ssize_t>(count);
}
nghttp2_nv nv(const char* name,const std::string& value) {
    nghttp2_nv x{}; x.name=reinterpret_cast<uint8_t*>(const_cast<char*>(name)); x.namelen=std::strlen(name);
    x.value=reinterpret_cast<uint8_t*>(const_cast<char*>(value.data())); x.valuelen=value.size(); x.flags=NGHTTP2_NV_FLAG_NONE; return x;
}
std::vector<uint8_t> sseEvent(const std::string& text) {
    // Every line needs its own data: prefix. Blank line terminates one SSE event.
    std::string event="data: ";
    for (char c:text) { if (c=='\n') event+="\ndata: "; else if (c!='\r') event+=c; }
    event+="\n\n"; return {event.begin(),event.end()};
}
}
struct Http2Server::Impl {
    Http2Server& owner;
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context rng;
    mbedtls_x509_crt certificate;
    mbedtls_pk_context key;
    mbedtls_ssl_config config; // lives until ALL SSL contexts have been destroyed
    mbedtls_net_context listener;
    nghttp2_session_callbacks* callbacks=nullptr;
    std::vector<std::unique_ptr<Connection>> clients;
    std::atomic<bool> stopping{false},active{false};
    std::thread thread;
    explicit Impl(Http2Server& s):owner(s) {
        mbedtls_entropy_init(&entropy); mbedtls_ctr_drbg_init(&rng); mbedtls_x509_crt_init(&certificate);
        mbedtls_pk_init(&key); mbedtls_ssl_config_init(&config); mbedtls_net_init(&listener);
    }
    ~Impl() {
        clients.clear(); mbedtls_net_free(&listener);
        if (callbacks) nghttp2_session_callbacks_del(callbacks);
        mbedtls_ssl_config_free(&config); mbedtls_x509_crt_free(&certificate);
        mbedtls_pk_free(&key); mbedtls_ctr_drbg_free(&rng); mbedtls_entropy_free(&entropy);
    }
    bool setup() {
        if (owner.certPath_.empty() || owner.keyPath_.empty() || owner.port_<1 || owner.port_>65535) return false;
        const unsigned char label[]="gvio-dev-server";
        if (mbedtls_ctr_drbg_seed(&rng,mbedtls_entropy_func,&entropy,label,sizeof(label))!=0 ||
            mbedtls_x509_crt_parse_file(&certificate,owner.certPath_.c_str())!=0) return false;
#if MBEDTLS_VERSION_MAJOR >= 3
        int r=mbedtls_pk_parse_keyfile(&key,owner.keyPath_.c_str(),nullptr,mbedtls_ctr_drbg_random,&rng);
#else
        int r=mbedtls_pk_parse_keyfile(&key,owner.keyPath_.c_str(),nullptr);
#endif
        if (r!=0 || mbedtls_ssl_config_defaults(&config,MBEDTLS_SSL_IS_SERVER,MBEDTLS_SSL_TRANSPORT_STREAM,MBEDTLS_SSL_PRESET_DEFAULT)!=0) return false;
        mbedtls_ssl_conf_rng(&config,mbedtls_ctr_drbg_random,&rng);
        mbedtls_ssl_conf_authmode(&config,MBEDTLS_SSL_VERIFY_NONE); // no client certificate; local development only
        if (mbedtls_ssl_conf_own_cert(&config,&certificate,&key)!=0) return false;
        static const char* alpn[]={"h2",nullptr};
        if (mbedtls_ssl_conf_alpn_protocols(&config,alpn)!=0) return false;
        if (nghttp2_session_callbacks_new(&callbacks)!=0) return false;
        nghttp2_session_callbacks_set_on_begin_headers_callback(callbacks,beginHeaders);
        nghttp2_session_callbacks_set_on_header_callback(callbacks,header);
        nghttp2_session_callbacks_set_on_data_chunk_recv_callback(callbacks,dataChunk);
        nghttp2_session_callbacks_set_on_frame_recv_callback(callbacks,frameReceived);
        nghttp2_session_callbacks_set_on_stream_close_callback(callbacks,streamClosed);
        std::string port=std::to_string(owner.port_);
        return mbedtls_net_bind(&listener,owner.host_.c_str(),port.c_str(),MBEDTLS_NET_PROTO_TCP)==0 &&
               mbedtls_net_set_nonblock(&listener)==0;
    }
    void acceptClients() {
        for (int i=0;i<4 && clients.size()<kMaxConnections;++i) {
            auto c=std::make_unique<Connection>();
            if (mbedtls_net_accept(&listener,&c->net,nullptr,0,nullptr)!=0) return;
            if (mbedtls_net_set_nonblock(&c->net)!=0 || mbedtls_ssl_setup(&c->ssl,&config)!=0) continue;
            mbedtls_ssl_set_bio(&c->ssl,&c->net,mbedtls_net_send,mbedtls_net_recv,nullptr);
            clients.push_back(std::move(c));
        }
    }
    void read(Connection& c) {
        if (!c.handshaked) {
            int r=mbedtls_ssl_handshake(&c.ssl);
            if (retry(r)) return;
            const char* selected=mbedtls_ssl_get_alpn_protocol(&c.ssl);
            if (r!=0 || !selected || std::strcmp(selected,"h2")!=0 ||
                nghttp2_session_server_new(&c.h2,callbacks,&c)!=0) { c.dead=true; return; }
            nghttp2_settings_entry settings[]={{NGHTTP2_SETTINGS_MAX_CONCURRENT_STREAMS,uint32_t(kMaxStreams)},
                                               {NGHTTP2_SETTINGS_MAX_HEADER_LIST_SIZE,uint32_t(kMaxHeaders)}};
            if (nghttp2_submit_settings(c.h2,NGHTTP2_FLAG_NONE,settings,2)!=0) { c.dead=true; return; }
            c.handshaked=true; c.activity=nowMs();
        }
        for (int i=0;i<8;++i) {
            uint8_t buffer[16384]; int r=mbedtls_ssl_read(&c.ssl,buffer,sizeof(buffer));
            if (retry(r)) break;
            if (r<=0) { c.dead=true; break; }
            c.activity=nowMs();
            ssize_t consumed=nghttp2_session_mem_recv(c.h2,buffer,size_t(r));
            if (consumed!=r) { c.dead=true; break; }
        }
    }
    void respond(Connection& c,int32_t id,Stream& s) {
        if (!s.complete || s.responded) return;
        s.responded=true;
        RouteResult route;
        try {
            route=routeRequest(s.method,s.path,s.body,owner.webRoot_,"",owner.controlFn_);
        } catch (const std::exception&) { route.status=500; route.contentType="text/plain"; }
        std::string status=std::to_string(route.status),noCache="no-store";
        nghttp2_nv headers[]={nv(":status",status),nv("content-type",route.contentType),nv("cache-control",noCache)};
        s.sse=route.sse; s.data=std::move(route.body); s.offset=0;
        if (s.sse) {
            s.data=sseEvent(owner.statusFn_?owner.statusFn_():"{}"); s.nextEvent=nowMs()+200;
        }
        nghttp2_data_provider provider{}; provider.source.ptr=&s; provider.read_callback=readBody;
        // A provider is required even for SSE's initial headers, otherwise h2
        // ends the stream. Each stream owns its own deferred payload.
        if (nghttp2_submit_response(c.h2,id,headers,3,&provider)!=0) c.dead=true;
        s.body.clear();
    }
    void streamEvents(Connection& c) {
        uint64_t now=nowMs();
        for (auto& kv:c.streams) {
            auto& s=*kv.second; respond(c,kv.first,s);
            if (!s.sse || now<s.nextEvent || s.offset!=s.data.size()) continue;
            std::string text=owner.statusFn_?owner.statusFn_():"{}";
            if (text.size()>1024*1024) { c.dead=true; return; }
            s.data=sseEvent(text); s.offset=0; s.nextEvent=now+200;
            // INVALID_ARGUMENT is harmless when the data provider has not yet
            // become deferred; it will consume the new buffer on its next call.
            int r=nghttp2_session_resume_data(c.h2,kv.first);
            if (r!=0 && r!=NGHTTP2_ERR_INVALID_ARGUMENT) { c.dead=true; return; }
        }
    }
    void flush(Connection& c) {
        if (c.output.empty()) return;
        // Keep buffer address AND remaining length unchanged across WANT_*.
        int r=mbedtls_ssl_write(&c.ssl,c.output.data()+c.offset,c.output.size()-c.offset);
        if (retry(r)) return;
        if (r<=0) { c.dead=true; return; }
        c.offset+=size_t(r); c.activity=nowMs();
        if (c.offset==c.output.size()) { c.output.clear(); c.offset=0; }
    }
    void pump(Connection& c) {
        for (int i=0;i<16 && !c.dead;++i) {
            if (!c.output.empty()) { flush(c); if (!c.output.empty()) return; }
            const uint8_t* data=nullptr; ssize_t n=nghttp2_session_mem_send(c.h2,&data);
            if (n<0) { c.dead=true; return; }
            if (!n) return;
            c.output.assign(data,data+n); c.offset=0; flush(c);
        }
    }
    void run() {
        try {
            while (!stopping) {
                acceptClients();
                for (auto& c:clients) {
                    read(*c);
                    if (!c->dead && c->handshaked) { streamEvents(*c); pump(*c); }
                    if (nowMs()-c->activity>60000) c->dead=true;
                }
                clients.erase(std::remove_if(clients.begin(),clients.end(),[](const auto& c){return c->dead;}),clients.end());
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        } catch (const std::exception&) { stopping=true; }
        clients.clear(); mbedtls_net_free(&listener); active=false;
    }
};
Http2Server::Http2Server(const std::string& host,int port,const std::string& cert,const std::string& key,const std::string& root)
    :host_(host),port_(port),certPath_(cert),keyPath_(key),webRoot_(root) {}
Http2Server::~Http2Server() { stop(); }
bool Http2Server::start() {
    if (impl_ && impl_->active) return true;
    stop(); impl_=std::make_unique<Impl>(*this);
    if (!impl_->setup()) { impl_.reset(); return false; }
    impl_->active=true;
    try { impl_->thread=std::thread([this]{impl_->run();}); }
    catch (...) { impl_.reset(); return false; }
    return true;
}
void Http2Server::stop() {
    if (!impl_) return;
    impl_->stopping=true;
    if (impl_->thread.joinable() && impl_->thread.get_id()!=std::this_thread::get_id()) impl_->thread.join();
}
} // namespace gvio
#else
namespace gvio {
struct Http2Server::Impl {};
Http2Server::Http2Server(const std::string& host,int port,const std::string& cert,const std::string& key,const std::string& root)
    :host_(host),port_(port),certPath_(cert),keyPath_(key),webRoot_(root) {}
Http2Server::~Http2Server()=default;
bool Http2Server::start() { return false; }
void Http2Server::stop() {}
}
#endif
