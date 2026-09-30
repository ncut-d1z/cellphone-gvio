#include "gvio/fusion.h"
#include <iostream>
#include <stdexcept>
using namespace gvio;
int main() {
    try {
        Config c; c.valid=true; c.filter.gpsEnabled=false; c.filter.magEnabled=false;
        c.filter.cam.width=80; c.filter.cam.height=60; c.filter.reorderWindowNs=1000000000;
        auto decoded=Config::fromJson(c.toJson());
        if (!decoded.valid || Json::parse(decoded.toJson())!=Json::parse(c.toJson())) throw std::runtime_error("configuration round trip");
        FusionEngine engine(c); engine.setClockOffset(0);
        for (int run=0;run<3;++run) {
            if (!engine.start()) throw std::runtime_error("headless start requires no certificates");
            float values[6]={0,0,9.80665f,0,0,0};
            for (int i=0;i<=100;++i) engine.feedImu(i*5000000LL,values,nullptr);
            std::vector<uint8_t> image(80*60,100);
            engine.feedImage(497500000,0,100,80,60,80,image.data(),image.size());
            engine.stop(); // deterministic final drain; no sleep-based correctness checks
            auto j=Json::parse(engine.statusJson());
            if (j.at("mode")!="running" || j.at("stats").at("frames")!=1 || !j.at("error").get<std::string>().empty())
                throw std::runtime_error("headless initialization/final image drain: "+j.dump());
            engine.stop();
        }
        Config invalid=c; invalid.filter.serverEnabled=true; invalid.filter.certPath="missing.crt"; invalid.filter.keyPath="missing.key";
        FusionEngine failing(invalid); if (failing.start()) throw std::runtime_error("TLS failure must propagate");
        std::cout<<"PASS engine lifecycle/configuration\n"; return 0;
    } catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
