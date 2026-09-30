#include "gvio/msckf.h"
#include "gvio/timeline.h"
#include <iostream>
#include <stdexcept>
using namespace gvio;
void require(bool value,const char* message) { if (!value) throw std::runtime_error(message); }
int main() {
    try {
        FilterConfig cfg; cfg.maxFeatures=1;
        MsckfFilter f(cfg); f.initialize(Vec3::Zero(),Quat::Identity(),0);
        f.feedImage(0,{},{{1,Vec2(100,100)}});
        ImuSample s; s.accel=-GRAVITY; s.t=5000000;
        f.feedImu(s); f.feedImage(s.t,{},{{2,Vec2(120,100)}});
        require(f.trackCount()==1,"lost slots must be reusable in the SAME frame");
        s.t=10000000; f.feedImu(s); f.feedImage(s.t,{{2,Vec2(120,100)}},{});
        require(f.trackCount()==1,"replacement track survives");
        SensorTimeline noImu;
        noImu.pushObservation(10,40,[]{throw std::runtime_error("observation without IMU");});
        noImu.drain([](const ImuSample&){},true);
        require(noImu.unbracketed==1 && noImu.queued()==0,"release unbracketed observations at final drain");
        std::cout<<"PASS full-budget feature replacement and no-IMU shutdown\n";
        return 0;
    } catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
