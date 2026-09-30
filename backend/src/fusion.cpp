#include "gvio/fusion.h"
#include "gvio/server.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace gvio {
namespace {
Vec3 ecef(double lat,double lon,double alt) {
    constexpr double a=6378137.,f=1./298.257223563,e2=f*(2.-f);
    double phi=lat*kPi/180.,lam=lon*kPi/180.,N=a/std::sqrt(1.-e2*std::sin(phi)*std::sin(phi));
    return {(N+alt)*std::cos(phi)*std::cos(lam),(N+alt)*std::cos(phi)*std::sin(lam),(N*(1.-e2)+alt)*std::sin(phi)};
}
}
FusionEngine::FusionEngine(const Config& cfg):cfg_(cfg),filter_(cfg.filter),timeline_(cfg.filter.reorderWindowNs) {}
FusionEngine::~FusionEngine() { stop(); }
bool FusionEngine::start() {
    std::lock_guard<std::mutex> life(lifecycleMtx_);
    if (running_) return true;
    if (thread_.joinable()) thread_.join();
    { std::lock_guard<std::mutex> lk(mtx_); stop_=false; pending_.clear(); commands_.clear(); }
    clearState(); paused_=false; error_.clear();
    try {
        if (cfg_.filter.serverEnabled) {
            auto& f=cfg_.filter;
            server_=std::make_unique<Http2Server>(f.serverHost,f.serverPort,f.certPath,f.keyPath,f.webRoot);
            server_->setStatusFn([this]{return statusJson();});
            server_->setControlFn([this](const std::string& body) {
                try {
                    std::string cmd=Json::parse(body).value("cmd","");
                    if (cmd=="stop") command(Command::Pause);
                    else if (cmd=="start") command(Command::Resume);
                    else if (cmd=="reset") command(Command::Reset);
                    else if (cmd=="magcal") command(Command::MagCal);
                    else return std::string("{\"ok\":false,\"msg\":\"unknown command\"}");
                    return std::string("{\"ok\":true,\"msg\":\"queued\"}");
                } catch (const std::exception&) { return std::string("{\"ok\":false,\"msg\":\"bad command\"}"); }
            });
            if (!server_->start()) { server_.reset(); return false; }
        }
        running_=true;
        thread_=std::thread(&FusionEngine::loop,this);
    } catch (const std::exception&) {
        running_=false; if (server_) server_->stop(); server_.reset(); return false;
    }
    return true;
}
void FusionEngine::stop() {
    std::lock_guard<std::mutex> life(lifecycleMtx_);
    { std::lock_guard<std::mutex> lk(mtx_); stop_=true; }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
    if (server_) { server_->stop(); server_.reset(); }
    running_=false;
}
void FusionEngine::command(Command cmd) {
    { std::lock_guard<std::mutex> lk(mtx_);
      if (commands_.size()<32) commands_.push_back(cmd); else ++admissionDrops_; }
    cv_.notify_one();
}
void FusionEngine::reset() { command(Command::Reset); }
void FusionEngine::startMagCal() { command(Command::MagCal); }
void FusionEngine::setClockOffset(double ns) {
    if (!std::isfinite(ns) || std::abs(ns)>9e18) throw std::invalid_argument("invalid clock offset");
    std::lock_guard<std::mutex> lk(mtx_);
    int64_t value=static_cast<int64_t>(ns);
    if (running_ && hasClockOffset_ && value!=clockOffsetNs_)
        throw std::logic_error("stop acquisition before changing clock domain");
    clockOffsetNs_=value; hasClockOffset_=true;
}
void FusionEngine::enqueue(Pending p) {
    { std::lock_guard<std::mutex> lk(mtx_);
      if (stop_ || !running_ || pending_.size()>=8192) { ++admissionDrops_; return; }
      pending_.push_back(std::move(p)); }
    cv_.notify_one();
}
void FusionEngine::feedImu(int64_t t,const float* v,const float*) {
    if (!v) { ++admissionDrops_; return; }
    Pending p; p.imu=true; p.sample.t=t; p.sample.accel=Vec3(v[0],v[1],v[2]); p.sample.gyro=Vec3(v[3],v[4],v[5]);
    enqueue(std::move(p));
}
void FusionEngine::feedMag(int64_t t,const float* v,const float* bias) {
    if (!v || !bias) { ++admissionDrops_; return; }
    MagSample s; s.t=t; s.value=Vec3(v[0],v[1],v[2]); s.bias=Vec3(bias[0],bias[1],bias[2]);
    Pending p; p.t=t; p.priority=10; p.apply=[this,s]{processMag(s);}; enqueue(std::move(p));
}
void FusionEngine::feedGpsFix(int64_t t,double lat,double lon,double alt,float accuracy,float speed,float bearing) {
    if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(alt) || std::abs(lat)>90. ||
        std::abs(lon)>180. || !std::isfinite(accuracy) || accuracy<=0.) { ++admissionDrops_; return; }
    GpsFix f; f.t=t; f.lat=lat; f.lon=lon; f.alt=alt; f.sigma=accuracy; f.speed=speed; f.bearing=bearing;
    Pending p; p.t=t; p.priority=20; p.apply=[this,f]{processGps(f);}; enqueue(std::move(p));
}
void FusionEngine::feedGnssSats(int64_t t,const int* types,const float* cn0,int n) {
    if (n<0 || n>1024 || (n && (!types || !cn0))) { ++admissionDrops_; return; }
    GnssSats s; s.t=t;
    if (n) { s.constellations.assign(types,types+n); s.cn0.assign(cn0,cn0+n); }
    Pending p; p.t=t; p.priority=30; p.apply=[this,s]{processSats(s);}; enqueue(std::move(p));
}
void FusionEngine::feedImage(int64_t t,int64_t exposure,float iso,int w,int h,int stride,const uint8_t* y,size_t len) {
    if (!hasClockOffset_ || !y || w<1 || h<1 || w>16384 || h>16384 || stride<w || stride>1048576 ||
        size_t(h-1)*size_t(stride)+size_t(w)>len || size_t(w)*h>32*1024*1024) { ++admissionDrops_; return; }
    int64_t offset=clockOffsetNs_.load();
    if (t<0 || (offset>0 && t>std::numeric_limits<int64_t>::max()-offset) || (offset<0 && t<-offset)) { ++admissionDrops_; return; }
    if (imageQuota_->fetch_add(1)>=8) { imageQuota_->fetch_sub(1); ++admissionDrops_; return; }
    std::shared_ptr<ImageMsg> frame;
    try { frame=std::make_shared<ImageMsg>(); }
    catch (...) { imageQuota_->fetch_sub(1); throw; }
    frame->quota=imageQuota_; frame->t=t+offset; frame->exposureNs=exposure; frame->iso=iso; frame->w=w; frame->h=h;
    grayFromY(y,w,h,stride,frame->gray);
    Pending p; p.t=frame->t; p.priority=40; p.apply=[this,frame]{processImage(*frame);}; enqueue(std::move(p));
}
void FusionEngine::clearState() {
    filter_.reset(); timeline_.clear(); magCal_=MagCalibrator{};
    initDone_=initHasOrigin_=magBiasSet_=havePrev_=false;
    initAccel_.clear(); initMag_.clear(); gpsPath_.clear();
    prevGray_.clear(); prevKps_.clear(); prevIds_.clear(); magBias_.setZero();
    originEcef_.setZero(); ecefToEnu_.setIdentity(); originLat_=originLon_=originAlt_=0.;
    gpsSats_=0; gpsCn0Avg_=0.; frames_=prevTracks_=rejectedImages_=0;
}
void FusionEngine::loop() {
    while (true) {
        std::deque<Pending> batch; std::deque<Command> commands; bool final;
        { std::unique_lock<std::mutex> lk(mtx_);
          cv_.wait(lk,[this]{return stop_ || !pending_.empty() || !commands_.empty();});
          final=stop_; batch.swap(pending_); commands.swap(commands_); }
        try {
            for (Command c:commands) {
                if (c==Command::MagCal) magCal_.start();
                else {
                    clearState(); batch.clear(); error_.clear();
                    if (c==Command::Pause) paused_=true;
                    if (c==Command::Resume) paused_=false;
                }
            }
            if (!paused_) {
                for (auto& p:batch) {
                    if (p.imu) timeline_.pushImu(p.sample);
                    else timeline_.pushObservation(p.t,p.priority,std::move(p.apply));
                }
                timeline_.drain([this](const ImuSample& s){processImu(s);},final);
            } else admissionDrops_+=batch.size();
        } catch (const std::exception& e) {
            error_=e.what(); paused_=true; timeline_.clear();
        }
        buildSnapshot();
        if (final) break;
    }
    running_=false;
}
void FusionEngine::processImu(const ImuSample& s) {
    if (!initDone_) {
        initAccel_.push_back(s.accel); if (initAccel_.size()>80) initAccel_.pop_front();
        filter_.feedImu(s); maybeInit(s.t);
    } else filter_.feedImu(s);
}
void FusionEngine::processMag(const MagSample& s) {
    if (!s.value.allFinite() || !s.bias.allFinite()) return;
    if (magCal_.isCollecting()) {
        magCal_.addSample(s.value,s.t); // fit RAW samples: the fitted bias is absolute
        if (magCal_.isDone()) { magCal_.finish(); magBias_=magCal_.bias(); magBiasSet_=magBias_.allFinite(); }
    }
    Vec3 bias=magBiasSet_?magBias_:s.bias;
    if (!initDone_) {
        initMag_.push_back(s.value-bias); if (initMag_.size()>20) initMag_.pop_front(); maybeInit(s.t);
    } else { MagSample corrected=s; corrected.bias=bias; filter_.feedMag(corrected); }
}
void FusionEngine::processGps(const GpsFix& f) {
    if (!cfg_.filter.gpsEnabled) return;
    if (!initHasOrigin_) {
        originLat_=f.lat; originLon_=f.lon; originAlt_=f.alt; originEcef_=ecef(f.lat,f.lon,f.alt);
        double p=f.lat*kPi/180.,l=f.lon*kPi/180.;
        ecefToEnu_ << -std::sin(l),std::cos(l),0.,
            -std::sin(p)*std::cos(l),-std::sin(p)*std::sin(l),std::cos(p),
            std::cos(p)*std::cos(l),std::cos(p)*std::sin(l),std::sin(p);
        initHasOrigin_=true; maybeInit(f.t);
    }
    if (!initDone_) return;
    Vec3 enu=ecefToEnu_*(ecef(f.lat,f.lon,f.alt)-originEcef_);
    gpsPath_.push_back(enu); if (gpsPath_.size()>800) gpsPath_.erase(gpsPath_.begin(),gpsPath_.begin()+100);
    filter_.feedGpsPosition(f,enu);
}
void FusionEngine::processSats(const GnssSats& s) {
    gpsSats_=int(s.cn0.size()); double sum=0;
    for (float x:s.cn0) if (std::isfinite(x)) sum+=x;
    gpsCn0Avg_=gpsSats_?sum/gpsSats_:0.;
}
void FusionEngine::maybeInit(int64_t t) {
    if (initDone_ || initAccel_.size()<80 || (cfg_.filter.gpsEnabled && !initHasOrigin_) ||
        (cfg_.filter.magEnabled && initMag_.size()<3)) return;
    Vec3 a=Vec3::Zero(); for (const auto& x:initAccel_) a+=x;
    if (a.norm()<1e-8) return; Vec3 z=a.normalized();
    Quat q=Quat::FromTwoVectors(Vec3::UnitZ(),z);
    if (cfg_.filter.magEnabled) {
        Vec3 m=Vec3::Zero(); for (const auto& x:initMag_) m+=x;
        Vec3 y=m-z*z.dot(m); if (y.norm()<1e-8) return; y.normalize(); Vec3 x=y.cross(z);
        Mat3 R; R.col(0)=x; R.col(1)=y; R.col(2)=z;
        q=Quat(R)*Quat(Eigen::AngleAxisd(cfg_.filter.magDeclinationDeg*kPi/180.,Vec3::UnitZ()));
    }
    filter_.initialize(Vec3::Zero(),q,t); initDone_=true;
}
void FusionEngine::processImage(const ImageMsg& frame) {
    if (!initDone_ || frame.t!=filter_.imuTime() || frame.t<=filter_.lastImageTime() ||
        frame.w!=cfg_.filter.cam.width || frame.h!=cfg_.filter.cam.height) { ++rejectedImages_; return; }
    std::vector<Keypoint> live; std::vector<uint32_t> ids;
    std::vector<std::pair<uint32_t,Vec2>> matched,fresh;
    if (havePrev_ && !prevKps_.empty()) {
        std::vector<Keypoint> current; std::vector<uint8_t> status;
        trackKlt(prevGray_,frame.gray,frame.w,frame.h,prevKps_,current,status);
        for (size_t i=0;i<prevIds_.size();++i) if (status[i]) {
            live.push_back(current[i]); ids.push_back(prevIds_[i]);
            matched.push_back({prevIds_[i],Vec2(current[i].x,current[i].y)});
        }
    }
    const auto& c=cfg_.filter;
    if (live.size()<size_t(c.maxFeatures)) {
        std::vector<Keypoint> detected;
        detectFast(frame.gray.data(),frame.w,frame.h,c.fastThreshold,detected,c.maxFeatures,c.maxCellsX,c.maxCellsY,c.maxPerCell);
        for (const auto& kp:detected) {
            if (live.size()>=size_t(c.maxFeatures)) break;
            bool near=false;
            for (const auto& p:live) if (std::hypot(kp.x-p.x,kp.y-p.y)<18.f) { near=true; break; }
            if (near) continue;
            uint32_t id=filter_.allocateTrackId(); ids.push_back(id); live.push_back(kp); fresh.push_back({id,Vec2(kp.x,kp.y)});
        }
    }
    if (filter_.feedImage(frame.t,matched,fresh)!=ImageResult::Accepted) { ++rejectedImages_; return; }
    prevKps_=std::move(live); prevIds_=std::move(ids); prevGray_=frame.gray;
    havePrev_=true; ++frames_; prevTracks_=matched.size();
}
void FusionEngine::buildSnapshot() {
    Json j;
    j["mode"]=paused_?"paused":initDone_?"running":"init";
    j["error"]=error_; j["t"]=filter_.imuTime();
    j["gpsSats"]=gpsSats_; j["gpsCn0"]=gpsCn0Avg_;
    j["accelSamples"]=initAccel_.size(); j["magSamples"]=initMag_.size(); j["hasOrigin"]=initHasOrigin_;
    Quat q=filter_.imuQuat(); Vec3 p=filter_.imuPos(),v=filter_.imuVel(); Quat bodyToGlobal=q.conjugate();
    j["pose"]={{"qx",q.x()},{"qy",q.y()},{"qz",q.z()},{"qw",q.w()},
        {"x",p.x()},{"y",p.y()},{"z",p.z()},{"vx",v.x()},{"vy",v.y()},{"vz",v.z()},
        {"yawDeg",quatYaw(bodyToGlobal)*180./kPi},
        {"pitchDeg",std::asin(std::clamp(2.*(bodyToGlobal.w()*bodyToGlobal.y()-bodyToGlobal.z()*bodyToGlobal.x()),-1.,1.))*180./kPi},
        {"rollDeg",std::atan2(2.*(bodyToGlobal.w()*bodyToGlobal.x()+bodyToGlobal.y()*bodyToGlobal.z()),1.-2.*(bodyToGlobal.x()*bodyToGlobal.x()+bodyToGlobal.y()*bodyToGlobal.y()))*180./kPi}};
    j["window"]=filter_.cameraCount(); j["tracks"]=filter_.trackCount(); j["tracked"]=prevTracks_;
    j["features"]=Json::array(); for (const auto& x:filter_.recentPoints()) j["features"].push_back({x.x(),x.y(),x.z()});
    j["gpsPath"]=Json::array(); for (const auto& x:gpsPath_) j["gpsPath"].push_back({x.x(),x.y(),x.z()});
    j["origin"]={{"lat",originLat_},{"lon",originLon_},{"alt",originAlt_}};
    j["mag"]={{"state",magCal_.isCollecting()?"collecting":magCal_.isDone()?"done":"idle"},
        {"samples",magCal_.collected()},{"bias",{magBias_.x(),magBias_.y(),magBias_.z()}},{"radius",magCal_.radius()}};
    j["stats"]={{"imuHz",filter_.imuRate()},{"imgHz",filter_.imgRate()},{"gpsHz",filter_.gpsRate()},
        {"updates",filter_.updateCount()},{"visualUpdates",filter_.visualUpdateCount()},
        {"frames",frames_},{"updateUs",filter_.lastUpdateDurationUs()},
        {"clockOffsetNs",clockOffsetNs_.load()},{"clockReady",hasClockOffset_.load()},
        {"late",timeline_.late},{"overflow",timeline_.overflow},{"invalid",timeline_.invalid},
        {"unbracketed",timeline_.unbracketed},{"admissionDrops",admissionDrops_.load()},
        {"rejectedImages",rejectedImages_},{"queued",timeline_.queued()}};
    Vec3 bg=filter_.imuBiasG(),ba=filter_.imuBiasA();
    j["calib"]={{"bg",{bg.x(),bg.y(),bg.z()}},{"ba",{ba.x(),ba.y(),ba.z()}},{"declDeg",cfg_.filter.magDeclinationDeg}};
    std::lock_guard<std::mutex> lk(snapMtx_); snap_=std::move(j);
}
std::string FusionEngine::statusJson() {
    std::lock_guard<std::mutex> lk(snapMtx_);
    return snap_.is_null()?std::string("{\"mode\":\"idle\"}"):snap_.dump();
}
} // namespace gvio
