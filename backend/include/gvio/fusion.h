#pragma once
#include <Eigen/QR>
#include "gvio/config.h"
#include "gvio/magcalib.h"
#include "gvio/msckf.h"
#include "gvio/timeline.h"
#include "gvio/vision.h"
#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace gvio {
class Http2Server;
// Sensor admission is thread safe; all filter, calibration and timeline state is
// owned exclusively by the worker. HTTP callbacks enqueue commands, never join.
class FusionEngine {
public:
    explicit FusionEngine(const Config& cfg);
    ~FusionEngine();
    bool start();
    void stop();
    void reset();
    void startMagCal();
    void feedImu(int64_t t,const float* values,const float* biases);
    void feedMag(int64_t t,const float* values,const float* biases);
    void feedGpsFix(int64_t t,double lat,double lon,double alt,float accuracy,float speed,float bearing);
    void feedGnssSats(int64_t t,const int* constellations,const float* cn0,int n);
    void feedImage(int64_t t,int64_t exposure,float iso,int width,int height,int stride,const uint8_t* y,size_t len);
    void setClockOffset(double offsetNs);
    bool clockOffsetReady() const { return hasClockOffset_.load(); }
    std::string statusJson();
    const Config& config() const { return cfg_; }
    bool running() const { return running_.load(); }
private:
    struct ImageMsg {
        int64_t t=0,exposureNs=0; float iso=0; int w=0,h=0;
        std::vector<uint8_t> gray;
        std::shared_ptr<std::atomic<int>> quota;
        ~ImageMsg() { if (quota) quota->fetch_sub(1); }
    };
    enum class Command { Reset, MagCal, Pause, Resume };
    struct Pending {
        bool imu=false; ImuSample sample;
        int64_t t=0; int priority=0; std::function<void()> apply;
    };
    Config cfg_;
    MsckfFilter filter_;
    MagCalibrator magCal_;
    SensorTimeline timeline_;
    std::mutex lifecycleMtx_,mtx_,snapMtx_;
    std::condition_variable cv_;
    std::deque<Pending> pending_;
    std::deque<Command> commands_;
    bool stop_=false;
    std::thread thread_;
    std::atomic<bool> running_{false},hasClockOffset_{false};
    std::atomic<int64_t> clockOffsetNs_{0};
    std::atomic<uint64_t> admissionDrops_{0};
    std::shared_ptr<std::atomic<int>> imageQuota_=std::make_shared<std::atomic<int>>(0);
    std::unique_ptr<Http2Server> server_;
    Json snap_;
    // Worker-only fields below.
    bool paused_=false,initDone_=false,initHasOrigin_=false,magBiasSet_=false,havePrev_=false;
    double originLat_=0,originLon_=0,originAlt_=0,gpsCn0Avg_=0;
    Vec3 originEcef_=Vec3::Zero(),magBias_=Vec3::Zero();
    Mat3 ecefToEnu_=Mat3::Identity();
    int gpsSats_=0;
    std::deque<Vec3> initAccel_,initMag_;
    std::vector<Vec3> gpsPath_;
    std::vector<Keypoint> prevKps_;
    std::vector<uint32_t> prevIds_;
    std::vector<uint8_t> prevGray_;
    uint64_t frames_=0,prevTracks_=0,rejectedImages_=0;
    std::string error_;
    void enqueue(Pending p);
    void command(Command cmd);
    void loop();
    void clearState();
    void processImu(const ImuSample& s);
    void processMag(const MagSample& s);
    void processGps(const GpsFix& f);
    void processSats(const GnssSats& s);
    void processImage(const ImageMsg& frame);
    void maybeInit(int64_t t);
    void buildSnapshot();
};
} // namespace gvio
