#pragma once

#include "gvio/config.h"
#include "gvio/magcalib.h"
#include "gvio/msckf.h"
#include "gvio/vision.h"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

namespace gvio {

class Http2Server;

// 融合引擎: 采集线程(JNI)与融合线程之间的队列调度 + 初始化 + 快照 + HTTP/2 服务器。
class FusionEngine {
public:
    explicit FusionEngine(const Config& cfg);
    ~FusionEngine();

    bool start();
    void stop();
    void reset();
    void startMagCal();

    // 采集侧(线程安全)
    void feedImu(int64_t tNs, const float* values, const float* biases);
    void feedMag(int64_t tNs, const float* values, const float* biases);
    void feedGpsFix(int64_t tNs, double lat, double lon, double alt,
                    float accH, float speed, float bearing);
    void feedGnssSats(int64_t tNs, const int* constellations, const float* cn0, int n);
    void feedImage(int64_t tNs, int64_t exposureNs, float iso,
                   int width, int height, int yStride,
                   const uint8_t* yPlane, size_t yLen);
    void setClockOffset(double offsetNs);
    bool clockOffsetReady() const;

    std::string statusJson();   // 线程安全

    const Config& config() const { return cfg_; }
    bool running() const { return running_.load(); }

private:
    struct ImageMsg {
        int64_t t = 0;
        int64_t exposureNs = 0;
        float iso = 0;
        int w = 0, h = 0, stride = 0;
        std::vector<uint8_t> y;
    };

    Config cfg_;
    MsckfFilter filter_;
    MagCalibrator magCal_;

    std::mutex mtx_;
    std::condition_variable cv_;
    std::deque<ImuSample> imuQ_;
    std::deque<MagSample> magQ_;
    std::deque<GpsFix> gpsQ_;
    std::deque<GnssSats> satsQ_;
    std::deque<ImageMsg> imgQ_;
    bool stop_ = false;
    bool resetPending_ = false;

    // 初始化缓冲
    bool initHasGps_ = false;
    bool initHasOrigin_ = false;
    double originLat_ = 0, originLon_ = 0, originAlt_ = 0;
    Vec3 originEcef_ = Vec3::Zero();
    std::vector<Vec3> initAccel_;
    std::vector<Vec3> initMag_;
    Vec3 magBias_ = Vec3::Zero();
    bool magBiasSet_ = false;
    int64_t initT0_ = 0;
    bool initDone_ = false;
    int gpsSats_ = 0;
    double gpsCn0Avg_ = 0;

    // 快照
    mutable std::mutex snapMtx_;
    nlohmann::json snap_;
    double clockOffsetNs_ = 0;
    std::atomic<bool> hasClockOffset_{false};

    std::thread thread_;
    std::atomic<bool> running_{false};
    uint64_t frames_ = 0;
    uint64_t prevTracks_ = 0;

    // 视觉跟踪状态(融合线程独占)
    std::vector<Keypoint> prevKps_;
    std::vector<uint32_t> prevIds_;
    std::vector<uint8_t> prevGray_;
    bool havePrev_ = false;
    std::vector<Vec3> gpsPath_;

    Http2Server* server_ = nullptr;

    void loop();
    void processImu(const ImuSample& s);
    void processMag(const MagSample& s);
    void processGps(const GpsFix& f);
    void processSats(const GnssSats& s);
    void processImage(ImageMsg& m);
    void maybeInit(int64_t t);
    void buildSnapshot();
    void wgs84ToEnu(double lat, double lon, double alt, Vec3& enu) const;
    void enuToWgs84(const Vec3& enu, double& lat, double& lon, double& alt) const;
};

}  // namespace gvio
