#include "gvio/fusion.h"

#include "gvio/server.h"

#include <cmath>

namespace gvio {

namespace {
const double kA = 6378137.0;
const double kF = 1.0 / 298.257223563;
const double kE2 = kF * (2.0 - kF);
const double kInitAccelCount = 80;   // ~0.4s @ 200Hz
const double kInitMagCount = 3;
const int kMaxGpsPath = 800;
}  // namespace

FusionEngine::FusionEngine(const Config& cfg)
    : cfg_(cfg), filter_(cfg.filter), running_(false) {
    server_ = nullptr;
}

FusionEngine::~FusionEngine() {
    stop();
}

// ---------------------------------------------------------------------------
// 启动/停止
// ---------------------------------------------------------------------------
bool FusionEngine::start() {
    if (running_.load()) return true;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        stop_ = false;
    }
    running_ = true;
    thread_ = std::thread(&FusionEngine::loop, this);

    // HTTP/2 服务器
    try {
        auto& f = cfg_.filter;
        server_ = new Http2Server(f.serverHost, f.serverPort, f.certPath, f.keyPath,
                                  f.webRoot);
        server_->setStatusFn([this]() { return statusJson(); });
        server_->setControlFn([this](const std::string& body) {
            std::string resp;
            try {
                Json j = Json::parse(body);
                std::string cmd = j.value("cmd", "");
                if (cmd == "start") {
                    resp = "{\"ok\":true,\"msg\":\"already running\"}";
                } else if (cmd == "stop") {
                    stop();
                    resp = "{\"ok\":true,\"msg\":\"stopping\"}";
                } else if (cmd == "reset") {
                    reset();
                    resp = "{\"ok\":true,\"msg\":\"reset\"}";
                } else if (cmd == "magcal") {
                    startMagCal();
                    resp = "{\"ok\":true,\"msg\":\"mag calibrating, 旋转手机\"}";
                } else {
                    resp = "{\"ok\":false,\"msg\":\"unknown cmd\"}";
                }
            } catch (...) {
                resp = "{\"ok\":false,\"msg\":\"bad json\"}";
            }
            return resp;
        });
        server_->start();
    } catch (const std::exception& e) {
        delete server_;
        server_ = nullptr;
        stop();
        return false;
    }
    return true;
}

void FusionEngine::stop() {
    running_ = false;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        stop_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
    if (server_) {
        server_->stop();
        delete server_;
        server_ = nullptr;
    }
}

void FusionEngine::reset() {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        imuQ_.clear();
        magQ_.clear();
        gpsQ_.clear();
        satsQ_.clear();
        imgQ_.clear();
        resetPending_ = true;
    }
    cv_.notify_all();
}

void FusionEngine::startMagCal() {
    std::lock_guard<std::mutex> lk(mtx_);
    magCal_.start();
}

void FusionEngine::setClockOffset(double offsetNs) {
    clockOffsetNs_ = offsetNs;
    hasClockOffset_.store(true);
}

bool FusionEngine::clockOffsetReady() const {
    return hasClockOffset_.load();
}

// ---------------------------------------------------------------------------
// 采集侧入队
// ---------------------------------------------------------------------------
void FusionEngine::feedImu(int64_t tNs, const float* values, const float* biases) {
    ImuSample s;
    s.t = tNs;
    s.gyro = Vec3(values[3], values[4], values[5]);
    s.accel = Vec3(values[0], values[1], values[2]);
    {
        std::lock_guard<std::mutex> lk(mtx_);
        imuQ_.push_back(s);
    }
    cv_.notify_one();
}

void FusionEngine::feedMag(int64_t tNs, const float* values, const float* biases) {
    MagSample s;
    s.t = tNs;
    s.value = Vec3(values[0], values[1], values[2]);
    s.bias = Vec3(biases[0], biases[1], biases[2]);
    {
        std::lock_guard<std::mutex> lk(mtx_);
        magQ_.push_back(s);
    }
    cv_.notify_one();
}

void FusionEngine::feedGpsFix(int64_t tNs, double lat, double lon, double alt,
                              float accH, float speed, float bearing) {
    GpsFix f;
    f.t = tNs;
    f.lat = lat;
    f.lon = lon;
    f.alt = alt;
    f.sigma = accH > 0.5 ? accH : 10.0;
    f.speed = speed;
    f.bearing = bearing;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        gpsQ_.push_back(f);
    }
    cv_.notify_one();
}

void FusionEngine::feedGnssSats(int64_t tNs, const int* constellations,
                                const float* cn0, int n) {
    GnssSats s;
    s.t = tNs;
    s.constellations.assign(constellations, constellations + n);
    s.cn0.assign(cn0, cn0 + n);
    {
        std::lock_guard<std::mutex> lk(mtx_);
        satsQ_.push_back(s);
    }
}

void FusionEngine::feedImage(int64_t tNs, int64_t exposureNs, float iso,
                             int width, int height, int yStride,
                             const uint8_t* yPlane, size_t yLen) {
    ImageMsg m;
    m.t = tNs;
    m.exposureNs = exposureNs;
    m.iso = iso;
    m.w = width;
    m.h = height;
    m.stride = yStride;
    m.y.assign(yPlane, yPlane + yLen);
    {
        std::lock_guard<std::mutex> lk(mtx_);
        imgQ_.push_back(std::move(m));
    }
    cv_.notify_one();
}

// ---------------------------------------------------------------------------
// 坐标换算
// ---------------------------------------------------------------------------
void FusionEngine::wgs84ToEnu(double lat, double lon, double alt, Vec3& enu) const {
    double phi = lat * M_PI / 180.0, lam = lon * M_PI / 180.0;
    double N = kA / std::sqrt(1.0 - kE2 * std::sin(phi) * std::sin(phi));
    Vec3 p;
    p.x() = (N + alt) * std::cos(phi) * std::cos(lam);
    p.y() = (N + alt) * std::cos(phi) * std::sin(lam);
    p.z() = (N * (1.0 - kE2) + alt) * std::sin(phi);

    Vec3 d = p - originEcef_;
    enu.x() = -std::sin(lam) * d.x() + std::cos(lam) * d.y();
    enu.y() = -std::sin(phi) * std::cos(lam) * d.x()
              - std::sin(phi) * std::sin(lam) * d.y()
              + std::cos(phi) * d.z();
    enu.z() = std::cos(phi) * std::cos(lam) * d.x()
              + std::cos(phi) * std::sin(lam) * d.y()
              + std::sin(phi) * d.z();
}

// ---------------------------------------------------------------------------
// 融合线程
// ---------------------------------------------------------------------------
void FusionEngine::loop() {
    while (!stop_) {
        {
            std::unique_lock<std::mutex> lk(mtx_);
            if (resetPending_) {
                resetPending_ = false;
                initHasGps_ = false;
                initHasOrigin_ = false;
                initAccel_.clear();
                initMag_.clear();
                magBiasSet_ = false;
                initDone_ = false;
                gpsPath_.clear();
                filter_.reset();
                frames_ = 0;
                prevTracks_ = 0;
                havePrev_ = false;
                prevKps_.clear();
                prevIds_.clear();
                prevGray_.clear();
            }
            cv_.wait_for(lk, std::chrono::milliseconds(20),
                         [this]() {
                             return !imuQ_.empty() || !magQ_.empty() || !gpsQ_.empty() ||
                                    !satsQ_.empty() || !imgQ_.empty();
                         });
            if (stop_) break;

            // 先 IMU(传播), 再其它
            int dbgN = 0;
            while (!imuQ_.empty()) {
                ImuSample s = imuQ_.front();
                imuQ_.pop_front();
                if (dbgN++ % 25 == 0)
                    fprintf(stderr, "[dbg] drain imu #%d t=%.6f accelS=%zu\n", dbgN,
                            s.t * 1e-9, initAccel_.size());
                processImu(s);
                if (dbgN % 25 == 0)
                    fprintf(stderr, "[dbg]   done imu #%d t=%.6f accelS=%zu\n", dbgN,
                            s.t * 1e-9, initAccel_.size());
            }
            {
                static int dbg = 0;
                if (++dbg % 200 == 1)
                    fprintf(stderr, "[fusion] imu=%zu mag=%zu gps=%zu sats=%zu img=%zu initDone=%d\n",
                            imuQ_.size(), magQ_.size(), gpsQ_.size(), satsQ_.size(),
                            imgQ_.size(), initDone_ ? 1 : 0);
            }
            while (!magQ_.empty()) {
                MagSample s = magQ_.front();
                magQ_.pop_front();
                processMag(s);
            }
            while (!gpsQ_.empty()) {
                GpsFix f = gpsQ_.front();
                gpsQ_.pop_front();
                processGps(f);
            }
            while (!satsQ_.empty()) {
                GnssSats s = satsQ_.front();
                satsQ_.pop_front();
                processSats(s);
            }
            while (!imgQ_.empty()) {
                ImageMsg m = std::move(imgQ_.front());
                imgQ_.pop_front();
                processImage(m);
            }
        }
        buildSnapshot();
    }
}

void FusionEngine::processImu(const ImuSample& s) {
    fprintf(stderr, "[dbg] processImu enter t=%.6f initDone=%d\n", s.t * 1e-9,
            initDone_ ? 1 : 0);
    if (!initDone_) {
        // 初始化前: 累积加速度用于重力方向
        initAccel_.push_back(s.accel);
        if (initAccel_.size() > 4000) initAccel_.erase(initAccel_.begin(),
                                                       initAccel_.begin() + 2000);
        filter_.feedImu(s);
        maybeInit(s.t);
        return;
    }
    filter_.feedImu(s);
}

void FusionEngine::processMag(const MagSample& s) {
    if (!magBiasSet_ && !magCal_.isDone()) {
        magBias_ = s.bias;  // 系统 UNCALIBRATED 估计的硬磁偏置
        magBiasSet_ = true;
    }
    if (magCal_.isCollecting()) {
        magCal_.addSample(s.value - s.bias, s.t);
        if (magCal_.isDone()) {
            magCal_.finish();
            magBias_ = magCal_.bias();
        }
    }
    if (initDone_ && magBiasSet_) {
        MagSample c = s;
        c.bias = magBias_;
        filter_.feedMag(c);
    } else if (!initDone_) {
        initMag_.push_back(s.value - (magBiasSet_ ? magBias_ : Vec3::Zero()));
        if (initMag_.size() > 200) initMag_.erase(initMag_.begin());
        maybeInit(s.t);
    }
}

void FusionEngine::processGps(const GpsFix& f) {
    if (!initHasOrigin_) {
        initHasGps_ = true;
        initHasOrigin_ = true;
        originLat_ = f.lat;
        originLon_ = f.lon;
        originAlt_ = f.alt;
        double phi = originLat_ * M_PI / 180.0, lam = originLon_ * M_PI / 180.0;
        double N = kA / std::sqrt(1.0 - kE2 * std::sin(phi) * std::sin(phi));
        originEcef_.x() = (N + originAlt_) * std::cos(phi) * std::cos(lam);
        originEcef_.y() = (N + originAlt_) * std::cos(phi) * std::sin(lam);
        originEcef_.z() = (N * (1.0 - kE2) + originAlt_) * std::sin(phi);
        maybeInit(f.t);
        return;
    }
    if (!initDone_) {
        maybeInit(f.t);
        return;
    }
    Vec3 enu;
    wgs84ToEnu(f.lat, f.lon, f.alt, enu);
    gpsPath_.push_back(enu);
    if (static_cast<int>(gpsPath_.size()) > kMaxGpsPath)
            gpsPath_.erase(gpsPath_.begin(), gpsPath_.begin() + 100);
    filter_.feedGpsPosition(f, enu);
}

void FusionEngine::processSats(const GnssSats& s) {
    gpsSats_ = static_cast<int>(s.cn0.size());
    double sum = 0;
    for (float c : s.cn0) sum += c;
    gpsCn0Avg_ = gpsSats_ > 0 ? sum / gpsSats_ : 0;
}

void FusionEngine::maybeInit(int64_t t) {
    if (initDone_) return;
    if (!initHasOrigin_ || initAccel_.size() < kInitAccelCount ||
        initMag_.size() < kInitMagCount) {
        return;
    }
    // 重力方向 -> roll/pitch
    Vec3 a = Vec3::Zero();
    for (auto& v : initAccel_) a += v;
    a /= static_cast<double>(initAccel_.size());
    a.normalize();
    double pitch = std::atan2(-a.x(), std::sqrt(a.y() * a.y() + a.z() * a.z()));
    double roll = std::atan2(a.y(), a.z());

    // 磁力计 -> yaw (倾角补偿)
    Vec3 m = Vec3::Zero();
    for (auto& v : initMag_) m += v;
    m /= static_cast<double>(initMag_.size());
    double cp = std::cos(roll), sp = std::sin(roll);
    double ct = std::cos(pitch), st = std::sin(pitch);
    Vec3 mComp;
    mComp.x() = m.x() * ct + m.y() * sp * st + m.z() * cp * st;
    mComp.y() = m.y() * cp - m.z() * sp;
    double declRad = cfg_.filter.magDeclinationDeg * M_PI / 180.0;
    double yaw = std::atan2(mComp.x(), mComp.y()) - declRad;

    // R_IG = R_z(-yaw)·R_y(pitch)·R_x(roll): quatYaw(q.conjugate()) = 航向
    Quat q = (Eigen::AngleAxisd(-yaw, Vec3::UnitZ()) *
              Eigen::AngleAxisd(pitch, Vec3::UnitY()) *
              Eigen::AngleAxisd(roll, Vec3::UnitX()))
                 .normalized();

    filter_.initialize(Vec3::Zero(), q, t);
    initDone_ = true;
    initT0_ = t;
    std::fprintf(stderr, "[dbg] maybeInit fired t=%g yaw=%g accelS=%zu magS=%zu\n", t * 1e-9,
                 yaw * 180.0 / M_PI, initAccel_.size(), initMag_.size());
}

// ---------------------------------------------------------------------------
// 图像处理(视觉跟踪)
// ---------------------------------------------------------------------------
void FusionEngine::processImage(ImageMsg& m) {
    if (!initDone_) {
        // 初始化前也保留第一帧灰度, 避免首帧全新
        grayFromY(m.y.data(), m.w, m.h, m.stride, prevGray_);
        havePrev_ = false;
        return;
    }
    if (!hasClockOffset_.load()) {
        // 时钟偏移未就绪: 无法对齐相机时间戳, 跳过
        return;
    }
    int64_t tMono = m.t + static_cast<int64_t>(clockOffsetNs_);

    std::vector<uint8_t> gray;
    grayFromY(m.y.data(), m.w, m.h, m.stride, gray);

    auto& f = cfg_.filter;
    std::vector<Keypoint> detected;
    detectFast(gray.data(), m.w, m.h, f.fastThreshold, detected, f.maxFeatures,
               f.maxCellsX, f.maxCellsY, f.maxPerCell);

    std::vector<std::pair<uint32_t, Vec2>> matched;
    std::vector<std::pair<uint32_t, Vec2>> fresh;

    if (havePrev_ && !prevKps_.empty()) {
        std::vector<Keypoint> curKps;
        std::vector<uint8_t> status;
        trackKlt(prevGray_, gray, m.w, m.h, prevKps_, curKps, status);

        std::vector<Keypoint> curTracked;
        std::vector<uint32_t> curIds;
        for (size_t i = 0; i < prevIds_.size(); ++i) {
            if (status[i]) {
                matched.push_back({prevIds_[i], Vec2(curKps[i].x, curKps[i].y)});
                curTracked.push_back(curKps[i]);
                curIds.push_back(prevIds_[i]);
            }
        }

        // 新检测: 与已跟踪点距离足够远
        size_t budget = f.maxFeatures > filter_.trackCount()
                            ? f.maxFeatures - filter_.trackCount()
                            : 0;
        for (auto& kp : detected) {
            if (budget == 0) break;
            bool near = false;
            for (auto& tk : curTracked) {
                float dx = kp.x - tk.x, dy = kp.y - tk.y;
                if (dx * dx + dy * dy < 18.0f * 18.0f) {
                    near = true;
                    break;
                }
            }
            if (!near) {
                uint32_t id = filter_.allocateTrackId();
                fresh.push_back({id, Vec2(kp.x, kp.y)});
                curTracked.push_back(kp);
                curIds.push_back(id);
                budget--;
            }
        }
        prevKps_ = curTracked;
        prevIds_ = curIds;
    } else {
        for (auto& kp : detected) {
            uint32_t id = filter_.allocateTrackId();
            fresh.push_back({id, Vec2(kp.x, kp.y)});
        }
        prevKps_ = detected;
        for (auto& fr : fresh) prevIds_.push_back(fr.first);
    }

    if (havePrev_ || fresh.size() > 10) {
        filter_.feedImage(tMono, matched, fresh);
    }
    prevGray_ = std::move(gray);
    havePrev_ = true;
    frames_++;
    prevTracks_ = matched.size();
}

// ---------------------------------------------------------------------------
// 快照
// ---------------------------------------------------------------------------
void FusionEngine::buildSnapshot() {
    Json j;
    double tNow = 0;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!initDone_) {
            j["mode"] = "init";
            j["gpsSats"] = gpsSats_;
            j["accelSamples"] = static_cast<int>(initAccel_.size());
            j["magSamples"] = static_cast<int>(initMag_.size());
            j["hasOrigin"] = initHasOrigin_;
            snap_ = j;
            return;
        }
    }
    j["mode"] = "running";
    j["t"] = filter_.imuTime();
    Quat q = filter_.imuQuat();
    Vec3 p = filter_.imuPos();
    Vec3 v = filter_.imuVel();
    j["pose"] = {
        {"qx", q.x()}, {"qy", q.y()}, {"qz", q.z()}, {"qw", q.w()},
        {"x", p.x()}, {"y", p.y()}, {"z", p.z()},
        {"vx", v.x()}, {"vy", v.y()}, {"vz", v.z()},
        {"yawDeg", quatYaw(q.conjugate()) * 180.0 / M_PI},
        {"pitchDeg", std::asin(std::max(-1.0, std::min(1.0, 2.0 * (q.w() * q.y() - q.z() * q.x())))) * 180.0 / M_PI},
        {"rollDeg", std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                               1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y())) * 180.0 / M_PI}
    };
    j["window"] = static_cast<int>(filter_.cameraCount());
    j["tracks"] = static_cast<int>(filter_.trackCount());
    j["tracked"] = prevTracks_;

    {
        nlohmann::json arr = nlohmann::json::array();
        for (auto& pt : filter_.recentPoints()) {
            arr.push_back({pt.x(), pt.y(), pt.z()});
        }
        j["features"] = arr;
    }
    {
        nlohmann::json arr = nlohmann::json::array();
        std::lock_guard<std::mutex> lk(mtx_);
        for (auto& pt : gpsPath_) arr.push_back({pt.x(), pt.y(), pt.z()});
        j["gpsPath"] = arr;
        j["gpsSats"] = gpsSats_;
        j["gpsCn0"] = gpsCn0Avg_;
    }
    j["origin"] = {{"lat", originLat_}, {"lon", originLon_}, {"alt", originAlt_}};
    j["mag"] = {
        {"state", magCal_.isCollecting() ? "collecting"
                 : magCal_.isDone() ? "done" : "idle"},
        {"samples", magCal_.collected()},
        {"bias", {magBias_.x(), magBias_.y(), magBias_.z()}},
        {"radius", magCal_.radius()}
    };
    j["stats"] = {
        {"imuHz", filter_.imuRate()},
        {"imgHz", filter_.imgRate()},
        {"gpsHz", filter_.gpsRate()},
        {"updates", filter_.updateCount()},
        {"frames", filter_.imgCount()},
        {"updateUs", filter_.lastUpdateDurationUs()},
        {"clockOffsetNs", clockOffsetNs_},
        {"clockReady", hasClockOffset_.load()}
    };
    j["calib"] = {
        {"bg", {filter_.imuBiasG().x(), filter_.imuBiasG().y(), filter_.imuBiasG().z()}},
        {"ba", {filter_.imuBiasA().x(), filter_.imuBiasA().y(), filter_.imuBiasA().z()}},
        {"declDeg", cfg_.filter.magDeclinationDeg}
    };
    std::lock_guard<std::mutex> lk(snapMtx_);
    snap_ = j;
}

std::string FusionEngine::statusJson() {
    std::lock_guard<std::mutex> lk(snapMtx_);
    if (snap_.is_null()) {
        Json j;
        j["mode"] = "idle";
        return j.dump();
    }
    return snap_.dump();
}

}  // namespace gvio
