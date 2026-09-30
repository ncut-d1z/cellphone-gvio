#include "synthetic.h"

#include <cmath>
#include <random>

namespace gvio {

namespace {
const double kA = 6378137.0;
const double kF = 1.0 / 298.257223563;
const double kE2 = kF * (2.0 - kF);
const double kDeg = M_PI / 180.0;

double yawFunc(double t) {
    return 40.0 * kDeg * std::sin(0.25 * t) + 8.0 * kDeg * std::sin(0.05 * t);
}
}  // namespace

void SynthWorld::generate(double dtImu, double dtImg, double dtMag, double dtGps,
                          double duration, double originLat, double originLon,
                          double originAlt) {
    originLat_ = originLat;
    originLon_ = originLon;
    originAlt_ = originAlt;

    // ENU -> ECEF 旋转矩阵(列 = 东/北/天 在 ECEF 中的方向)
    double phi = originLat_ * kDeg, lam = originLon_ * kDeg;
    rotEnuToEcef_.col(0) << -std::sin(lam), std::cos(lam), 0.0;
    rotEnuToEcef_.col(1) << -std::sin(phi) * std::cos(lam), -std::sin(phi) * std::sin(lam),
        std::cos(phi);
    rotEnuToEcef_.col(2) << std::cos(phi) * std::cos(lam), std::cos(phi) * std::sin(lam),
        std::sin(phi);
    double N = kA / std::sqrt(1.0 - kE2 * std::sin(phi) * std::sin(phi));
    originEcef_ << (N + originAlt_) * std::cos(phi) * std::cos(lam),
        (N + originAlt_) * std::cos(phi) * std::sin(lam),
        (N * (1.0 - kE2) + originAlt_) * std::sin(phi);

    // 场景特征点: 地面点阵(间距 1m) + 每隔 4m 的竖直立柱(z = 1..3m)
    worldPts_.clear();
    for (double x = -10; x <= 10.0 + 1e-9; x += 1.0) {
        for (double y = -10; y <= 10.0 + 1e-9; y += 1.0) {
            worldPts_.push_back(Vec3(x, y, 0.0));
            if (std::abs(x - std::round(x / 4.0) * 4.0) < 1e-6) {
                worldPts_.push_back(Vec3(x, y, 1.0));
                worldPts_.push_back(Vec3(x, y, 2.0));
                worldPts_.push_back(Vec3(x, y, 3.0));
            }
        }
    }

    // 真值轨迹: 恒定高度, 缓慢转向的行走
    const Vec3 accBias(0.02, -0.03, 0.05);
    const Vec3 gyroBias(0.002, -0.001, 0.004);
    const Vec3 magBias(0.5, -1.2, 0.8);
    const double g = 9.80665;
    const double incl = 60.0 * kDeg, decl = 0.0;
    Vec3 bEnu = 50.0 * Vec3(std::cos(incl) * std::sin(decl), std::cos(incl) * std::cos(decl),
                            std::sin(incl));

    std::normal_distribution<double> nAcc(0.0, 0.02), nGyr(0.0, 0.002), nMag(0.0, 0.3),
        nGps(0.0, 2.0);

    double t = 0;
    Vec3 p(0, 0, 1.5), v(2.0, 0, 0);
    while (t <= duration + 1e-9) {
        double yaw = yawFunc(t);
        double yawDot = 40.0 * kDeg * 0.25 * std::cos(0.25 * t) +
                        8.0 * kDeg * 0.05 * std::cos(0.05 * t);
        Vec3 aG(0.0, 1.2 * 0.2 * std::cos(0.2 * t), 0.0);
        Vec3 vCur(2.0, 1.2 * std::sin(0.2 * t), 0.0);

        Quat q(std::cos(yaw / 2.0), 0.0, 0.0, -std::sin(yaw / 2.0));  // R_IG = R_z(-yaw)
        Mat3 R = q.toRotationMatrix();

        SynthPose sp;
        sp.t = t;
        sp.p = p;
        sp.v = vCur;
        sp.q = q;
        sp.gyroB = Vec3(0.0, 0.0, yawDot);
        sp.accelB = R * (aG + Vec3(0, 0, g));
        sp.magB = R * bEnu;
        states_.push_back(sp);

        // IMU 样本(200Hz 等效 dtImu)
        {
            SynthImuSample s;
            s.tNs = static_cast<int64_t>(t * 1e9);
            Vec3 aNoisy = sp.accelB + accBias + Vec3(nAcc(rng_), nAcc(rng_), nAcc(rng_));
            Vec3 gNoisy = sp.gyroB + gyroBias + Vec3(nGyr(rng_), nGyr(rng_), nGyr(rng_));
            s.values[0] = static_cast<float>(aNoisy.x());
            s.values[1] = static_cast<float>(aNoisy.y());
            s.values[2] = static_cast<float>(aNoisy.z());
            s.values[3] = static_cast<float>(gNoisy.x());
            s.values[4] = static_cast<float>(gNoisy.y());
            s.values[5] = static_cast<float>(gNoisy.z());
            s.biases[0] = static_cast<float>(accBias.x());
            s.biases[1] = static_cast<float>(accBias.y());
            s.biases[2] = static_cast<float>(accBias.z());
            s.biases[3] = static_cast<float>(gyroBias.x());
            s.biases[4] = static_cast<float>(gyroBias.y());
            s.biases[5] = static_cast<float>(gyroBias.z());
            imuSamples_.push_back(s);
        }
        // 磁力计
        if (t + 1e-9 >= static_cast<double>(magSamples_.size()) * dtMag) {
            SynthMagSample s;
            s.tNs = static_cast<int64_t>(t * 1e9);
            Vec3 m = sp.magB + magBias + Vec3(nMag(rng_), nMag(rng_), nMag(rng_));
            s.values[0] = static_cast<float>(m.x());
            s.values[1] = static_cast<float>(m.y());
            s.values[2] = static_cast<float>(m.z());
            s.biases[0] = static_cast<float>(magBias.x());
            s.biases[1] = static_cast<float>(magBias.y());
            s.biases[2] = static_cast<float>(magBias.z());
            magSamples_.push_back(s);
        }
        // GPS
        if (t + 1e-9 >= static_cast<double>(gpsSamples_.size()) * dtGps) {
            SynthGpsFix f;
            f.tNs = static_cast<int64_t>(t * 1e9);
            Vec3 enu = sp.p + Vec3(nGps(rng_), nGps(rng_), 0.5 * nGps(rng_));
            enuToWgs84(enu, f.lat, f.lon, f.alt);
            f.accH = 2.0f;
            gpsSamples_.push_back(f);
        }
        // 图像帧(插值渲染)
        if (t + 1e-9 >= static_cast<double>(imgFrames_.size()) * dtImg) {
            SynthImgFrame fr;
            fr.tNs = static_cast<int64_t>(t * 1e9);
            fr.width = W_;
            fr.height = H_;
            fr.y.resize(static_cast<size_t>(W_) * H_);
            renderAt(sp, fr.y);
            imgFrames_.push_back(std::move(fr));
        }

        p += 0.5 * (v + vCur) * dtImu;
        v = vCur;
        t += dtImu;
    }
}

void SynthWorld::renderAt(const SynthPose& pose, std::vector<uint8_t>& gray) const {
    std::uniform_int_distribution<int> noise(-8, 8);
    for (auto& g : gray) g = static_cast<uint8_t>(40 + noise(rng_));

    Quat qGC = (pose.q * cam_.qIC()).normalized();
    Mat3 R_CG = (qGC.toRotationMatrix()).transpose();
    Vec3 pGC = pose.p + quatInvRotate(pose.q, cam_.pIC());

    for (auto& P : worldPts_) {
        Vec3 pc = R_CG * (P - pGC);
        if (pc.z() < 0.3) continue;
        Vec2 uv = cam_.project(pc);
        int u = static_cast<int>(std::lround(uv.x()));
        int v = static_cast<int>(std::lround(uv.y()));
        if (u < 2 || v < 2 || u > W_ - 3 || v > H_ - 3) continue;
        for (int dy = -2; dy <= 2; ++dy) {
            for (int dx = -2; dx <= 2; ++dx) {
                if (dx * dx + dy * dy > 4) continue;
                gray[(v + dy) * W_ + (u + dx)] = 235;
            }
        }
    }
}

void SynthWorld::enuToWgs84(const Vec3& enu, double& lat, double& lon, double& alt) const {
    Vec3 ecef = originEcef_ + rotEnuToEcef_ * enu;
    double x = ecef.x(), y = ecef.y(), z = ecef.z();
    double p = std::sqrt(x * x + y * y);
    double lam = std::atan2(y, x);
    double phi = std::atan2(z, p * (1.0 - kE2));
    for (int i = 0; i < 10; ++i) {
        double N = kA / std::sqrt(1.0 - kE2 * std::sin(phi) * std::sin(phi));
        double h = p / std::cos(phi) - N;
        double phiNew = std::atan2(z, p * (1.0 - kE2 * N / (N + h)));
        if (std::abs(phiNew - phi) < 1e-13) {
            phi = phiNew;
            break;
        }
        phi = phiNew;
    }
    double N = kA / std::sqrt(1.0 - kE2 * std::sin(phi) * std::sin(phi));
    lat = phi / kDeg;
    lon = lam / kDeg;
    alt = p / std::cos(phi) - N;
}

bool SynthWorld::poseAt(double t, SynthPose& out) const {
    if (states_.empty() || t < states_.front().t || t > states_.back().t) return false;
    size_t lo = 0, hi = states_.size() - 1;
    while (hi - lo > 1) {
        size_t mid = (lo + hi) / 2;
        if (states_[mid].t <= t)
            lo = mid;
        else
            hi = mid;
    }
    const SynthPose& a = states_[lo];
    const SynthPose& b = states_[hi];
    double w = (b.t > a.t) ? (t - a.t) / (b.t - a.t) : 0.0;
    out.t = t;
    out.p = (1.0 - w) * a.p + w * b.p;
    out.v = (1.0 - w) * a.v + w * b.v;
    out.q = a.q.slerp(w, b.q).normalized();
    out.gyroB = (1.0 - w) * a.gyroB + w * b.gyroB;
    out.accelB = (1.0 - w) * a.accelB + w * b.accelB;
    out.magB = (1.0 - w) * a.magB + w * b.magB;
    return true;
}

double SynthWorld::yawTrue(double t) const {
    return yawFunc(t) / kDeg;
}

Vec3 SynthWorld::pTrue(double t) const {
    SynthPose p;
    if (poseAt(t, p)) return p.p;
    return Vec3::Zero();
}

}  // namespace gvio
