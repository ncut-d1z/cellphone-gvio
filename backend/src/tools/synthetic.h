#pragma once

#include "gvio/types.h"

#include <random>
#include <vector>

namespace gvio {

// 合成的物理上自洽的采集数据(IMU/磁力计/GPS/图像), 用于桌面端验证滤波器。
// 约定: q_GI 满足 R_IG = R_z(-yaw)·R_y(pitch)·R_x(roll) (quatYaw(q⁻¹) = 航向)。
struct SynthPose {
    double t = 0;
    Vec3 p = Vec3::Zero();      // IMU 在 ENU 中的位置
    Vec3 v = Vec3::Zero();
    Quat q = Quat::Identity();  // q_GI
    Vec3 gyroB = Vec3::Zero();
    Vec3 accelB = Vec3::Zero();
    Vec3 magB = Vec3::Zero();
};

struct SynthImuSample {
    int64_t tNs = 0;
    float values[6] = {};   // ax,ay,az,gx,gy,gz (含偏置+噪声)
    float biases[6] = {};   // 模拟 UNCALIBRATED 报告的偏置
};

struct SynthMagSample {
    int64_t tNs = 0;
    float values[3] = {};
    float biases[3] = {};
};

struct SynthGpsFix {
    int64_t tNs = 0;
    double lat = 0, lon = 0, alt = 0;
    float accH = 0;
};

struct SynthImgFrame {
    int64_t tNs = 0;
    int width = 0, height = 0;
    std::vector<uint8_t> y;
};

class SynthWorld {
public:
    SynthWorld(int width, int height, const CameraParams& cam)
        : W_(width), H_(height), cam_(cam) {}

    void generate(double dtImu, double dtImg, double dtMag, double dtGps,
                  double duration, double originLat, double originLon, double originAlt);

    const std::vector<SynthImuSample>& imu() const { return imuSamples_; }
    const std::vector<SynthMagSample>& mag() const { return magSamples_; }
    const std::vector<SynthGpsFix>& gps() const { return gpsSamples_; }
    const std::vector<SynthImgFrame>& imgs() const { return imgFrames_; }

    double yawTrue(double t) const;     // 度
    Vec3 pTrue(double t) const;
    void enuToWgs84(const Vec3& enu, double& lat, double& lon, double& alt) const;
    bool poseAt(double t, SynthPose& out) const;

private:
    void renderAt(const SynthPose& pose, std::vector<uint8_t>& gray) const;

    int W_ = 0, H_ = 0;
    CameraParams cam_;
    std::vector<SynthPose> states_;   // dtImu 间隔的真值轨迹
    std::vector<SynthImuSample> imuSamples_;
    std::vector<SynthMagSample> magSamples_;
    std::vector<SynthGpsFix> gpsSamples_;
    std::vector<SynthImgFrame> imgFrames_;
    std::vector<Vec3> worldPts_;

    double originLat_ = 0, originLon_ = 0, originAlt_ = 0;
    Eigen::Matrix3d rotEnuToEcef_ = Eigen::Matrix3d::Identity();
    Vec3 originEcef_ = Vec3::Zero();

    mutable std::mt19937 rng_ = std::mt19937(42);
};

}  // namespace gvio
