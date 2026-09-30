#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <string>
#include <vector>

namespace gvio {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;
using Quat = Eigen::Quaterniond;
using MatXd = Eigen::MatrixXd;
using VecXd = Eigen::VectorXd;

// 全局参考系: ENU (x=东, y=北, z=上)。重力在全局系。
inline const Vec3 GRAVITY(0.0, 0.0, -9.80665);

// 四元数积分: q' = q ⊗ exp([0, ω*dt/2])  (Hamilton, v_I = R(q) v_G)
inline Quat quatIntegrate(const Quat& q, const Vec3& omega, double dt) {
    double half = 0.5 * dt;
    Vec3 axis = omega * half;
    double theta2 = axis.squaredNorm();
    Quat dq;
    if (theta2 < 1e-12) {
        dq = Quat(1.0, 0.0, 0.0, 0.0);
    } else {
        double theta = std::sqrt(theta2);
        double s = std::sin(theta) / theta;
        dq = Quat(std::cos(theta), s * axis.x(), s * axis.y(), s * axis.z());
    }
    return (q * dq).normalized();
}

inline Vec3 quatRotate(const Quat& q, const Vec3& v) { return q * v; }        // v_I = R v_G
inline Vec3 quatInvRotate(const Quat& q, const Vec3& v) { return q.conjugate() * v; } // v_G = Rᵀ v_I

// 反对称矩阵
inline Mat3 skewM(const Vec3& v) {
    Mat3 m;
    m << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
         -v.y(), v.x(), 0.0;
    return m;
}

// 角度归一化到 (-π, π]
inline double wrapAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

// 从四元数提取 ENU yaw (Hamilton)
inline double quatYaw(const Quat& qIG) {
    return std::atan2(2.0 * (qIG.w() * qIG.z() + qIG.x() * qIG.y()),
                      1.0 - 2.0 * (qIG.y() * qIG.y() + qIG.z() * qIG.z()));
}

struct ImuSample {
    int64_t t = 0;          // CLOCK_MONOTONIC 纳秒
    Vec3 gyro = Vec3::Zero();   // rad/s
    Vec3 accel = Vec3::Zero();  // m/s²
};

struct MagSample {
    int64_t t = 0;
    Vec3 value = Vec3::Zero();      // 原始读数 µT
    Vec3 bias = Vec3::Zero();       // 硬磁零偏估计(来自 UNCALIBRATED)
};

struct GpsFix {
    int64_t t = 0;                 // elapsedRealtimeNanos
    double lat = 0, lon = 0, alt = 0;
    double sigma = 10.0;           // 水平精度(米)
    double speed = 0;
    double bearing = 0;
};

struct GnssSats {
    int64_t t = 0;
    std::vector<int> constellations;
    std::vector<float> cn0;
};

// 相机参数(针孔 + 前向畸变, 未建模畸变时默认零)
struct CameraParams {
    int width = 1920, height = 1080;
    double fx = 1100.0, fy = 1100.0, cx = 960.0, cy = 540.0;
    double k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;
    bool hasDistortion = false;
    Quat imuToCamQ = Quat::Identity();   // IMU -> Camera 旋转 (v_C = R_IC v_I)
    Vec3 imuToCamP = Vec3::Zero();       // 相机在 IMU 系中的位置

    const Quat& qIC() const { return imuToCamQ; }
    const Vec3& pIC() const { return imuToCamP; }

    Vec2 project(const Vec3& pc) const {
        double x = pc.x() / pc.z(), y = pc.y() / pc.z();
        double r2 = x * x + y * y;
        double radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
        double tx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        double ty = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        return Vec2(fx * (x * radial + tx) + cx, fy * (y * radial + ty) + cy);
    }
};

struct FilterConfig {
    // 相机
    CameraParams cam;
    int cameraWindowSize = 20;
    double featureNoisePx = 1.5;
    int maxFeatures = 300;
    double minParallaxDeg = 1.0;
    int minTriObs = 3;
    // IMU 噪声(Allan/实测标定占位)
    double sigmaG = 3.0e-4;      // rad/s/√Hz
    double sigmaA = 2.0e-2;      // m/s²/√Hz
    double sigmaBg = 1.0e-6;     // rad/s²/√Hz
    double sigmaBa = 1.0e-4;     // m/s³/√Hz
    // 磁力计
    bool magEnabled = true;
    double magDeclinationDeg = 0.0;
    double magInclinationDeg = 0.0;
    double magFieldUT = 50.0;
    double magYawNoiseRad = 0.087;
    // GPS
    bool gpsEnabled = true;
    int gpsMinSats = 4;
    // 服务器
    std::string serverHost = "127.0.0.1";
    int serverPort = 8443;
    std::string certPath;
    std::string keyPath;
    std::string webRoot = "frontend";
    // 内参初始协方差
    double initPosSigma = 5.0;
    double initYawSigmaDeg = 10.0;
    double initVelSigma = 1.0;
    double initBiasGSigma = 1e-2;
    double initBiasASigma = 0.1;
    // FAST/KLT
    int fastThreshold = 25;
    int maxCellsX = 8, maxCellsY = 6;
    int maxPerCell = 30;
};

}  // namespace gvio
