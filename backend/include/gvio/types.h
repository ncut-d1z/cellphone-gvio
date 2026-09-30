#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
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
inline constexpr double kPi = 3.14159265358979323846;
inline const Vec3 GRAVITY(0., 0., -9.80665); // ENU
// Hamilton right increment. This is NOT physical integration of a G->I attitude:
// that integration is Exp(-omega_I * dt) * q_GI (see MsckfFilter).
inline Quat quatIntegrate(const Quat& q, const Vec3& omega, double dt) {
    Vec3 h = omega * (0.5 * dt);
    double a2 = h.squaredNorm();
    double a = std::sqrt(a2);
    double sinc = a2 < 1e-12 ? 1. - a2 / 6. + a2 * a2 / 120. : std::sin(a) / a;
    return (q * Quat(std::cos(a), sinc*h.x(), sinc*h.y(), sinc*h.z())).normalized();
}
inline Vec3 quatRotate(const Quat& q, const Vec3& v) { return q*v; }
inline Vec3 quatInvRotate(const Quat& q, const Vec3& v) { return q.conjugate()*v; }
inline Mat3 skewM(const Vec3& v) {
    Mat3 m; m << 0.,-v.z(),v.y(), v.z(),0.,-v.x(), -v.y(),v.x(),0.; return m;
}
inline double wrapAngle(double a) {
    if (!std::isfinite(a)) return a;
    double r = std::remainder(a, 2.*kPi);
    return r <= -kPi ? r + 2.*kPi : r;
}
inline double quatYaw(const Quat& q) {
    return std::atan2(2.*(q.w()*q.z()+q.x()*q.y()), 1.-2.*(q.y()*q.y()+q.z()*q.z()));
}
struct ImuSample {
    int64_t t = 0; // same monotonic nanosecond domain for EVERY sensor
    Vec3 gyro = Vec3::Zero(); // rad/s, I frame
    Vec3 accel = Vec3::Zero(); // specific force, m/s^2, I frame
};
struct MagSample { int64_t t = 0; Vec3 value = Vec3::Zero(), bias = Vec3::Zero(); };
struct GpsFix {
    int64_t t = 0;
    double lat = 0, lon = 0, alt = 0, sigma = 10., speed = 0, bearing = 0;
};
struct GnssSats { int64_t t = 0; std::vector<int> constellations; std::vector<float> cn0; };
struct CameraParams {
    int width = 1920, height = 1080;
    double fx = 1100., fy = 1100., cx = 960., cy = 540.;
    double k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;
    bool hasDistortion = false;
    Quat imuToCamQ = Quat::Identity(); // v_C = R_IC v_I
    Vec3 imuToCamP = Vec3::Zero(); // camera origin in I
    const Quat& qIC() const { return imuToCamQ; }
    const Vec3& pIC() const { return imuToCamP; }
    Vec2 project(const Vec3& p) const {
        double x=p.x()/p.z(), y=p.y()/p.z(), r=x*x+y*y;
        double d=1.+k1*r+k2*r*r+k3*r*r*r;
        return {fx*(x*d+2.*p1*x*y+p2*(r+2.*x*x))+cx,
                fy*(y*d+p1*(r+2.*y*y)+2.*p2*x*y)+cy};
    }
};
struct FilterConfig {
    CameraParams cam;
    int cameraWindowSize = 20;
    double featureNoisePx = 1.5;
    int maxFeatures = 300;
    double minParallaxDeg = 1.;
    int minTriObs = 3;
    double sigmaG = 3e-4, sigmaA = 2e-2, sigmaBg = 1e-6, sigmaBa = 1e-4;
    bool magEnabled = true;
    double magDeclinationDeg = 0, magInclinationDeg = 0, magFieldUT = 50., magYawNoiseRad = .087;
    bool gpsEnabled = true;
    int gpsMinSats = 4;
    bool serverEnabled = false; // headless algorithms never require TLS credentials
    std::string serverHost = "127.0.0.1";
    int serverPort = 8443;
    std::string certPath, keyPath, webRoot = "frontend";
    int64_t reorderWindowNs = 50000000; // bounded lateness; no retrospective EKF replay
    double initPosSigma = 5., initYawSigmaDeg = 10., initVelSigma = 1., initBiasGSigma = .01, initBiasASigma = .1;
    int fastThreshold = 25, maxCellsX = 8, maxCellsY = 6, maxPerCell = 30;
};
} // namespace gvio
