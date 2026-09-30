#pragma once
#include "gvio/types.h"
#include <map>

namespace gvio {
struct FeatureTrack {
    uint32_t id = 0;
    Vec2 uv = Vec2::Zero();
    std::vector<std::pair<int, Vec2>> obs;
    bool lost = false;
};
struct StateIndex {
    static constexpr int Theta=0, Position=3, Velocity=6, GyroBias=9, AccelBias=12, ImuSize=15, CameraSize=6;
};
using Mat15 = Eigen::Matrix<double,15,15>;
enum class ImageResult { Accepted, NotInitialized, DuplicateOrStale, NotAtImuTime };
// R_GI maps global -> IMU. Right error: R_true = R_hat Exp(delta_theta_G).
// The right perturbation is in GLOBAL coordinates, not body coordinates.
// State order: theta_G, p_G, v_G, bg_I, ba_I, then (theta_G,p_G) per clone.
class MsckfFilter {
public:
    explicit MsckfFilter(const FilterConfig& cfg);
    void initialize(const Vec3& p0,const Quat& q0,int64_t t0);
    bool initialized() const { return initialized_; }
    void feedImu(const ImuSample& s);
    void feedMag(const MagSample& s);
    void feedGpsPosition(const GpsFix& fix,const Vec3& enu);
    ImageResult feedImage(int64_t t,const std::vector<std::pair<uint32_t,Vec2>>& matched,
                          const std::vector<std::pair<uint32_t,Vec2>>& fresh);
    struct CamPose { Quat q; Vec3 p; int64_t t=0; };
    size_t cameraCount() const { return cams_.size(); }
    CamPose cameraPose(size_t idx) const { return cams_.at(idx); }
    Quat imuQuat() const { return imu_.q; }
    Vec3 imuPos() const { return imu_.p; }
    Vec3 imuVel() const { return imu_.v; }
    Vec3 imuBiasG() const { return imu_.bg; }
    Vec3 imuBiasA() const { return imu_.ba; }
    int64_t imuTime() const { return imu_.t; }
    int64_t lastImageTime() const { return lastImageT_; }
    const FilterConfig& cfg() const { return cfg_; }
    const MatXd& covariance() const { return P_; }
    size_t trackCount() const { return tracks_.size(); }
    uint32_t nextTrackId() const { return nextTrackId_; }
    uint32_t allocateTrackId() { return nextTrackId_++; }
    const std::vector<Vec3>& recentPoints() const { return recentPts_; }
    void reset();
    bool lastUpdateGated=false;
    void nanGuard(const char* tag);
    double imuRate() const { return imuRate_; }
    double imgRate() const { return imgRate_; }
    double gpsRate() const { return gpsRate_; }
    uint64_t imgCount() const { return imgCount_; }
    uint64_t updateCount() const { return updateCount_; }
    uint64_t gpsCount() const { return gpsCount_; }
    uint64_t magCount() const { return magCount_; }
    uint64_t visualUpdateCount() const { return visualUpdateCount_; }
    int64_t lastUpdateDurationUs() const { return lastUpdateDurationUs_; }
    // Exposed pure mathematical helpers for finite-difference regression tests.
    static Mat15 dynamics(const Quat& q,const Vec3& accel);
    static Eigen::Matrix<double,6,15> augmentationJacobian(const Quat& q,const Vec3& pIC);
private:
    struct ImuState {
        Quat q=Quat::Identity(); Vec3 p=Vec3::Zero(),v=Vec3::Zero(),bg=Vec3::Zero(),ba=Vec3::Zero(); int64_t t=0;
    };
    FilterConfig cfg_;
    ImuState imu_;
    std::vector<CamPose> cams_;
    MatXd P_;
    bool initialized_=false, havePreviousImu_=false;
    ImuSample previousImu_;
    int64_t lastImageT_=-1;
    std::map<uint32_t,FeatureTrack> tracks_; // deterministic feature order
    uint32_t nextTrackId_=1;
    std::vector<Vec3> recentPts_;
    double imuRate_=0,imgRate_=0,gpsRate_=0;
    int64_t lastImuT_=0,lastImgT_=0,lastGpsT_=0,lastUpdateDurationUs_=0;
    uint64_t imgCount_=0,updateCount_=0,gpsCount_=0,magCount_=0,visualUpdateCount_=0;
    void propagateOne(const ImuSample& s);
    void augment();
    void prune();
    void attachObs(const std::vector<std::pair<uint32_t,Vec2>>& matched,const std::vector<std::pair<uint32_t,Vec2>>& fresh);
    void finalizeLost();
    void consumeTracks(const std::vector<uint32_t>& ids);
    bool triangulate(const FeatureTrack& ft,Vec3& fG) const;
    bool ekfUpdate(const VecXd& r,const MatXd& H,const MatXd& R);
};
} // namespace gvio
