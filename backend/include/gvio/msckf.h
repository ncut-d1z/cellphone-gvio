#pragma once

#include "gvio/types.h"
#include <map>
#include <unordered_map>

namespace gvio {

struct FeatureTrack {
    uint32_t id = 0;
    Vec2 uv = Vec2::Zero();                     // 最新帧坐标
    std::vector<std::pair<int, Vec2>> obs;      // (窗口相机索引, 观测)
    bool lost = false;
};

// MSCKF 滤波核心: 状态 = IMU(15) + N×相机位姿(6N)
// 误差态约定(右乘): q = q̂ ⊗ [1, δθ/2]，δθ 位于该刚体自身坐标系。
// 全局系 ENU；四元数 Hamilton: v_I = q_GI ⊗ v_G ⊗ q_GI*。
class MsckfFilter {
public:
    explicit MsckfFilter(const FilterConfig& cfg);

    void initialize(const Vec3& p0, const Quat& q0, int64_t t0);
    bool initialized() const { return initialized_; }

    // IMU 传播(按样本逐段积分)
    void feedImu(const ImuSample& s);

    // 磁力计 yaw 量测更新(已过硬磁校准与倾角补偿)
    void feedMag(const MagSample& s);

    // GPS 位置量测更新(ENU 坐标, 由外部换算)
    void feedGpsPosition(const GpsFix& fix, const Vec3& enu);

    // 图像: 更新窗口、增广相机状态、处理特征观测
    // matched: 上一帧跟踪成功并在本帧匹配上的 (trackId -> uv)
    // fresh: 本帧新检测的角点 (由外部分配 id)
    void feedImage(int64_t t, const std::vector<std::pair<uint32_t, Vec2>>& matched,
                   const std::vector<std::pair<uint32_t, Vec2>>& fresh);

    // 观测快照(供三角化/可视化)
    struct CamPose {
        Quat q;
        Vec3 p;
        int64_t t = 0;
    };
    size_t cameraCount() const { return cams_.size(); }
    CamPose cameraPose(size_t idx) const;

    Quat imuQuat() const { return imu_.q; }
    Vec3 imuPos() const { return imu_.p; }
    Vec3 imuVel() const { return imu_.v; }
    Vec3 imuBiasG() const { return imu_.bg; }
    Vec3 imuBiasA() const { return imu_.ba; }
    int64_t imuTime() const { return imu_.t; }
    int64_t lastImageTime() const { return lastImageT_; }

    const FilterConfig& cfg() const { return cfg_; }
    size_t trackCount() const { return tracks_.size(); }
    uint32_t nextTrackId() const { return nextTrackId_; }
    uint32_t allocateTrackId() { return nextTrackId_++; }

    // 最近成功三角化的特征点(用于可视化/调试, ENU)
    const std::vector<Vec3>& recentPoints() const { return recentPts_; }

    void reset();
    bool lastUpdateGated = false;
    void nanGuard(const char* tag);

private:
    struct ImuState {
        Quat q = Quat::Identity();   // q_GI: global -> IMU
        Vec3 p = Vec3::Zero();       // ENU
        Vec3 v = Vec3::Zero();
        Vec3 bg = Vec3::Zero();
        Vec3 ba = Vec3::Zero();
        int64_t t = 0;
    };

    FilterConfig cfg_;
    ImuState imu_;
    std::vector<CamPose> cams_;
    Eigen::MatrixXd P_;
    bool initialized_ = false;
    bool imuStateBad_ = false;
    bool initDbg_ = false;
    int dtDbg_ = 0;
    int64_t lastImageT_ = -1;

    std::unordered_map<uint32_t, FeatureTrack> tracks_;
    uint32_t nextTrackId_ = 1;
    std::vector<uint32_t> finalized_;
    std::vector<Vec3> recentPts_;

    // 统计
    double imuRate_ = 0, imgRate_ = 0, gpsRate_ = 0;
    int64_t lastImuT_ = 0, lastImgT_ = 0, lastGpsT_ = 0;
    uint64_t imgCount_ = 0, updateCount_ = 0, gpsCount_ = 0, magCount_ = 0;
    int64_t lastUpdateDurationUs_ = 0;

    // 内部
    void propagateTo(int64_t t);
    void propagateOne(const ImuSample& s);
    void augment();
    void prune();
    void attachObs(const std::vector<std::pair<uint32_t, Vec2>>& matched,
                   const std::vector<std::pair<uint32_t, Vec2>>& fresh);
    void finalizeLost();
    void batchUpdate();
    bool triangulate(const FeatureTrack& ft, Vec3& fG) const;
    void ekfUpdate(const VecXd& r, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R);

public:
    // 统计访问(供状态快照)
    double imuRate() const { return imuRate_; }
    double imgRate() const { return imgRate_; }
    double gpsRate() const { return gpsRate_; }
    uint64_t imgCount() const { return imgCount_; }
    uint64_t updateCount() const { return updateCount_; }
    uint64_t gpsCount() const { return gpsCount_; }
    uint64_t magCount() const { return magCount_; }
    int64_t lastUpdateDurationUs() const { return lastUpdateDurationUs_; }
};

}  // namespace gvio
