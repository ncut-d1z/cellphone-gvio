#include "gvio/msckf.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>

namespace gvio {

void MsckfFilter::nanGuard(const char* tag) {
    if (!imuStateBad_ &&
        (!P_.allFinite() || !imu_.q.coeffs().allFinite() || !imu_.p.allFinite() ||
         !imu_.v.allFinite())) {
        imuStateBad_ = true;
        std::fprintf(stderr, "[NaN] first detected at %s\n", tag);
        std::fprintf(stderr, "  imu t=%lld q=(%g,%g,%g,%g) p=(%g,%g,%g) v=(%g,%g,%g) bg=(%g,%g,%g) ba=(%g,%g,%g)\n",
                     static_cast<long long>(imu_.t), imu_.q.w(), imu_.q.x(), imu_.q.y(),
                     imu_.q.z(), imu_.p.x(), imu_.p.y(), imu_.p.z(), imu_.v.x(), imu_.v.y(),
                     imu_.v.z(), imu_.bg.x(), imu_.bg.y(), imu_.bg.z(), imu_.ba.x(), imu_.ba.y(),
                     imu_.ba.z());
        std::fprintf(stderr, "  P_ %lldx%lld diag[0..16]:", static_cast<long long>(P_.rows()),
                     static_cast<long long>(P_.cols()));
        for (long long i = 0; i < P_.rows() && i < 17; ++i)
            std::fprintf(stderr, " %.3e", P_(i, i));
        std::fprintf(stderr, "\n");
    }
}

MsckfFilter::MsckfFilter(const FilterConfig& cfg) : cfg_(cfg) {
    reset();
}

void MsckfFilter::reset() {
    imu_ = ImuState();
    cams_.clear();
    tracks_.clear();
    finalized_.clear();
    recentPts_.clear();
    initialized_ = false;
    lastImageT_ = -1;
    imuRate_ = imgRate_ = gpsRate_ = 0;
    lastImuT_ = lastImgT_ = lastGpsT_ = 0;
    imgCount_ = updateCount_ = gpsCount_ = magCount_ = 0;
    lastUpdateDurationUs_ = 0;
    nextTrackId_ = 1;
}

void MsckfFilter::initialize(const Vec3& p0, const Quat& q0, int64_t t0) {
    imu_.q = q0.normalized();
    imu_.p = p0;
    imu_.v = Vec3::Zero();
    imu_.bg = Vec3::Zero();
    imu_.ba = Vec3::Zero();
    imu_.t = t0;
    cams_.clear();
    tracks_.clear();
    finalized_.clear();

    double sigTheta = cfg_.initYawSigmaDeg * M_PI / 180.0;
    double sigPos = cfg_.initPosSigma;
    double sigVel = cfg_.initVelSigma;
    double sigBg = cfg_.initBiasGSigma;
    double sigBa = cfg_.initBiasASigma;
    VecXd diag(15);
    diag << sigTheta * sigTheta, sigTheta * sigTheta, sigTheta * sigTheta,
            sigPos * sigPos, sigPos * sigPos, sigPos * sigPos,
            sigVel * sigVel, sigVel * sigVel, sigVel * sigVel,
            sigBg * sigBg, sigBg * sigBg, sigBg * sigBg,
            sigBa * sigBa, sigBa * sigBa, sigBa * sigBa;
    P_ = diag.asDiagonal();
    initialized_ = true;
}

// ---------------------------------------------------------------------------
// IMU 传播
// ---------------------------------------------------------------------------
void MsckfFilter::feedImu(const ImuSample& s) {
    if (!initialized_) {
        imu_.t = s.t;
        return;
    }
    if (s.t <= imu_.t) return;
    if (lastImuT_ > 0 && s.t > lastImuT_) {
        imuRate_ = 1e9 / (s.t - lastImuT_);
    }
    lastImuT_ = s.t;
    propagateOne(s);
}

void MsckfFilter::propagateOne(const ImuSample& s) {
    double dt = (s.t - imu_.t) * 1e-9;
    if (dt <= 0) return;

    Vec3 omega = s.gyro - imu_.bg;
    Vec3 accel = s.accel - imu_.ba;

    if (!imu_.q.coeffs().allFinite() || !imu_.p.allFinite() || !imu_.v.allFinite() ||
        !P_.allFinite() || !omega.allFinite() || !accel.allFinite() || std::isnan(dt) ||
        dt > 1.0) {
        std::fprintf(stderr,
                     "[dbg] propagateOne bad input t=%lld dt=%g q=(%g,%g,%g,%g) p=(%g,%g,%g) "
                     "v=(%g,%g,%g) bg=(%g,%g,%g) ba=(%g,%g,%g) omega=(%g,%g,%g) "
                     "accel=(%g,%g,%g) P00=%g\n",
                     static_cast<long long>(s.t), dt, imu_.q.w(), imu_.q.x(), imu_.q.y(),
                     imu_.q.z(), imu_.p.x(), imu_.p.y(), imu_.p.z(), imu_.v.x(), imu_.v.y(),
                     imu_.v.z(), imu_.bg.x(), imu_.bg.y(), imu_.bg.z(), imu_.ba.x(), imu_.ba.y(),
                     imu_.ba.z(), omega.x(), omega.y(), omega.z(), accel.x(), accel.y(),
                     accel.z(), P_.rows() > 0 ? P_(0, 0) : -1);
    }

    // 中点法积分
    Quat q_mid = quatIntegrate(imu_.q, omega, 0.5 * dt);
    Vec3 aG_mid = quatInvRotate(q_mid, accel);
    Vec3 v_mid = imu_.v + (aG_mid + GRAVITY) * (0.5 * dt);
    Vec3 aG = quatInvRotate(imu_.q, accel);

    Quat q_next = quatIntegrate(imu_.q, omega, dt);
    Vec3 v_next = imu_.v + (aG + GRAVITY) * dt;
    Vec3 p_next = imu_.p + v_mid * dt;

    // 连续误差态(右乘, δθ 在 IMU 系):
    //   δθ̇ = −[ω]× δθ − δbg − n_g
    //   δṗ = δv
    //   δv̇ = [Rᵀa]× δθ − Rᵀ δba − Rᵀ n_a
    Mat3 R = imu_.q.toRotationMatrix();   // global -> IMU
    Mat3 R_a_x = skewM(quatInvRotate(imu_.q, accel));
    Mat3 F15 = Mat3::Zero();
    F15.block<3, 3>(0, 0) = -skewM(omega);
    F15.block<3, 3>(0, 9) = -Mat3::Identity();
    F15.block<3, 3>(1, 2) = Mat3::Identity();
    F15.block<3, 3>(2, 0) = R_a_x;
    F15.block<3, 3>(2, 12) = -R.transpose();

    MatXd Phi = MatXd::Identity(15, 15) + F15 * dt;
    MatXd Qd = MatXd::Zero(15, 15);
    {
        MatXd G = MatXd::Zero(15, 6);
        G.block<3, 3>(0, 0) = -Mat3::Identity();
        G.block<3, 3>(2, 3) = -R.transpose();
        MatXd Qc = MatXd::Zero(6, 6);
        Qc.block<3, 3>(0, 0) = Mat3::Identity() * (cfg_.sigmaG * cfg_.sigmaG);
        Qc.block<3, 3>(3, 3) = Mat3::Identity() * (cfg_.sigmaA * cfg_.sigmaA);
        Qd = G * Qc * G.transpose() * dt;
        Qd.block<3, 3>(9, 9) = Mat3::Identity() * (cfg_.sigmaBg * cfg_.sigmaBg) * dt;
        Qd.block<3, 3>(12, 12) = Mat3::Identity() * (cfg_.sigmaBa * cfg_.sigmaBa) * dt;
    }

    imu_.q = q_next;
    imu_.p = p_next;
    imu_.v = v_next;
    imu_.t = s.t;

    if (!initDbg_) {
        initDbg_ = true;
        std::fprintf(stderr, "[dbg] FIRST PROP t=%lld dt=%g Ppp=(%g,%g,%g) Pvv=(%g,%g,%g) "
                             "Pthetad=(%g,%g,%g) pmax=%g omega=(%g,%g,%g)\n",
                     static_cast<long long>(s.t), dt, P_(3, 3), P_(4, 4), P_(5, 5), P_(6, 6),
                     P_(7, 7), P_(8, 8), P_(0, 0), P_(1, 1), P_(2, 2), P_.cwiseAbs().maxCoeff(),
                     omega.x(), omega.y(), omega.z());
    }
    if (dtDbg_ < 8 && (s.t - imu_.t) < 0.01) {
        std::fprintf(stderr,
                     "[dbg] prop#%d t=%lld Ppp=(%g,%g,%g) Pvv=(%g,%g,%g) Ptt=(%g,%g,%g) "
                     "Ptp=(%g,%g,%g) Pvp=(%g,%g,%g) pmax=%g\n",
                     dtDbg_++, static_cast<long long>(s.t), P_(3, 3), P_(4, 4), P_(5, 5), P_(6, 6),
                     P_(7, 7), P_(8, 8), P_(0, 0), P_(1, 1), P_(2, 2), P_(0, 3), P_(1, 4), P_(2, 5),
                     P_(6, 3), P_(7, 4), P_(8, 5), P_.cwiseAbs().maxCoeff());
        if (dtDbg_ == 2) {
            std::fprintf(stderr, "[dbg] F15:\n");
            for (int i = 0; i < 15; ++i)
                std::fprintf(stderr, "  %13.6e %13.6e %13.6e %13.6e %13.6e %13.6e %13.6e %13.6e "
                                     "%13.6e %13.6e %13.6e %13.6e %13.6e %13.6e %13.6e\n",
                             F15(i, 0), F15(i, 1), F15(i, 2), F15(i, 3), F15(i, 4), F15(i, 5),
                             F15(i, 6), F15(i, 7), F15(i, 8), F15(i, 9), F15(i, 10), F15(i, 11),
                             F15(i, 12), F15(i, 13), F15(i, 14));
            std::fprintf(stderr, "[dbg] Phi(0..3,:) dt=%g omega=(%g,%g,%g)\n", dt, omega.x(),
                         omega.y(), omega.z());
            for (int i = 0; i < 4; ++i)
                std::fprintf(stderr, "  %13.6e %13.6e %13.6e %13.6e %13.6e %13.6e %13.6e %13.6e "
                                     "%13.6e %13.6e %13.6e %13.6e %13.6e %13.6e %13.6e\n",
                             Phi(i, 0), Phi(i, 1), Phi(i, 2), Phi(i, 3), Phi(i, 4), Phi(i, 5),
                             Phi(i, 6), Phi(i, 7), Phi(i, 8), Phi(i, 9), Phi(i, 10), Phi(i, 11),
                             Phi(i, 12), Phi(i, 13), Phi(i, 14));
        }
    }

    // 协方差传播(相机块仅经交叉项)
    int n = 15 + 6 * static_cast<int>(cams_.size());
    MatXd P15 = Phi * P_.topLeftCorner(15, 15) * Phi.transpose() + Qd;
    P_.topLeftCorner(15, 15) = P15;
    if (n > 15) {
        MatXd Pc = Phi * P_.block(0, 15, 15, n - 15);
        P_.block(0, 15, 15, n - 15) = Pc;
        P_.block(15, 0, n - 15, 15) = Pc.transpose();
    }
    nanGuard("propagateOne");
}

// ---------------------------------------------------------------------------
// 增广: 图像时刻把 IMU 位姿(含外参)加入相机窗口
// ---------------------------------------------------------------------------
void MsckfFilter::augment() {
    const Quat qIC = cfg_.cam.qIC();
    const Vec3 pIC = cfg_.cam.pIC();
    Quat qGC = (imu_.q * qIC).normalized();
    Vec3 pGC = imu_.p + quatInvRotate(imu_.q, pIC);

    int nOld = 15 + 6 * static_cast<int>(cams_.size());
    //   δθ_C = R_IC δθ
    //   δp_C = [(R_IG p_IC)×] δθ + δp
    Mat3 R_IG = imu_.q.toRotationMatrix().transpose();
    Mat3 R_IC = qIC.toRotationMatrix();
    Mat3 dP_dTh = skewM(R_IG * pIC);
    MatXd Jaug = MatXd::Zero(6, nOld);
    Jaug.block<3, 3>(0, 0) = R_IC;              // δθ_C / δθ
    Jaug.block<3, 3>(3, 0) = dP_dTh;            // δp_C / δθ
    Jaug.block<3, 3>(3, 1) = Mat3::Identity();  // δp_C / δp

    int nNew = nOld + 6;
    MatXd Pnew = MatXd::Zero(nNew, nNew);
    Pnew.topLeftCorner(nOld, nOld) = P_;
    MatXd JaugP = Jaug * P_;                          // 6 x nOld
    Pnew.block(nOld, 0, 6, nOld) = JaugP;
    Pnew.block(0, nOld, nOld, 6) = JaugP.transpose();
    Pnew.block(nOld, nOld, 6, 6) = JaugP * Jaug.transpose();

    cams_.push_back({qGC, pGC, imu_.t});
    P_ = Pnew;
    lastImageT_ = imu_.t;
    nanGuard("augment");
}

// ---------------------------------------------------------------------------
// 窗口裁剪
// ---------------------------------------------------------------------------
void MsckfFilter::prune() {
    while (static_cast<int>(cams_.size()) > cfg_.cameraWindowSize) {
        finalized_.clear();
        for (auto& kv : tracks_) {
            auto& ft = kv.second;
            if (!ft.obs.empty() && ft.obs.front().first == 0) {
                ft.lost = true;
                finalized_.push_back(ft.id);
            }
        }
        if (!finalized_.empty()) batchUpdate();

        cams_.erase(cams_.begin());
        for (auto& kv : tracks_) {
            auto& ft = kv.second;
            std::vector<std::pair<int, Vec2>> nv;
            nv.reserve(ft.obs.size());
            for (auto& ob : ft.obs) {
                if (ob.first > 0) nv.push_back({ob.first - 1, ob.second});
            }
            ft.obs = std::move(nv);
            if (ft.obs.empty()) ft.lost = true;
        }
        // 删除第 15..20 行列(相机 0)
        int nOld = P_.rows();
        int n = nOld - 6;
        MatXd Pnew = MatXd::Zero(n, n);
        Pnew.topLeftCorner(15, 15) = P_.topLeftCorner(15, 15);
        Pnew.block(15, 15, n - 15, n - 15) = P_.block(21, 21, n - 15, n - 15);
        Pnew.block(15, 0, n - 15, 15) = P_.block(21, 0, n - 15, 15);
        Pnew.block(0, 15, 15, n - 15) = P_.block(0, 21, 15, n - 15);
        P_ = Pnew;
    }
}

// ---------------------------------------------------------------------------
// 特征观测管理
// ---------------------------------------------------------------------------
void MsckfFilter::attachObs(const std::vector<std::pair<uint32_t, Vec2>>& matched,
                            const std::vector<std::pair<uint32_t, Vec2>>& fresh) {
    int camIdx = static_cast<int>(cams_.size()) - 1;
    for (auto& kv : tracks_) kv.second.lost = true;
    for (auto& m : matched) {
        auto it = tracks_.find(m.first);
        if (it != tracks_.end()) {
            it->second.uv = m.second;
            it->second.obs.push_back({camIdx, m.second});
            it->second.lost = false;
        }
    }
    for (auto& f : fresh) {
        if (tracks_.find(f.first) != tracks_.end()) continue;
        FeatureTrack ft;
        ft.id = f.first;
        ft.uv = f.second;
        ft.obs.push_back({camIdx, f.second});
        tracks_[ft.id] = ft;
    }
}

void MsckfFilter::finalizeLost() {
    finalized_.clear();
    for (auto& kv : tracks_) {
        if (kv.second.lost) finalized_.push_back(kv.first);
    }
    if (!finalized_.empty()) batchUpdate();
    // 清理: lost 特征已在 batchUpdate 中处理(或丢弃), 全部删除
    for (uint32_t id : finalized_) {
        tracks_.erase(id);
    }
}

// ---------------------------------------------------------------------------
// 三角化: DLT + SVD
// ---------------------------------------------------------------------------
bool MsckfFilter::triangulate(const FeatureTrack& ft, Vec3& fG) const {
    int K = static_cast<int>(ft.obs.size());
    if (K < 2) return false;
    Eigen::MatrixXd A(2 * K, 4);
    int row = 0;
    for (auto& ob : ft.obs) {
        int ci = ob.first;
        if (ci < 0 || ci >= static_cast<int>(cams_.size())) return false;
        const CamPose& cp = cams_[ci];
        Mat3 R = cp.q.toRotationMatrix();
        Eigen::Matrix<double, 3, 4> Pk;
        Pk.block<3, 3>(0, 0) = R;
        Pk.col(3) = -R * cp.p;
        double u = (ob.second.x() - cfg_.cam.cx) / cfg_.cam.fx;
        double v = (ob.second.y() - cfg_.cam.cy) / cfg_.cam.fy;
        A.row(row++) = u * Pk.row(2) - Pk.row(0);
        A.row(row++) = v * Pk.row(2) - Pk.row(1);
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    Vec4 hv = svd.matrixV().col(3);
    if (std::abs(hv.w()) < 1e-12) return false;
    fG = Vec3(hv.x() / hv.w(), hv.y() / hv.w(), hv.z() / hv.w());
    // 深度检查在各观测的相机系内完成(fC.z()); 全局系 z 检查无意义(特征可在地面)
    double sum = 0;
    for (auto& ob : ft.obs) {
        int ci = ob.first;
        const CamPose& cp = cams_[ci];
        Vec3 fC = cp.q * (fG - cp.p);
        if (fC.z() <= 0.05) return false;
        Vec2 pr = cfg_.cam.project(fC);
        sum += (pr - ob.second).squaredNorm();
    }
    double rms = std::sqrt(sum / K);
    return rms < 3.0;
}

// ---------------------------------------------------------------------------
// 批量空空间更新(特征作为隐变量被空空间消去, 更新仅涉及相机状态)
// ---------------------------------------------------------------------------
void MsckfFilter::batchUpdate() {
    int n = 15 + 6 * static_cast<int>(cams_.size());
    if (n <= 15 || finalized_.empty()) {
        finalized_.clear();
        return;
    }

    std::vector<VecXd> rs;
    std::vector<Eigen::MatrixXd> Hs;
    double sigmaPx = cfg_.featureNoisePx;

    for (uint32_t id : finalized_) {
        auto it = tracks_.find(id);
        if (it == tracks_.end()) continue;
        const FeatureTrack& ft = it->second;
        int K = static_cast<int>(ft.obs.size());
        if (K < cfg_.minTriObs) continue;
        // 观测过多时截尾(限制行数, 防止堆叠矩阵爆炸)
        const int kMaxObs = 32;
        std::vector<std::pair<int, Vec2>> obs =
            (K > kMaxObs) ? std::vector<std::pair<int, Vec2>>(ft.obs.end() - kMaxObs,
                                                              ft.obs.end())
                          : ft.obs;
        K = static_cast<int>(obs.size());
        if (K < cfg_.minTriObs) continue;

        Vec3 fG;
        if (!triangulate(ft, fG)) continue;
        recentPts_.push_back(fG);
        if (recentPts_.size() > 400) recentPts_.erase(recentPts_.begin(),
                                                      recentPts_.begin() + recentPts_.size() - 400);

        // 视差检查
        const CamPose& ca = cams_[ft.obs.front().first];
        const CamPose& cb = cams_[ft.obs.back().first];
        Vec3 ra = ca.q * (fG - ca.p);
        Vec3 rb = cb.q * (fG - cb.p);
        if (ra.z() <= 0.05 || rb.z() <= 0.05) continue;
        ra.normalize();
        rb.normalize();
        double cosA = std::max(-1.0, std::min(1.0, ra.dot(rb)));
        double parallax = std::acos(cosA) * 180.0 / M_PI;
        if (parallax < cfg_.minParallaxDeg) continue;

        Eigen::MatrixXd Hf(2 * K, 3);   // 对 f_G
        Eigen::MatrixXd Hx(2 * K, n);   // 对相机位姿误差
        VecXd r(2 * K);
        Hx.setZero();
        int row = 0;
        bool ok = true;
        for (auto& ob : obs) {
            int ci = ob.first;
            if (ci < 0 || ci >= static_cast<int>(cams_.size())) { ok = false; break; }
            const CamPose& cp = cams_[ci];
            Mat3 R = cp.q.toRotationMatrix();
            Vec3 fC = cp.q * (fG - cp.p);
            if (fC.z() <= 0.05) { ok = false; break; }
            Vec2 z = cfg_.cam.project(fC);
            r.segment<2>(row) = ob.second - z;

            Eigen::Matrix<double, 2, 3> Jp;
            double x = fC.x(), y = fC.y(), zc = fC.z();
            Jp << cfg_.cam.fx / zc, 0, -cfg_.cam.fx * x / (zc * zc),
                  0, cfg_.cam.fy / zc, -cfg_.cam.fy * y / (zc * zc);

            Hf.block<2, 3>(row, 0) = Jp * R;             // ∂z/∂f_G
            int col = 15 + 6 * ci;
            Hx.block<2, 3>(row, col) = -Jp * skewM(fC) * R;  // ∂z/∂δθ_C (右乘, 相机系)
            Hx.block<2, 3>(row, col + 3) = -Jp * R;      // ∂z/∂δp_C
            row += 2;
        }
        if (!ok) continue;

        // 空空间: H_f = Q [R; 0], N = Q[:, 3:]
        if (2 * K - 3 <= 0) continue;
        Eigen::HouseholderQR<Eigen::MatrixXd> qr(Hf);
        Eigen::MatrixXd Q = qr.householderQ();
        Eigen::MatrixXd N = Q.block(0, 3, 2 * K, 2 * K - 3);
        VecXd rn = N.transpose() * r;
        Eigen::MatrixXd Hn = N.transpose() * Hx;
        double Rn = sigmaPx * sigmaPx;

        // 逐特征马氏距离门限
        Eigen::MatrixXd Sf = Hn * P_ * Hn.transpose();
        Sf.diagonal().array() += Rn;
        double d = rn.dot(Sf.ldlt().solve(rn));
        if (d > 3.0 * (2 * K - 3)) continue;

        rs.push_back(rn);
        Hs.push_back(Hn);
    }
    finalized_.clear();

    if (rs.empty()) return;

    // 分批顺序更新: 避免单次堆叠矩阵过大导致 S 求解过慢
    const size_t kChunkRows = 400;
    size_t total = 0;
    for (auto& rv : rs) total += rv.size();
    if (total <= kChunkRows) {
        size_t m = 0;
        for (auto& rv : rs) m += rv.size();
        VecXd rAll(m);
        Eigen::MatrixXd HAll = Eigen::MatrixXd::Zero(m, n);
        size_t off = 0;
        for (size_t k = 0; k < rs.size(); ++k) {
            rAll.segment(off, rs[k].size()) = rs[k];
            HAll.block(off, 0, rs[k].size(), n) = Hs[k];
            off += rs[k].size();
        }
        Eigen::MatrixXd RAll = Eigen::MatrixXd::Identity(m, m) * (sigmaPx * sigmaPx);
        ekfUpdate(rAll, HAll, RAll);
    } else {
        size_t idx = 0;
        while (idx < rs.size()) {
            size_t m = 0, j = idx;
            while (j < rs.size() && m + rs[j].size() <= kChunkRows) {
                m += rs[j].size();
                ++j;
            }
            if (j == idx) {
                m = rs[idx].size();
                ++j;
            }
            VecXd rc(m);
            Eigen::MatrixXd Hc = Eigen::MatrixXd::Zero(m, n);
            size_t off = 0;
            for (size_t k = idx; k < j; ++k) {
                rc.segment(off, rs[k].size()) = rs[k];
                Hc.block(off, 0, rs[k].size(), n) = Hs[k];
                off += rs[k].size();
            }
            Eigen::MatrixXd Rc = Eigen::MatrixXd::Identity(m, m) * (sigmaPx * sigmaPx);
            ekfUpdate(rc, Hc, Rc);
            idx = j;
        }
    }
    nanGuard("batchUpdate");
}

void MsckfFilter::ekfUpdate(const VecXd& r, const Eigen::MatrixXd& H,
                            const Eigen::MatrixXd& R) {
    if (H.rows() == 0 || H.cols() != P_.rows()) return;

    auto t0 = std::chrono::steady_clock::now();
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() *
                        S.ldlt().solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));
    VecXd dx = K * r;
    {
        double pmax = P_.cwiseAbs().maxCoeff();
        double sMin = S.minCoeff();
        double rn2 = r.norm();
        if (pmax > 1e8 || !P_.allFinite())
            std::fprintf(stderr,
                         "[dbg] P LARGE before update: pmax=%g rows=%lld cols=%lld |r|=%g Smin=%g "
                         "Pdiag(3,4,5)=(%g,%g,%g)\n",
                         pmax, static_cast<long long>(H.rows()), static_cast<long long>(H.cols()),
                         rn2, sMin, P_(3, 3), P_(4, 4), P_(5, 5));
    }
    if (dx.cwiseAbs().maxCoeff() > 1e4) {
        std::fprintf(stderr,
                     "[dbg] BIG dx: max|dx|=%g n=%lld rows=%lld cols=%lld |r|=%g Pdiag=(%g,%g,%g) "
                     "v_before=(%g,%g,%g)\n",
                     dx.cwiseAbs().maxCoeff(), static_cast<long long>(P_.rows()),
                     static_cast<long long>(H.rows()), static_cast<long long>(H.cols()), r.norm(),
                     P_(3, 3), P_(4, 4), P_(5, 5), imu_.v.x(), imu_.v.y(), imu_.v.z());
    }
    // Joseph 形式: 保持半正定, 避免浮点减法产生负方差
    Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(P_.rows(), P_.cols()) - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
    P_.diagonal().array() = P_.diagonal().array().max(1e-12);
    P_ = 0.5 * (P_ + P_.transpose());

    imu_.q = (imu_.q * quatIntegrate(Quat::Identity(), dx.segment<3>(0), 1.0)).normalized();
    imu_.p += dx.segment<3>(3);
    imu_.v += dx.segment<3>(6);
    imu_.bg += dx.segment<3>(9);
    imu_.ba += dx.segment<3>(12);
    for (size_t k = 0; k < cams_.size(); ++k) {
        int col = 15 + 6 * static_cast<int>(k);
        cams_[k].q = (cams_[k].q *
                      quatIntegrate(Quat::Identity(), dx.segment<3>(col), 1.0)).normalized();
        cams_[k].p += dx.segment<3>(col + 3);
    }
    updateCount_++;
    lastUpdateDurationUs_ = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - t0).count();
    if (!P_.allFinite() || P_(3, 3) < 0 || P_(4, 4) < 0) {
        std::fprintf(stderr,
                     "[dbg] POST-UPDATE BAD P_ rows=%lld Pdiag3,4,5=(%g,%g,%g) "
                     "state p=(%g,%g,%g) v=(%g,%g,%g) |r|=%g\n",
                     static_cast<long long>(P_.rows()), P_(3, 3), P_(4, 4), P_(5, 5), imu_.p.x(),
                     imu_.p.y(), imu_.p.z(), imu_.v.x(), imu_.v.y(), imu_.v.z(), r.norm());
    }
    nanGuard("ekfUpdate");
}

// ---------------------------------------------------------------------------
// 磁力计 yaw 量测
// ---------------------------------------------------------------------------
void MsckfFilter::feedMag(const MagSample& s) {
    if (!initialized_ || !cfg_.magEnabled) return;
    // 体坐标系量测(已减硬磁零偏): psiM = atan2(mx, my) - decl
    Vec3 m_b = s.value - s.bias;
    double mH2 = m_b.x() * m_b.x() + m_b.y() * m_b.y();
    if (mH2 < 1e-8) return;

    double declRad = cfg_.magDeclinationDeg * M_PI / 180.0;
    double psiM = std::atan2(m_b.x(), m_b.y()) - declRad;
    double psiHat = quatYaw(imu_.q.conjugate());
    double res = wrapAngle(psiM - psiHat);

    // dψ/dm_b = [m_y, −m_x, 0]/mH² (体系)
    // m_b = R_IG m_G, 右乘误差: m_b' = m_b − R[m_G]× δθ
    // => dψ/dδθ = −(dψ/dm_b) R [m_G]×
    Mat3 R = imu_.q.toRotationMatrix();          // G -> I
    Vec3 mG = quatInvRotate(imu_.q, m_b);        // 线性化点: 全局场
    Eigen::Matrix<double, 1, 3> dPsiDmb;
    dPsiDmb << m_b.y() / mH2, -m_b.x() / mH2, 0.0;
    Eigen::Matrix<double, 1, 3> Htheta = -dPsiDmb * R * skewM(mG);
    Eigen::Matrix<double, 1, 15> Himu = Eigen::Matrix<double, 1, 15>::Zero();
    Himu.head<3>() = Htheta;

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, P_.rows());
    H.topLeftCorner(1, 15) = Himu;
    VecXd r(1);
    r << res;
    Eigen::Matrix<double, 1, 1> Rm;
    Rm << cfg_.magYawNoiseRad * cfg_.magYawNoiseRad;
    ekfUpdate(r, H, Rm);
    magCount_++;
}

// ---------------------------------------------------------------------------
// GPS 位置量测(ENU)
// ---------------------------------------------------------------------------
void MsckfFilter::feedGpsPosition(const GpsFix& fix, const Vec3& enu) {
    if (!initialized_ || !cfg_.gpsEnabled) return;
    if (lastGpsT_ > 0 && fix.t > lastGpsT_) {
        gpsRate_ = 1e9 / (fix.t - lastGpsT_);
    }
    lastGpsT_ = fix.t;

    Eigen::Matrix<double, 3, 15> Himu = Eigen::Matrix<double, 3, 15>::Zero();
    Himu.block<3, 3>(0, 3) = Mat3::Identity();
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, P_.rows());
    H.topLeftCorner(3, 15) = Himu;
    VecXd r(3);
    r << enu.x() - imu_.p.x(), enu.y() - imu_.p.y(), enu.z() - imu_.p.z();
    std::fprintf(stderr,
                 "[dbg] gps t=%lld enu=(%g,%g,%g) p=(%g,%g,%g) r=(%g,%g,%g) Ppp=(%g,%g,%g) "
                 "theta=(%g,%g,%g) v=(%g,%g,%g) ba=(%g,%g,%g)\n",
                 static_cast<long long>(fix.t), enu.x(), enu.y(), enu.z(), imu_.p.x(), imu_.p.y(),
                 imu_.p.z(), r.x(), r.y(), r.z(), P_(3, 3), P_(4, 4), P_(5, 5),
                 imu_.q.toRotationMatrix().eulerAngles(0, 1, 2).x(),
                 imu_.q.toRotationMatrix().eulerAngles(0, 1, 2).y(),
                 imu_.q.toRotationMatrix().eulerAngles(0, 1, 2).z(), imu_.v.x(), imu_.v.y(),
                 imu_.v.z(), imu_.ba.x(), imu_.ba.y(), imu_.ba.z());
    Mat3 R = Mat3::Identity() * (fix.sigma * fix.sigma);
    ekfUpdate(r, H, R);
    gpsCount_++;
    std::fprintf(stderr, "[dbg] gps POST Ppp=(%g,%g,%g) Pvv=(%g,%g,%g) pmax=%g\n", P_(3, 3), P_(4, 4),
                 P_(5, 5), P_(6, 6), P_(7, 7), P_(8, 8), P_.cwiseAbs().maxCoeff());
}

// ---------------------------------------------------------------------------
// 图像入口
// ---------------------------------------------------------------------------
void MsckfFilter::feedImage(int64_t t,
                            const std::vector<std::pair<uint32_t, Vec2>>& matched,
                            const std::vector<std::pair<uint32_t, Vec2>>& fresh) {
    if (!initialized_) return;
    if (t <= lastImageT_) {
        std::fprintf(stderr, "[dbg] img drop stale-last t=%g last=%g\n", t * 1e-9,
                     lastImageT_ * 1e-9);
        return;
    }
    if (t <= imu_.t) {
        std::fprintf(stderr, "[dbg] img drop stale-imu t=%g imu=%g\n", t * 1e-9, imu_.t * 1e-9);
        return;
    }
    std::fprintf(stderr, "[dbg] img ACCEPT t=%g imu=%g last=%g\n", t * 1e-9, imu_.t * 1e-9,
                 lastImageT_ * 1e-9);

    if (lastImgT_ > 0 && t > lastImgT_) {
        imgRate_ = 1e9 / (t - lastImgT_);
    }
    lastImgT_ = t;
    imgCount_++;

    prune();
    augment();
    attachObs(matched, fresh);
    finalizeLost();
    prune();
}

MsckfFilter::CamPose MsckfFilter::cameraPose(size_t idx) const {
    return cams_[idx];
}

}  // namespace gvio
