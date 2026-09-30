#include "gvio/msckf.h"
#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace gvio {
namespace {
using I = StateIndex;
Mat3 rightJacobian(const Vec3& v) {
    double a2=v.squaredNorm(); Mat3 V=skewM(v);
    if (a2<1e-10) return Mat3::Identity()-.5*V+(1./6.)*V*V;
    double a=std::sqrt(a2);
    return Mat3::Identity()-(1.-std::cos(a))/a2*V+(a-std::sin(a))/(a2*a)*V*V;
}
Eigen::Matrix<double,2,3> projectionJacobian(const CameraParams& c,const Vec3& p) {
    double x=p.x()/p.z(), y=p.y()/p.z(), r=x*x+y*y;
    double d=1.+c.k1*r+c.k2*r*r+c.k3*r*r*r;
    double dr=c.k1+2.*c.k2*r+3.*c.k3*r*r;
    Eigen::Matrix2d D;
    D << c.fx*(d+2.*x*x*dr+2.*c.p1*y+6.*c.p2*x), c.fx*(2.*x*y*dr+2.*c.p1*x+2.*c.p2*y),
         c.fy*(2.*x*y*dr+2.*c.p1*x+2.*c.p2*y), c.fy*(d+2.*y*y*dr+6.*c.p1*y+2.*c.p2*x);
    Eigen::Matrix<double,2,3> J;
    J << 1./p.z(),0.,-x/p.z(),0.,1./p.z(),-y/p.z();
    return D*J;
}
bool bearing(const CameraParams& c,const Vec2& pixel,Vec2& uv) {
    uv={(pixel.x()-c.cx)/c.fx,(pixel.y()-c.cy)/c.fy};
    for (int k=0;k<12;++k) {
        Vec3 p(uv.x(),uv.y(),1.);
        Vec2 r=c.project(p)-pixel;
        if (!r.allFinite()) return false;
        if (r.norm()<1e-8) return true;
        Eigen::Matrix2d J=projectionJacobian(c,p).leftCols<2>();
        if (std::abs(J.determinant())<1e-12) return false;
        uv-=J.fullPivLu().solve(r);
    }
    return (c.project(Vec3(uv.x(),uv.y(),1.))-pixel).norm()<1e-5;
}
}
MsckfFilter::MsckfFilter(const FilterConfig& cfg):cfg_(cfg) {
    if (cfg_.cameraWindowSize<1 || cfg_.cameraWindowSize>64 || cfg_.minTriObs<2 ||
        cfg_.maxFeatures<1 || cfg_.maxFeatures>5000 || !(cfg_.featureNoisePx>0.) ||
        !std::isfinite(cfg_.featureNoisePx) || !(cfg_.cam.fx>0.) || !(cfg_.cam.fy>0.) ||
        !std::isfinite(cfg_.cam.fx) || !std::isfinite(cfg_.cam.fy) ||
        !cfg_.cam.qIC().coeffs().allFinite() || cfg_.cam.qIC().norm()<1e-10 ||
        !cfg_.cam.pIC().allFinite()) throw std::invalid_argument("invalid filter/camera configuration");
    cfg_.cam.imuToCamQ.normalize();
    for (double x : {cfg_.sigmaG,cfg_.sigmaA,cfg_.sigmaBg,cfg_.sigmaBa,
                     cfg_.initPosSigma,cfg_.initVelSigma,cfg_.initYawSigmaDeg,
                     cfg_.initBiasGSigma,cfg_.initBiasASigma})
        if (!std::isfinite(x) || x<0.) throw std::invalid_argument("invalid noise standard deviation");
    reset();
}
void MsckfFilter::reset() {
    imu_=ImuState{}; P_=Mat15::Zero(); cams_.clear(); tracks_.clear(); recentPts_.clear();
    initialized_=havePreviousImu_=false; previousImu_=ImuSample{}; lastImageT_=-1;
    nextTrackId_=1; imuRate_=imgRate_=gpsRate_=0.;
    lastImuT_=lastImgT_=lastGpsT_=lastUpdateDurationUs_=0;
    imgCount_=updateCount_=gpsCount_=magCount_=visualUpdateCount_=0;
    lastUpdateGated=false;
}
void MsckfFilter::initialize(const Vec3& p0,const Quat& q0,int64_t t0) {
    if (!p0.allFinite() || !q0.coeffs().allFinite() || q0.norm()<1e-10 || t0<0)
        throw std::invalid_argument("invalid initial state");
    ImuSample seed=previousImu_; bool haveSeed=havePreviousImu_ && seed.t==t0;
    reset(); imu_.q=q0.normalized(); imu_.p=p0; imu_.t=t0;
    Eigen::Matrix<double,15,1> d;
    d.segment<3>(I::Theta).setConstant(cfg_.initYawSigmaDeg*kPi/180.);
    d.segment<3>(I::Position).setConstant(cfg_.initPosSigma);
    d.segment<3>(I::Velocity).setConstant(cfg_.initVelSigma);
    d.segment<3>(I::GyroBias).setConstant(cfg_.initBiasGSigma);
    d.segment<3>(I::AccelBias).setConstant(cfg_.initBiasASigma);
    P_=d.array().square().matrix().asDiagonal();
    previousImu_=seed; havePreviousImu_=haveSeed; initialized_=true;
}
void MsckfFilter::nanGuard(const char* tag) {
    if (!P_.allFinite() || !imu_.q.coeffs().allFinite() || !imu_.p.allFinite() ||
        !imu_.v.allFinite() || !imu_.bg.allFinite() || !imu_.ba.allFinite())
        throw std::runtime_error(std::string("non-finite filter state: ")+tag);
}
Mat15 MsckfFilter::dynamics(const Quat& q,const Vec3& accel) {
    Mat3 Rt=q.toRotationMatrix().transpose();
    Mat15 F=Mat15::Zero();
    // R_true=R_hat Exp(dtheta_G): dtheta_G_dot=R^T dbg_I.
    F.block<3,3>(I::Theta,I::GyroBias)=Rt;
    F.block<3,3>(I::Position,I::Velocity)=Mat3::Identity();
    F.block<3,3>(I::Velocity,I::Theta)=skewM(Rt*accel);
    F.block<3,3>(I::Velocity,I::AccelBias)=-Rt;
    return F;
}
Eigen::Matrix<double,6,15> MsckfFilter::augmentationJacobian(const Quat& q,const Vec3& pIC) {
    Eigen::Matrix<double,6,15> J=Eigen::Matrix<double,6,15>::Zero();
    J.block<3,3>(0,I::Theta)=Mat3::Identity();
    J.block<3,3>(3,I::Theta)=skewM(q.conjugate()*pIC);
    J.block<3,3>(3,I::Position)=Mat3::Identity();
    return J;
}
void MsckfFilter::feedImu(const ImuSample& s) {
    if (s.t<0 || !s.accel.allFinite() || !s.gyro.allFinite())
        throw std::invalid_argument("invalid IMU sample");
    if (!initialized_) { previousImu_=s; havePreviousImu_=true; imu_.t=s.t; return; }
    if (s.t<imu_.t) return;
    if (s.t==imu_.t) { previousImu_=s; havePreviousImu_=true; return; }
    if (s.t-imu_.t>1000000000LL) throw std::runtime_error("IMU gap exceeds one second; reset required");
    if (lastImuT_>0) imuRate_=1e9/double(s.t-lastImuT_);
    lastImuT_=s.t; propagateOne(s); previousImu_=s; havePreviousImu_=true;
}
void MsckfFilter::propagateOne(const ImuSample& s) {
    double total=(s.t-imu_.t)*1e-9;
    const ImuSample start=havePreviousImu_ ? previousImu_ : s;
    int steps=std::max(1,int(std::ceil(total/.01))); double dt=total/steps;
    for (int step=0;step<steps;++step) {
        double alpha=(step+.5)/steps;
        Vec3 w=(1.-alpha)*start.gyro+alpha*s.gyro-imu_.bg;
        Vec3 a=(1.-alpha)*start.accel+alpha*s.accel-imu_.ba;
        Quat qm=(quatIntegrate(Quat::Identity(),-w,dt*.5)*imu_.q).normalized();
        Vec3 ag=qm.conjugate()*a+GRAVITY;
        Mat15 F=dynamics(qm,a);
        Mat15 Phi=Mat15::Identity()+F*dt+.5*F*F*dt*dt;
        Mat15 L=Mat15::Zero();
        L.block<3,3>(I::Theta,I::Theta).diagonal().setConstant(cfg_.sigmaG*cfg_.sigmaG);
        L.block<3,3>(I::Velocity,I::Velocity).diagonal().setConstant(cfg_.sigmaA*cfg_.sigmaA);
        L.block<3,3>(I::GyroBias,I::GyroBias).diagonal().setConstant(cfg_.sigmaBg*cfg_.sigmaBg);
        L.block<3,3>(I::AccelBias,I::AccelBias).diagonal().setConstant(cfg_.sigmaBa*cfg_.sigmaBa);
        Mat15 Q=.5*dt*(L+Phi*L*Phi.transpose()); // PSD trapezoidal noise integral
        Mat15 Pii=Phi*P_.topLeftCorner<15,15>()*Phi.transpose()+Q;
        if (P_.cols()>15) {
            MatXd cross=Phi*P_.block(0,15,15,P_.cols()-15);
            P_.block(0,15,15,P_.cols()-15)=cross;
            P_.block(15,0,P_.rows()-15,15)=cross.transpose();
        }
        P_.topLeftCorner<15,15>()=Pii;
        imu_.p+=imu_.v*dt+.5*ag*dt*dt; imu_.v+=ag*dt;
        imu_.q=(quatIntegrate(Quat::Identity(),-w,dt)*imu_.q).normalized();
    }
    imu_.t=s.t; P_=(.5*(P_+P_.transpose())).eval(); nanGuard("propagate");
}
void MsckfFilter::augment() {
    int n=int(P_.rows()); MatXd J=MatXd::Zero(6,n);
    J.leftCols<15>()=augmentationJacobian(imu_.q,cfg_.cam.pIC());
    MatXd JP=J*P_, out(n+6,n+6);
    out.topLeftCorner(n,n)=P_;
    out.bottomLeftCorner(6,n)=JP; out.topRightCorner(n,6)=JP.transpose();
    out.bottomRightCorner<6,6>()=JP*J.transpose(); P_=std::move(out);
    cams_.push_back({(cfg_.cam.qIC()*imu_.q).normalized(),imu_.p+imu_.q.conjugate()*cfg_.cam.pIC(),imu_.t});
    lastImageT_=imu_.t;
}
void MsckfFilter::prune() {
    while (int(cams_.size())>cfg_.cameraWindowSize) {
        std::vector<uint32_t> ids;
        for (const auto& kv:tracks_)
            if (!kv.second.obs.empty() && kv.second.obs.front().first==0) ids.push_back(kv.first);
        consumeTracks(ids); // exactly once, before removing the referenced clone
        cams_.erase(cams_.begin());
        for (auto& kv:tracks_) for (auto& ob:kv.second.obs) --ob.first;
        int n=int(P_.rows())-6; MatXd out(n,n);
        for (int i=0;i<n;++i) for (int j=0;j<n;++j)
            out(i,j)=P_(i<15?i:i+6,j<15?j:j+6);
        P_=std::move(out);
    }
}
void MsckfFilter::attachObs(const std::vector<std::pair<uint32_t,Vec2>>& matched,
                            const std::vector<std::pair<uint32_t,Vec2>>& fresh) {
    int ci=int(cams_.size())-1;
    for (auto& kv:tracks_) kv.second.lost=true;
    auto add=[&](const auto& values,bool allowNew) {
        for (const auto& ob:values) {
            if (!ob.second.allFinite()) continue;
            auto it=tracks_.find(ob.first);
            if (it==tracks_.end()) {
                if (!allowNew || tracks_.size()>=size_t(cfg_.maxFeatures)) continue;
                it=tracks_.emplace(ob.first,FeatureTrack{}).first;
                it->second.id=ob.first;
            }
            auto& ft=it->second;
            if (!ft.obs.empty() && ft.obs.back().first==ci) continue;
            ft.uv=ob.second; ft.obs.push_back({ci,ob.second}); ft.lost=false;
        }
    };
    // Mark surviving existing tracks first; free lost slots before replenishment.
    // Otherwise a full previous frame suppresses all replacements for one frame.
    add(matched,false);
    finalizeLost();
    add(matched,true); add(fresh,true);
}
void MsckfFilter::finalizeLost() {
    std::vector<uint32_t> ids;
    for (const auto& kv:tracks_) if (kv.second.lost) ids.push_back(kv.first);
    consumeTracks(ids);
}
bool MsckfFilter::triangulate(const FeatureTrack& ft,Vec3& fG) const {
    int K=int(ft.obs.size()); if (K<cfg_.minTriObs) return false;
    MatXd A(2*K,4); int row=0;
    for (const auto& ob:ft.obs) {
        if (ob.first<0 || ob.first>=int(cams_.size())) return false;
        const auto& cp=cams_[ob.first]; Mat3 R=cp.q.toRotationMatrix();
        Eigen::Matrix<double,3,4> P; P.leftCols<3>()=R; P.col(3)=-R*cp.p;
        Vec2 uv; if (!bearing(cfg_.cam,ob.second,uv)) return false;
        A.row(row++)=uv.x()*P.row(2)-P.row(0); A.row(row++)=uv.y()*P.row(2)-P.row(1);
    }
    Eigen::JacobiSVD<MatXd> svd(A,Eigen::ComputeFullV);
    Vec4 h=svd.matrixV().col(3); if (!h.allFinite() || std::abs(h.w())<1e-12) return false;
    fG=h.head<3>()/h.w(); if (!fG.allFinite()) return false;
    // Compare rays in ONE frame. Camera rotations alone do not create baseline.
    const Vec3 da=fG-cams_[ft.obs.front().first].p, db=fG-cams_[ft.obs.back().first].p;
    if (da.norm()<1e-8 || db.norm()<1e-8) return false;
    double angle=std::acos(std::clamp(da.normalized().dot(db.normalized()),-1.,1.));
    if (angle<cfg_.minParallaxDeg*kPi/180.) return false;
    double err=0.;
    for (const auto& ob:ft.obs) {
        const auto& cp=cams_[ob.first]; Vec3 pc=cp.q*(fG-cp.p);
        if (pc.z()<=.05) return false;
        err+=(cfg_.cam.project(pc)-ob.second).squaredNorm();
    }
    return std::isfinite(err) && std::sqrt(err/K)<3.;
}
void MsckfFilter::consumeTracks(const std::vector<uint32_t>& ids) {
    int n=int(P_.rows()); std::vector<MatXd> hs; std::vector<VecXd> rs;
    size_t rows=0;
    for (uint32_t id:ids) {
        auto it=tracks_.find(id); if (it==tracks_.end()) continue;
        // Erase regardless of triangulation/gating outcome. Never clear caller's id list.
        FeatureTrack ft=std::move(it->second); tracks_.erase(it);
        if (ft.obs.size()>32) ft.obs.erase(ft.obs.begin(),ft.obs.end()-32);
        Vec3 point; if (!triangulate(ft,point)) continue;
        int k=int(ft.obs.size()); MatXd Hf(2*k,3), Hx=MatXd::Zero(2*k,n); VecXd r(2*k);
        int row=0;
        for (const auto& ob:ft.obs) {
            const auto& cp=cams_[ob.first]; Mat3 R=cp.q.toRotationMatrix();
            Vec3 pc=R*(point-cp.p); auto J=projectionJacobian(cfg_.cam,pc);
            r.segment<2>(row)=ob.second-cfg_.cam.project(pc);
            Hf.block<2,3>(row,0)=J*R;
            Hx.block<2,3>(row,15+6*ob.first)=-J*R*skewM(point-cp.p);
            Hx.block<2,3>(row,18+6*ob.first)=-J*R; row+=2;
        }
        Eigen::JacobiSVD<MatXd> svd(Hf,Eigen::ComputeFullU);
        if (svd.singularValues()(2)<=svd.singularValues()(0)*1e-8) continue;
        MatXd N=svd.matrixU().rightCols(2*k-3);
        MatXd H=N.transpose()*Hx; VecXd residual=N.transpose()*r;
        MatXd S=H*P_*H.transpose(); S.diagonal().array()+=cfg_.featureNoisePx*cfg_.featureNoisePx;
        Eigen::LDLT<MatXd> ldlt(S);
        if (ldlt.info()!=Eigen::Success || !ldlt.isPositive()) continue;
        double d=residual.dot(ldlt.solve(residual));
        if (!std::isfinite(d) || d>3.*residual.size()) continue;
        rows+=residual.size(); hs.push_back(std::move(H)); rs.push_back(std::move(residual));
        recentPts_.push_back(point);
    }
    if (recentPts_.size()>400) recentPts_.erase(recentPts_.begin(),recentPts_.end()-400);
    if (!rows) return;
    MatXd H(rows,n); VecXd r(rows); Eigen::Index off=0;
    for (size_t i=0;i<rs.size();++i) {
        H.middleRows(off,rs[i].size())=hs[i]; r.segment(off,rs[i].size())=rs[i]; off+=rs[i].size();
    }
    // One common linearization and ONE injection. QR avoids large innovation inverses
    // without the old stale-residual sequential-chunk updates.
    if (H.rows()>n) {
        Eigen::HouseholderQR<MatXd> qr(H);
        VecXd qrR=qr.householderQ().adjoint()*r;
        MatXd reduced=qr.matrixQR().topRows(n).triangularView<Eigen::Upper>();
        H=std::move(reduced); r=qrR.head(n);
    }
    MatXd R=MatXd::Identity(r.size(),r.size())*(cfg_.featureNoisePx*cfg_.featureNoisePx);
    if (ekfUpdate(r,H,R)) ++visualUpdateCount_;
}
bool MsckfFilter::ekfUpdate(const VecXd& r,const MatXd& H,const MatXd& R) {
    lastUpdateGated=true;
    if (!r.size() || H.rows()!=r.size() || H.cols()!=P_.rows() || R.rows()!=r.size() ||
        R.cols()!=r.size() || !r.allFinite() || !H.allFinite() || !R.allFinite()) return false;
    auto begin=std::chrono::steady_clock::now();
    MatXd PHt=P_*H.transpose(), S=H*PHt+R;
    Eigen::LDLT<MatXd> ldlt(S);
    if (ldlt.info()!=Eigen::Success || !ldlt.isPositive() || (ldlt.vectorD().array()<=0.).any()) return false;
    MatXd K=ldlt.solve(PHt.transpose()).transpose(); VecXd dx=K*r;
    if (!dx.allFinite()) return false;
    MatXd A=MatXd::Identity(P_.rows(),P_.cols())-K*H;
    MatXd pn=A*P_*A.transpose()+K*R*K.transpose();
    MatXd resetJ=MatXd::Identity(P_.rows(),P_.cols());
    resetJ.block<3,3>(0,0)=rightJacobian(dx.head<3>());
    for (size_t k=0;k<cams_.size();++k) resetJ.block<3,3>(15+6*k,15+6*k)=rightJacobian(dx.segment<3>(15+6*k));
    pn=(resetJ*pn*resetJ.transpose()).eval(); pn=(.5*(pn+pn.transpose())).eval();
    ImuState next=imu_;
    next.q=quatIntegrate(imu_.q,dx.head<3>(),1.);
    next.p+=dx.segment<3>(3); next.v+=dx.segment<3>(6); next.bg+=dx.segment<3>(9); next.ba+=dx.segment<3>(12);
    if (!pn.allFinite() || !next.q.coeffs().allFinite() || !next.p.allFinite() || !next.v.allFinite() ||
        !next.bg.allFinite() || !next.ba.allFinite()) return false;
    imu_=next; P_=std::move(pn);
    for (size_t k=0;k<cams_.size();++k) {
        cams_[k].q=quatIntegrate(cams_[k].q,dx.segment<3>(15+6*k),1.);
        cams_[k].p+=dx.segment<3>(18+6*k);
    }
    ++updateCount_; lastUpdateGated=false;
    lastUpdateDurationUs_=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-begin).count();
    return true;
}
void MsckfFilter::feedMag(const MagSample& s) {
    if (!initialized_ || !cfg_.magEnabled || s.t!=imu_.t || !s.value.allFinite() || !s.bias.allFinite()) return;
    Vec3 m=s.value-s.bias; if (m.norm()<1e-8 || !(cfg_.magYawNoiseRad>0.)) return;
    // Tilt-compensated heading observation. Numerical derivative uses exactly the
    // same right-global retraction as propagation and injection.
    double decl=cfg_.magDeclinationDeg*kPi/180.;
    auto residual=[&](const Quat& q) {
        Vec3 mg=q.conjugate()*m;
        return wrapAngle(std::atan2(mg.x(),mg.y())-decl);
    };
    Vec3 mg=imu_.q.conjugate()*m;
    if (mg.head<2>().norm()<.05*m.norm()) return;
    VecXd r(1); r(0)=residual(imu_.q);
    MatXd H=MatXd::Zero(1,P_.cols()); constexpr double e=1e-6;
    for (int j=0;j<3;++j) {
        Vec3 d=Vec3::Zero(); d(j)=e;
        H(0,j)=-wrapAngle(residual(quatIntegrate(imu_.q,d,1.))-residual(quatIntegrate(imu_.q,-d,1.)))/(2.*e);
    }
    MatXd R=MatXd::Constant(1,1,cfg_.magYawNoiseRad*cfg_.magYawNoiseRad);
    double S=(H*P_*H.transpose())(0,0)+R(0,0);
    if (r(0)*r(0)>9.*S) return;
    if (ekfUpdate(r,H,R)) ++magCount_;
}
void MsckfFilter::feedGpsPosition(const GpsFix& f,const Vec3& enu) {
    if (!initialized_ || !cfg_.gpsEnabled || f.t!=imu_.t || !enu.allFinite() || !std::isfinite(f.sigma) || f.sigma<=0.) return;
    if (lastGpsT_>0 && f.t>lastGpsT_) gpsRate_=1e9/double(f.t-lastGpsT_);
    lastGpsT_=f.t;
    MatXd H=MatXd::Zero(3,P_.cols()); H.block<3,3>(0,I::Position)=Mat3::Identity();
    if (ekfUpdate(enu-imu_.p,H,Mat3::Identity()*f.sigma*f.sigma)) ++gpsCount_;
}
ImageResult MsckfFilter::feedImage(int64_t t,const std::vector<std::pair<uint32_t,Vec2>>& matched,
                                  const std::vector<std::pair<uint32_t,Vec2>>& fresh) {
    if (!initialized_) return ImageResult::NotInitialized;
    if (t<=lastImageT_) return ImageResult::DuplicateOrStale;
    if (t!=imu_.t) return ImageResult::NotAtImuTime;
    if (lastImgT_>0) imgRate_=1e9/double(t-lastImgT_);
    lastImgT_=t; ++imgCount_;
    augment(); attachObs(matched,fresh); prune(); nanGuard("image");
    return ImageResult::Accepted;
}
} // namespace gvio
