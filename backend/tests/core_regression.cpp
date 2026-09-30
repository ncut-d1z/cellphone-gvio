#include "gvio/msckf.h"
#include "gvio/timeline.h"
#include "gvio/vision.h"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
using namespace gvio;
void require(bool x,const char* what) { if (!x) throw std::runtime_error(what); }
void covariance(const MsckfFilter& f) {
    const auto& p=f.covariance(); require(p.allFinite(),"finite covariance");
    require((p-p.transpose()).norm()<1e-8,"symmetric covariance");
    Eigen::SelfAdjointEigenSolver<MatXd> es(p);
    require(es.eigenvalues().minCoeff()>-1e-8,"PSD covariance");
}
void propagation() {
    FilterConfig cfg; MsckfFilter f(cfg); f.initialize(Vec3::Zero(),Quat::Identity(),0);
    ImuSample s; s.accel=-GRAVITY; f.feedImu(s);
    for (int i=1;i<=200;++i) { s.t=i*5000000LL; f.feedImu(s); }
    require(f.imuPos().norm()<1e-10 && f.imuVel().norm()<1e-10,"stationary gravity cancellation"); covariance(f);
    f.reset(); f.initialize(Vec3::Zero(),Quat::Identity(),0);
    s.t=0; s.gyro=Vec3(0,0,.4); f.feedImu(s);
    for (int i=1;i<=200;++i) { s.t=i*5000000LL; f.feedImu(s); }
    Quat expected(Eigen::AngleAxisd(-.4,Vec3::UnitZ()));
    require(f.imuQuat().angularDistance(expected)<1e-8,"physical angular velocity sign");
    require(f.imuPos().norm()<1e-8,"yaw rotation gravity cancellation");
    // Off-axis, nonidentity initial attitude: left physical integration must hold.
    Quat q(Eigen::AngleAxisd(.5,Vec3(1,2,3).normalized()));
    f.initialize(Vec3::Zero(),q,0); s.t=0; s.gyro=Vec3(.1,.2,.3); f.feedImu(s);
    s.t=10000000; f.feedImu(s);
    require(f.imuQuat().angularDistance(quatIntegrate(Quat::Identity(),-s.gyro,.01)*q)<1e-8,"noncommuting attitude integration");
    require(quatIntegrate(Quat::Identity(),Vec3(1e-8,0,0),1.).x()!=0.,"small-angle increment not discarded");
    covariance(f);
}
void jacobians() {
    Quat q(Eigen::AngleAxisd(.7,Vec3(1,2,3).normalized())); Vec3 lever(.1,-.2,.3),a(.4,.6,9.);
    auto J=MsckfFilter::augmentationJacobian(q,lever); auto F=MsckfFilter::dynamics(q,a);
    require(J.block<3,3>(3,3).isIdentity(1e-12),"position block offset");
    require(F.block<3,3>(3,6).isIdentity(1e-12),"velocity block offset");
    constexpr double e=1e-6;
    for (int k=0;k<3;++k) {
        Vec3 d=Vec3::Zero(); d(k)=e;
        Quat qp=quatIntegrate(q,d,1.),qm=quatIntegrate(q,-d,1.);
        Vec3 numerical=(qp.conjugate()*lever-qm.conjugate()*lever)/(2.*e);
        require((numerical-J.block<3,1>(3,k)).norm()<1e-8,"lever arm Jacobian");
        numerical=(qp.conjugate()*a-qm.conjugate()*a)/(2.*e);
        require((numerical-F.block<3,1>(6,k)).norm()<1e-8,"acceleration attitude Jacobian");
    }
    // Gyro bias perturbation differentiated through the actual rotation law.
    Vec3 w(.2,-.1,.3); double dt=1e-5;
    Quat nominal=quatIntegrate(Quat::Identity(),-w,dt)*q;
    for (int k=0;k<3;++k) {
        Vec3 d=Vec3::Zero(); d(k)=e;
        Quat truth=quatIntegrate(Quat::Identity(),-(w-d),dt)*q;
        Quat dq=nominal.conjugate()*truth;
        Vec3 numerical=2.*dq.vec()/(dt*e);
        require((numerical-F.block<3,1>(0,9+k)).norm()<1e-4,"gyro-bias attitude Jacobian");
    }
}
void clonesAndTracks() {
    FilterConfig cfg; cfg.cameraWindowSize=2; cfg.cam.imuToCamQ=Quat(Eigen::AngleAxisd(.4,Vec3::UnitY()));
    cfg.cam.imuToCamP=Vec3(.1,.2,.3);
    MsckfFilter f(cfg); Quat q(Eigen::AngleAxisd(.2,Vec3::UnitX())); f.initialize(Vec3::Zero(),q,0);
    require(f.feedImage(0,{},{{1,Vec2(100,100)}})==ImageResult::Accepted,"equal image/IMU time accepted");
    require(f.cameraPose(0).q.angularDistance(cfg.cam.qIC()*q)<1e-12,"extrinsic composition order");
    require((f.cameraPose(0).p-q.conjugate()*cfg.cam.pIC()).norm()<1e-12,"clone lever arm");
    require(f.feedImage(0,{}, {})==ImageResult::DuplicateOrStale,"duplicate rejected");
    require(f.feedImage(1,{}, {})==ImageResult::NotAtImuTime,"unaligned image rejected");
    ImuSample s; s.accel=q*(-GRAVITY);
    s.t=5000000; f.feedImu(s); f.feedImage(s.t,{},{});
    require(f.trackCount()==0,"lost untriangulatable track removed");
    for (int i=2;i<30;++i) {
        s.t=i*5000000LL; f.feedImu(s); f.feedImage(s.t,{{7,Vec2(100,100)}},{});
        require(f.cameraCount()<=2,"bounded clones"); require(f.trackCount()<=1,"bounded live tracks");
        require(f.covariance().rows()==15+6*int(f.cameraCount()),"covariance dimension");
    }
    covariance(f); f.reset(); require(f.covariance().rows()==15 && f.trackCount()==0,"complete reset");
}
void visualUpdate() {
    FilterConfig cfg; cfg.cam.k1=.03; cfg.cam.p1=.001; cfg.cam.hasDistortion=true;
    MsckfFilter f(cfg); f.initialize(Vec3::Zero(),Quat::Identity(),0);
    ImuSample imu; imu.accel=Vec3(2.,0.,9.80665); f.feedImu(imu);
    const Vec3 landmark(1.,.2,5.);
    for (int k=0;k<5;++k) {
        imu.t=k*200000000LL; f.feedImu(imu);
        Vec2 uv=cfg.cam.project(f.imuQuat()*(landmark-f.imuPos()));
        f.feedImage(imu.t,k?std::vector<std::pair<uint32_t,Vec2>>{{1,uv}}:std::vector<std::pair<uint32_t,Vec2>>{},
                    k?std::vector<std::pair<uint32_t,Vec2>>{}:std::vector<std::pair<uint32_t,Vec2>>{{1,uv}});
    }
    require(f.trackCount()==1,"live multi-view track retained");
    imu.t=1000000000; f.feedImu(imu); f.feedImage(imu.t,{},{});
    require(f.visualUpdateCount()==1 && f.trackCount()==0,"successful visual track consumed exactly once");
    require(f.recentPoints().size()==1 && (f.recentPoints()[0]-landmark).norm()<1e-6,"distorted multi-view triangulation");
    covariance(f);
    imu.t+=200000000; f.feedImu(imu); f.feedImage(imu.t,{},{});
    require(f.visualUpdateCount()==1,"lost track cannot update twice");
    // Noisy, finite GPS update exercises nonzero state injection/reset.
    GpsFix fix; fix.t=imu.t; fix.sigma=1.;
    Vec3 before=f.imuPos(); f.feedGpsPosition(fix,before+Vec3(.1,-.1,.05));
    require(f.gpsCount()==1 && (f.imuPos()-before).norm()>0.,"GPS updates state"); covariance(f);
}
void timeline() {
    SensorTimeline queue(20000000); std::vector<int64_t> obs; ImuSample last;
    auto feed=[&](int i) { ImuSample s; s.t=i*10000000LL; s.gyro=Vec3(double(i),0,0); queue.pushImu(s); };
    for (int i:{3,1,5,0,2,4}) feed(i);
    for (int64_t t:{25000000LL,10000000LL,15000000LL})
        queue.pushObservation(t,40,[&,t]{require(last.t==t,"observation aligned to interpolated IMU");require(std::abs(last.gyro.x()-double(t)/1e7)<1e-10,"linear interpolation");obs.push_back(t);});
    queue.drain([&](const ImuSample& s){last=s;});
    require(obs==std::vector<int64_t>({10000000,15000000,25000000}),"sorted observations");
    require(!queue.pushObservation(15000000,40,[]{}),"too late rejected");
    queue.pushObservation(60000000,40,[]{throw std::runtime_error("extrapolated observation");});
    queue.drain([&](const ImuSample& s){last=s;},true);
    require(queue.unbracketed==1 && queue.late==1,"drop counters");
}
float texture(float x,float y) { return 125.f+35.f*std::sin(.19f*x)+32.f*std::cos(.21f*y)+28.f*std::sin(.13f*x+.17f*y); }
void opticalFlow() {
    int w=160,h=120; std::vector<uint8_t> a(w*h),b(w*h);
    std::vector<Keypoint> pts={{55.25f,52.5f,1},{83,72,1},{110,60,1}}; std::vector<Keypoint> out; std::vector<uint8_t> status;
    for (auto shift:std::vector<Vec2>{{0,0},{3,-2},{.5,.75}}) {
        for (int y=0;y<h;++y) for (int x=0;x<w;++x) {
            a[y*w+x]=uint8_t(std::lround(texture(float(x),float(y))));
            b[y*w+x]=uint8_t(std::lround(texture(float(x-shift.x()),float(y-shift.y()))));
        }
        int n=trackKlt(a,b,w,h,pts,out,status); require(n==int(pts.size()),"textured tracks survive");
        for (size_t i=0;i<pts.size();++i) require(std::hypot(out[i].x-pts[i].x-shift.x(),out[i].y-pts[i].y-shift.y())<.12,"known optical flow");
    }
    std::fill(a.begin(),a.end(),100); require(trackKlt(a,a,w,h,pts,out,status)==0,"flat image rejected");
    require(trackKlt({},a,w,h,pts,out,status)==0,"invalid image rejected");
}
int main(int argc,char** argv) {
    try {
        std::string name=argc>1?argv[1]:"all";
        require(name=="all" || name=="propagation" || name=="jacobians" || name=="tracks" || name=="timeline" || name=="vision","unknown test case");
        if (name=="all" || name=="propagation") propagation();
        if (name=="all" || name=="jacobians") jacobians();
        if (name=="all" || name=="tracks") { clonesAndTracks(); visualUpdate(); }
        if (name=="all" || name=="timeline") timeline();
        if (name=="all" || name=="vision") opticalFlow();
        std::cout<<"PASS "<<name<<'\n'; return 0;
    } catch (const std::exception& e) { std::cerr<<"FAIL: "<<e.what()<<'\n'; return 1; }
}
