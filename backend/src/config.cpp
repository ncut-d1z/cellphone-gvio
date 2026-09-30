#include "gvio/config.h"
#include "gvio/msckf.h"
namespace gvio {
Config Config::fromJson(const std::string& text) {
    Config c;
    try {
        Json j=Json::parse(text); if (!j.is_object()) return c;
        auto& f=c.filter;
        const Json cam=j.value("camera",Json::object());
        f.cam.width=cam.value("width",f.cam.width); f.cam.height=cam.value("height",f.cam.height);
        f.cam.fx=cam.value("fx",f.cam.fx); f.cam.fy=cam.value("fy",f.cam.fy);
        f.cam.cx=cam.value("cx",f.cam.cx); f.cam.cy=cam.value("cy",f.cam.cy);
        if (cam.contains("intrinsics")) {
            const auto& a=cam.at("intrinsics"); if (a.size()!=4) return c;
            f.cam.fx=a.at(0); f.cam.fy=a.at(1); f.cam.cx=a.at(2); f.cam.cy=a.at(3);
        }
        if (cam.contains("distortion")) {
            const auto& a=cam.at("distortion"); if (a.size()!=5) return c;
            f.cam.k1=a.at(0); f.cam.k2=a.at(1); f.cam.p1=a.at(2); f.cam.p2=a.at(3); f.cam.k3=a.at(4);
            f.cam.hasDistortion=cam.value("hasDistortion",true);
        }
        if (cam.contains("imuToCamQ")) {
            const auto& a=cam.at("imuToCamQ"); if (a.size()!=4) return c;
            f.cam.imuToCamQ=Quat(a.at(3).get<double>(),a.at(0).get<double>(),a.at(1).get<double>(),a.at(2).get<double>());
        }
        if (cam.contains("imuToCamP")) {
            const auto& a=cam.at("imuToCamP"); if (a.size()!=3) return c;
            f.cam.imuToCamP=Vec3(a.at(0).get<double>(),a.at(1).get<double>(),a.at(2).get<double>());
        }
        const Json imu=j.value("imu",Json::object());
        f.sigmaG=imu.value("gyroNoise",f.sigmaG); f.sigmaA=imu.value("accelNoise",f.sigmaA);
        f.sigmaBg=imu.value("gyroBiasWalk",f.sigmaBg); f.sigmaBa=imu.value("accelBiasWalk",f.sigmaBa);
        const Json mag=j.value("mag",Json::object());
        f.magEnabled=mag.value("enabled",f.magEnabled); f.magDeclinationDeg=mag.value("declinationDeg",f.magDeclinationDeg);
        f.magInclinationDeg=mag.value("inclinationDeg",f.magInclinationDeg); f.magFieldUT=mag.value("fieldUT",f.magFieldUT);
        f.magYawNoiseRad=mag.value("yawNoiseRad",f.magYawNoiseRad);
        const Json gps=j.value("gps",Json::object());
        f.gpsEnabled=gps.value("enabled",f.gpsEnabled); f.gpsMinSats=gps.value("minSats",f.gpsMinSats);
        const Json ft=j.value("filter",Json::object());
        f.cameraWindowSize=ft.value("cameraWindowSize",ft.value("windowSize",f.cameraWindowSize));
#define READ_FIELD(name) f.name=ft.value(#name,f.name)
        READ_FIELD(featureNoisePx); READ_FIELD(maxFeatures); READ_FIELD(minParallaxDeg); READ_FIELD(minTriObs);
        READ_FIELD(reorderWindowNs); READ_FIELD(initPosSigma); READ_FIELD(initYawSigmaDeg); READ_FIELD(initVelSigma);
        READ_FIELD(initBiasGSigma); READ_FIELD(initBiasASigma); READ_FIELD(fastThreshold); READ_FIELD(maxCellsX);
        READ_FIELD(maxCellsY); READ_FIELD(maxPerCell);
#undef READ_FIELD
        const Json server=j.value("server",Json::object());
        f.serverEnabled=server.value("enabled",false); f.serverHost=server.value("host",f.serverHost);
        f.serverPort=server.value("port",f.serverPort); f.certPath=server.value("certPath",f.certPath);
        f.keyPath=server.value("keyPath",f.keyPath); f.webRoot=server.value("webRoot",f.webRoot);
        if (f.reorderWindowNs<0 || f.reorderWindowNs>1000000000LL || f.cam.width<1 || f.cam.height<1 ||
            f.serverPort<1 || f.serverPort>65535 || f.maxCellsX<1 || f.maxCellsY<1 || f.maxPerCell<1) return c;
        MsckfFilter checked(f); (void)checked;
        c.valid=true;
    } catch (const std::exception&) { c.valid=false; }
    return c;
}
std::string Config::toJson() const {
    const auto& f=filter; const auto& q=f.cam.qIC(); const auto& p=f.cam.pIC();
    Json j;
    j["camera"]={{"width",f.cam.width},{"height",f.cam.height},{"fx",f.cam.fx},{"fy",f.cam.fy},{"cx",f.cam.cx},{"cy",f.cam.cy},
        {"distortion",{f.cam.k1,f.cam.k2,f.cam.p1,f.cam.p2,f.cam.k3}},{"hasDistortion",f.cam.hasDistortion},
        {"imuToCamQ",{q.x(),q.y(),q.z(),q.w()}},{"imuToCamP",{p.x(),p.y(),p.z()}}};
    j["imu"]={{"gyroNoise",f.sigmaG},{"accelNoise",f.sigmaA},{"gyroBiasWalk",f.sigmaBg},{"accelBiasWalk",f.sigmaBa}};
    j["mag"]={{"enabled",f.magEnabled},{"declinationDeg",f.magDeclinationDeg},{"inclinationDeg",f.magInclinationDeg},
        {"fieldUT",f.magFieldUT},{"yawNoiseRad",f.magYawNoiseRad}};
    j["gps"]={{"enabled",f.gpsEnabled},{"minSats",f.gpsMinSats}};
    auto& ft=j["filter"];
#define WRITE_FIELD(name) ft[#name]=f.name
    WRITE_FIELD(cameraWindowSize); WRITE_FIELD(featureNoisePx); WRITE_FIELD(maxFeatures); WRITE_FIELD(minParallaxDeg);
    WRITE_FIELD(minTriObs); WRITE_FIELD(reorderWindowNs); WRITE_FIELD(initPosSigma); WRITE_FIELD(initYawSigmaDeg);
    WRITE_FIELD(initVelSigma); WRITE_FIELD(initBiasGSigma); WRITE_FIELD(initBiasASigma); WRITE_FIELD(fastThreshold);
    WRITE_FIELD(maxCellsX); WRITE_FIELD(maxCellsY); WRITE_FIELD(maxPerCell);
#undef WRITE_FIELD
    j["server"]={{"enabled",f.serverEnabled},{"host",f.serverHost},{"port",f.serverPort},
        {"certPath",f.certPath},{"keyPath",f.keyPath},{"webRoot",f.webRoot}};
    return j.dump();
}
} // namespace gvio
