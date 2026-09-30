#include "gvio/config.h"

namespace gvio {

Config Config::fromJson(const std::string& jsonStr) {
    Config c;
    try {
        Json j = Json::parse(jsonStr);
        auto& f = c.filter;

        if (j.contains("camera")) {
            auto& cam = j["camera"];
            f.cam.width = cam.value("width", f.cam.width);
            f.cam.height = cam.value("height", f.cam.height);
            f.cam.fx = cam.value("fx", f.cam.fx);
            f.cam.fy = cam.value("fy", f.cam.fy);
            f.cam.cx = cam.value("cx", f.cam.cx);
            f.cam.cy = cam.value("cy", f.cam.cy);
            if (cam.contains("intrinsics") && cam["intrinsics"].size() >= 4) {
                auto& k = cam["intrinsics"];
                f.cam.fx = k[0].get<double>();
                f.cam.fy = k[1].get<double>();
                f.cam.cx = k[2].get<double>();
                f.cam.cy = k[3].get<double>();
            }
            if (cam.contains("distortion") && cam["distortion"].size() >= 5) {
                auto& d = cam["distortion"];
                f.cam.k1 = d[0].get<double>();
                f.cam.k2 = d[1].get<double>();
                f.cam.p1 = d[2].get<double>();
                f.cam.p2 = d[3].get<double>();
                f.cam.k3 = d[4].get<double>();
                f.cam.hasDistortion = true;
            }
        }

        if (j.contains("imu")) {
            auto& imu = j["imu"];
            f.sigmaG = imu.value("gyroNoise", f.sigmaG);
            f.sigmaA = imu.value("accelNoise", f.sigmaA);
            f.sigmaBg = imu.value("gyroBiasWalk", f.sigmaBg);
            f.sigmaBa = imu.value("accelBiasWalk", f.sigmaBa);
        }

        if (j.contains("mag")) {
            auto& m = j["mag"];
            f.magEnabled = m.value("enabled", f.magEnabled);
            f.magDeclinationDeg = m.value("declinationDeg", f.magDeclinationDeg);
            f.magInclinationDeg = m.value("inclinationDeg", f.magInclinationDeg);
            f.magFieldUT = m.value("fieldUT", f.magFieldUT);
            f.magYawNoiseRad = m.value("yawNoiseRad", f.magYawNoiseRad);
        }

        if (j.contains("gps")) {
            auto& g = j["gps"];
            f.gpsEnabled = g.value("enabled", f.gpsEnabled);
            f.gpsMinSats = g.value("minSats", f.gpsMinSats);
        }

        if (j.contains("filter")) {
            auto& ft = j["filter"];
            f.cameraWindowSize = ft.value("cameraWindowSize", f.cameraWindowSize);
            f.featureNoisePx = ft.value("featureNoisePx", f.featureNoisePx);
            f.maxFeatures = ft.value("maxFeatures", f.maxFeatures);
            f.minParallaxDeg = ft.value("minParallaxDeg", f.minParallaxDeg);
            f.minTriObs = ft.value("minTriObs", f.minTriObs);
        }

        if (j.contains("server")) {
            auto& s = j["server"];
            f.serverHost = s.value("host", f.serverHost);
            f.serverPort = s.value("port", f.serverPort);
            f.certPath = s.value("certPath", f.certPath);
            f.keyPath = s.value("keyPath", f.keyPath);
            f.webRoot = s.value("webRoot", f.webRoot);
        }

        c.valid = true;
    } catch (const std::exception&) {
        c.valid = false;
    }
    return c;
}

std::string Config::toJson() const {
    Json j;
    auto& f = filter;
    j["camera"] = {
        {"width", f.cam.width}, {"height", f.cam.height},
        {"fx", f.cam.fx}, {"fy", f.cam.fy}, {"cx", f.cam.cx}, {"cy", f.cam.cy},
        {"hasDistortion", f.cam.hasDistortion}
    };
    j["filter"] = {
        {"windowSize", f.cameraWindowSize},
        {"featureNoisePx", f.featureNoisePx},
        {"maxFeatures", f.maxFeatures}
    };
    j["server"] = {{"port", f.serverPort}};
    return j.dump();
}

}  // namespace gvio
