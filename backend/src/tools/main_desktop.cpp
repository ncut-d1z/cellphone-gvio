// 桌面端验证工具: 合成数据 1x 实时送入 FusionEngine, 打印状态并对比真值。
// 用法: ./gvio_desktop [durationSec=60]
#include "gvio/config.h"
#include "gvio/fusion.h"

#include "synthetic.h"

#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

using namespace gvio;

int main(int argc, char** argv) {
    double duration = 60.0;
    if (argc > 1) duration = std::atof(argv[1]);

    Config cfg;
    auto& f = cfg.filter;
    f.cam.width = 320;
    f.cam.height = 240;
    f.cam.fx = 250.0;
    f.cam.fy = 250.0;
    f.cam.cx = 160.0;
    f.cam.cy = 120.0;
    f.cam.imuToCamQ = Quat::Identity();
    f.cam.imuToCamP = Vec3(0.03, -0.01, 0.02);
    f.cameraWindowSize = 20;
    f.featureNoisePx = 1.5;
    f.maxFeatures = 200;
    f.minParallaxDeg = 1.0;
    f.minTriObs = 3;
    f.fastThreshold = 25;
    f.sigmaG = 3.0e-4;
    f.sigmaA = 2.0e-2;
    f.sigmaBg = 1.0e-6;
    f.sigmaBa = 1.0e-4;
    f.magEnabled = true;
    f.magDeclinationDeg = 0.0;
    f.magInclinationDeg = 60.0;
    f.magFieldUT = 50.0;
    f.magYawNoiseRad = 0.087;
    f.gpsEnabled = true;
    f.gpsMinSats = 4;
    f.initPosSigma = 5.0;
    f.initYawSigmaDeg = 10.0;
    f.initVelSigma = 1.0;
    f.initBiasGSigma = 1e-2;
    f.initBiasASigma = 0.1;
    f.serverHost = "127.0.0.1";
    f.serverPort = 8443;
    cfg.valid = true;

    SynthWorld world(320, 240, f.cam);
    world.generate(0.005, 0.1, 0.02, 1.0, duration, 31.2304, 121.4737, 5.0);
    std::cout << "synthetic: imu=" << world.imu().size() << " mag=" << world.mag().size()
              << " gps=" << world.gps().size() << " img=" << world.imgs().size()
              << std::endl;

    FusionEngine engine(cfg);
    engine.setClockOffset(0.0);
    if (!engine.start()) {
        std::cerr << "engine start failed" << std::endl;
        return 2;
    }

    std::vector<int> consts(8, 1);
    std::vector<float> cn0(8, 40.0f);

    auto t0 = std::chrono::steady_clock::now();
    size_t ii = 0, im = 0, ig = 0, ip = 0;
    double lastPrint = -10.0;
    double simEnd = world.imu().back().tNs * 1e-9;

    while (ii < world.imu().size()) {
        double nowT = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
                          .count();
        if (nowT > simEnd + 1.0) break;

        while (ii < world.imu().size() && world.imu()[ii].tNs * 1e-9 <= nowT) {
            const auto& s = world.imu()[ii];
            engine.feedImu(s.tNs, s.values, s.biases);
            ii++;
        }
        while (im < world.mag().size() && world.mag()[im].tNs * 1e-9 <= nowT) {
            const auto& s = world.mag()[im];
            engine.feedMag(s.tNs, s.values, s.biases);
            im++;
        }
        while (ig < world.gps().size() && world.gps()[ig].tNs * 1e-9 <= nowT) {
            const auto& s = world.gps()[ig];
            engine.feedGpsFix(s.tNs, s.lat, s.lon, s.alt, s.accH, 0.0f, 0.0f);
            engine.feedGnssSats(s.tNs, consts.data(), cn0.data(), 8);
            ig++;
        }
        while (ip < world.imgs().size() && world.imgs()[ip].tNs * 1e-9 <= nowT) {
            const auto& fr = world.imgs()[ip];
            engine.feedImage(fr.tNs, 0, 100, fr.width, fr.height, fr.width, fr.y.data(),
                             fr.y.size());
            ip++;
        }

        if (nowT - lastPrint >= 5.0) {
            lastPrint = nowT;
            try {
                std::string raw = engine.statusJson();
                Json j = Json::parse(raw);
                double tSim = j.value("t", 0.0) * 1e-9;
                std::string mode = j.value("mode", "?");
                if (mode == "running") {
                    double yawDeg = j["pose"].value("yawDeg", 0.0);
                    Vec3 est(j["pose"]["x"], j["pose"]["y"], j["pose"]["z"]);
                    Vec3 tru = world.pTrue(tSim);
                    std::cout << "t=" << tSim << " mode=running yaw_est=" << yawDeg
                              << " yaw_true=" << world.yawTrue(tSim) << " |pos_err|="
                              << (est - tru).norm() << "m tracks=" << j.value("tracks", 0)
                              << " win=" << j.value("window", 0)
                              << " updates=" << j["stats"].value("updates", 0)
                              << " imuHz=" << j["stats"].value("imuHz", 0.0) << std::endl;
                } else {
                    std::cout << "t=" << tSim << " mode=" << mode
                              << " accelS=" << j.value("accelSamples", 0)
                              << " magS=" << j.value("magSamples", 0)
                              << " hasOrigin=" << j.value("hasOrigin", false) << std::endl;
                }
            } catch (const std::exception& e) {
                std::cerr << "[print] exception: " << e.what() << std::endl;
                std::cerr << "statusJson=" << engine.statusJson() << std::endl;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    engine.stop();

    // 最终对比
    double tEnd = world.imu().back().tNs * 1e-9;
    Json j = Json::parse(engine.statusJson());
    bool ok = j.value("mode", "") == "running";
    if (ok) {
        double tSim = j.value("t", 0.0) * 1e-9;
        Vec3 est(j["pose"]["x"], j["pose"]["y"], j["pose"]["z"]);
        Vec3 tru = world.pTrue(tSim);
        double posErr = (est - tru).norm();
        double yawErr = std::abs(j["pose"].value("yawDeg", 0.0) - world.yawTrue(tSim));
        std::cout << "\nfinal t=" << tSim << "/" << tEnd << std::endl;
        std::cout << "  pos  est=(" << est.x() << "," << est.y() << "," << est.z()
                  << ") true=(" << tru.x() << "," << tru.y() << "," << tru.z()
                  << ") err=" << posErr << "m" << std::endl;
        std::cout << "  yaw  est=" << j["pose"].value("yawDeg", 0.0)
                  << " true=" << world.yawTrue(tSim) << " err=" << yawErr << "deg"
                  << std::endl;
        std::cout << "  tracks=" << j.value("tracks", 0) << " window=" << j.value("window", 0)
                  << " updates=" << j["stats"].value("updates", 0)
                  << " features=" << j["features"].size() << std::endl;
        std::cout << "  biasG=(" << j["calib"]["bg"][0] << "," << j["calib"]["bg"][1] << ","
                  << j["calib"]["bg"][2] << ") true=(0.002,-0.001,0.004)" << std::endl;
        std::cout << "  biasA=(" << j["calib"]["ba"][0] << "," << j["calib"]["ba"][1] << ","
                  << j["calib"]["ba"][2] << ") true=(0.02,-0.03,0.05)" << std::endl;
        ok = posErr < 3.0 && yawErr < 8.0 && j.value("tracks", 0) > 10;
    }
    std::cout << (ok ? "PASS" : "FAIL") << std::endl;
    return ok ? 0 : 1;
}
