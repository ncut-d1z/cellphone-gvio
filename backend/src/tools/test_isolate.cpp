// 调试隔离: 逐阶段喂数据, 定位挂起点
#include "gvio/config.h"
#include "gvio/fusion.h"
#include "synthetic.h"

#include <chrono>
#include <iostream>
#include <thread>

using namespace gvio;

static void tick(const char* tag) {
    std::cout << tag << " @ " << std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count() << std::endl;
}

int main() {
    Config cfg;
    auto& f = cfg.filter;
    f.cam.width = 320; f.cam.height = 240;
    f.cam.fx = 250; f.cam.fy = 250; f.cam.cx = 160; f.cam.cy = 120;
    f.cam.imuToCamP = Vec3(0.03, -0.01, 0.02);
    f.cameraWindowSize = 20;
    f.maxFeatures = 200;
    f.magEnabled = true; f.gpsEnabled = true;
    cfg.valid = true;

    SynthWorld world(320, 240, f.cam);
    world.generate(0.005, 0.1, 0.02, 1.0, 8.0, 31.2304, 121.4737, 5.0);

    tick("start");
    FusionEngine engine(cfg);
    engine.setClockOffset(0.0);
    tick("before engine.start");
    if (!engine.start()) { std::cout << "ENGINE START FAILED" << std::endl; return 2; }
    tick("engine started");

    std::vector<int> consts(8, 1);
    std::vector<float> cn0(8, 40.0f);

    // 阶段1: 只喂 IMU
    for (size_t i = 0; i < 100; ++i) {
        const auto& s = world.imu()[i];
        engine.feedImu(s.tNs, s.values, s.biases);
    }
    tick("fed 100 imu");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "status: " << engine.statusJson() << std::endl;

    // 阶段2: IMU + mag + gps 到 1s
    for (size_t i = 100; i < 200; ++i) {
        const auto& s = world.imu()[i];
        engine.feedImu(s.tNs, s.values, s.biases);
    }
    for (size_t i = 0; i < world.mag().size() && world.mag()[i].tNs <= 1e9; ++i) {
        const auto& s = world.mag()[i];
        engine.feedMag(s.tNs, s.values, s.biases);
    }
    for (size_t i = 1; i < 3; ++i) {
        const auto& s = world.gps()[i];
        engine.feedGpsFix(s.tNs, s.lat, s.lon, s.alt, s.accH, 0, 0);
        engine.feedGnssSats(s.tNs, consts.data(), cn0.data(), 8);
    }
    tick("fed imu+mag+gps 1s");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "status: " << engine.statusJson() << std::endl;

    // 阶段3: 逐帧图像 + 时间同步的 IMU/磁力计/GPS
    size_t ii = 200, im = 0, ig = 3;
    while (im < world.mag().size() && world.mag()[im].tNs <= 1e9) im++;
    size_t ip0 = 0;
    while (ip0 < world.imgs().size() && world.imgs()[ip0].tNs <= 1e9) ip0++;
    for (size_t i = ip0; i < world.imgs().size(); ++i) {
        const auto& fr = world.imgs()[i];
        int64_t upTo = fr.tNs;
        while (ii < world.imu().size() && world.imu()[ii].tNs < upTo) {
            const auto& s = world.imu()[ii];
            engine.feedImu(s.tNs, s.values, s.biases);
            ii++;
        }
        while (im < world.mag().size() && world.mag()[im].tNs <= upTo) {
            const auto& s = world.mag()[im];
            engine.feedMag(s.tNs, s.values, s.biases);
            im++;
        }
        while (ig < world.gps().size() && world.gps()[ig].tNs <= upTo) {
            const auto& s = world.gps()[ig];
            engine.feedGpsFix(s.tNs, s.lat, s.lon, s.alt, s.accH, 0, 0);
            engine.feedGnssSats(s.tNs, consts.data(), cn0.data(), 8);
            ig++;
        }
        engine.feedImage(fr.tNs, 0, 100, fr.width, fr.height, fr.width, fr.y.data(), fr.y.size());
        tick((std::string("fed img ") + std::to_string(i)).c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        if (i % 5 == 0) std::cout << "s: " << engine.statusJson() << std::endl;
    }
    engine.stop();
    std::cout << "DONE" << std::endl;
    return 0;
}
