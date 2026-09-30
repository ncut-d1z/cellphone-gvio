#pragma once

#include "gvio/types.h"
#include <cstdint>
#include <vector>

namespace gvio {

// Y 平面 -> 灰度图(stride 可能大于 width)
void grayFromY(const uint8_t* y, int width, int height, int stride, std::vector<uint8_t>& out);

struct Keypoint {
    float x = 0, y = 0;
    float score = 0;
};

// FAST-9 角点检测 + 网格分布 + 非极大抑制
void detectFast(const uint8_t* gray, int width, int height, int threshold,
                std::vector<Keypoint>& kps, int maxFeatures,
                int cellsX = 8, int cellsY = 6, int maxPerCell = 30);

// 金字塔 KLT 稀疏光流。ptsIn 输入上一帧坐标(浮点)，返回当前帧坐标与丢失标记。
// 返回跟踪成功的点数量。
int trackKlt(const std::vector<uint8_t>& grayPrev, const std::vector<uint8_t>& grayCur,
             int width, int height, const std::vector<Keypoint>& ptsIn,
             std::vector<Keypoint>& ptsOut, std::vector<uint8_t>& status);

}  // namespace gvio
