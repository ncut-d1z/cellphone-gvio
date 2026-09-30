#include "gvio/vision.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace gvio {

void grayFromY(const uint8_t* y, int width, int height, int stride,
               std::vector<uint8_t>& out) {
    out.resize(static_cast<size_t>(width) * height);
    for (int r = 0; r < height; ++r) {
        std::memcpy(out.data() + static_cast<size_t>(r) * width,
                    y + static_cast<size_t>(r) * stride,
                    static_cast<size_t>(width));
    }
}

// ---------------------------------------------------------------------------
// FAST-9
// ---------------------------------------------------------------------------
namespace {

// Bresenham 16 点圆(半径3), 坐标偏移
const int kCircleDx[16] = {0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2, -1};
const int kCircleDy[16] = {-3, -3, -2, -1, 0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3};

int fastCornerScore(const uint8_t* g, int stride, int x, int y, int threshold) {
    const uint8_t center = g[y * stride + x];
    const int lo = center - threshold;
    const int hi = center + threshold;
    int start = -1, n = 0;
    // 找到最长的连续 9 段亮/暗
    int best = 0;
    for (int pass = 0; pass < 2; ++pass) {
        bool bright = (pass == 0);
        for (int i = 0; i < 16; ++i) {
            int idx = (start + i + 16) % 16;
            const uint8_t v = g[(y + kCircleDy[idx]) * stride + (x + kCircleDx[idx])];
            bool ok = bright ? (v >= hi) : (v <= lo);
            if (ok) {
                n++;
                if (n > best) best = n;
            } else {
                if (start < 0) start = i;
                if (n > best) best = n;
                n = 0;
            }
        }
    }
    if (best < 9) return 0;
    // 得分: 连续弧段与中心的绝对差之和
    int score = 0;
    for (int i = 0; i < 16; ++i) {
        const uint8_t v = g[(y + kCircleDy[i]) * stride + (x + kCircleDx[i])];
        int d = std::abs(int(v) - int(center));
        score += d;
    }
    return score;
}

}  // namespace

void detectFast(const uint8_t* gray, int width, int height, int threshold,
                std::vector<Keypoint>& kps, int maxFeatures,
                int cellsX, int cellsY, int maxPerCell) {
    kps.clear();
    if (width < 16 || height < 16 || maxFeatures <= 0) return;
    const int margin = 4;

    std::vector<Keypoint> all;
    all.reserve(static_cast<size_t>(maxFeatures) * 2);

    int cellW = (width - 2 * margin) / cellsX;
    int cellH = (height - 2 * margin) / cellsY;
    if (cellW < 8 || cellH < 8) return;

    std::vector<std::vector<Keypoint>> cells(static_cast<size_t>(cellsX) * cellsY);

    for (int cy = 0; cy < cellsY; ++cy) {
        for (int cx = 0; cx < cellsX; ++cx) {
            int x0 = margin + cx * cellW, x1 = margin + (cx + 1) * cellW;
            int y0 = margin + cy * cellH, y1 = margin + (cy + 1) * cellH;
            auto& cell = cells[static_cast<size_t>(cy) * cellsX + cx];
            for (int y = y0; y < y1; ++y) {
                for (int x = x0; x < x1; ++x) {
                    const uint8_t c = gray[y * width + x];
                    const int lo = c - threshold, hi = c + threshold;
                    // 快速预检: 圆上 4 个点至少 3 个同时亮/暗
                    const uint8_t p0 = gray[(y - 3) * width + x];
                    const uint8_t p4 = gray[y * width + x + 3];
                    const uint8_t p8 = gray[(y + 3) * width + x];
                    const uint8_t p12 = gray[y * width + x - 3];
                    int bright = (p0 >= hi) + (p4 >= hi) + (p8 >= hi) + (p12 >= hi);
                    int dark = (p0 <= lo) + (p4 <= lo) + (p8 <= lo) + (p12 <= lo);
                    if (bright < 3 && dark < 3) continue;
                    int score = fastCornerScore(gray, width, x, y, threshold);
                    if (score > 0) {
                        Keypoint kp;
                        kp.x = static_cast<float>(x);
                        kp.y = static_cast<float>(y);
                        kp.score = static_cast<float>(score);
                        cell.push_back(kp);
                    }
                }
            }
            // 每格按得分取前 maxPerCell
            std::sort(cell.begin(), cell.end(),
                      [](const Keypoint& a, const Keypoint& b) { return a.score > b.score; });
            if (static_cast<int>(cell.size()) > maxPerCell) cell.resize(maxPerCell);
            for (auto& kp : cell) all.push_back(kp);
        }
    }

    // 全局得分排序 + 非极大抑制(3x3)
    std::sort(all.begin(), all.end(),
              [](const Keypoint& a, const Keypoint& b) { return a.score > b.score; });

    std::vector<uint8_t> occupied(static_cast<size_t>(width) * height, 0);
    kps.reserve(static_cast<size_t>(maxFeatures));
    for (auto& kp : all) {
        int x = static_cast<int>(kp.x), y = static_cast<int>(kp.y);
        if (occupied[static_cast<size_t>(y) * width + x]) continue;
        kps.push_back(kp);
        if (static_cast<int>(kps.size()) >= maxFeatures) break;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int yy = y + dy, xx = x + dx;
                if (xx >= 0 && xx < width && yy >= 0 && yy < height)
                    occupied[static_cast<size_t>(yy) * width + xx] = 1;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 金字塔 KLT
// ---------------------------------------------------------------------------
namespace {

struct Pyramid {
    std::vector<std::vector<uint8_t>> levels;
    std::vector<int> widths, heights;
    int nlevels;
};

void buildPyramid(const std::vector<uint8_t>& gray, int width, int height,
                  Pyramid& pyr, int nlevels) {
    pyr.nlevels = nlevels;
    pyr.widths.resize(nlevels);
    pyr.heights.resize(nlevels);
    pyr.levels.resize(nlevels);
    pyr.levels[0] = gray;
    pyr.widths[0] = width;
    pyr.heights[0] = height;
    for (int l = 1; l < nlevels; ++l) {
        int pw = pyr.widths[l - 1], ph = pyr.heights[l - 1];
        int nw = (pw + 1) / 2, nh = (ph + 1) / 2;
        pyr.widths[l] = nw;
        pyr.heights[l] = nh;
        pyr.levels[l].resize(static_cast<size_t>(nw) * nh);
        const uint8_t* src = pyr.levels[l - 1].data();
        uint8_t* dst = pyr.levels[l].data();
        for (int y = 0; y < nh; ++y) {
            for (int x = 0; x < nw; ++x) {
                int sx = std::min(x * 2, pw - 1), sy = std::min(y * 2, ph - 1);
                // 2x2 平均
                int v = src[sy * pw + sx] + src[sy * pw + std::min(sx + 1, pw - 1)] +
                        src[std::min(sy + 1, ph - 1) * pw + sx] +
                        src[std::min(sy + 1, ph - 1) * pw + std::min(sx + 1, pw - 1)];
                dst[y * nw + x] = static_cast<uint8_t>(v >> 2);
            }
        }
    }
}

inline uint8_t sampleBilinear(const uint8_t* img, int w, int h, float x, float y) {
    int x0 = static_cast<int>(x), y0 = static_cast<int>(y);
    if (x0 < 0 || y0 < 0 || x0 >= w - 1 || y0 >= h - 1) return 0;
    float dx = x - x0, dy = y - y0;
    float v = img[y0 * w + x0] * (1 - dx) * (1 - dy) +
              img[y0 * w + x0 + 1] * dx * (1 - dy) +
              img[(y0 + 1) * w + x0] * (1 - dx) * dy +
              img[(y0 + 1) * w + x0 + 1] * dx * dy;
    return static_cast<uint8_t>(v + 0.5f);
}

bool lkIterate(const uint8_t* prev, const uint8_t* cur, int w, int h,
               float& ux, float& uy, int halfWin, float maxDisp) {
    const int win = 2 * halfWin + 1;
    // 预计算上一帧窗口梯度(需要 ±1 邻域)
    float gxx = 0, gyy = 0, gxy = 0;
    int x0 = static_cast<int>(ux), y0 = static_cast<int>(uy);
    if (x0 - halfWin - 1 < 0 || y0 - halfWin - 1 < 0 || x0 + halfWin + 1 >= w ||
        y0 + halfWin + 1 >= h)
        return false;
    for (int dy = -halfWin; dy <= halfWin; ++dy) {
        const uint8_t* row = prev + (y0 + dy) * w + (x0 - halfWin);
        for (int dx = -halfWin; dx <= halfWin; ++dx) {
            int ix = row[dx + 1] - row[dx - 1];
            int iy = row[dx + w] - row[dx - w];
            gxx += static_cast<float>(ix * ix);
            gyy += static_cast<float>(iy * iy);
            gxy += static_cast<float>(ix * iy);
        }
    }
    float det = gxx * gyy - gxy * gxy;
    if (det < 1e-6f) return false;
    det = 1.0f / det;

    float dx = 0, dy = 0;
    for (int iter = 0; iter < 30; ++iter) {
        float ex = 0, ey = 0;
        float bx = ux + dx, by = uy + dy;
        int bx0 = static_cast<int>(bx), by0 = static_cast<int>(by);
        if (bx0 - halfWin < 0 || by0 - halfWin < 0 ||
            bx0 + halfWin >= w || by0 + halfWin >= h)
            return false;
        // 迭代中 prev 梯度取自固定窗口(x0,y0), 边界检查已包含 ±1
        if (x0 + halfWin + 1 >= w || y0 + halfWin + 1 >= h) return false;
        for (int j = -halfWin; j <= halfWin; ++j) {
            for (int i = -halfWin; i <= halfWin; ++i) {
                float it = sampleBilinear(cur, w, h, bx + i, by + j);
                float ip = prev[(y0 + j) * w + (x0 + i)];
                ex += (it - ip) * static_cast<float>(prev[(y0 + j) * w + (x0 + i + 1)] -
                                                     prev[(y0 + j) * w + (x0 + i - 1)]);
                ey += (it - ip) * static_cast<float>(prev[(y0 + j + 1) * w + (x0 + i)] -
                                                     prev[(y0 + j - 1) * w + (x0 + i)]);
            }
        }
        float nx = (gyy * ex - gxy * ey) * det;
        float ny = (gxx * ey - gxy * ex) * det;
        dx += nx;
        dy += ny;
        if (nx * nx + ny * ny < 0.01f * 0.01f) break;
    }
    if (std::abs(dx) > maxDisp || std::abs(dy) > maxDisp) return false;
    ux += dx;
    uy += dy;
    return true;
}

}  // namespace

int trackKlt(const std::vector<uint8_t>& grayPrev, const std::vector<uint8_t>& grayCur,
             int width, int height, const std::vector<Keypoint>& ptsIn,
             std::vector<Keypoint>& ptsOut, std::vector<uint8_t>& status) {
    const int nlevels = 3;
    Pyramid pa, pb;
    buildPyramid(grayPrev, width, height, pa, nlevels);
    buildPyramid(grayCur, width, height, pb, nlevels);

    ptsOut.resize(ptsIn.size());
    status.assign(ptsIn.size(), 0);

    int nOk = 0;
    for (size_t i = 0; i < ptsIn.size(); ++i) {
        float x = ptsIn[i].x, y = ptsIn[i].y;
        bool ok = true;
        for (int l = nlevels - 1; l >= 0; --l) {
            float sx = x / static_cast<float>(1 << l);
            float sy = y / static_cast<float>(1 << l);
            float maxDisp = (l > 0) ? 30.0f : 3.0f;  // 粗层允许较大位移
            if (!lkIterate(pa.levels[l].data(), pb.levels[l].data(),
                           pa.widths[l], pa.heights[l], sx, sy, 7, maxDisp)) {
                ok = false;
                break;
            }
            if (l > 0) { sx *= 2; sy *= 2; }  // 下一层初始位置
            x = sx;
            y = sy;
        }
        if (ok && x > 2 && y > 2 && x < width - 3 && y < height - 3) {
            Keypoint kp;
            kp.x = x;
            kp.y = y;
            kp.score = ptsIn[i].score;
            ptsOut[i] = kp;
            status[i] = 1;
            nOk++;
        }
    }
    return nOk;
}

}  // namespace gvio
