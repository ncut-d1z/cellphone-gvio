#pragma once

#include "gvio/types.h"

namespace gvio {

// 磁力计硬磁校准: 采集样本, 球面拟合估计零偏(硬铁)。
// 软磁(尺度/非正交)为后续扩展, 当前仅估计平移偏置。
class MagCalibrator {
public:
    enum class State { Idle, Collecting, Done };

    void start() {
        state_ = State::Collecting;
        samples_.clear();
    }

    void addSample(const Vec3& m, int64_t t) {
        if (state_ != State::Collecting) return;
        samples_.push_back(m);
        if (static_cast<int>(samples_.size()) >= target_) state_ = State::Done;
    }

    // 球面最小二乘: ||m - b||² = r²
    void finish() {
        if (state_ == State::Done || samples_.size() >= 6) {
            fit();
            state_ = State::Done;
        }
    }

    const Vec3& bias() const { return bias_; }
    double radius() const { return radius_; }
    State state() const { return state_; }
    int collected() const { return static_cast<int>(samples_.size()); }

    bool isIdle() const { return state_ == State::Idle; }
    bool isCollecting() const { return state_ == State::Collecting; }
    bool isDone() const { return state_ == State::Done; }

private:
    void fit() {
        int n = static_cast<int>(samples_.size());
        if (n < 9) return;
        Eigen::MatrixXd A(n, 4);
        VecXd b(n);
        for (int i = 0; i < n; ++i) {
            const Vec3& m = samples_[i];
            A.row(i) << -2.0 * m.x(), -2.0 * m.y(), -2.0 * m.z(), 1.0;
            b(i) = -m.squaredNorm();
        }
        VecXd x = A.colPivHouseholderQr().solve(b);
        bias_ = Vec3(x(0), x(1), x(2));
        double c = x(3);
        double r2 = bias_.squaredNorm() - c;
        radius_ = r2 > 0 ? std::sqrt(r2) : 0.0;
    }

    State state_ = State::Idle;
    std::vector<Vec3> samples_;
    int target_ = 200;
    Vec3 bias_ = Vec3::Zero();
    double radius_ = 0.0;
};

}  // namespace gvio
