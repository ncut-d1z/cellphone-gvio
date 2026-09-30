#pragma once
#include "gvio/types.h"
#include <algorithm>
#include <functional>
#include <map>
#include <tuple>
#include <utility>

namespace gvio {
// Single-writer bounded reorder buffer. Arrival time NEVER replaces acquisition
// time. The owner transfers input under its queue lock, then drains without it.
class SensorTimeline {
public:
    explicit SensorTimeline(int64_t delayNs=50000000):delay_(std::max<int64_t>(0,delayNs)) {}
    uint64_t late=0,overflow=0,invalid=0,unbracketed=0;
    bool pushImu(const ImuSample& s) {
        if (s.t<0 || !s.accel.allFinite() || !s.gyro.allFinite()) { ++invalid; return false; }
        if ((haveLast_ && s.t<=last_.t) || imus_.count(s.t)) { ++late; return false; }
        if (imus_.size()>=4096) { ++overflow; return false; }
        imus_.emplace(s.t,s); latest_=std::max(latest_,s.t); return true;
    }
    bool pushObservation(int64_t t,int priority,std::function<void()> apply) {
        // At the committed watermark, only observations already in the queue are
        // ordered deterministically. New equal-time observations are too late.
        if (t<0) { ++invalid; return false; }
        if (haveLast_ && t<=committed_) { ++late; return false; }
        if (observations_.size()>=256) { ++overflow; return false; }
        observations_.emplace(std::make_tuple(t,priority,sequence_++),std::move(apply)); return true;
    }
    void drain(const std::function<void(const ImuSample&)>& onImu,bool final=false) {
        int64_t watermark=final ? latest_ : latest_-delay_;
        if (!haveLast_) {
            if (imus_.empty() || imus_.begin()->first>watermark) {
                if (final) { unbracketed+=observations_.size(); observations_.clear(); }
                return;
            }
            last_=imus_.begin()->second; imus_.erase(imus_.begin()); haveLast_=true;
            onImu(last_);
        }
        while (true) {
            bool hasObs=!observations_.empty() && std::get<0>(observations_.begin()->first)<=watermark;
            bool hasImu=!imus_.empty() && imus_.begin()->first<=watermark;
            if (!hasObs && !hasImu) break;
            if (hasImu && (!hasObs || imus_.begin()->first<=std::get<0>(observations_.begin()->first))) {
                last_=imus_.begin()->second; imus_.erase(imus_.begin()); onImu(last_); continue;
            }
            auto ob=observations_.begin(); int64_t t=std::get<0>(ob->first);
            if (t<last_.t) { ++late; observations_.erase(ob); continue; }
            if (t>last_.t) {
                if (imus_.empty()) break; // do not extrapolate past final IMU
                const auto& right=imus_.begin()->second;
                double alpha=double(t-last_.t)/double(right.t-last_.t);
                ImuSample mid; mid.t=t; mid.gyro=(1.-alpha)*last_.gyro+alpha*right.gyro;
                mid.accel=(1.-alpha)*last_.accel+alpha*right.accel;
                onImu(mid); last_=mid;
            }
            auto apply=std::move(ob->second); observations_.erase(ob); apply();
        }
        // Commit only time covered by processed IMU, not arbitrary future time.
        committed_=last_.t;
        if (final) { unbracketed+=observations_.size(); observations_.clear(); }
    }
    void clear() {
        imus_.clear(); observations_.clear(); haveLast_=false;
        last_=ImuSample{}; committed_=-1; latest_=-1; sequence_=0;
    }
    size_t queued() const { return imus_.size()+observations_.size(); }
    int64_t time() const { return haveLast_?last_.t:-1; }
private:
    int64_t delay_,latest_=-1,committed_=-1;
    uint64_t sequence_=0;
    bool haveLast_=false;
    ImuSample last_;
    std::map<int64_t,ImuSample> imus_;
    std::map<std::tuple<int64_t,int,uint64_t>,std::function<void()>> observations_;
};
} // namespace gvio
