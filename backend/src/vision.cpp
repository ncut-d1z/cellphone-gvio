#include "gvio/vision.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
namespace gvio {
namespace {
bool dimensions(int w,int h) { return w>0 && h>0 && w<=16384 && h<=16384; }
constexpr int cx[16]={0,1,2,3,3,3,2,1,0,-1,-2,-3,-3,-3,-2,-1};
constexpr int cy[16]={-3,-3,-2,-1,0,1,2,3,3,3,2,1,0,-1,-2,-3};
struct Level { int w,h; std::vector<float> pixels; };
float sample(const Level& im,float x,float y) {
    int ix=int(std::floor(x)),iy=int(std::floor(y)); float u=x-ix,v=y-iy;
    return (1-v)*((1-u)*im.pixels[iy*im.w+ix]+u*im.pixels[iy*im.w+ix+1])+
           v*((1-u)*im.pixels[(iy+1)*im.w+ix]+u*im.pixels[(iy+1)*im.w+ix+1]);
}
bool inside(const Level& im,float x,float y,int margin) {
    return std::isfinite(x) && std::isfinite(y) && x>=margin && y>=margin &&
           x<float(im.w-margin-1) && y<float(im.h-margin-1);
}
std::vector<Level> pyramid(const std::vector<uint8_t>& data,int w,int h) {
    std::vector<Level> p; p.push_back({w,h,std::vector<float>(data.begin(),data.end())});
    constexpr int kernel[5]={1,4,6,4,1};
    while (p.size()<3 && p.back().w>=40 && p.back().h>=40) {
        const auto& a=p.back(); int nw=(a.w+1)/2,nh=(a.h+1)/2;
        Level b{nw,nh,std::vector<float>(size_t(nw)*nh)};
        for (int y=0;y<nh;++y) for (int x=0;x<nw;++x) {
            float sum=0;
            for (int j=-2;j<=2;++j) for (int i=-2;i<=2;++i)
                sum+=kernel[i+2]*kernel[j+2]*a.pixels[std::clamp(2*y+j,0,a.h-1)*a.w+std::clamp(2*x+i,0,a.w-1)];
            b.pixels[y*nw+x]=sum/256.f;
        }
        p.push_back(std::move(b));
    }
    return p;
}
// Inverse-compositional LK: reference point NEVER moves with the target estimate.
bool lk(const Level& a,const Level& b,float px,float py,float& qx,float& qy) {
    constexpr int half=5,n=121;
    if (!inside(a,px,py,half+1)) return false;
    std::array<float,n> ref{},gx{},gy{}; int k=0;
    double xx=0,xy=0,yy=0;
    for (int j=-half;j<=half;++j) for (int i=-half;i<=half;++i,++k) {
        ref[k]=sample(a,px+i,py+j);
        gx[k]=.5f*(sample(a,px+i+1,py+j)-sample(a,px+i-1,py+j));
        gy[k]=.5f*(sample(a,px+i,py+j+1)-sample(a,px+i,py+j-1));
        xx+=gx[k]*gx[k]; xy+=gx[k]*gy[k]; yy+=gy[k]*gy[k];
    }
    double det=xx*yy-xy*xy;
    double eig=.5*(xx+yy-std::hypot(xx-yy,2.*xy));
    if (eig/n<1. || det<1e-9) return false;
    bool converged=false;
    for (int it=0;it<40;++it) {
        if (!inside(b,qx,qy,half)) return false;
        double ex=0,ey=0; k=0;
        for (int j=-half;j<=half;++j) for (int i=-half;i<=half;++i,++k) {
            double r=sample(b,qx+i,qy+j)-ref[k]; ex+=gx[k]*r; ey+=gy[k]*r;
        }
        double dx=-(yy*ex-xy*ey)/det,dy=-(xx*ey-xy*ex)/det;
        if (!std::isfinite(dx) || !std::isfinite(dy) || std::abs(dx)>10. || std::abs(dy)>10.) return false;
        qx+=float(dx); qy+=float(dy);
        if (dx*dx+dy*dy<1e-4) { converged=true; break; }
    }
    if (!converged || !inside(b,qx,qy,half)) return false;
    double err=0; k=0;
    for (int j=-half;j<=half;++j) for (int i=-half;i<=half;++i,++k) {
        double r=sample(b,qx+i,qy+j)-ref[k]; err+=r*r;
    }
    return err/n<30.*30.;
}
bool trackOne(const std::vector<Level>& a,const std::vector<Level>& b,float px,float py,float& qx,float& qy) {
    int top=int(a.size())-1;
    while (top>0 && !inside(a[top],px/float(1<<top),py/float(1<<top),6)) --top;
    qx=px/float(1<<top); qy=py/float(1<<top);
    for (int l=top;l>=0;--l) {
        if (l<top) { qx*=2; qy*=2; } // upscale target ONCE between levels
        float scale=float(1<<l);
        if (!lk(a[l],b[l],px/scale,py/scale,qx,qy)) return false;
    }
    return true;
}
}
void grayFromY(const uint8_t* y,int w,int h,int stride,std::vector<uint8_t>& out) {
    if (!y || !dimensions(w,h) || stride<w) throw std::invalid_argument("invalid Y plane layout");
    out.resize(size_t(w)*h);
    for (int r=0;r<h;++r) std::memcpy(out.data()+size_t(r)*w,y+size_t(r)*stride,size_t(w));
}
void detectFast(const uint8_t* gray,int w,int h,int threshold,std::vector<Keypoint>& out,
                int maxFeatures,int nx,int ny,int maxPerCell) {
    out.clear();
    if (!gray || !dimensions(w,h) || w<9 || h<9 || nx<1 || ny<1 || nx>256 || ny>256 ||
        threshold<1 || threshold>255 || maxFeatures<1 || maxPerCell<1) return;
    std::vector<Keypoint> candidates;
    for (int y=3;y<h-3;++y) for (int x=3;x<w-3;++x) {
        int d[16],score=0,center=gray[y*w+x];
        for (int i=0;i<16;++i) d[i]=int(gray[(y+cy[i])*w+x+cx[i]])-center;
        for (int sign:{-1,1}) for (int start=0;start<16;++start) {
            int arc=256;
            for (int j=0;j<9;++j) { arc=std::min(arc,sign*d[(start+j)%16]); if (arc<=threshold) break; }
            score=std::max(score,arc);
        }
        if (score>threshold) candidates.push_back({float(x),float(y),float(score)});
    }
    std::stable_sort(candidates.begin(),candidates.end(),[](const Keypoint& a,const Keypoint& b){return a.score>b.score;});
    std::vector<uint8_t> used(size_t(w)*h,0); std::vector<int> count(size_t(nx)*ny,0);
    for (const auto& p:candidates) {
        int x=int(p.x),y=int(p.y),cell=std::min(ny-1,y*ny/h)*nx+std::min(nx-1,x*nx/w);
        if (used[y*w+x] || count[cell]>=maxPerCell) continue;
        out.push_back(p); ++count[cell];
        if (out.size()>=size_t(maxFeatures)) break;
        for (int j=-1;j<=1;++j) for (int i=-1;i<=1;++i) used[(y+j)*w+x+i]=1;
    }
}
int trackKlt(const std::vector<uint8_t>& prev,const std::vector<uint8_t>& cur,int w,int h,
             const std::vector<Keypoint>& input,std::vector<Keypoint>& output,std::vector<uint8_t>& status) {
    output=input; status.assign(input.size(),0);
    if (!dimensions(w,h) || prev.size()!=size_t(w)*h || cur.size()!=size_t(w)*h) return 0;
    auto a=pyramid(prev,w,h),b=pyramid(cur,w,h); int count=0;
    for (size_t i=0;i<input.size();++i) {
        float x=0,y=0,backX=0,backY=0;
        if (!trackOne(a,b,input[i].x,input[i].y,x,y) || !trackOne(b,a,x,y,backX,backY)) continue;
        if (std::hypot(backX-input[i].x,backY-input[i].y)>.75f) continue;
        output[i]={x,y,input[i].score}; status[i]=1; ++count;
    }
    return count;
}
} // namespace gvio
