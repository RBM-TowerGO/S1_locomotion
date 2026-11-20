#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

namespace towergo {
struct RoutePoint {
  double x, y;
  bool operator==(const RoutePoint &p) const {return x==p.x && y==p.y;}
};
inline double distance(RoutePoint a, RoutePoint b) {return std::hypot(a.x-b.x,a.y-b.y);}

// Progress is measured on the unpruned controller route, in metres of arc length.
class RouteHeading {
public:
  void setPath(std::vector<RoutePoint> points, bool same_task, bool full) {
    full_=full;
    if (same_task && points==points_) return;
    base_=same_task ? completed_ : 0.;
    completed_=base_;
    points_=std::move(points);
    arc_.clear();
    for(size_t i=0;i<points_.size();++i)
      arc_.push_back(i ? arc_.back()+distance(points_[i-1],points_[i]) : 0.);
    along_=0.;
    rebase_=same_task;
  }

  std::optional<double> direction(RoutePoint robot) {
    if(points_.size()<2) return {};
    // Search the contiguous neighbourhood of the cursor. A self-intersection
    // farther along the same route must not advance the task into another leg.
    const double lower=std::max(0.,along_-.30), upper=along_+2.;
    auto first=std::upper_bound(arc_.begin(),arc_.end(),lower);
    size_t i=std::max<size_t>(1,first-arc_.begin());
    double best=INFINITY, nearest=along_;
    for(;i<arc_.size() && arc_[i-1]<=upper;++i) {
      const auto a=points_[i-1], b=points_[i];
      const auto len=arc_[i]-arc_[i-1];
      if(len<=1e-8) continue;
      const double ux=(b.x-a.x)/len, uy=(b.y-a.y)/len;
      const auto step=std::clamp((robot.x-a.x)*ux+(robot.y-a.y)*uy,0.,len);
      const double d=distance(robot,{a.x+step*ux,a.y+step*uy});
      if(d<best) {best=d; nearest=arc_[i-1]+step;}
    }
    if(best>0.75) return {};
    along_=nearest;
    if(rebase_) {base_-=along_; rebase_=false;}
    completed_=std::max(0.,base_+along_);
    const double total=base_+arc_.back();
    if(!full_ || total<=10.+1e-6 || completed_<.2*total || completed_>=.8*total) return {};
    const auto a=at(along_), b=at(std::min(along_+.65,arc_.back()));
    if(distance(a,b)<.05) return {};
    return std::atan2(b.y-a.y,b.x-a.x);
  }

private:
  RoutePoint at(double s) const {
    const size_t i=std::upper_bound(arc_.begin(),arc_.end(),s)-arc_.begin();
    if(i==arc_.size()) return points_.back();
    const double t=(s-arc_[i-1])/(arc_[i]-arc_[i-1]);
    return {points_[i-1].x+t*(points_[i].x-points_[i-1].x),
            points_[i-1].y+t*(points_[i].y-points_[i-1].y)};
  }
  std::vector<RoutePoint> points_;
  std::vector<double> arc_;
  double base_=0., along_=0., completed_=0.;
  bool rebase_=false, full_=false;
};
}
