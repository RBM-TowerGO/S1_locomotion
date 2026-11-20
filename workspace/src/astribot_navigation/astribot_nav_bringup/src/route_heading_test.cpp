#include "route_heading.hpp"
#include <stdexcept>

using towergo::RoutePoint;
using towergo::RouteHeading;
void check(bool condition) {
  if(!condition) throw std::runtime_error("route heading regression");
}
std::vector<RoutePoint> line(int first, int last) {
  std::vector<RoutePoint> result;
  for(int i=first;i<=last;++i) result.push_back({double(i),0.});
  return result;
}
int main() {
  RouteHeading rule;
  for(int length: {5,10,11,20}) {
    rule.setPath(line(0,length),false,true);
    for(int i=0;i<=length*10;i++) {
      double x=i/10.;
      auto yaw=rule.direction({x,0});
      check(bool(yaw)==(length>10 && x>=.2*length && x<.8*length));
      if(yaw) check(std::abs(*yaw)<1e-8);
    }
  }
  rule.setPath(line(0,20),false,true);
  for(int i=0;i<=9;i++)rule.direction({double(i),0.});
  rule.setPath(line(9,20),true,true);
  check(rule.direction({9.,0.}).has_value());
  for(int i=10;i<16;i++)check(rule.direction({double(i),0.}).has_value());
  check(!rule.direction({16.,0.}));
  // Arc length is 16 m, though the goal is only 8 m from the start.
  rule.setPath({{0,0},{4,0},{4,8},{0,8}},false,true);
  for(int i=0;i<=4;i++)rule.direction({double(i),0});
  check(std::abs(rule.direction({4,1}).value()-M_PI/2)<1e-8);
  rule.setPath(line(0,20),false,false);
  for(int i=0;i<18;i++)check(!rule.direction({double(i),0}));
}
