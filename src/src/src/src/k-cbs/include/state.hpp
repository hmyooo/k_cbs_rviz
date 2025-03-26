#pragma once

#include <boost/functional/hash.hpp>
#include "constants.hpp"





//###################################################
// 类                        State
//###################################################
namespace libMultiRobotPlanning{
struct State {
    State(double x, double y, double yaw, double v, double time)
      : time(time), x(x), y(y), yaw(yaw), v(v){
    }
    State(double x, double y, double yaw, double time)
      : time(time), x(x), y(y), yaw(yaw), v(0){
    }
    State(double x, double y, double yaw)
      : time(0), x(x), y(y), yaw(yaw), v(0){
    }
    State(double x, double y)
      : time(0), x(x), y(y), yaw(0), v(0){
    }
    State(State s, double time)
      : time(time), x(s.x), y(s.y), yaw(s.yaw),v(0){
    }
    State() = default;

    bool operator==(const State& s) const {
        return std::tie(time, x, y, yaw, v) == std::tie(s.time, s.x, s.y, s.yaw, s.v);
    }

    friend std::ostream& operator<<(std::ostream& os, const State& s) {
        return os << "(" << s.x << "," << s.y << ":" << s.yaw << ")@"
                  "("<<s.v << ")"<< s.time;
    }

    double time;
    double x;
    double y;
    double yaw;
    double v;
  // boost::numeric::ublas::matrix<double> rot;
};
}
namespace std {
template <>
struct hash<libMultiRobotPlanning::State> {
  size_t operator()(const libMultiRobotPlanning::State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    boost::hash_combine(seed, s.v);
    boost::hash_combine(seed, s.yaw); 
    return seed;
  }
};
}  // namespace std

using Action = int;
using Cost = double;


//###################################################
// 类                        Location
//###################################################
namespace libMultiRobotPlanning {
struct Location {
  Location() :r(0) {}
  Location(double x, double y) : x(x), y(y),r(0) {}
  Location(double x, double y, double r) : x(x), y(y), r(r) {}
  Location(State s, double r) : x(s.x), y(s.y), r(r) {}
  double x;
  double y;
  double r;

  bool operator<(const Location& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const Location& c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
};
}
namespace std {
template <>
struct hash<libMultiRobotPlanning::Location> {
  size_t operator()(const libMultiRobotPlanning::Location& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

