#pragma once

#include <boost/functional/hash.hpp>
#include <boost/numeric/ublas/matrix.hpp>

//###################################################
// 类                        State
//###################################################
struct State
{
  State(double x, double y, double yaw, int time = 0, double v = 0)
      : time(time), x(x), y(y), yaw(yaw), v(v){}

  State() = default;
  bool operator==(const State &s) const
  {
    return std::tie(time, x, y, yaw, v) == std::tie(s.time, s.x, s.y, s.yaw, s.v);
  }

  bool except_time(const State &s)
  {
    return std::tie(x, y, yaw, v) == std::tie(s.x, s.y, s.yaw, s.v);
  }

  friend std::ostream &operator<<(std::ostream &os, const State &s)
  {
    return os << "(" << s.x << "," << s.y << ":" << s.yaw << ")" << ")@" << s.time<< std::endl ;
  }
  
  int time;
  double x;
  double y;
  double yaw;
  double v;

  // Location location;
  // boost::numeric::ublas::matrix<double> rot;
};

namespace std
{
  template <>
  struct hash<State>
  {
    size_t operator()(const State &s) const
    {
      size_t seed = 0;
      boost::hash_combine(seed, s.time);
      boost::hash_combine(seed, s.x);
      boost::hash_combine(seed, s.y);
      boost::hash_combine(seed, s.yaw);
      boost::hash_combine(seed, s.v);
      return seed;
    }
  };
} // namespace std

