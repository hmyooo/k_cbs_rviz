#pragma once

#include <iostream>

#include "neighbor.hpp"
#include "state.hpp"

using namespace libMultiRobotPlanning;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::State;
struct Ackerman{
    double LB, LF, width, steerphi;
};
struct Holonomic{
     double width;
};
struct Model_Type{
    Model_Type(double w){
        holo.width = w;
        str = "Holonomic";
    };
    Model_Type(double b,double f,double w,double phi){
        ack.LB = b;
        ack.LF = f;
        ack.width = w;
        ack.steerphi = phi;
        str = "Ackerman";
    };
    Holonomic holo;
    Ackerman ack;
    std::string str;
};
//###################################################
//                           类 Model  
// ackermann_model和holonomic_model的一个模板
// 均为model的子类
//###################################################
class Model {
 public:
  Model(){};
  ~Model(){};
  virtual std::vector<Neighbor> getNeighbors(const State &) = 0;
  virtual std::vector<Neighbor> getReNeighbors(const State &) = 0;
  virtual bool isSolution(const State &, const State &,
                  std::vector<std::vector<std::tuple<State, double>>> &) = 0;
  virtual double admissibleHeuristic(const State &, const State &) = 0;
  // virtual bool collideObstacle(const State &s, const Location &obstacle) = 0;
  virtual double safetyRadius() = 0;
  virtual void collisionLookup(Constants::config* lookup) = 0;
  Model_Type *type;
  Constants::config modellookup[100000];
  unsigned int index;
};
