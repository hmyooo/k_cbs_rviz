#pragma once
#include "model.hpp"



class HolonomicModel : public Model {

 public:
  //###################################################
  // 构造函数                        HolonomicModel
  // width 为全向模型的半径
  //   v   为最小速度 默认为1 
  //###################################################
  HolonomicModel() : Model(){
        collisionLookup(Model::modellookup);
        type = new Model_Type(width);
    };
  HolonomicModel(double w, double step_length,double theta): Model(), width(w) {
    stepLength *= 1;
    type = new Model_Type(width);
    collisionLookup(Model::modellookup);
    std::cout << "Successfully construct a Holonomic model with width" << width << std::endl;
  }
  ~HolonomicModel(){}





  //###################################################
  // 找neighbor                        getNeighbors
  // input  State, action (需看主函数 定义act大小 现在是8个自由度方向)
  // stepLength目前是r*delta 还需要改正 需要改为地图栅格大小
  //###################################################
  std::vector<Neighbor> getNeighbors(const State &s) {
    std::vector<Neighbor> neighbors;
    double g = stepLength;

    for (int act = 0; act < 8; act++) {
      State tempState(s.x + g * dx[act],
                      s.y + g * dy[act], Constants::normalizeHeadingRad(s.yaw + dyaw[act]), s.time + 1);
        neighbors.emplace_back(Neighbor(tempState, g));
    }
    // wait
    g = stepLength;
    State tempState(s.x, s.y, s.yaw, s.time + 1);
    neighbors.emplace_back(Neighbor(tempState, g));
    return neighbors;
  }



  std::vector<Neighbor> getReNeighbors(const State &s) {
    std::vector<Neighbor> neighbors;
    double g = stepLength;

    for (int act = 0; act < 8; act++) {
      State tempState(s.x + g * dx[act],
                      s.y + g * dy[act], Constants::normalizeHeadingRad(s.yaw + dyaw[act]), s.time + 1);
        neighbors.emplace_back(Neighbor(tempState, g));
    }
    // wait
    g = stepLength;
    State tempState(s.x, s.y, s.yaw, s.time + 1);
    neighbors.emplace_back(Neighbor(tempState, g));
    return neighbors;
  }

  //###################################################
  // 当距离goal距离 小于3倍车身 判断是否到终点     isSolution
  // input  State, goal, path
  //###################################################  
  bool isSolution(const State &state, const State &goal,
                  std::vector<std::vector<std::tuple<State, Cost>>> &paths) {
      std::vector<std::tuple<State, Cost>> path;
    // 当当前位置距离终点小于 3倍车身 开始进行rs判断
    double goal_distance =
        sqrt(pow(state.x - goal.x, 2) + pow(state.y - goal.y, 2));
    if (goal_distance > 1.5 * width) return false;
    State tempState = state;
    path.clear();
    path.emplace_back(std::make_tuple<>(state, 0));
    // X direction
    // 车身位置靠前 需向后走
    int act = (goal.x - state.x > 0) ? 0 : 5;
    for (int i = 0; i < (int)(abs(goal.x - state.x) / stepLength); i++) {
      tempState.x += stepLength * dx[act];
      tempState.time++;
      path.emplace_back(std::make_tuple<>(tempState, stepLength));
    }
    double ratio = abs(tempState.x - goal.x) / stepLength;
    tempState.x = goal.x;
    tempState.time++;
    path.emplace_back(std::make_tuple<>(tempState, ratio * stepLength));
    // Y direction
    act = (goal.y - state.y > 0) ? 1 : 4;
    for (int i = 0; i < (int)(abs(goal.y - state.y) / stepLength); i++) {
      tempState.y += stepLength * dy[act];
      tempState.time++;
      path.emplace_back(std::make_tuple<>(tempState, stepLength));
    }
    ratio = abs(tempState.y - goal.y) / stepLength;
    tempState.y = goal.y;
    tempState.time++;
    path.emplace_back(std::make_tuple<>(tempState, ratio * stepLength));

    // std::cout << state << " " << goal << std::endl;
    // for (auto p : path) {
    //   std::cout << std::get<0>(p) << " " << std::get<1>(p) << " "
    //             << std::get<2>(p) << std::endl;
    // }
    // std::cout << "------------\n";
    paths.push_back(path);
    return true;
  }





  //###################################################
  // 计算到目标的启发值(cost)       admissibleHeuristic
  // 当前状态         State, 
  // 目标状态         goal, 
  // 由于是全向模型，只计算了距离目标点的欧氏距离
  // "holonomic-with-obstacles" heuristic：（用于发现U形转弯(U-shaped obstacles)/死路(dead-ends)）
  //    （不受运动学约束的）有障约束启发式值(即：A*) 在environment.cpp中计算
  //###################################################  
  double admissibleHeuristic(const State &s, const State &goal) {
    return sqrt(pow(goal.x - s.x, 2) + pow(goal.y - s.y, 2));
  }





  //###################################################
  // 安全距离         safetyRadius
  //###################################################  
  double safetyRadius() {
    return sqrt(pow(width / 2.0, 2) + pow(width / 2.0, 2));
  }

 
 
  //###################################################
  //                      变量及常量定义         
  //###################################################  
 private:
  std::vector<double> dx   = {  1.f,     1.f,     1.f,         0.f,      
                                0.f,    -1.f,     -1.f,       -1.f,     };
  std::vector<double> dy   = {  0.f,     1.f,     -1.f,        1.f,
                                -1.f,    0.f,     1.f,        -1.f,     };
  std::vector<double> dyaw = {   0,     M_PI_4,  -M_PI_4,     M_PI_2, 
                              -M_PI_2,   M_PI,  3.f*M_PI_4, -3.f*M_PI_4 };
  double width = Constants::holoCarWidth;
  double stepLength = 1 * Constants::holoStepLength;

  // collisionlookup查找表需要用到的一些变量
  /// [m] -- The bounding box size length and width to precompute all possible headings
  //用于预计算所有可能的转向的框的大小
  // 为 (车的面积+4)/cellsize
  double safetyradius = 0.5;
  // 存放车辆检测碰撞的查找表
  // 生成collisionlookup查找表的参数
  // 在constants.h中定义的一些常量
  int headings = Constants::headings;//车体朝向的离散数量
  int positionResolution = Constants::positionResolution;
  int positions = Constants::positions;
  struct point {//定义点的数据结构
        double x;
        double y;
      };
  /// A structure describing the relative position of the occupied cell based on the center of the vehicle





  public:  
  //###################################################
  //                    COLLISION LOOKUP
  // 障碍查找表
  //###################################################
  void collisionLookup(Constants::config* lookup) {
    // 是否终端调试显示参数
    bool DEBUG = true; 
    std::cout << "I am building the collision lookup table...";

    // VARIABLES FOR LOOKUP CREATION 创建查找表的变量
    int count = 0;
    point points[positions];//转化为点表示

    // generate all discrete positions within one cell
    for (int i = 0; i < positionResolution; ++i) {
      for (int j = 0; j < positionResolution; ++j) {
        points[positionResolution * i + j].x = 1.f / positionResolution * j;
        points[positionResolution * i + j].y = 1.f / positionResolution * i;
      }//从左上方开始，给每个点的x、y赋值，实际上是计算出每个点的位置(偏移量)
    }

  

    for (int q = 0; q < positions; ++q) {

      // set the starting angle to zero;
      int tMaxX = ceil(points[q].x + width + safetyradius);
      int tMinX = floor(points[q].x - width - safetyradius);
      int tMaxY = ceil(points[q].y + width + safetyradius);
      int tMinY = floor(points[q].y - width - safetyradius);
      count = 0;
      for(int X = tMinX; X < tMaxX; X++)
        for(int Y = tMinY; Y < tMaxY; Y++)
          if(pow(X,2)+pow(Y,2) < pow(width + safetyradius,2)){
            count++;
            for(int o = 0; o < Constants::headings; ++o){
              // compute the relative position of the car cells
              lookup[q * Constants::headings + o].pos[count].x = X ;
              lookup[q * Constants::headings + o].pos[count].y = Y ;
              // add one for the length of the current list
            }
          }
             
      for(int o = 0; o < Constants::headings; ++o)
        lookup[q * Constants::headings + o].length = count ;
              
    }
  }

};
