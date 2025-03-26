#pragma once
#include <ompl/base/State.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <iostream>

#include "constants.hpp"
#include "model.hpp"

typedef ompl::base::SE2StateSpace::StateType OmplState;

// TODO: add agent size

class AckermannModel : public Model {
 public:
  //###################################################
  //                             构造函数 AckermannModel
  //    LB  为后轮中心到车尾距离
  //    LF  为前轮中心到车头距离
  //  width 为车辆宽度
  //    r   为车辆最小转弯半径
  //    v   为车辆最小速度 默认为1
  //###################################################
  AckermannModel() : Model(){
      collisionLookup(Model::modellookup);
      type = new Model_Type(LB,LF,width, maxSteerphi);
    };
  AckermannModel(double b, double f, double w)
      : Model(), LB(b), LF(f), width(w), ackerl(LB + LF), maxSteerphi(Constants::maxSteerphi),minR(ackerl/maxSteerphi) {
    type = new Model_Type(b,f,w, Constants::maxSteerphi);
      // 前三个向前 后三个向后
    LB = LB + boating;
    LF = LF + boating;
    length = LB + LF;
    width = width + boating * 2;
    collisionLookup(Model::modellookup);
    // std::cout << "Successfully build a Ackermann Model with " << LB << " " <<
    // LF
    //           << " " << width << std::endl;
  }
  ~AckermannModel(){};




  //###################################################
    // 当距离goal距离 小于6倍车款 判断是否到终点   isSolution
    // input  State, goal, path
    //###################################################
    bool isSolution(const State &state, const State &goal,
                    std::vector<std::vector<std::tuple<State, double>>> &paths) {
        double goal_distance =
        sqrt(pow(state.x - goal.x, 2) + pow(state.y - goal.y, 2));
        if (goal_distance > 10 * (LB + LF) || state.v > minVel) return false;
//        if (goal_distance > 1 * (LB + LF) ) return false;
        // Analytical expansion
        std::vector<double> steers =std::vector<double>({30.f/180.f* M_PI,45.f/180.f* M_PI,60.f/180.f* M_PI});
        for(auto theta:steers){
            minR = ackerl/theta;
            std::vector<std::tuple<State, double>> path;
            ompl::base::ReedsSheppStateSpace reedsSheppSpace(minR);
            OmplState *rsStart = (OmplState *)reedsSheppSpace.allocState();
            OmplState *rsEnd = (OmplState *)reedsSheppSpace.allocState();
            rsStart->setXY(state.x, state.y);
            rsStart->setYaw(-state.yaw);
            rsEnd->setXY(goal.x, goal.y);
            rsEnd->setYaw(-goal.yaw);
            ompl::base::ReedsSheppStateSpace::ReedsSheppPath reedsShepppath =
            reedsSheppSpace.reedsShepp(rsStart, rsEnd);
            // std::cout<<"distance" <<  reedsSheppSpace.distance(rsStart, rsEnd) << std::endl;
            // std::cout<<"type" <<  reedsShepppath.type_ << std::endl;
            // std::cout<<"totalLength_" <<  reedsShepppath.totalLength_ << std::endl;
            path.clear();
            path.emplace_back(std::make_tuple<>(state, 0));
            for (auto pathidx = 0; pathidx < 5; pathidx++) {
                // std::cout<<"length_   " << reedsShepppath.length_[pathidx] << std::endl;
                // std::cout<<"type" <<  reedsShepppath.type_[pathidx] << std::endl;
                if (fabs(reedsShepppath.length_[pathidx]) < 1e-6) continue;
                double deltat = 0, deltat_dis = 0, act = 0, cost = 0;
                switch (reedsShepppath.type_[pathidx]) {
                case 0:  // RS_NOP
                    continue;
                    break;
                case 1:  // RS_LEFT
                    deltat = -reedsShepppath.length_[pathidx];
                    deltat_dis = minR * sin(-deltat);
                    act = 2;
                    cost =
                    reedsShepppath.length_[pathidx] * minR * Constants::penaltyTurning;
                    break;
                case 2:  // RS_STRAIGHT
                    deltat = 0;
                    deltat_dis = reedsShepppath.length_[pathidx] * minR;
                    act = 0;
                    cost = deltat_dis;
                    break;
                case 3:  // RS_RIGHT
                    deltat = reedsShepppath.length_[pathidx];
                    deltat_dis = minR * sin(deltat);
                    act = 1;
                    cost =
                    reedsShepppath.length_[pathidx] * minR * Constants::penaltyTurning;
                    break;
                default:
                    std::cout << "\033[1m\033[31m"
                        << "Warning: Receive unknown ReedsSheppPath type"
                        << "\033[0m\n";
                    break;
                }
                State current_s = std::get<0>(path.back());
                double reserve = 1;
                if (cost < 0){
                    cost = -cost * Constants::penaltyReversing;
                    reserve = -1;
                }
                generatePath(act, deltat, deltat_dis, path,reserve);
            }
            paths.push_back(path);
        }
        return true;
    };




    //###################################################
    // 生成一组neighbor                    getNeighbors
    // 当前状态                   State,
    // 当前移动状态action         action,
    // 返回一组neighbor         neighbor
    //###################################################
    std::vector<Neighbor> getNeighbors(const State &s) {
      std::vector<Neighbor> neighbors;
      std::vector<double> accl;
//      std::vector<double> accl = {-1.0,-0.5,0,0.5,1.0};
    //  accl = {-1.0,-0.5,0,0.5,1.0};
    //  accl ={-20.0,-18.0,-16.0,-14.0,-12.0,-10.0,-8.0,-6.0,-4.0,-2.0,0,2.0,4.0,6.0,8.0,10.0,12.0,14.0,16.0,18.0,20.0};

      /******** input0 参数 *****/
//      accl ={ -1.5,-1.25,-1.0,-0.75, -0.5, -0.25, 0, 0.25, 0.5,0.75,1.0,1.25,1.5};
      /******** input1 参数 *****/
//      accl ={ -2.0,-1.5,-1.0, -0.5, 0, 0.5,1.0,1.5,2.0};
      /******** input7 参数 *****/
//      accl ={ -2.0,-1.5,-1.0, -0.5, 0, 0.5,1.0,1.5,2.0};
        // accl ={-4.0,-3.6,-2.8,-2.0,-1.6,-0.8,0, 0.8,1.6,2.0,2.8,3.6,4.0};
        // accl ={-2.0,-1.6,-0.8,0, 0.8,1.6,2.0};
        // accl ={-4.0,-3.6,-2.8,-2.0,-1.6,-1.0, 0, 1.0,1.6,2.0,2.8,3.6,4.0};
        accl ={-3.0,-2.0, -1.0,0,1.0,2.0,3.0};
        //  accl ={-3.0,-2.0,-1.0,1.0,0,2.0,3.0};
        //  accl ={-1.6,-1.0,0,1.0,1.6};
        //  accl ={-10.0,-8.0,-6.0,-4.0,-2.0,0,2.0,4.0,6.0,8.0,10.0};
        //  accl ={-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8};//改

//      accl ={-4.0,-2.0, 0,2.0,4.0};
//      accl ={-6.0,-4.0,-2.0, 0,2.0,4.0,6.0};
      for(auto acc : accl){
          for(auto theta:steerphi){
              double deltat_dis = s.v * timeResolution;
              double deltat_v = 0;
              if(fabs(s.v + acc * timeResolution) <= maxVel){
                  deltat_v = acc * timeResolution;
                  deltat_dis = s.v * timeResolution + 0.5 * acc * pow(timeResolution, 2);
              }

              if(deltat_dis == 0) continue;
              double r = ackerl/theta;
              double deltat = theta * deltat_dis*1.f/ackerl;
              dyaw =
                  std::vector<double>({     0,              deltat,                  -deltat});
              dx =
                  std::vector<double>({  deltat_dis,    r *sin(fabs(deltat)),       r *sin(fabs(deltat))});
              dy =
                  std::vector<double>({     0,        -fabs(r) *(1 - cos(deltat)),      fabs(r) *(1 - cos(deltat))});
              for (Action act = 0; act < 3; act++) {  // has 6 directions for Reeds-Shepp
                  double xSucc = s.x + dx[act] * cos(-s.yaw) - dy[act] * sin(-s.yaw);
                  double ySucc = s.y + dx[act] * sin(-s.yaw) + dy[act] * cos(-s.yaw);
                  double yawSucc = Constants::normalizeHeadingRad(s.yaw + dyaw[act]);
                  double vSucc = s.v + deltat_v;
                 double g =   0.25*fabs(acc) + exp(cos(theta));
                if (act  != 0) {  // penalize turning
                  g = g * Constants::penaltyTurning;
                }
                if (s.v * vSucc < 0) {
                  // penalize change of direction
                  g = g * Constants::penaltyCOD;
                }
                if (vSucc < 0) {  // backwards
                  g = g * Constants::penaltyReversing;
                }
                State tempState(xSucc, ySucc, yawSucc,vSucc, s.time + timeResolution);
                neighbors.emplace_back(Neighbor(tempState, g));
              }
          }
        }
      // wait
      if(s.v < minVel){
          double g = (0.25*fabs(2) + exp(cos(maxSteerphi))) * Constants::penaltyWaiting;
          State tempState(s.x, s.y, s.yaw, 0 , s.time + timeResolution);
          neighbors.emplace_back(Neighbor(tempState, g));
      }
      return neighbors;
    };


    std::vector<Neighbor> getReNeighbors(const State &s) {
        std::vector<Neighbor> neighbors;  
        std::vector<double> accel;
      //  accel ={ -1.0, -0.75, -0.5, -0.25, 0, 0.25, 0.5, 0.75, 1.0};
//        accel ={-0.5, -0.25, 0, 0.25, 0.5};
        /******** 0805 参数 *****/
//        accel ={ -0.6, -0.4, -0.2, 0, 0.2, 0.4, 0.6};
        /******** 0806 适用于小场景参数 *****/
        // accel ={ -0.4, -0.2, 0, 0.2, 0.4};
        /******** 0810 适用于大场景参数 *****/
      //  accel ={ -0.8, -0.4, 0,  0.4,0.8};
      //  accel ={-4.0,-3.0,-2.0,-1.0, 0,1.0,2.0,3.0,4.0};
       accel ={-3.0,-2.0, -1.0,0,1.0,2.0,3.0};
      //  accel ={-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8};

        // accel ={-3.0,-2.0,-1.0,1.0,0,2.0,3.0};
        //  accel ={-1.6,-1.0,0,1.0,1.6};
        //  accel ={-20.0,-18.0,-16.0,-14.0,-12.0,-10.0,-8.0,-6.0,-4.0,-2.0,0,2.0,4.0,6.0,8.0,10.0,12.0,14.0,16.0,18.0,20.0};
        //  accel ={-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5};

//        accel ={ -1.0, -0.5,  0,  0.5, 1.0};
        for(auto theta: steerphi)
        for (auto acc:accel){
            double deltat_dis = s.v * timeResolution;
            double deltat_v = 0;
            if(fabs(s.v + acc * timeResolution) <= maxVel){
                deltat_v = acc * timeResolution;
                deltat_dis = s.v * timeResolution + 0.5 * acc * pow(timeResolution, 2);
            }
//            if(acc == 0 ) deltat_dis= 0.8;
            if(deltat_dis == 0) continue;
            double r = ackerl/theta;
            double deltat = theta * deltat_dis*1.f/ackerl;
            dyaw =
                std::vector<double>({     0,              deltat,                  -deltat});
            dx =
                std::vector<double>({  deltat_dis,    r *sin(fabs(deltat)),       r *sin(fabs(deltat))});
            dy =
                std::vector<double>({     0,        -fabs(r) *(1 - cos(deltat)),      fabs(r) *(1 - cos(deltat))});
            for (Action act = 0; act < 3; act++) {  // has 6 directions for Reeds-Shepp
              double g =   0.25*fabs(acc) + exp(cos(theta));
              if (act != 0) {  // penalize turning
                g = g * Constants::penaltyTurning;
              }
              if(s.v * acc > 0) g = g *Constants::penaltyReversing;
              State tempState(s.x + dx[act] * cos(-s.yaw) - dy[act] * sin(-s.yaw),
                              s.y + dx[act] * sin(-s.yaw) + dy[act] * cos(-s.yaw),
                              Constants::normalizeHeadingRad(s.yaw + dyaw[act]),
                              s.v + deltat_v, s.time + timeResolution);
              neighbors.emplace_back(Neighbor(tempState, g));
            }
        }
        if(s.v < minVel){
            double g = Constants::penaltyWaiting*(0.25*fabs(maxAcc) + exp(cos(maxSteerphi)));
            State tempState(s.x, s.y, s.yaw, 0, s.time + timeResolution);
            neighbors.emplace_back(Neighbor(tempState, g));
        }
        return neighbors;
    };




    //###################################################
    // 启发函数计算                    admissibleHeuristic
    // 当前状态                   State,
    // 目标状态                   goal,
    // 在模型内部计算的取的欧式距离和非完整约束的cost中的最大值
    // 在environment.cpp中会计算A*的启发值 并比较
    // 计算到目标的启发值(cost)
    // 这里的cost由三项组成：《Practical Search Techniques in Path Planning for Autonomous Driving》
    // 1) "non-holonomic-without-obstacles" heuristic:（用于指导搜索向目标方向前进）
    //    受运动学约束的无障碍启发式值。论文的计算建议为： max(Reed-Shepp距离/Dubins距离, 欧氏距离) 表示
    //    至于用Reed-Shepp距离还是Dubins距离取决于车辆是否可倒退
    // 2) "holonomic-with-obstacles" heuristic：（用于发现U形转弯(U-shaped obstacles)/死路(dead-ends)）
    //    （不受运动学约束的）有障约束启发式值(即：A*)
    // 注1： 实际计算时，优先考虑运动学启发式值，A*作为可选项。至于是否启用欧氏距离和A*的启发式值，取决于计算
    //      的精度和CPU性能（可作为调优手段之一）
    // 注2： 实际计算与论文中的描述存在差异：
    //      （1）实际计算的第一步用的启发式值为“Reed-Shepp距离/Dubins距离”，而论文为“max(Reed-Shepp距离/Dubins距离, 欧氏距离)”
    //      （2）实际计算的第二步用的启发式值为A*的启发式值 减去 “start与goal各自相对自身所在2D网格的偏移量(二维向量)的欧氏距离”
    //          该步计算的意义还需仔细分析，目前我还没想明白代码这样设计的理由。
    // 也就是在模型内部 只计算了第一个部分
    //###################################################
    double admissibleHeuristic(const State &s, const State &goal) {
        // std::cout << "come to Ackermann function\n";
        // non-holonomic-without-obstacles heuristic: use a Reeds-Shepp
        double nonholonomic_Cost = 0;

        // if dubins heuristic is activated calculate the shortest path
        // constrained without obstacles
        // if (Constants::dubins) {
          //这里改用open motion planning library的算法
          ompl::base::DubinsStateSpace dubinsSpace(minR);
          OmplState* dbStart = (OmplState*)dubinsSpace.allocState();
          OmplState* dbEnd = (OmplState*)dubinsSpace.allocState();
          dbStart->setXY(s.x, s.y);
          dbStart->setYaw(s.yaw);
          dbEnd->setXY(goal.x, goal.y);
          dbEnd->setYaw(goal.yaw);
          nonholonomic_Cost = dubinsSpace.distance(dbStart, dbEnd);
        // }

        // if reversing is active use a Reeds-Shepp
        //假如车子可以后退，则可以启动Reeds-Shepp 算法
        // if (Constants::reverse && !Constants::dubins) {
        //   //    ros::Time t0 = ros::Time::now();
        //   ompl::base::ReedsSheppStateSpace rsSpace(minR);
        //   OmplState* rsStart = (OmplState*)rsSpace.allocState();
        //   OmplState* rsEnd = (OmplState*)rsSpace.allocState();
        //   rsStart->setXY(s.x, s.y);
        //   rsStart->setYaw(s.yaw);
        //   rsEnd->setXY(goal.x, goal.y);
        //   rsEnd->setYaw(goal.yaw);
        //   nonholonomic_Cost = rsSpace.distance(rsStart, rsEnd);
        //   //    ros::Time t1 = ros::Time::now();
        //   //    ros::Duration d(t1 - t0);
        //   //    std::cout << "calculated Reed-Sheep Heuristic in ms: " << d * 1000 << std::endl;
        // }
        // 欧式距离的代价
        double euclidean_Cost = sqrt(pow(goal.x - s.x, 2) + pow(goal.y - s.y, 2));
        // std::cout << "Euclidean cost:" << euclideanCost << std::endl;
        // std::cout << "holonomic cost:" << nonholonomic_Cost << std::endl;
        return std::max({nonholonomic_Cost, euclidean_Cost});
      };





 private:

    //###################################################
      // 生成rs曲线的路径点                    generatePath
      // rs曲线转过的角度              deltaSteer,
      // rs曲线在x方向上的 delta长度    deltaLength,
      // 需要传递的路径点path             result
      //###################################################
      bool generatePath(int act, double deltaSteer, double deltaLength,
                        std::vector<std::tuple<State, double>> &result,double reserve) {
        double xSucc, ySucc, yawSucc, restx, resty, restyaw, ratio;
        double dis =  minVel*reserve*timeResolution;
        double deltat =  dis/ minR;
        dyaw = std::vector<double>({0,deltat, -deltat});
        dx = std::vector<double>({dis,minR *sin(deltat),minR *sin(deltat)});
        dy = std::vector<double>({0,  -minR *(1 - cos(deltat)),minR *(1 - cos(deltat))});
        //        直走或者后退
        if (act == 0) {
            for (size_t i = 0; i < (size_t)(deltaLength / dx[act]); i++) {
                State s = std::get<0>(result.back());
                xSucc = s.x + dx[act] * cos(-s.yaw) - dy[act] * sin(-s.yaw);
                ySucc = s.y + dx[act] * sin(-s.yaw) + dy[act] * cos(-s.yaw);
                yawSucc = Constants::normalizeHeadingRad(s.yaw + dyaw[act]);
                State nextState(xSucc, ySucc, yawSucc,minVel*reserve,
                                std::get<0>(result.back()).time + timeResolution);
                result.emplace_back(std::make_tuple<>(nextState, pow(accResolution,2) + maxSteerphi));
            }
            ratio = fmodf(deltaLength, dx[act])/ dx[act];
            restyaw = 0;
            restx = ratio * dx[act];
            resty = 0;
        } else {
           for (size_t i = 0; i < (size_t)(deltaSteer / dyaw[act]); i++) {
               State s = std::get<0>(result.back());
                xSucc = s.x + dx[act] * cos(-s.yaw) - dy[act] * sin(-s.yaw);
                ySucc = s.y + dx[act] * sin(-s.yaw) + dy[act] * cos(-s.yaw);
                yawSucc = Constants::normalizeHeadingRad(s.yaw + dyaw[act]);
                State nextState(xSucc, ySucc, yawSucc,minVel*reserve,
                                std::get<0>(result.back()).time + timeResolution);
                result.emplace_back(std::make_tuple<>(
                    nextState, (pow(accResolution,2) + maxSteerphi) * Constants::penaltyTurning));
            }
            ratio = fmodf(deltaSteer,dyaw[act]) / dyaw[act];
            restyaw = ratio * dyaw[act];
            restx = minR * sin(restyaw);
            resty = -minR * (1 - cos(restyaw));
            if (act == 2) {
                restx = -restx;
                resty = -resty;
            }
        }
        State s = std::get<0>(result.back());
        xSucc = s.x + restx * cos(-s.yaw) - resty * sin(-s.yaw);
        ySucc = s.y + restx * sin(-s.yaw) + resty * cos(-s.yaw);
        yawSucc = Constants::normalizeHeadingRad(s.yaw + restyaw);
        // std::cout << m_agentIdx << " ratio::" << ratio << std::endl;
        State nextState(xSucc, ySucc, yawSucc, minVel*reserve, std::get<0>(result.back()).time + timeResolution);
        result.emplace_back(std::make_tuple<>(nextState, ratio * (pow(accResolution,2) + maxSteerphi)));
        return true;
      }
  
  



  
  
    //###################################################
    // 安全距离         safetyRadius
    //###################################################
    double safetyRadius() {
//        return width;
     return (sqrt(pow(width , 2) + pow(LB+LF, 2))/2)*1.5 ;
    }






  //###################################################
  //                      变量及常量定义         
  //###################################################  
 private:
    // ackerman模型自身的一些参数
    double boating = Constants::ackerboating;
    double LB = Constants::ackerLB + boating;
    double LF = Constants::ackerLF + boating;
    double width = Constants::ackerWidth + boating*2; //车的宽度
    double length = LB + LF;
    double ackerl = Constants::ackerLB + Constants::ackerLF;
    double maxAcc = Constants::maxAcc;
    double maxVel = Constants::maxVel;
    double minVel = Constants::minVel;
    double accResolution = Constants::accResolution;
    double timeResolution = Constants::timeResolution;
    double maxSteerphi = Constants::maxSteerphi;
    double minR = ackerl/maxSteerphi;;   //最小转弯半径
    std::vector<double> steerphi = Constants::steerphi;

    // collisionlookup查找表需要用到的一些变量
    /// [m] -- The bounding box size length and width to precompute all possible headings
    //用于预计算所有可能的转向的框的大小
    // 存放车辆检测碰撞的查找表
    // 生成collisionlookup查找表的参数
    // 在constants.h中定义的一些常量
    int headings = Constants::headings;//车体朝向的离散数量
    float deltaHeadingRad = Constants::deltaHeadingRad; //朝向离散步长(以弧度表示)
    int positionResolution = Constants::positionResolution;
    int positions = Constants::positions;
    struct point {//定义点的数据结构
        double x;
        double y;
      };
    /// A structure describing the relative position of the occupied cell based on the center of the vehicle
    // 前三个向前 后三个向后
    std::vector<double> dyaw,dx,dy;





 public:  
  //###################################################
  //                    COLLISION LOOKUP
  // 障碍查找表
  //###################################################
    //###################################################
     //                    COLLISION LOOKUP
     // 障碍查找表
     //###################################################
     // COLLISION LOOKUP CREATION
     double GetCross(point p1, point p2, point p){
         return (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y);
     }


     bool IsPointInMatrix(point* pts,point p){
         double a = GetCross(pts[0], pts[1], p);
         double b = GetCross(pts[2], pts[3], p);
         double c = GetCross(pts[1], pts[2], p);
         double d = GetCross(pts[3], pts[0], p);
         if((a >= 0 && b >= 0 && c >= 0 && d >= 0) || (a <= 0 && b <= 0 && c <= 0 && d <= 0))
             return true;
         else
             return false;
     }

     bool ispointaviliable(point p0,point p,point p1,point p2){
         point linemin,linemax;
         linemin.x = std::min(p1.x, p2.x);
         linemax.x = std::max(p1.x, p2.x);
         linemin.y = std::min(p1.y, p2.y);
         linemax.y = std::max(p1.y, p2.y);

         if (p0.x <= linemax.x && p0.x >= linemin.x &&
               p0.y <= linemax.y && p0.y >= linemin.y &&
               p0.x <= p.x+1 && p0.x >= p.x  &&
               p0.y <= p.y+1 && p0.y >= p.y )
             return true;
         return  false;
     }
       bool GeneralEquation(point p1,point p2,point p){
         double A = p2.y-p1.y;
         double B = p1.x-p2.x;
         double C = p2.x*p1.y-p1.x*p2.y;
        if (B==0) {
           point calp0,calp1;
           calp0.x = -C/double(A);
           calp0.y = p.y ;
           calp1.x = -C/double(A);
           calp1.y = p.y +1;
           if(ispointaviliable(calp0,p,p1,p2)||ispointaviliable(calp1,p,p1,p2))
               return true;
        }
        if (A == 0){
            point calp0,calp1;
            calp0.x = p.x;
            calp0.y = -C / double(B);
            calp1.x = p.x +1;
            calp1.y = -C / double(B);
            if(ispointaviliable(calp0,p,p1,p2)||ispointaviliable(calp1,p,p1,p2))
                return true;
        }
         if (B != 0){
            point calp0,calp1;
            calp0.x = p.x;
            calp0.y = -(C + A * p.x) / double(B);
            calp1.x = p.x +1;
            calp1.y = -(C + A * (p.x + 1)) / double(B);
            if(ispointaviliable(calp0,p,p1,p2)||ispointaviliable(calp1,p,p1,p2))
                return true;
         }
         if (A != 0){
             point calp0,calp1;
             calp0.x = -(C + B * p.y) / double(A);
             calp0.y = p.y ;
             calp1.x =  -(C + B * (p.y + 1)) / double(A);
             calp1.y = p.y +1;
             if(ispointaviliable(calp0,p,p1,p2)||ispointaviliable(calp1,p,p1,p2))
                 return true;
         }
         return false;
       }
       bool Generalrec(point* pts,point p){
         if (GeneralEquation(pts[0],pts[3],p)) return true;
         for(int i =0; i <3; i++)
             if(GeneralEquation(pts[i], pts[i+1], p))
                 return true;
         return false;
       }





     void collisionLookup(Constants::config* lookup) {
       // 是否终端调试显示参数
       std::cout << "I am building the collision lookup table...";

       // VARIABLES FOR ROTATION c 表示方形中心    temp 临时值     p[4] 四个点的坐标
       point c, pts[4];
       point maxx, minn;
       double theta;

       // VARIABLES FOR LOOKUP CREATION 创建查找表的变量
       int count = 0;
       point points[positions];//转化为点表示

       // generate all discrete positions within one cell
       for (int i = 0; i < positionResolution; ++i) {
         for (int j = 0; j < positionResolution; ++j) {
           points[positionResolution * j + i].x = 1.f / positionResolution * i;
           points[positionResolution * j + i].y = 1.f / positionResolution * j;
         }//从左上方开始，给每个点的x、y赋值，实际上是计算出每个点的位置(偏移量)
       }


       for (int q = 0; q < positions; ++q) {
         // set the starting angle to zero;
         theta = 0;




         for (int o = 0; o < headings; ++o) {//将朝向划分为72份(离散化)
             // set points of rectangle(cell中心为(size/2, size/2)，加上偏移量是在分辨率下的小网格的位置)
           c.x = ((type->ack.LB + type->ack.LF)/2.0 - type->ack.LB)*cos(theta) + (double) points[q].x;
           c.y = ((type->ack.LB + type->ack.LF)/2.0 - type->ack.LB)*sin(theta) + (double) points[q].y;
           float cal_a = sin(theta) * 0.5;
           float cal_b = cos(theta) * 0.5;
           pts[0].x = c.x + cal_a * width - cal_b * length;
           pts[0].y = c.y - cal_b * width - cal_a * length;
           pts[2].x = 2 * c.x - pts[0].x;
           pts[2].y = 2 * c.y - pts[0].y;
           pts[1].x = c.x - cal_a * width - cal_b * length;
           pts[1].y = c.y + cal_b * width - cal_a * length;
           pts[3].x = 2 * c.x - pts[1].x;
           pts[3].y = 2 * c.y - pts[1].y;

           maxx.x = std::ceil(std::max(pts[0].x,std::max(pts[1].x,std::max(pts[2].x,pts[3].x))));
           maxx.y = std::ceil(std::max(pts[0].y,std::max(pts[1].y,std::max(pts[2].y,pts[3].y))));
           minn.x = std::floor(std::min(pts[0].x,std::min(pts[1].x,std::min(pts[2].x,pts[3].x))));
           minn.y = std::floor(std::min(pts[0].y,std::min(pts[1].y,std::min(pts[2].y,pts[3].y))));


           count = 0;//生成查找表
           for (int i = minn.x - 2; i < maxx.x + 2; i++)
             for (int j = minn.y - 2; j < maxx.y + 2; j++){
                 point temp;
                 temp.x = i;
                 temp.y = j;
               if (IsPointInMatrix(pts,temp)){
                   lookup[q * headings + o].pos[count].x = i;
                   lookup[q * headings + o].pos[count].y = j;
                   count++;
               }
               else if(Generalrec(pts,temp)){
                   lookup[q * headings + o].pos[count].x = i;
                   lookup[q * headings + o].pos[count].y = j;
                   count++;
               }
             }
           lookup[q * headings + o].length = count;

           // create the next angle 下个角度(360度分成72份，步长是: deltaHeadingRad=2*pi/72)
           theta += deltaHeadingRad;

         }
       }
       std::cout << " done!" << std::endl;
     }



  
};
