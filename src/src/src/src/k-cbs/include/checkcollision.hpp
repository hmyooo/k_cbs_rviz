#pragma once

#include <typeindex>

#include "models/ackermann_model.hpp"
#include "models/holonomic_model.hpp"
#include "state.hpp"
//###################################################
// 类                        CheckCollision
//###################################################
template <typename Model, typename State>
class CheckCollision {
 public:


  // //###################################################
  // // 类的静态成员函数                     getInstance
  // // 作用 创造一个instance 即CheckCollision类出来 
  // // 调用方法 CheckCollision::getInstance()
  // //###################################################
  // static CheckCollision& getInstance(std::shared_ptr<Constants::map> map) {
  //   // 类的静态成员函数 
  //   static CheckCollision instance(map);  // Guaranteed to be destroyed.
  //   return instance;
  // }

  CheckCollision(std::shared_ptr<Constants::map> map,
                 std::vector<Location> static_obs = std::vector<Location>()):grid(map),m_static_obs(static_obs) {}



  //###################################################
  //###################################################
  //###################################################
  //###################################################
  //##############  need to update  ###################
  //###################################################
  //###################################################
  //###################################################
  //###################################################
  /// The occupancy grid  需要修改
  // nav_msgs::OccupancyGrid::Ptr grid;
   std::shared_ptr<Constants::map > grid;
  //###################################################
  //###################################################
  //###################################################
  //###################################################
  //##############  need to update  ###################
  //###################################################
  //###################################################
  //###################################################
  //###################################################

    std::vector<Location> m_static_obs;











 public:
  // 构造函数 每次创建新类的时候删除旧类
  CheckCollision(CheckCollision const&) = delete;
  void operator=(CheckCollision const&) = delete;



  //###################################################
  // 检查碰撞                        collideAgents
  // agent1的状态     state s1
  // agent2的状态     state s2
  // agent1的类型     model1
  // agent2的类型     model2
  //###################################################
  bool collideAgents(const State& s1, const State& s2,
                     const std::shared_ptr<Model>& model1,
                     const std::shared_ptr<Model>& model2) {
      State c1,c2;
      if(model1->type->str == "Ackerman" ){
          double length1 = model1->type->ack.LB + model1->type->ack.LF;
          c1.x = (length1/2 - model1->type->ack.LB)*cos(-s1.yaw) + s1.x;
          c1.y = (length1/2 - model1->type->ack.LB)*sin(-s1.yaw) + s1.y;
      }
      else c1 = s1;
      if(model2->type->str == "Ackerman" ){
           double length2 = model2->type->ack.LB + model2->type->ack.LF;
          c2.x = (length2/2 - model2->type->ack.LB)*cos(-s2.yaw) + s2.x;
          c2.y = (length2/2 - model2->type->ack.LB)*sin(-s2.yaw) + s2.y;
      }
      else c2 = s2;
    if (pow(c1.x - c2.x, 2) + pow(c1.y - c2.y, 2) <
        pow(model1->safetyRadius() + model2->safetyRadius(), 2))
      return true;
    else
      return false;
  }









//###################################################
// 检查碰撞                        collideobs
//假设障碍物是圆形
//###################################################

    // 计算两点之间的距离
    double DistanceBetweenTwoPoints(State s1, State s2){
        return sqrt(pow(s2.x - s1.x,2) + pow(s2.y - s1.y,2) );
    }

    // 计算点(x, y)到经过两点(x1, y1)和(x2, y2)的直线的距离
    double DistanceFromPointToLine(Location s, State s1, State s2){
        double a = s2.y - s1.y;
        double b = s1.x - s2.x;
        double c = s2.x * s1.y - s1.x * s2.y;

        assert(fabs(a) > 0.00001f || fabs(b) > 0.00001f);

        return fabs(a * s.x + b * s.y + c) / sqrt(a * a + b * b);
    }

    // 圆与矩形碰撞检测
    // 圆心(x, y), 半径r, 矩形中心(x0, y0), 矩形上边中心(x1, y1), 矩形右边中心(x2, y2)
    bool IsCircleIntersectRectangle(Location obs, State s0, State s1, State s2){
        float w1 = DistanceBetweenTwoPoints(s0, s2);
        float h1 = DistanceBetweenTwoPoints(s0, s1);
        float w2 = DistanceFromPointToLine(obs, s0, s1);
        float h2 = DistanceFromPointToLine(obs, s0, s2);
        if (w2 > w1 + obs.r)
            return false;
        if (h2 > h1 + obs.r)
            return false;
        if (w2 <= w1)
            return true;
        if (h2 <= h1)
            return true;
        return (w2 - w1) * (w2 - w1) + (h2 - h1) * (h2 - h1) <= obs.r * obs.r;
    }



  bool collideobs(const State& s, const std::shared_ptr<Model>& model) {
      double w = model->type->ack.LB + model->type->ack.LF;
      State s0 = State((w/2 - model->type->ack.LB)*cos(-s.yaw) + s.x,
              (w/2 - model->type->ack.LB)*sin(-s.yaw) + s.y);
      State s1 = State(model->type->ack.LF*cos(-s.yaw) + s.x,
                        model->type->ack.LF*sin(-s.yaw) + s.y);
      State s2 = State(model->type->ack.width*sin(-s.yaw) + s0.x,
                       - model->type->ack.width*cos(-s.yaw) + s0.y);
    if(m_static_obs.size()==0) return false;
     for(auto obs:m_static_obs){
        if (IsCircleIntersectRectangle(obs,s0,s1,s2))
            return true;
    }
    return false;

  }

  bool collidgrideobs(double x,double y) {
    if(m_static_obs.size()==0) return false;
     for(auto obs:m_static_obs){
        if(x < std::ceil(obs.x + obs.r) && x> std::floor(obs.x - obs.r)
                &&y < std::ceil(obs.y + obs.r)  && y> std::floor(obs.y - obs.r))
            return true;
    }
    return false;

  }


  //###################################################
  //###################################################
  //###################################################
  //###################################################
  //##############  need to update  ###################
  //###################################################
  //###################################################
  //###################################################
  //###################################################
  /*!
     \brief updates the grid with the world map 需要修改 
  */
  // void updateGrid(nav_msgs::OccupancyGrid::Ptr map) {grid = map;}

  void updateGrid(std::shared_ptr<Constants::map > map) {grid = map;}
  //###################################################
  //###################################################
  //###################################################
  //###################################################
  //##############  need to update  ###################
  //###################################################
  //###################################################
  //###################################################
  //###################################################

















  //###################################################
  // 检查碰撞                        collideObstacle
  // agent的状态        state s
  // agent的类型        model
  //###################################################
  bool collideObstacle(const State& s,const std::shared_ptr<Model>& model) {
    // return model->collideObstacle(s, obstacle);
    int X = std::floor(s.x);
    int Y = std::floor(s.y);
    int iX = (int)((s.x - std::floor(s.x)) * Constants::positionResolution);//得出X方向在cell中的偏移量
    int iY = (int)((s.y - std::floor(s.y)) * Constants::positionResolution);//Y方向在cell中的偏移量
    int iYaw = (int)(s.yaw / Constants::deltaHeadingRad);
    int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iYaw;
    int cX;
    int cY;

    for (int i = 0; i < model->modellookup[idx].length; ++i) {
      cX = (X + model->modellookup[idx].pos[i].x);
      cY = (Y + model->modellookup[idx].pos[i].y);
      for(int ddx=-1;ddx<=1;ddx++)
      {
      for(int ddy=-1;ddy<=1;ddy++)
        {
          if(ddx==0 && ddy==0) continue;
          cX=cX+ddx;
          cY=cY+ddy;

          if (cX >= 0 && (unsigned int)cX < grid->info.width && cY >= 0 && (unsigned int)cY < grid->info.height) {
        if (!grid->data[cY * grid->info.width + cX]||collidgrideobs(cX,cY)) {
          return true;
        }//若grid的某个小网格存在值，说明有障碍，则返回false表示不在自由网格
      }
      else return true;
        }
      }
      // make sure the configuration coordinates are actually on the grid

    }

    return false;//所有检测都没有检测到被占用，说明没有障碍，可以通行
  }
  bool startcollideObstacle(const State& s,const std::shared_ptr<Model>& model) {
    // return model->collideObstacle(s, obstacle);
    int X = std::floor(s.x);
    int Y = std::floor(s.y);
    int iX = (int)((s.x - std::floor(s.x)) * Constants::positionResolution);//得出X方向在cell中的偏移量
    int iY = (int)((s.y - std::floor(s.y)) * Constants::positionResolution);//Y方向在cell中的偏移量
    int iYaw = (int)(s.yaw / Constants::deltaHeadingRad);
    int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iYaw;
    int cX;
    int cY;

    for (int i = 0; i < model->modellookup[idx].length; ++i) {
      cX = (X + model->modellookup[idx].pos[i].x);
      cY = (Y + model->modellookup[idx].pos[i].y);
      for(int ddx=-1;ddx<=1;ddx++)
      {
      for(int ddy=-1;ddy<=1;ddy++)
        {
          if(ddx==0 && ddy==0) continue;
          cX=cX+ddx;
          cY=cY+ddy;

          if (cX >= 0 && (unsigned int)cX < grid->info.width && cY >= 0 && (unsigned int)cY < grid->info.height) {
        if (!grid->data[cY * grid->info.width + cX]||collidgrideobs(cX,cY)) {
          return true;
        }//若grid的某个小网格存在值，说明有障碍，则返回false表示不在自由网格
      }
      else return true;
        }
      }
      // make sure the configuration coordinates are actually on the grid

    }

    return false;//所有检测都没有检测到被占用，说明没有障碍，可以通行
  }
  bool corrid_collideObstacle(const State& s,const std::shared_ptr<Model>& model) {
    // return model->collideObstacle(s, obstacle);
    int cX = std::floor(s.x);
    int cY = std::floor(s.y);

      // for(int ddx=-1;ddx<=1;ddx++)
      // {
      // for(int ddy=-1;ddy<=1;ddy++)
      //   {
          // if(ddx==0 && ddy==0) continue;
          // cX=cX+ddx;
          // cY=cY+ddy;

          if (cX >= 0 && (unsigned int)cX < grid->info.width && cY >= 0 && (unsigned int)cY < grid->info.height) {
        if (!grid->data[cY * grid->info.width + cX]) {
          return true;
        }//若grid的某个小网格存在值，说明有障碍，则返回false表示不在自由网格
      }
      else return true;
      //   }
      // }
      // make sure the configuration coordinates are actually on the grid

    

    return false;//所有检测都没有检测到被占用，说明没有障碍，可以通行
  }


};
