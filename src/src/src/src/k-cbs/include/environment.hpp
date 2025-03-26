
#pragma once
#pragma warning(disable:4996)
#include <boost/heap/fibonacci_heap.hpp>
#include <unordered_map>
#include <unordered_set>
#include "constraints.hpp"
#include "checkcollision.hpp"
#include "neighbor.hpp"
#include "planresult.hpp"
#include "state.hpp"
#include "demo_agent.hpp"

#include <stdio.h>
#include <stdlib.h>

namespace libMultiRobotPlanning {

using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::Agent;
using libMultiRobotPlanning::State;
using namespace libMultiRobotPlanning;

/**
 * @brief  Environment class
 *
 * @tparam Location
 * @tparam State
 * @tparam Action
 * @tparam Cost
 * @tparam Conflict
 * @tparam Constraint
 * @tparam Constraints
 */

//  std::move() 拷贝
class Environment {
 public:
  Environment(std::multimap<int, std::pair<State, std::shared_ptr<Model>>> dynamic_obstacles,
    std::shared_ptr<Constants::map > map, std::vector<Agent>& _agents,
              std::vector<Location> static_obs = std::vector<Location>())
      : agents(_agents),m_dynamic_obstacles(std::move(dynamic_obstacles)),
        m_map(map),m_agentIdx(0),m_constraints(nullptr),m_lastGoalConstraint(-1),m_highLevelExpanded(0),
        m_lowLevelExpanded(0),m_collision(map,static_obs){
        holonomic_cost_maps = std::vector<std::vector<std::vector<double>>>
        (agents.size(), std::vector<std::vector<double>>(m_map->info.width, std::vector<double>(m_map->info.height, 0)));
        m_unique_dynamic.resize(agents.size());
        for(int i = 0; i < agents.size();i++){
            agents[i].goal = State(agents[i].goal.x, agents[i].goal.y, Constants::normalizeHeadingRad(agents[i].goal.yaw));
    //        agents[i].start = State(agents[i].start.x, agents[i].start.y, Constants::normalizeHeadingRad(agents[i].start.yaw));
            if(!startValid(agents[i].start)){
                std::cout<< "agents"<<agents[i].model->index<<"  start collide!!"<<std::endl;
                return;
            }
            if(static_obs.size()!=0){
                if(m_collision.collideobs(agents[i].goal,agents[i].model)||m_collision.collideobs(agents[i].start,agents[i].model)) {
                    std::cout << "collide with static obs"<<std::endl;
                    return;
                  }
                }
            for(int k = 0; k < agents.size();k++){
                if(i==k) continue;
                if(agents[i].start.time  < agents[k].start.time){
                    for(int j = agents[i].start.time; j <= agents[k].start.time; j++)
                        m_unique_dynamic[i].insert(std::pair<int, std::pair<State, std::shared_ptr<Model>>>
                       (j ,std::make_pair(agents[k].solution.states[j],agents[k].model)));
                }

            }

        }
    updateCostmap();

  }
  Environment( std::shared_ptr<Constants::map > map, std::vector<Agent>& _agents,
              std::vector<Location> static_obs = std::vector<Location>())
      : agents(_agents),
        m_map(map),m_agentIdx(0),m_constraints(nullptr),m_lastGoalConstraint(-1),m_highLevelExpanded(0),
        m_lowLevelExpanded(0),m_collision(map,static_obs){
        holonomic_cost_maps = std::vector<std::vector<std::vector<double>>>
        (agents.size(), std::vector<std::vector<double>>(m_map->info.width, std::vector<double>(m_map->info.height, 0)));
        m_unique_dynamic.resize(agents.size());
        for(int i = 0; i < agents.size();i++){
            agents[i].goal = State(agents[i].goal.x, agents[i].goal.y, Constants::normalizeHeadingRad(agents[i].goal.yaw));
    //        agents[i].start = State(agents[i].start.x, agents[i].start.y, Constants::normalizeHeadingRad(agents[i].start.yaw));
            if(!startValid(agents[i].start)){
                std::cout<< "agents"<<agents[i].model->index<<"  start collide!!"<<std::endl;
                return;
            }
            if(static_obs.size()!=0){
                if(m_collision.collideobs(agents[i].goal,agents[i].model)||m_collision.collideobs(agents[i].start,agents[i].model)) {
                    std::cout << "collide with static obs"<<std::endl;
                    return;
                  }
                }
            for(int k = 0; k < agents.size();k++){
                if(i==k) continue;
                if(agents[i].start.time  < agents[k].start.time){
                    for(int j = agents[i].start.time; j <= agents[k].start.time; j++)
                        m_unique_dynamic[i].insert(std::pair<int, std::pair<State, std::shared_ptr<Model>>>
                       (j ,std::make_pair(agents[k].solution.states[j],agents[k].model)));
                }

            }

        }
    // updateCostmap();

  }



  Environment(const Environment &) = delete;
  // Environment &operator=(const Environment &) = delete;



  //###################################################
  //获取冲突               getFirstConflict
  // solution     low-level 生成的各agent的path
  // result       若有碰撞返回result
  //###################################################
  /// High Level Environment functions
  bool getFirstConflict(
      const std::vector<PlanResult> &solution,Conflict &result) {
    int max_t = 0;
    int min_t = 9999;

      // 检查t时刻 agent之间有无碰撞
      for (size_t i = 0; i < solution.size(); ++i) {
        for (size_t j = i + 1; j < solution.size(); ++j) {
            max_t = std::max<int>(agents[i].start.time , agents[j].start.time);
            min_t = std::max<int>(agents[i].start.time + solution[i].states.size()*Constants::timeResolution,
                                  agents[j].start.time + solution[j].states.size()*Constants::timeResolution);
            if(max_t > min_t) continue;
            for (int t = max_t+Constants::timeResolution; t < min_t; t=t+Constants::timeResolution){
             State state1 = getState(i, solution, t - agents[i].start.time);
              State state2 = getState(j, solution, t - agents[j].start.time);
              if (m_collision.collideAgents(state1, state2, agents[i].model, agents[j].model)) {
                    result.time = t;
                    result.agent1 = i;
                    result.agent2 = j;
                    result.s1 =  state1;
                    result.s2 =  state2;
                     std::cout << "Found conflict: " << t << " collide "<<agents[i].model->index <<state1 <<" with "<<agents[j].model->index<<state2 << std::endl;
                    return true;
                }
            }
        }
      }
    return false;
  }




  //###################################################
  //获取冲突               getFirstConflict
  // solution     low-level 生成的各agent的path
  // result       若有碰撞返回result
  //###################################################
  void createConstraintsFromConflict(
      const Conflict &conflict, std::map<size_t, Constraints> &constraints,bool flag = false) {
//      double dis_goal2 = pow(agents[conflict.agent2].goal.x - conflict.s2.x,2 )+
//              pow(agents[conflict.agent2].goal.y - conflict.s2.y,2 );
//      double dis_goal1 = pow(agents[conflict.agent1].goal.x - conflict.s1.x,2 )+
//              pow(agents[conflict.agent1].goal.y - conflict.s1.y,2 );
//      if(flag == false){
//          if(dis_goal1 > dis_goal2){
//              Constraints c1;
//              c1.addConflict(conflict.time, conflict.s2, conflict.agent2);
//              constraints[conflict.agent1] = c1;
//          }
//          else{
//              Constraints c2;
//              c2.addConflict(conflict.time, conflict.s1, conflict.agent1);
//              constraints[conflict.agent2] = c2;
//          }
//      }
//     if(flag == true){
//         if(dis_goal1 <= dis_goal2){
//             Constraints c1;
//             c1.addConflict(conflict.time, conflict.s2, conflict.agent2);
//             constraints[conflict.agent1] = c1;
//         }
//         else{
//             Constraints c2;
//             c2.addConflict(conflict.time, conflict.s1, conflict.agent1);
//             constraints[conflict.agent2] = c2;
//         }
//      }
     Constraints c2;
     c2.addConflict(conflict.time, conflict.s1, conflict.agent1);
     constraints[conflict.agent2] = c2;
     Constraints c1;
     c1.addConflict(conflict.time, conflict.s2, conflict.agent2);
     constraints[conflict.agent1] = c1;

  }


  


  //###################################################
  //计数 进行high_level搜索次数     onExpandHighLevelNode
  //###################################################
  void onExpandHighLevelNode(int /*cost*/) {
    m_highLevelExpanded++;

//    if (m_highLevelExpanded % 50 == 0)
//      std::cout << "Now expand " << m_highLevelExpanded
//                << " high level nodes.\n"<<std::endl;
  }
  //###################################################
  // 计数 返回high_level搜索次数       highLevelExpanded
  //###################################################
  int highLevelExpanded() { return m_highLevelExpanded; }



  //###################################################
  // 初始化low_level搜索环境      setLowLevelContext
  //        agentIdx        agent编号
  //       constraints      约束
  //###################################################
  /// Low Level Environment functions
  void setLowLevelContext(size_t agentIdx,const Constraints *constraints) {
  // Constraints *constraints) {
    assert(constraints);  // NOLINT
    m_agentIdx = agentIdx;
    m_constraints = constraints;
    m_lastGoalConstraint = -1;
    for (const auto &c : constraints->constraints) {
      if (m_collision.collideAgents(agents[m_agentIdx].goal, c.s,
          agents[m_agentIdx].model, agents[c.agentid].model)) {
        m_lastGoalConstraint = std::max(m_lastGoalConstraint, c.time);
      }
    }

    // std::cout << "Setting Lowlevel agent idx:" << agentIdx
    //           << " Constraints:" << constraints->constraints.size()
    //           << "  lastGoalConstraints:" << m_lastGoalConstraint <<
    //           std::endl;
  }




  //###################################################
  // 返回 heuristic函数的值      admissibleHeuristic
  // 理论上是三个值的最大值 在environment里面去计算astar的值
  // 即计算holonomic-with-obstacles heuristic
  // 在model.h中已计算剩下两个值的最大值
  //###################################################
  double admissibleHeuristic(const State &s) {
      // holonomic-with-obstacles heuristic
      double twoDoffset =
          sqrt(pow((s.x - (int)s.x) -
                       (agents[m_agentIdx].goal.x - (int)agents[m_agentIdx].goal.x),
                   2) +
               pow((s.y - (int)s.y) -
                       (agents[m_agentIdx].goal.y - (int)agents[m_agentIdx].goal.y),
                   2));
      double twoDCost =
          holonomic_cost_maps[m_agentIdx][(int)s.x / Constants::mapResolution]
                             [(int)s.y / Constants::mapResolution] -
          twoDoffset;
      // std::cout << "holonomic cost:" << twoDCost << std::endl;

      double modelUniqueAdmissible =
          agents[m_agentIdx].model->admissibleHeuristic(s, agents[m_agentIdx].goal);
        modelUniqueAdmissible = 0;
      return std::max({modelUniqueAdmissible, twoDCost});
    }



  // 当前时刻的state,和其他同时刻路径共碰撞几次
    double focalStateHeuristic(const State &s,const std::vector<PlanResult> &solution){
      double numConflicts = 0;
      for (size_t i = 0; i < solution.size(); ++i) {
        if (i != m_agentIdx && !solution[i].states.empty()) {
          //所有高层解集里的其他非空路径解都进行判断
          State state2 = getState(i, solution, s.time - agents[i].start.time);
          if (m_collision.collideAgents(s,state2,agents[m_agentIdx].model, agents[i].model)) {
            ++numConflicts;
          }
        }
      }//TODO: check
      return numConflicts;
    }

  // 统计一个路径总共碰了多少次
    double focalHeuristic(const std::vector<PlanResult> &solution){
      double numConflicts = 0;

      int max_t = 0;//所有solution最大时刻
      for (const auto& sol : solution) {
        max_t = std::max<int>(max_t, sol.states.size() - 1);
      }

      for (int t = 0; t < max_t; ++t) {
        // check drive-drive vertex collisions
        for (size_t i = 0; i < solution.size(); ++i) {
          State state1 = getState(i, solution, t);
          for (size_t j = i + 1; j < solution.size(); ++j) {
            State state2 = getState(j, solution, t);
            if (m_collision.collideAgents(state1,state2, agents[i].model, agents[j].model)) {
              ++numConflicts;
            }
          }
        }

      }//TODO：check
      return numConflicts;
  }


  //###################################################
  //                                isSolution
  // 对于ackerman模型
  // 判断从当前位置到终点是否能生成一条无碰撞的rs曲线
  // 对于holonomic模型
  // 判断从当前位置到终点是否有无碰撞曲线
  // 并返回生成路径
  //###################################################
  bool isSolution(
      const State &state, double gscore,
      std::unordered_map<State, std::tuple<State, double, double>,std::hash<State>> &_camefrom) {
    std::vector<std::vector<std::tuple<State, double>>> paths;
//    for (auto it = m_constraints->constraints.begin();
//             it != m_constraints->constraints.end(); it++)
//        if (state.time + agents[m_agentIdx].start.time >=std::max(it->time - Constants::constraintWaitTime,0)
//              && state.time + agents[m_agentIdx].start.time <= it->time +Constants::constraintWaitTime )
//            return false;
    // 对于模型本身进行判断 是否有path
    if (agents[m_agentIdx].model->isSolution(state, agents[m_agentIdx].goal, paths) ==
        false)
      return false;
    for(auto path:paths){
        if (std::get<0>(path.back()).time <= m_lastGoalConstraint) {
            continue;
        }
        bool collide_flag = false;
        // 生成的路径是否有碰撞
        for (auto iter = path.begin(); iter != path.end(); iter++)
          if (!stateValid(std::get<0>(*iter)))
              collide_flag = true;
        if(collide_flag)    continue;
        // 返回生成路径
        for (auto iter = path.begin() + 1; iter != path.end(); iter++) {
          gscore += std::get<1>(*iter);
          _camefrom.insert(std::make_pair<>(
              std::get<0>(*iter),
              std::make_tuple<>(std::get<0>(*(iter - 1)),std::get<1>(*iter), gscore)));
        }

        agents[m_agentIdx].goal = std::get<0>(path.back());
        return true;

    }

    return false;
  }





  //###################################################
  //                                getNeighbors
  // 找neighbor 并判断是否满足约束 返回一组neighbor
  //###################################################
  void getNeighbors(const State &s,std::vector<Neighbor> &neighbors) {
    neighbors.clear();
    std::vector<Neighbor> modelNeighbours =
        agents[m_agentIdx].model->getNeighbors(s);
    for (auto n : modelNeighbours) {
      if (stateValid(n.state)) {
          if(std::sqrt(pow(n.state.x - s.x,2) + pow(n.state.y - s.y,2)) > Constants::mapResolution
                  && check_line(n.state, s))
                 neighbors.emplace_back(n);
          else if(std::sqrt(pow(n.state.x - s.x,2) + pow(n.state.y - s.y,2)) < Constants::mapResolution)
              neighbors.emplace_back(n);
      }
    }
  }





  //###################################################
  //                                getGoal
  // 返回当前agent的目标goal
  //##################################################
  State getGoal() { return agents[m_agentIdx].goal; }







  //#######################################################################################
  // 计算index                calcIndex
  // 状态                   state,
  // 计算   y *height *yaw *width + x *height *yaw +yaw *height +time
  //#######################################################################################
  uint64_t calcIndex(const State &s) {
    return (uint64_t)(s.y / Constants::mapResolution) *m_map->info.width *Constants::headings *m_map->info.height +
           (uint64_t)(s.x / Constants::mapResolution) *Constants::headings *m_map->info.height +
           (uint64_t)(Constants::normalizeHeadingRad(s.yaw) / Constants::yawResolution) *m_map->info.height +
           (uint64_t)s.time
           ;
  }





  //###################################################
  // 计数 进行low_level搜索次数      onExpandLowLevelNode
  //###################################################
  void onExpandLowLevelNode(const State & /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
  }
  //###################################################
  // 返回 进行low_level搜索次数      lowLevelExpanded
  //###################################################
  int lowLevelExpanded() const { return m_lowLevelExpanded; }





  //###################################################
  // 检查起始初始位置是否发生碰撞      startAndGoalValid
  //###################################################
  bool startAndGoalValid() {
    for (size_t i = 0; i < agents.size(); i++)
      for (size_t j = i + 1; j < agents.size(); j++) {
        if (m_collision.collideAgents(
                agents[i].goal, agents[j].goal, agents[i].model, agents[j].model)) {
          std::cout << "ERROR: Goal point of  collide!\n";
          return false;
        }
        if (m_collision.collideAgents(
                agents[i].start, agents[j].start, agents[i].model, agents[j].model)) {
          std::cout << "ERROR: Start point of collide!\n";
          return false;
        }
      }
    return true;
  }



  //###################################################
  //                 getLocation
  // 将state 转换成location
  //###################################################
  Location getLocation(const State& s) { return Location((int)s.x, (int)s.y); }





  //###################################################
  //                 isCommandValid
  // 返回最早可出发时间
  //###################################################
  bool isCommandValid(int earliestStartTime,      // can start motion at this time
      int earliestArrivalTime,    // can only arrive at (s+cmd)
      int& t) {
    t = std::max<int>(earliestArrivalTime, earliestStartTime);
    return true;
  }


 private:
  //###################################################
  //                 getState
  // 获取时刻t的agent[agentIdx]的状态state
  //###################################################
  State getState(size_t agentIdx,
       const std::vector<PlanResult> &solution,size_t t) {
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size() && t>=0) {
      int t_int=floor(t/Constants::timeResolution);
      return solution[agentIdx].states[t_int];
    }
    else if (t<0) {
        return solution[agentIdx].states.front();
    }
    assert(!solution[agentIdx].states.empty());
    return solution[agentIdx].states.back();
  }








  //###################################################
  //###################################################
  //###################################################
  //##############       还需要修改        ##############
  //###################################################
  //###################################################
  //###################################################

  //###################################################
  // 检查起始初始位置是否发生碰撞      startValid
  //###################################################
  bool startValid(const State &s) {
    double x_ind = s.x / Constants::mapResolution;
    double y_ind = s.y / Constants::mapResolution;
    if (x_ind < 0 || x_ind >= m_map -> info.width || y_ind < 0 || y_ind >= m_map -> info.height )
      return false;


    // 判断与障碍有无冲突
    if (m_collision.collideObstacle(s,agents[m_agentIdx].model))
      return false;


    // 若 cbs失败 把整条路径加到m_dynamic_obstacles中
    auto it = m_dynamic_obstacles.equal_range(s.time + agents[m_agentIdx].start.time);

    for (auto itr = it.first; itr != it.second; ++itr) {
      if (m_collision.collideAgents(
              s, itr->second.first, agents[m_agentIdx].model, itr->second.second))
        return false;
    }
    // 检查跟其他机器人的终点位置是否有碰撞
    auto itlow = m_dynamic_obstacles.lower_bound(-(s.time + agents[m_agentIdx].start.time));
    auto itup = m_dynamic_obstacles.upper_bound(-1);
    for (auto itr = itlow; itr != itup; ++itr)
      if (m_collision.collideAgents(
              s, itr->second.first, agents[m_agentIdx].model, itr->second.second))
        return false;

    if(m_collision.collideobs(s,agents[m_agentIdx].model))
        return false;
    return  true;
  }



  //###################################################
  // 位置是否发送冲突     stateValid
  // input              State
  //
  //###################################################
  bool stateValid(const State &s) {
    double x_ind = s.x / Constants::mapResolution;
    double y_ind = s.y / Constants::mapResolution;
    if (x_ind < 0 || x_ind >= m_map -> info.width || y_ind < 0 || y_ind >= m_map -> info.height )
      return false;


    // 判断与障碍有无冲突
    if (m_collision.collideObstacle(s,agents[m_agentIdx].model))
      return false;


    // 若 cbs失败 把整条路径加到m_dynamic_obstacles中
//    int lowwer = std::max(int(s.time + agents[m_agentIdx].start.time),0);
//    auto itlow = m_dynamic_obstacles.lower_bound(lowwer);
//    auto itup = m_dynamic_obstacles.upper_bound(s.time + agents[m_agentIdx].start.time );
    auto it = m_dynamic_obstacles.equal_range(s.time + agents[m_agentIdx].start.time);
    for (auto itr = it.first; itr != it.second; ++itr) {
      if (m_collision.collideAgents(
              s, itr->second.first, agents[m_agentIdx].model, itr->second.second))
        return false;
    }
    // 检查跟其他机器人的终点位置是否有碰撞
     auto itlow = m_dynamic_obstacles.lower_bound(-(s.time + agents[m_agentIdx].start.time));
     auto itup = m_dynamic_obstacles.upper_bound(-1);
    for (auto itr = itlow; itr != itup; ++itr)
      if (m_collision.collideAgents(
              s, itr->second.first, agents[m_agentIdx].model, itr->second.second))
        return false;

    if(m_collision.collideobs(s,agents[m_agentIdx].model))
        return false;

     it = m_unique_dynamic[m_agentIdx].equal_range(s.time + agents[m_agentIdx].start.time);
    for (auto itr = it.first; itr != it.second; ++itr) {
      if (m_collision.collideAgents(
              s, itr->second.first, agents[m_agentIdx].model, itr->second.second))
        return false;
    }

    // t时刻加了constraints 判断是否在之内
    for (auto it = m_constraints->constraints.begin();
         it != m_constraints->constraints.end(); it++)
      if (s.time + agents[m_agentIdx].start.time >=std::max(it->time - Constants::constraintWaitTime,0)
        && s.time + agents[m_agentIdx].start.time <= it->time + Constants::constraintWaitTime &&
        m_collision.collideAgents(it->s, s, agents[it->agentid].model,agents[m_agentIdx].model))
        return false;




    return true;
  }
bool struct_stateValid(const State &s) {
    double x_ind = s.x / Constants::mapResolution;
    double y_ind = s.y / Constants::mapResolution;
    if (x_ind < 0 || x_ind >= m_map -> info.width || y_ind < 0 || y_ind >= m_map -> info.height )
      return false;


    // 判断与障碍有无冲突
    if (m_collision.corrid_collideObstacle(s,agents[m_agentIdx].model))
      return false;


    // 若 cbs失败 把整条路径加到m_dynamic_obstacles中
//    int lowwer = std::max(int(s.time + agents[m_agentIdx].start.time),0);
//    auto itlow = m_dynamic_obstacles.lower_bound(lowwer);
//    auto itup = m_dynamic_obstacles.upper_bound(s.time + agents[m_agentIdx].start.time );
    // auto it = m_dynamic_obstacles.equal_range(s.time + agents[m_agentIdx].start.time);
    // for (auto itr = it.first; itr != it.second; ++itr) {
    //   if (m_collision.collideAgents(
    //           s, itr->second.first, agents[m_agentIdx].model, itr->second.second))
    //     return false;
    // }
    // // 检查跟其他机器人的终点位置是否有碰撞
    //  auto itlow = m_dynamic_obstacles.lower_bound(-(s.time + agents[m_agentIdx].start.time));
    //  auto itup = m_dynamic_obstacles.upper_bound(-1);
    // for (auto itr = itlow; itr != itup; ++itr)
    //   if (m_collision.collideAgents(
    //           s, itr->second.first, agents[m_agentIdx].model, itr->second.second))
    //     return false;

    // if(m_collision.collideobs(s,agents[m_agentIdx].model))
    //     return false;

    //  it = m_unique_dynamic[m_agentIdx].equal_range(s.time + agents[m_agentIdx].start.time);
    // for (auto itr = it.first; itr != it.second; ++itr) {
    //   if (m_collision.collideAgents(
    //           s, itr->second.first, agents[m_agentIdx].model, itr->second.second))
    //     return false;
    // }

    // t时刻加了constraints 判断是否在之内
    // for (auto it = m_constraints->constraints.begin();
    //      it != m_constraints->constraints.end(); it++)
    //   if (s.time + agents[m_agentIdx].start.time >=std::max(it->time - Constants::constraintWaitTime,0)
    //     && s.time + agents[m_agentIdx].start.time <= it->time + Constants::constraintWaitTime &&
    //     m_collision.collideAgents(it->s, s, agents[it->agentid].model,agents[m_agentIdx].model))
    //     return false;




    return true;
  }

  //###################################################
  //###################################################
  //###################################################
  //##############       还需要修改        ##############
  //###################################################
  //###################################################
  //###################################################


  //###################################################
  // 两点之间有无碰撞     check_line
  // input              State
  //
  //###################################################
  public:
    bool check_line(const State& s1,const State& s2){
        if(s1.x != s2.x){
            double k = (s2.y - s1.y)/(s2.x - s1.x);
            double minx,miny;

            minx=std::min(s1.x,s2.x);
            miny=std::min(s1.y,s2.y);
            for(double i = std::min(s1.x,s2.x); i <= std::max(s1.x,s2.x);
                i += 1){
                if(!stateValid(State(i, k*(i - minx)+miny))) return false;
            }
        }else
        for(double i = std::min(s1.y,s2.y); i <= std::max(s1.y,s2.y);
            i += 1){
            if(!stateValid(State(s1.x, i)))  return false;
        }
        return true;
    }

    bool  struct_corrid_check_line(const State& s1,const State& s2){
        if(s1.x != s2.x){
            double k = (s2.y - s1.y)/(s2.x - s1.x);
            double minx,miny;

            minx=std::min(s1.x,s2.x);
            miny=std::min(s1.y,s2.y);
            for(double i = std::min(s1.x,s2.x); i <= std::max(s1.x,s2.x);
                i += 1){
                if(!struct_stateValid(State(i, k*(i - minx)+miny))) return false;
            }
        }else
        for(double i = std::min(s1.y,s2.y); i <= std::max(s1.y,s2.y);
            i += 1){
            if(!struct_stateValid(State(s1.x, i)))  return false;
        }
        return true;
    }


  //###################################################
  //###################################################
  //                updateCostmap
  //###################################################
  //###################################################
  //###################################################
  // 堆中比较函数的定义 返回gcost中较小的      compare_node
  //###################################################
  struct compare_node {
    bool operator()(const std::pair<State, double> &n1,
                    const std::pair<State, double> &n2) const {
      return (n1.second > n2.second);
    }
  };

  //###################################################
  // 更新地图 计算astar到终点的参数      updateCostmap
  // 生成一张costmap
  //###################################################
  void updateCostmap() {

    // 定义一个堆 用的比较函数在上面定义了
    // 堆里面的元素 包括state 和 点到终点的gcost
    boost::heap::fibonacci_heap<std::pair<State, double>,
                boost::heap::compare<compare_node>>   heap;

    for (size_t idx = 0; idx < agents.size(); idx++) {
      heap.clear();
      int goal_x = (int)agents[idx].goal.x / Constants::mapResolution;
      int goal_y = (int)agents[idx].goal.y / Constants::mapResolution;
      heap.push(std::make_pair(State(goal_x, goal_y, 0), 0));

      while (!heap.empty()) {
        std::pair<State, double> node = heap.top();
        heap.pop();

        int x = node.first.x;
        int y = node.first.y;
        for (int dx = -1; dx <= 1; dx++)
          for (int dy = -1; dy <= 1; dy++) {
            if (dx == 0 && dy == 0) continue;
            int new_x = x + dx;
            int new_y = y + dy;
            if (new_x == goal_x && new_y == goal_y) continue;
            if (new_x >= 0 && new_x < m_map -> info.width && new_y >= 0 && new_y < m_map -> info.height
              &&holonomic_cost_maps[idx][new_x][new_y] == 0 && m_map ->data[new_y * m_map->info.width + new_x]
                    &&!m_collision.collidgrideobs(new_x,new_y)){
              holonomic_cost_maps[idx][new_x][new_y] =  holonomic_cost_maps[idx][x][y] +
                  sqrt(pow(dx * Constants::mapResolution, 2) + pow(dy * Constants::mapResolution, 2));
              heap.push(std::make_pair(State(new_x, new_y, 0),holonomic_cost_maps[idx][new_x][new_y]));

            }
          }
      }
    }

  }





 public:
  std::vector<Agent>& agents;
 private:
  std::vector<std::vector<std::vector<double>>> holonomic_cost_maps;
  std::vector<std::multimap<int, std::pair<State, std::shared_ptr<Model>>>>  m_unique_dynamic;
  std::multimap<int, std::pair<State, std::shared_ptr<Model>>>
      m_dynamic_obstacles;
  std::shared_ptr<Constants::map >  m_map;
  size_t m_agentIdx;
  const Constraints *m_constraints;
  int m_lastGoalConstraint;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
  CheckCollision<Model,State> m_collision;
  // double hight_level_conflicts = 0;
  // double 
};
}  // namespace libMultiRobotPlanning
