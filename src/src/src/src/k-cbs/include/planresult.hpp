#pragma once

#include <vector>
#include "state.hpp"

namespace libMultiRobotPlanning {

/*! \brief Represents the path for an agent

    This class is used to store the result of a planner for a single agent.
    It has both the ordered list of states that need to be traversed as well as
   the ordered
    list of actions together with their respective costs

    \tparam State Custom state for the search. Needs to be copy'able
    \tparam Action Custom action for the search. Needs to be copy'able
    \tparam Cost Custom Cost type (integer or floating point types)
*/
//###################################################
//                           类 PlanResult  
// 该类用于存储单个agent的规划的结果。它既有需要遍历的有序状态列表，也有有序状态列表
// 搜索的自定义状态。需要可复制      State
// 搜索的自定义操作。需要可复制      Action
// 实际代价值                     Cost
// 最小代价                       fmin
//###################################################
struct PlanResult {
  //! states and their gScore
  std::vector<State> states;
  //! actual cost of the result
  Cost cost;
  //! lower bound of the cost (for suboptimal solvers)
  Cost fmin;
  bool operator<(const PlanResult& n) const {
    return cost > n.cost;
  }
};

}  // namespace libMultiRobotPlanning
