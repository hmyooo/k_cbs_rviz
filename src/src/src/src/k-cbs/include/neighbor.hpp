#pragma once
#include "state.hpp"

namespace libMultiRobotPlanning {

/*! \brief Represents state transations

    This class represents a transition from a start state applying an action
   with the given cost.

    \tparam State Custom state for the search. Needs to be copy'able
    \tparam Action Custom action for the search. Needs to be copy'able
    \tparam Cost Custom Cost type (integer or floating point types)
*/
//###################################################
//                           类 Neighbor  
// 这个类表示从应用具有给定代价的操作的开始状态的转换。
// 搜索的自定义状态。需要可复制      State
// 搜索的自定义操作。需要可复制      Action
// 自定义成本类型(整数或浮点类型)    Cost
//###################################################
struct Neighbor {
  Neighbor(const State& state, Cost cost)
      : state(state), cost(cost) {}

  //! neighboring state
  State state;
  //! cost to get to the neighboring state
  Cost cost;
};

}  // namespace libMultiRobotPlanning
