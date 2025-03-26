#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
#define USE_FIBONACCI_HEAP
#include <boost/heap/d_ary_heap.hpp>

#include <boost/program_options.hpp>
#ifdef USE_FIBONACCI_HEAP
#include <boost/heap/fibonacci_heap.hpp>
#endif
#include "neighbor.hpp"
#include "planresult.hpp"
// #define REBUILT_FOCAL_LIST
// #define CHECK_FOCAL_LIST



namespace libMultiRobotPlanning {


/*! \brief Spatiotemporal Hybrid A* Algorithm.*/
// 这个类实现了时空混合状态A*算法。**注意到，如果执行正确，它也可以作为传统的Hybrid-State star
// 这个类可以使用fibonacci堆，也可以使用d-ary堆。前者是默认值。注释“USE FIBONACCI HEAP”改为使用d-ary堆。
// \tparam State   搜索的自定义状态。需要可复制
// \tparam Action  搜索的自定义操作。需要可复制
// \tparam Cost    自定义成本类型(整数或浮点类型)
// \tparam Environment 他的类需要提供自定义的混合A*逻辑。需要支持以下函数:
//   - Cost admissibleHeuristic(const State& s)
//     如果没有合适的启发式方法可用，该函数将返回0。
//
//   - bool isSolution(const State& s, Cost gscore,
//       std::unordered_map<State, std::tuple<State, Action, Cost, Cost>， StateHasher>& cameFrom)
//      如果给定状态接近目标状态，并且它有一个到目标的无碰撞路径，则返回true。还需要将生成路径到目标插入到当前的cameFrom映射中。
//
//   - State getGoal()
//     返回当前agent的目标状态。
//
//   - void getNeighbors(const State& s, std::vector<Neighbor<State, Action,Cost> >& neighbors)`
//     返回当前state的neighbor
//
//   - int calcIndex(const State& s)
//     此函数计算给定状态的索引(int)，该索引用作closelist中的键。
//
//   - void onExpandNode(const State& s, Cost fScore, Cost gScore)
//     该函数在每次展开时调用，并可用于统计目的。（已不用）
//
//   - void onDiscover(const State& s, Cost fScore, Cost gScore)
//     该函数在每个节点发现时调用，并可用于统计目的。（已不用）
//
// \tparam StateHasher 将state转换为hash的类 Default:std::hash<State>




template <typename State, typename Cost, typename Environment,
          typename StateHasher = std::hash<State>>
class HybridAStar {
 public:
  HybridAStar(Environment& environment, float w): m_env(environment), m_w(w) {}
  ~HybridAStar() {}







  //###################################################
  // hybrid_astar查找函数          search
  // 起始状态                   startState,
  // astar得到的路径             solution,
  // 初始cost默认为0           initialCost
  //###################################################
  bool search(const State& startState,
              PlanResult& solution, Cost initialCost = 0) {

    // 对solution进行初始化 清0
    solution.states.clear();
    solution.cost = 0;
    

    // openset      容器存放Node
    // closeset     容器存放Node
    // cameForm     容器是存放 state parent_state action fsocre gscore
    //              （主要是存放neighbor和生成的无碰撞轨迹）  其实跟close 功能一样
    // stateToHeap  容器存放index和node
    //              （存放所有被找过的节点 只要被search过就放入statetoheap） 跟open功能一样
    openSet_t openSet;
    focalSet_t  focalSet;  // subset of open nodes that are within suboptimality bound
    std::unordered_map<uint64_t, fibHeapHandle_t, std::hash<uint64_t>> stateToHeap;
    std::unordered_set<uint64_t, std::hash<uint64_t>> closedSet;
    std::unordered_map<State, std::tuple<State, Cost, Cost>,StateHasher>   cameFrom;
    auto handle =
        openSet.push(Node(startState,m_env.admissibleHeuristic(startState), initialCost,0));
    stateToHeap.insert(std::make_pair<>(m_env.calcIndex(startState), handle));
     (*handle).handle = handle;
    focalSet.push(handle);

    // 为存储 neighbor 容器留足空间
    std::vector<Neighbor> neighbors;
    neighbors.reserve(10);
    Cost bestFScore = (*handle).fScore;
    
    while (!openSet.empty()) {
#ifdef REBUILT_FOCAL_LIST
      focalSet.clear();
      const auto& top = openSet.top();
      
      Cost bestVal = top.fScore;
      auto iter = openSet.ordered_begin();
      auto iterEnd = openSet.ordered_end();
      for (; iter != iterEnd; ++iter) {
        Cost val = iter->fScore;
        if (val <= bestVal * m_w) {
          const auto& s = *iter;
          focalSet.push(s.handle);
        } else 
          break;
      }
#else
      {
        Cost oldBestFScore = bestFScore;
        bestFScore = openSet.top().fScore;
        // std::cout << "bestFScore: " << bestFScore << std::endl;
        if (bestFScore > oldBestFScore) {
          // std::cout << "oldBestFScore: " << oldBestFScore << " newBestFScore:
          // " << bestFScore << std::endl;
          auto iter = openSet.ordered_begin();
          auto iterEnd = openSet.ordered_end();
          for (; iter != iterEnd; ++iter) {
            Cost val = iter->fScore;
            if (val > oldBestFScore * m_w && val <= bestFScore * m_w) {
              const Node& n = *iter;
              focalSet.push(n.handle);
            }
            if (val > bestFScore * m_w) 
              break;
          }
        }
      }
#endif
// check focal list/open list consistency
#ifdef CHECK_FOCAL_LIST
      {
        // focalSet_t focalSetGolden;
        bool mismatch = false;
        const auto& top = openSet.top();
        Cost bestVal = top.fScore;
        auto iter = openSet.ordered_begin();
        auto iterEnd = openSet.ordered_end();
        for (; iter != iterEnd; ++iter) {
          const auto& s = *iter;
          Cost val = s.fScore;
          if (val <= bestVal * m_w) {
            // std::cout << "should: " << s << std::endl;
            // focalSetGolden.push(s.handle);
            if (std::find(focalSet.begin(), focalSet.end(), s.handle) ==
                focalSet.end()) {
              std::cout << "focalSet misses: " << s << std::endl;
              mismatch = true;
            }

          } else {
            if (std::find(focalSet.begin(), focalSet.end(), s.handle) !=
                focalSet.end()) {
              std::cout << "focalSet shouldn't have: " << s << std::endl;
              mismatch = true;
            }
            // break;
          }
        }
        assert(!mismatch);
        // assert(focalSet == focalSetGolden);
      }
#endif
      // 找出f最小的 openset的node 如果f值相同比较g值
      auto currentHandle = focalSet.top();
      Node current = *currentHandle;
      // Node current = openSet.top();
      
      // 计数进行low-level次数
      m_env.onExpandNode(current.state, current.fScore, current.gScore);
      // 是否能直接生成路径
      if (m_env.isSolution(current.state, current.gScore, cameFrom)) {
//         std::cout << "current"<<current.state<<std::endl;
        solution.states.clear();
        auto iter = cameFrom.find(m_env.getGoal());
        solution.cost = std::get<2>(iter->second);
         // current.fScore;
        solution.fmin = std::get<2>(iter->second) +
                              m_env.admissibleHeuristic(iter->first);

        // 将存储在cameFrom容器中的 放入solution中
        while (iter != cameFrom.end()) {
          // std::cout << " From " << std::get<0>(iter->second)
          //           << " to Node:" << iter->first
          //           << " with ACTION: " << std::get<1>(iter->second) << "
          //           cost "
          //           << std::get<2>(iter->second) << " g_score "
          //           << std::get<3>(iter->second) << std::endl;
          solution.states.push_back(iter->first);
          iter = cameFrom.find(std::get<0>(iter->second));//父亲状态
        }
        solution.states.push_back(startState);
        std::reverse(solution.states.begin(), solution.states.end());
        solution.states.back().v = 0;
        solution.cost = current.gScore;
        solution.fmin = openSet.top().fScore;
        openSet.clear();

        return true;
      }

      // openSet.pop();

      focalSet.pop();
      openSet.erase(currentHandle);
      stateToHeap.erase(m_env.calcIndex(current.state));
      closedSet.insert(m_env.calcIndex(current.state));
      // traverse neighbors 找neighbor
      neighbors.clear();
      m_env.getNeighbors(current.state, neighbors);
      for (const Neighbor& neighbor : neighbors) {
        // not in closed set 判断neighbor是否在close list中
        if (closedSet.find(m_env.calcIndex(neighbor.state)) ==  closedSet.end()) {
            Cost tentative_gScore = current.gScore + neighbor.cost;
            auto iter = stateToHeap.find(m_env.calcIndex(neighbor.state));
            // Discover a new node 如果这个节点之前未被search过
            if (iter == stateToHeap.end()) {
              Cost fScore =
                  tentative_gScore + m_env.admissibleHeuristic(neighbor.state);
              Cost focalHeuristic =
                current.focalHeuristic +
                m_env.focalStateHeuristic(neighbor.state);
              auto handle = openSet.push(Node(neighbor.state,fScore, tentative_gScore, focalHeuristic));
              (*handle).handle = handle;
              stateToHeap.insert(std::make_pair<>(m_env.calcIndex(neighbor.state), handle));
              if (fScore <= bestFScore * m_w) {
              // std::cout << "focalAdd: " << *handle << std::endl;
              focalSet.push(handle);
              }
            }
            // 否则更新节点数据
            else {
              auto handle = iter->second;
              // std::cout << "  this is an old node: " << tentative_gScore << ","
              // << (*handle).gScore << std::endl;
              // We found this node before with a better path
              if (tentative_gScore >= (*handle).gScore) {
                continue;
              }
              Cost last_gScore = (*handle).gScore;
              Cost last_fScore = (*handle).fScore;
              // update f and gScore
              Cost delta = last_gScore - tentative_gScore;
              (*handle).gScore = tentative_gScore;
              (*handle).fScore -= delta;
              (*handle).state = neighbor.state;
              openSet.increase(handle);
              // m_env.onDiscover(neighbor.state, (*handle).fScore,
                              //  (*handle).gScore);
              if ((*handle).fScore <= bestFScore * m_w &&
                last_fScore > bestFScore * m_w) {
                // std::cout << "focalAdd: " << *handle << std::endl;
                focalSet.push(handle);
              }
            }

            // Best path for this node so far
            // TODO: this is not the best way to update "cameFrom", but otherwise
            // default c'tors of State and Action are required
            cameFrom.erase(neighbor.state);
            cameFrom.insert(std::make_pair<>(neighbor.state,std::make_tuple<>(current.state,neighbor.cost,tentative_gScore)));
          }
        }
    }
    openSet.clear();
    return false;
  }






 private:
  struct Node;

  //###################################################
  // 类                           Node
  // 状态                     State,
  // 移动状态action           action,
  // cost f = g + h          fScore
  // gcost                   gScore
  // 比较方式           1. lowest fScore
  //                   2. highest gScore
  //###################################################
  
#ifdef USE_FIBONACCI_HEAP
  typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
  typedef typename openSet_t::handle_type fibHeapHandle_t;
#else
  typedef typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                           boost::heap::mutable_<true> >
      openSet_t;
  typedef typename openSet_t::handle_type fibHeapHandle_t;

#endif

  struct Node {
    Node(const State& state, Cost fScore, Cost gScore, Cost focalHeuristic)
        : state(state),
          fScore(fScore),
          gScore(gScore),
          focalHeuristic(focalHeuristic) {}

    bool operator<(const Node& other) const {
      // Sort order
      // 1. lowest fScore
      // 2. highest gScore

      // Our heap is a maximum heap, so we invert the comperator function here左大于右建立小顶堆
      if (fScore != other.fScore) {
        return fScore > other.fScore;
      } else {
        return gScore < other.gScore;
      }
    }

    friend std::ostream& operator<<(std::ostream& os, const Node& node) {
      os << "state: " << node.state << " fScore: " << node.fScore
         << " gScore: " << node.gScore << " focal: " << node.focalHeuristic;
      return os;
    }

    State state;

    Cost fScore;
    Cost gScore;
    Cost focalHeuristic;

    fibHeapHandle_t handle;
  };

struct compareFocalHeuristic {
  bool operator()(const fibHeapHandle_t& h1,
                  const fibHeapHandle_t& h2) const {
    // Sort order (see "Improved Solvers for Bounded-Suboptimal Multi-Agent
    // Path Finding" by Cohen et. al.)
    // 1. lowest focalHeuristic
    // 2. lowest fScore
    // 3. highest gScore

    // Our heap is a maximum heap, so we invert the comperator function here
    if ((*h1).focalHeuristic != (*h2).focalHeuristic) {
      return (*h1).focalHeuristic > (*h2).focalHeuristic;
    } else if ((*h1).fScore != (*h2).fScore) {
      return (*h1).fScore > (*h2).fScore;
    } else {
      return (*h1).gScore < (*h2).gScore;
    }
  }
};

#ifdef USE_FIBONACCI_HEAP
  typedef typename boost::heap::fibonacci_heap<
      fibHeapHandle_t, boost::heap::compare<compareFocalHeuristic> >
      focalSet_t;
#else
  typedef typename boost::heap::d_ary_heap<
      fibHeapHandle_t, boost::heap::arity<2>, boost::heap::mutable_<true>,
      boost::heap::compare<compareFocalHeuristic> >
      focalSet_t;
#endif


 private:
  Environment& m_env;
  float m_w;
};

}  // namespace libMultiRobotPlanning
