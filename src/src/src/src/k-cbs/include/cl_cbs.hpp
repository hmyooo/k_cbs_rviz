#pragma once

#include <chrono>
#include <map>
#include "demo_agent.hpp"
#include "hybrid_astar.hpp"

#define MAX_RUNTIME 9999
namespace libMultiRobotPlanning {

/*! \brief 
CAR_Like CBS
类车CBS冲突搜索算法解决类车机器人多agent寻路问题

它应用身体冲突树来处理考虑代理形状的冲突。
它采用一种新的时空混合状态A*算法作为单智能体路径规划器，生成同时满足运动学和时空约束的路径。
为了提高效率，该文件还集成了CL-CBS方法的顺序规划版本。

\tparam State       搜索的自定义状态。需要可复制
\tparam Action      搜索的自定义操作。需要可复制
\tparam Cost        自定义成本类型(整数或浮点类型)
\tparam Conflict    自定义冲突描述。冲突需要能够转化为约束。
\tparam Constraints 定义约束描述.
\tparam Environment 这个类需要提供自定义逻辑。特别地，它需要支持以下功能
  - void setLowLevelContext(size_t agentIdx, const Constraints* constraints)
    将当前设置为具有给定约束集的特定代理

  - Cost admissibleHeuristic(const State& s)
    启发函数的代价.需要考虑到当前的环境。

  - bool isSolution(const State& s, Cost gscore,
      std::unordered_map<State, std::tuple<State, Action, Cost, Cost>,StateHasher>& cameFrom)
     如果给定状态是当前代理的目标状态，则返回true。在from结构中添加解析展开。

  - void getNeighbors(const State& s, std::vector<Neighbor<State, Action, int>>& neighbors)
    为给定的状态和当前agent找到neighbor列表。

  - bool getFirstConflict(const std::vector<PlanResult<State, Action, int> >&solution, Conflict& result)
    为每个agent找到给定解决方案的第一个主体冲突。如果发现冲突冲突则返回true，否则返回false。

  - void createConstraintsFromConflict(const Conflict& conflict,std::map<size_t, Constraints>& constraints)
    为给定的冲突创建约束列表。

  - void onExpandHighLevelNode(Cost cost)
    该函数在每次高级扩展时调用，并可用于统计目的。

  - void onExpandLowLevelNode(const State& s, Cost fScore, Cost gScore)
    该函数在每个低级展开时都被调用，并可用于统计目的。
*/

template <typename State, typename Action, typename Cost, typename Conflict,
          typename Constraints, typename Environment>
class CL_CBS {
 public:
  //###################################################
  // 构造函数                        CL_CBS
  // environment 搜索的环境
  //###################################################
  CL_CBS(Environment& environment, double w) : m_env(environment), m_w(w) {}
  ~CL_CBS() {}




  //###################################################
  // 搜索path                        search
  // initialStates    初始状态
  // solution         应该返回的path
  //###################################################
  bool search(void) {
    HighLevelNode start;
    start.solution.resize(m_env.agents.size());
    start.constraints.resize(m_env.agents.size());
    start.cost = 0;
    start.LB = 0;
    start.id = 0;
    std::chrono::high_resolution_clock::time_point
        low_startTime,low_endTime;

    // 初始化 对每个agent 进行一次low-level search 
    for (size_t i = 0; i < m_env.agents.size(); ++i) {
//        if(m_env.agents[i].start.time == m_env.agents[i].solution.states.size()){
//            start.solution[i] = m_env.agents[i].solution;
//            start.cost += start.solution[i].cost;
//            continue;
//        }
      State start_s = m_env.agents[i].start;
//      start_s.v = 0;
      start_s.time = 0;
      // 初始化low-level的search LowLevelSearch_t即hybrid A*
      LowLevelEnvironment llenv(m_env, i, start.constraints[i],start.solution);
      LowLevelSearch_t lowLevel(llenv, m_w);
      // 进行low-level search
      low_startTime = std::chrono::high_resolution_clock::now();
      bool success = lowLevel.search(start_s, start.solution[i]);
      if (!success) {
        return false;
      }
      start.cost += start.solution[i].cost;
      start.LB += start.solution[i].fmin;
      low_endTime = std::chrono::high_resolution_clock::now();
      std::cout << "\033[1m\033[31m No." << m_env.agents[i].model->index << " low level success: " << std::chrono::duration_cast<std::chrono::duration<double>>
                   (low_endTime -low_startTime).count() << "  \033[0m\n";;

//        solution = P.solution;

    }
    start.focalHeuristic = m_env.focalHeuristic(start.solution);//碰撞次数
//    for(size_t i = 0;i< start.solution.size();i++){
//        m_env.agents[i].solution = start.solution[i];
//    }
//    return true;
    // std::priority_queue<HighLevelNode> open;
    // 优先级队列 std::priority_queue<HighLevelNode> open;
    openSet_t open; //高层节点HighLevelNode 自己定义了大小判断符 > 的标准(比较总cost),故直接排列就好
    // focal需要再次改变 判断大小函数的 定义改为比较focal Herisutic
    focalSet_t focal;
       
    auto handle = open.push(start);
    (*handle).handle = handle; //每个高层节点都有自己在openlist里的位置handle(底层也是)
    focal.push(handle);
    Cost bestCost = (*handle).cost;
    std::chrono::high_resolution_clock::time_point
        startTime = std::chrono::high_resolution_clock::now(),endTime;

//    solution.clear();
    int id = 1;
    while (!open.empty()) {
#ifdef REBUILT_FOCAL_LIST
      focal.clear();
      Cost LB = open.top().LB;
      auto iter = open.ordered_begin();
      auto iterEnd = open.ordered_end();
      for (; iter != iterEnd; ++iter){
        float val = iter->cost;
        // std::cout << "  cost: " << val << std::endl;
        if (val <= LB * m_w){
          const HighLevelNode &node = *iter;
          focal.push(node.handle);
        }
        else
          break;
      }
#else
      {
        //更新高层的界限cost
        Cost oldBestCost = bestCost;
        bestCost = open.top().cost;
        // std::cout << "bestFScore: " << bestFScore << std::endl;
        if (bestCost > oldBestCost){
          // std::cout << "oldBestCost: " << oldBestCost << " bestCost: " <<
          // bestCost << std::endl;
          auto iter = open.ordered_begin();
          auto iterEnd = open.ordered_end();
          for (; iter != iterEnd; ++iter){
            Cost val = iter->cost;
            if (val > oldBestCost * m_w && val <= bestCost * m_w){
              const HighLevelNode &n = *iter;
              focal.push(n.handle);
            }
            if (val > bestCost * m_w)
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
        const auto &top = open.top();
        Cost bestCost = top.cost;
        auto iter = open.ordered_begin();
        auto iterEnd = open.ordered_end();
        for (; iter != iterEnd; ++iter){
          const auto &s = *iter;
          Cost val = s.cost;
          if (val <= bestCost * m_w){
            // std::cout << "should: " << s << std::endl;
            // focalSetGolden.push(s.handle);
            if (std::find(focal.begin(), focal.end(), s.handle) ==
                focal.end()){
              std::cout << "focal misses: " << s << std::endl;
              mismatch = true;
            }
          }
          else{
            if (std::find(focal.begin(), focal.end(), s.handle) !=
                focal.end()){
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



      endTime = std::chrono::high_resolution_clock::now();
//      if (std::chrono::duration_cast<std::chrono::duration<double>>
//            (endTime -startTime).count() > MAX_RUNTIME) {
//        open.clear();
//        std::cout << "\033[1m\033[31m Plan out of runtime time! \033[0m\n";
//        return false;
//      }

      auto h = focal.top(); //返回focal里的值还是指针呀?
      HighLevelNode P = *h;
      // 计数high-level搜索次数//       std::cout << "expand: " << P << std::endl;

      focal.pop();
      open.erase(h);
      Conflict conflict;
      // 若没有碰撞 则返回true
      if (!m_env.getFirstConflict(P.solution, conflict)) {
        // std::cout << "done; cost: " << P.cost << std::endl;
          for(size_t i = 0;i< P.solution.size();i++){
              P.solution[i].states.insert(P.solution[i].states.begin(),
                                          m_env.agents[i].solution.states.begin(), m_env.agents[i].solution.states.end());
              m_env.agents[i].solution = P.solution[i];
          }
//        solution = P.solution;
        return true;
      }

      // 若有碰撞 则生成constraints
      // create additional nodes to resolve conflict
//       std::cout << "Found conflict: " << conflict << std::endl;

      std::map<size_t, Constraints> constraints;
      m_env.createConstraintsFromConflict(conflict, constraints);
      for (const auto& c : constraints) {
//         std::cout << "Add HL node for " << c.first << std::endl;
        size_t i = c.first;
        // std::cout << "create child with id " << id << std::endl;
        HighLevelNode newNode = P;
        newNode.id = id;
        // (optional) check that this constraint was not included already
//         std::cout << newNode.constraints[i] << std::endl;
//         std::cout << c.second << std::endl;{
        assert(!newNode.constraints[i].overlap(c.second));

        newNode.constraints[i].add(c.second);
        newNode.LB -= newNode.solution[i].fmin; // solution.fmin = openSet.top().fScore;
        newNode.cost -= newNode.solution[i].cost;

        LowLevelEnvironment llenv(m_env, i, newNode.constraints[i],newNode.solution);
        LowLevelSearch_t lowLevel(llenv,m_w);
        State start_s = State(m_env.agents[i].start,0);
        std::cout << "\033[1m\033[31m No." << m_env.agents[i].model->index << " plan \033[0m\n"<<std::endl;
        low_startTime = std::chrono::high_resolution_clock::now();

        bool success = lowLevel.search(start_s, newNode.solution[i]);
        newNode.cost += newNode.solution[i].cost;
        if (success) {
            low_endTime = std::chrono::high_resolution_clock::now();
            std::cout << "\033[1m\033[31m No." << m_env.agents[i].model->index << " low level success: " << std::chrono::duration_cast<std::chrono::duration<double>>
                         (low_endTime -low_startTime).count() << "  \033[0m\n";
//           std::cout << "  success. cost: " << newNode.cost << std::endl;
            auto handle = open.push(newNode);
            (*handle).handle = handle;
            if (newNode.cost <= bestCost * m_w)
              focal.push(handle);
        }
        else{
//            m_env.createConstraintsFromConflict(conflict, constraints,true);
            std::cout << "\033[1m\033[31m No." << m_env.agents[i].model->index << " plan fail \033[0m\n";
        }
            ++id;

      }
    }

    return false;
  }

 
 
 
 private:
  //###################################################
  //                        结构体 HighLevelNode
  //    solution    cbs结果 生成的路径点
  //  constraints    动态约束
  //      cost        代价
  //       id       agent的编号
  //###################################################
  struct HighLevelNode {
    std::vector<PlanResult> solution;
    std::vector<Constraints> constraints;
    Cost cost;
    Cost LB;            // 所有solution的fmin
    Cost focalHeuristic; //碰撞启发
    int id;
    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>::handle_type  handle;

    bool operator<(const HighLevelNode& n) const {
      // if (cost != n.cost)
      return cost > n.cost;
      // return id > n.id;
    }

    friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& c) {
      os << "id: " << c.id << " cost: " << c.cost << std::endl;
      for (size_t i = 0; i < c.solution.size(); ++i) {
        os << "Agent: " << i << std::endl;
        os << " States:" << std::endl;
        for (size_t t = 0; t < c.solution[i].states.size(); ++t) {
          os << "  " << c.solution[i].states[t] << std::endl;
        }
        os << " Constraints:" << std::endl;
        os << c.constraints[i];
        os << " cost: " << c.solution[i].cost << std::endl;
      }
      return os;
    }
  };
  

  typedef typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                           boost::heap::mutable_<true> >
      openSet_t;
  typedef typename openSet_t::handle_type handle_t;
  struct compareFocalHeuristic
    {
      bool operator()(const handle_t &h1, const handle_t &h2) const
      {
        // Our heap is a maximum heap, so we invert the comperator function here
        if ((*h1).focalHeuristic != (*h2).focalHeuristic)
        {
          return (*h1).focalHeuristic > (*h2).focalHeuristic;
        }
        return (*h1).cost > (*h2).cost;
      }
    };

    typedef typename boost::heap::d_ary_heap<
      handle_t, boost::heap::arity<2>, boost::heap::mutable_<true>,
      boost::heap::compare<compareFocalHeuristic> >
      focalSet_t;
   
  //###################################################
  //                  结构体 LowLevelEnvironment
  //    env    底层environment 用的environment.cpp函数
  //###################################################
  struct LowLevelEnvironment {
    LowLevelEnvironment(Environment& env, size_t agentIdx,
                        const Constraints& constraints,const std::vector<PlanResult>& solution)
        : m_env(env),m_solution(solution){
      m_env.setLowLevelContext(agentIdx, &constraints);
    }

    Cost admissibleHeuristic(const State& s) {
      return m_env.admissibleHeuristic(s);
    }
    Cost focalStateHeuristic(const State& s) {
        return m_env.focalStateHeuristic(s, m_solution);
    }
    bool isSolution(
        const State& s, Cost g,
        std::unordered_map<State, std::tuple<State, Cost, Cost>,std::hash<State>>& camefrom) {
      return m_env.isSolution(s, g, camefrom);
    }

    void getNeighbors(const State& s,std::vector<Neighbor>& neighbors) {
      m_env.getNeighbors(s, neighbors);
    }

    State getGoal() { return m_env.getGoal(); }

    int calcIndex(const State& s) { return m_env.calcIndex(s); }

    void onExpandNode(const State& s, Cost fScore, Cost gScore) {
      // std::cout << "LL expand: " << s << std::endl;
      m_env.onExpandLowLevelNode(s, fScore, gScore);
    }

    void onDiscover(const State& /*s*/, Cost /*fScore*/, Cost /*gScore*/) {
      // std::cout << "LL discover: " << s << std::endl;
      // m_env.onDiscoverLowLevel(s, m_agentIdx, m_constraints);
    }

   private:
    Environment& m_env;
    const std::vector<PlanResult>& m_solution;
  };




  //###################################################
  //                      变量及常量定义         
  //################################################### 
 private:
  Environment& m_env;
  double m_w;
  typedef HybridAStar<State, Cost, LowLevelEnvironment>   LowLevelSearch_t;
};
}
