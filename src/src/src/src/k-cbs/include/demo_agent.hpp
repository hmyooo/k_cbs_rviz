#pragma once

#include <vector>
#include "planresult.hpp"
#include "state.hpp"
#include "model.hpp"

namespace libMultiRobotPlanning {
struct staticobs{
    State location;
    double radius;
    int time;
    bool operator < (const staticobs& ti) const {
        return time < ti.time;
    }
    bool operator==(const staticobs& s) const {
      return std::tie(time, radius, location) == std::tie(s.time, s.radius, s.location);
    }
};

struct Agent {
    Agent() = default;
    Agent(std::shared_ptr<Model> model):model(model){
        repaln_flag = 0;
    }
    Agent(State begin,State end,std::shared_ptr<Model> model):
        model(model),start(begin),goal(end){
        repaln_flag = 0;
    }
    Agent(State begin,State end,Agent a):
        model(a.model),start(begin),goal(end){
        repaln_flag = a.repaln_flag;
    }
    ~Agent(){}
    bool operator < (const Agent& a) const {
        return solution.states.size() < a.solution.states.size();
    }


//    是否需要重规划
    unsigned int repaln_flag;

//    是否出现故障
    bool running_flag = true;

//    与故障碰撞时间(第一个int 代表故障编号 第二个int 代表 碰撞事件)
     std::vector<std::pair<staticobs, int>> collide_time;

    std::shared_ptr<Model> model;

    State start;

    State goal;

    std::vector<std::pair<double, double>> initial_path;

    PlanResult solution;

    std::vector<std::vector<std::pair<State, Cost>>> show_solution;
};

}  // namespace libMultiRobotPlanning
