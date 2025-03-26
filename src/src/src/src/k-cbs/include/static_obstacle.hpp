#pragma once
#include<algorithm>
#include "demo_agent.hpp"
namespace libMultiRobotPlanning {

class Static_obs {
 public:
    Static_obs(){};
    ~Static_obs(){};
    int cal_time = Constants::CALTime;
    void clear(){
        m_obs.clear();
    }
    static bool cmp_time(staticobs a, staticobs b){
        return a.time < b.time;
    }
    void set_current_time(int _t){current_time = _t;}
    void add_obs(State obs_s,int obs_t,double obs_r,int t){
        staticobs obs;
        obs.radius = obs_r;
        obs.location = obs_s;
        obs.time = obs_t;
        m_obs.push_back(obs);
        current_time = t;
        std::sort(m_obs.begin(),m_obs.end(),cmp_time);
    }
    void add_obs(Agent& agent,int obs_t){
        staticobs obs;
        obs.radius = agent.model->safetyRadius();
        obs.location = agent.solution.states.back();
        obs.time = obs_t;
        m_obs.push_back(obs);
        std::sort(m_obs.begin(),m_obs.end(),cmp_time);
    }
    int obs_collide_time(){
        return m_obs.back().time;
    }
    void cal_avoid_collide_agent(std::vector<Agent>& agents){
         for(auto obstacle:m_obs){
             for(size_t i = 0; i < agents.size();i++){
                 int size = agents[i].solution.states.size();
                 /*如果当前车辆可能受到该obs影响*/
                 if(size > obstacle.time){
                     for(int j = obstacle.time; j < size;j++)
                         if(collideobs(agents[i].solution.states[j],agents[i].model,obstacle)){
                             staticobs safeobs;
                             safeobs = obstacle;
                             safeobs.radius += Constants::obssafety_r;
//                             safeobs.radius = sqrt(pow(agents[i].solution.states[current_time].first.x -obstacle.location.x ,2)+
//                                                   pow(agents[i].solution.states[current_time].first.y -obstacle.location.y ,2)) -
//                                     Constants::obssafety_r;
//                             if(safeobs.radius < obstacle.radius + 20) safeobs.radius = obstacle.radius + 20;
                             for(int itr = obstacle.time; itr <= j;itr++){
                                if(collideobs(agents[i].solution.states[itr],agents[i].model,safeobs)){
                                 int time = std::max(current_time + 1 , itr);
//                                 time = current_time+1;
                                 if(agents[i].repaln_flag&&agents[i].start.time < time)
                                     break;
                                 agents[i].initial_path.clear();
                                 for(int k = time; k < agents[i].solution.states.size(); k++){
                                     auto s = agents[i].solution.states[k];
                                     agents[i].initial_path.emplace_back(std::pair<double, double>(s.x,s.y));
                                 }
                                 agents[i].repaln_flag =1;
                                 agents[i].start = agents[i].solution.states[time];
                                 agents[i].solution.states.assign(agents[i].solution.states.begin(),
                                 agents[i].solution.states.begin() + agents[i].start.time);

                                 break;
                                }
                             }
                             break;
                         }
                 }
             }
         }
    }


    /*当前时间已经超过start时间 需要重新规划 计算出 需要重规划的agent 和 需要加入的 dynamic*/
    void cal_changestart(std::vector<Agent>& agents,std::vector<Agent>& re_agents,
                        std::multimap<int, std::pair<State, std::shared_ptr<Model>>>& m_dynamic,int _cur){
         current_time = _cur;
         int replan_time = current_time ;
         int min_time,max_time;
         min_time = 999;max_time=0;
         for(auto a:agents){
             if(a.repaln_flag && a.solution.states.size() > replan_time)
                 replan_time = a.solution.states.size();
             if(a.repaln_flag && a.solution.states.size() < min_time)
                 min_time = a.solution.states.size();
             if( a.solution.states.size() > max_time)
                 max_time = a.solution.states.size();
         }
//         replan_time += 20;
         for(size_t i = 0; i < agents.size();i++){
             if(!agents[i].repaln_flag){
                 if(current_time >= (int)agents[i].solution.states.size()){
                     m_dynamic.insert(std::pair<int, std::pair<State, std::shared_ptr<Model>>>
                                (-agents[i].solution.states.size() ,std::make_pair(agents[i].solution.states.back(),agents[i].model)));
                 }
                 else
                     if(!Constants::same_time_falg){
                     for(int j = current_time; j < (int)agents[i].solution.states.size();j++)
                         m_dynamic.insert(std::pair<int, std::pair<State, std::shared_ptr<Model>>>
                                    (j ,std::make_pair(agents[i].solution.states[j],agents[i].model)));}
                     m_dynamic.insert(std::pair<int, std::pair<State, std::shared_ptr<Model>>>
                           (-agents[i].solution.states.size() ,std::make_pair(agents[i].solution.states.back(),agents[i].model)));
             }
             else{
                 agents[i].start = agents[i].solution.states.back();
                 if(!Constants::same_time_falg){

                      agents[i].start.time  += Constants::CALTime;
                      agents[i].start.v = 0;
                      auto terminal_s = agents[i].solution.states.back();
                     if(agents[i].start.time > agents[i].solution.states.size())
                     for(int j = agents[i].solution.states.size(); j <= agents[i].start.time; ++j)
                         agents[i].solution.states.push_back(State(terminal_s,j));
                    re_agents.push_back(agents[i]);
                 }
                 else {
                     agents[i].start.time  =  replan_time;
                     agents[i].start.v = 0;
                     auto terminal_s = agents[i].solution.states.back();
                    if(agents[i].start.time > agents[i].solution.states.size())
                    for(int j = agents[i].solution.states.size(); j <= agents[i].start.time; ++j)
                        agents[i].solution.states.push_back(State(terminal_s,j));
                   re_agents.push_back(agents[i]);
                 }
             }
         }

    }


    /* 直接加入obs的时候计算出 需要重规划的agent 和 需要加入的 dynamic
     * 由于 不会等待 所以不需要考虑 当计算不出来 需要等待的时候 车是否会影响其他车 */
    int add_dynaobs(std::vector<Agent>& agents,std::vector<Agent>& re_agents,
        std::multimap<int, std::pair<State, std::shared_ptr<Model>>>& m_dynamic,int _cur){
        current_time = _cur;
        max_time = current_time;

        for(size_t i =0; i < agents.size();i++){
            auto a = agents[i];
            if(a.solution.states.size() > max_time) max_time = a.solution.states.size();
            if(!a.repaln_flag){
                if(current_time >= (int)a.solution.states.size()){
                    m_dynamic.insert(std::pair<int, std::pair<State, std::shared_ptr<Model>>>
                               (-a.solution.states.size() ,
                                std::make_pair(a.solution.states.back(),a.model)));
                }
                else

                    for(int j = current_time; j < (int)a.solution.states.size();j++){
                        m_dynamic.insert(std::pair<int, std::pair<State, std::shared_ptr<Model>>>
                                   (a.solution.states[j].time, std::make_pair(a.solution.states[j],a.model)));
                        }
                        m_dynamic.insert(std::pair<int, std::pair<State, std::shared_ptr<Model>>>
                                   (-a.solution.states.size() ,
                                    std::make_pair(a.solution.states.back(),a.model)));


            }
            else{
                re_agents.push_back(agents[i]);
            }
        }
        return max_time;
    }




private:
    // 计算两点之间的距离
    double DistanceBetweenTwoPoints(State s1, State s2){
        return sqrt(pow(s2.x - s1.x,2) + pow(s2.y - s1.y,2) );
    }

    // 计算点(x, y)到经过两点(x1, y1)和(x2, y2)的直线的距离
    double DistanceFromPointToLine(State s, State s1, State s2){
        double a = s2.y - s1.y;
        double b = s1.x - s2.x;
        double c = s2.x * s1.y - s1.x * s2.y;

        assert(fabs(a) > 0.00001f || fabs(b) > 0.00001f);

        return fabs(a * s.x + b * s.y + c) / sqrt(a * a + b * b);
    }

    // 圆与矩形碰撞检测
    // 圆心(x, y), 半径r, 矩形中心(x0, y0), 矩形上边中心(x1, y1), 矩形右边中心(x2, y2)
    bool IsCircleIntersectRectangle(staticobs obs, State s0, State s1, State s2){
        float w1 = DistanceBetweenTwoPoints(s0, s2);
        float h1 = DistanceBetweenTwoPoints(s0, s1);
        float w2 = DistanceFromPointToLine(obs.location, s0, s1);
        float h2 = DistanceFromPointToLine(obs.location, s0, s2);
        if (w2 > w1 + obs.radius)
            return false;
        if (h2 > h1 + obs.radius)
            return false;
        if (w2 <= w1)
            return true;
        if (h2 <= h1)
            return true;
        return (w2 - w1) * (w2 - w1) + (h2 - h1) * (h2 - h1) <= obs.radius * obs.radius;
    }



  bool collideobs(const State& s, const std::shared_ptr<Model>& model,staticobs obs) {
      double w = model->type->ack.LB + model->type->ack.LF;
      State s0 = State((w/2 - model->type->ack.LB)*cos(-s.yaw) + s.x,
              (w/2 - model->type->ack.LB)*sin(-s.yaw) + s.y);
      State s1 = State(model->type->ack.LF*cos(-s.yaw) + s.x,
                        model->type->ack.LF*sin(-s.yaw) + s.y);
      State s2 = State(model->type->ack.width*sin(-s.yaw) + s0.x,
                       - model->type->ack.width*cos(-s.yaw) + s0.y);
        if (IsCircleIntersectRectangle(obs,s0,s1,s2))
            return true;
    return false;

  }

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

    std::vector<staticobs> m_obs;
    int current_time;
    int max_time;
};
}
