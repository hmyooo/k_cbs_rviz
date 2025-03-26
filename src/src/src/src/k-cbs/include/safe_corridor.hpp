// // #ifndef _SAFE_CORRIDOR_H_
// // #define _SAFE_CORRIDOR_H_
#pragma once
#include <vector>
#include <cmath>
#include <string.h>
#include <iostream>
#include <Eigen/Eigen>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <list>
#include <visualization_msgs/Marker.h>
#include "constants.hpp"
#include "environment.hpp"
#include <ros/console.h>
#include <ros/ros.h>
#include "opt_corrid.hpp"
#include "opt_corrid_OBB.hpp"
#include "opt_corrid_zhouxin.hpp"


using std::shared_ptr;
struct Candidate_collideinfo
{
    int id1;
    int id2;
    double time;


};
struct Candidate_Adjust_collideinfo
{
    int adjust_car_id;
    int corid_id;
    int other_car_id;

};
class Safe_Corridor
{
 private:   
    std::vector<Agent>  agents;
    Environment& m_env;
    ros::Publisher corrids_pub_markerarray;
    ros::NodeHandle nh_;
    int numid,dbl_max;
    visualization_msgs::MarkerArray marker_array;
    std::vector<Candidate_collideinfo> candidate_collideinfo_;
    std::vector<Candidate_Adjust_collideinfo> candidate_adjust_collideinfo_;
    std::vector<std::vector<Eigen::Vector4d>> cars_expanding;
    

public:

    Safe_Corridor(Environment& environment, std::vector<Agent>  m_agents,ros::NodeHandle &nh): m_env(environment), agents(m_agents),nh_(nh)
    {    
    corrids_pub_markerarray = nh_.advertise<visualization_msgs::MarkerArray>("/corrids", 10);
    numid=1;
    dbl_max=99999;
    
    }
    ~Safe_Corridor(){}
    std::vector<std::vector<Eigen::MatrixXd>> all_corrid_;
    std::vector<std::vector<Eigen::MatrixXd>> smallest_all_corrid_;

    std::vector<std::vector<double>> t_ts;
    void update(){
        std::cout << "check_line"<<m_env.struct_corrid_check_line(agents[0].solution.states[0],agents[0].solution.states[1]);
    }

void getRectangleConst_ts()
{
    // hPolys_.clear();
    double resolution = Constants::mapResolution;
    double step = resolution ;
    double limitBound = 60.0;
    std::vector<std::vector<Eigen::Vector4d>> statelists;
    std::vector<int> tnum_id;
    std::vector<int> add_t_id;
    std::vector<std::vector<double>> yaws;
    int addt=0;
    for(int i=0;i<agents.size();i++)
    {
     std::vector<Eigen::Vector4d> statelist;  
     std::vector<Eigen::MatrixXd> hPolys_t;
     std::vector<Eigen::MatrixXd> hPolys_smallest;

     std::vector<double> t_t;
     std::vector<double> yaw_s;
     std::vector<Eigen::Vector4d> ego_expending;
     int count_=0;
     for(const auto state_onpiece : agents[i].solution.states)
     {
        Eigen::Vector4d state_one;
        state_one(0)=state_onpiece.x;
        state_one(1)=state_onpiece.y;
        state_one(2)=state_onpiece.yaw;
        state_one(3)=state_onpiece.time;
        statelist.push_back(state_one);
        yaw_s.push_back(state_onpiece.yaw);

     }
    for(const auto state : statelist)
    {
        Eigen::Vector2d pos = state.head(2);
        double yaw = -state(2);
        Eigen::Matrix2d ego_R;
        ego_R << cos(yaw), -sin(yaw),
                 sin(yaw), cos(yaw);
        
        Eigen::Vector4d distance2center;
        // distance2center << Constants::ackerWidth/ 2.0, Constants::ackerLF, Constants::ackerWidth / 2.0,Constants::ackerLB;
        distance2center << Constants::ackerWidth/ 2.0, Constants::ackerWidth/ 2.0, Constants::ackerWidth / 2.0,Constants::ackerWidth/ 2.0;

        // distance2center << Constants::ackerLF*2.5, Constants::ackerLF*2.5, Constants::ackerLF*2.5,Constants::ackerLF*2.5;

        Eigen::Vector4d have_stopped_expanding;
        have_stopped_expanding << 1.0, 1.0, 1.0, 1.0;

        // Eigen::MatrixXd hPoly,hPoly_t;
        Eigen::MatrixXd hPoly;
        Eigen::MatrixXd one_smallesthPoly;

        Eigen::Vector2d expend_direction;
        one_smallesthPoly.resize(4,4);

        if(count_ !=statelist.size()-1) 
        {
     
            // expend_direction= (statelist[count_+1]-statelist[count_]).head(2);

            one_smallesthPoly.col(0).tail(2)=statelist[count_+1].head(2);
            one_smallesthPoly.col(2).tail(2)=statelist[count_].head(2);

            one_smallesthPoly.col(1)(2)= one_smallesthPoly.col(0)(2);
            one_smallesthPoly.col(1)(3)=one_smallesthPoly.col(2)(3);
            one_smallesthPoly.col(3)(2)= one_smallesthPoly.col(2)(2);
            one_smallesthPoly.col(3)(3)=one_smallesthPoly.col(0)(3);
            one_smallesthPoly.col(0).head(2)<<0,1;
            one_smallesthPoly.col(1).head(2)<<1,0;
            one_smallesthPoly.col(2).head(2)<<0,-1;
            one_smallesthPoly.col(3).head(2)<<-1,0;
        }
        // if(count_!=statelist.size()-1)
        // {
            

        // }
        else
        {
              one_smallesthPoly=hPolys_smallest.back();
             
        }
        hPolys_smallest.push_back(one_smallesthPoly);
        hPoly.resize(4, 4);
        // hPoly_t.resize(6,4);
        while(have_stopped_expanding.norm() > 1e-6)
        {
            for(int i = 0; i < 4; i++)
            {
            
                Eigen::Vector2d point1, point2, newpoint1, newpoint2; 
                State s1,s2,snew1,snew2;
                // bool isocc = false;               
                switch(i)
                {
                case 0: //上
                
                    point1 = pos +   Eigen::Vector2d(-distance2center(3), distance2center(0));
                    point2 = pos +  Eigen::Vector2d(distance2center(1), distance2center(0));
                    newpoint1 = pos +Eigen::Vector2d(-distance2center(3), distance2center(0) + step);
                    newpoint2 = pos +  Eigen::Vector2d(distance2center(1), distance2center(0) + step);
                    s1.x=point1(0);
                    s1.y=point1(1);
                    s2.x=point2(0);
                    s2.y=point2(1);
                    snew1.x=newpoint1(0);
                    snew1.y=newpoint1(1);
                    snew2.x=newpoint2(0);
                    snew2.y=newpoint2(1);
                    if(!m_env.struct_corrid_check_line(s1, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(s2, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(snew1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s2, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    distance2center(i) += step;
                    // if(expend_direction(1)>0)
                    // {
                        if(distance2center(i) > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    // }
                    // else
                    // {
                    // if(distance2center(i) > limitBound*0.5)
                    // {
                    //     have_stopped_expanding(i) = 0.0;
                    //     break;
                    // }
                    // }
                   
                    break;

                case 1:// x正
                    point1 = pos +Eigen::Vector2d(distance2center(1), distance2center(0));
                    point2 = pos +Eigen::Vector2d(distance2center(1), -distance2center(2));
                    newpoint1 = pos + Eigen::Vector2d(distance2center(1) + step, distance2center(0));
                    newpoint2 = pos + Eigen::Vector2d(distance2center(1) + step, -distance2center(2));
                    s1.x=point1(0);
                    s1.y=point1(1);
                    s2.x=point2(0);
                    s2.y=point2(1);
                    snew1.x=newpoint1(0);
                    snew1.y=newpoint1(1);
                    snew2.x=newpoint2(0);
                    snew2.y=newpoint2(1);
                     if(!m_env.struct_corrid_check_line(s1, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(s2, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(snew1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s2, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                //    if(expend_direction(0)>0)
                //     {
                        if(distance2center(i) > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    // }
                    // else
                    // {
                    // if(distance2center(i) > limitBound*0.5)
                    // {
                    //     have_stopped_expanding(i) = 0.0;
                    //     break;
                    // }
                    // }
                    break;

                case 2:// 下
                    point1 = pos +   Eigen::Vector2d(distance2center(1), -distance2center(2));
                    point2 = pos + Eigen::Vector2d(-distance2center(3), -distance2center(2));
                    newpoint1 = pos + Eigen::Vector2d(distance2center(1), -distance2center(2) - step);
                    newpoint2 = pos + Eigen::Vector2d(-distance2center(3), -distance2center(2) - step);
                    s1.x=point1(0);
                    s1.y=point1(1);
                    s2.x=point2(0);
                    s2.y=point2(1);
                    snew1.x=newpoint1(0);
                    snew1.y=newpoint1(1);
                    snew2.x=newpoint2(0);
                    snew2.y=newpoint2(1);
                     if(!m_env.struct_corrid_check_line(s1, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(s2, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(snew1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s2, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                    // if(expend_direction(3)>0)
                    // {
                        if(distance2center(i) > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    // }
                    // else
                    // {
                    // if(distance2center(i) > limitBound*0.5)
                    // {
                    //     have_stopped_expanding(i) = 0.0;
                    //     break;
                    // }
                    // }
                    break;

                case 3: // x负
                    point1 = pos +  Eigen::Vector2d(-distance2center(3), -distance2center(2));
                    point2 = pos +  Eigen::Vector2d(-distance2center(3), distance2center(0));
                    newpoint1 = pos + Eigen::Vector2d(-distance2center(3) - step, -distance2center(2));
                    newpoint2 = pos +  Eigen::Vector2d(-distance2center(3) - step, distance2center(0));
                    s1.x=point1(0);
                    s1.y=point1(1);
                    s2.x=point2(0);
                    s2.y=point2(1);
                    snew1.x=newpoint1(0);
                    snew1.y=newpoint1(1);
                    snew2.x=newpoint2(0);
                    snew2.y=newpoint2(1);   
                     if(!m_env.struct_corrid_check_line(s1, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(s2, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(snew1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s2, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                //   if(expend_direction(2)>0)
                //     {
                        if(distance2center(i) > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    // }
                    // else
                    // {
                    // if(distance2center(i) > limitBound*0.5)
                    // {
                    //     have_stopped_expanding(i) = 0.0;
                    //     break;
                    // }
                    // }
                    break;

                }
            }
        }
Eigen::Vector2d point1, norm1;
        point1 << pos +  Eigen::Vector2d(distance2center(1), distance2center(0));
        // norm1 << -sin(yaw), cos(yaw);
        norm1 <<0, 1;

        hPoly.col(0).head<2>() = norm1;
        hPoly.col(0).tail<2>() = point1;

        // hPoly_t.col(0).head<3>() = norm1;
        // hPoly_t.col(0).tail<3>() = point1;
      

        Eigen::Vector2d point2, norm2;
        point2 << pos + Eigen::Vector2d(distance2center(1), -distance2center(2));
        // norm2 << cos(yaw), sin(yaw);
        norm2 <<-1, 0;

        hPoly.col(1).head<2>() = norm2;
        hPoly.col(1).tail<2>() = point2;
        // hPoly_t.col(1).head<3>() = norm2;
        // hPoly_t.col(1).tail<3>() = point2;
        Eigen::Vector2d point3, norm3;
        point3 << pos +  Eigen::Vector2d(-distance2center(3), -distance2center(2));
        // norm3 << sin(yaw), -cos(yaw);
        norm3 <<0, -1;

        hPoly.col(2).head<2>() = norm3;
        hPoly.col(2).tail<2>() = point3;
        // hPoly_t.col(2).head<3>() = norm3;
        // hPoly_t.col(2).tail<3>() = point3;
        Eigen::Vector2d point4, norm4;
        point4 << pos +  Eigen::Vector2d(-distance2center(3), distance2center(0));
        // norm4 << -cos(yaw), -sin(yaw);
        norm4 << 1, 0;

        hPoly.col(3).head<2>() = norm4;
        hPoly.col(3).tail<2>() = point4;   
        // hPoly_t.col(3).head<3>() = norm4;
        // hPoly_t.col(3).tail<3>() = point4;
        // hPolys_.push_back(hPoly);    
        
        hPolys_t.push_back(hPoly);
        ego_expending.push_back(distance2center);
        t_t.push_back(state(3));
        count_++;

    }
    all_corrid_.push_back(hPolys_t);
    cars_expanding.push_back(ego_expending);
    smallest_all_corrid_.push_back(hPolys_smallest);
    t_ts.push_back(t_t);
    tnum_id.push_back(t_t.size());
    add_t_id.push_back(addt);
    addt=addt+t_t.size();


    yaws.push_back(yaw_s);
    statelists.push_back(statelist);
    }
    std::cout<<"all_corrid_ size "<<all_corrid_.size()<<std::endl;

    // std::vector<Rectangle_X> optimization_variables = initializeOptimizationVariables(statelists, Constants::ackerLB+Constants::ackerLF, Constants::ackerWidth);
    // SmartPtr<IpoptApplication> app = new IpoptApplication();
    MyOptimizationProblem cor_opt(agents.size(),tnum_id,all_corrid_,t_ts,statelists,Constants::ackerLB+Constants::ackerLF, Constants::ackerWidth,Constants::maxVel,smallest_all_corrid_);
    // // 创建问题实例并传递变量
    //     SmartPtr<TNLP> my_nlp = new MyOptimizationProblem(optimization_variables,agents.size(),tnum_id,add_t_id,all_corrid_,t_ts,yaws,statelists,Constants::ackerLB+Constants::ackerLF, Constants::ackerWidth);
    cor_opt.opt_corrid();
    all_corrid_.swap(cor_opt.opt_after_all_corrid_);
    
}
// 函数来计算给定车辆位置和尺寸的走廊矩形
  void getRectangleConst_ts_zhouxin()
{
    // hPolys_.clear();
    double resolution = Constants::mapResolution;
    double step = resolution ;
    double limitBound = 40.0;
    std::vector<std::vector<Eigen::Vector4d>> statelists;
    std::vector<int> tnum_id;
    std::vector<int> add_t_id;
    std::vector<std::vector<double>> yaws;
    int addt=0;
    for(int i=0;i<agents.size();i++)
    {
     std::vector<Eigen::Vector4d> statelist;  
     std::vector<Eigen::MatrixXd> hPolys_t;
     std::vector<Eigen::MatrixXd> hPolys_smallest;

     std::vector<double> t_t;
     std::vector<double> yaw_s;
     std::vector<Eigen::Vector4d> ego_expending;
     int count_=0;
     for(const auto state_onpiece : agents[i].solution.states)
     {
        Eigen::Vector4d state_one;
        state_one(0)=state_onpiece.x;
        state_one(1)=state_onpiece.y;
        state_one(2)=state_onpiece.yaw;
        state_one(3)=state_onpiece.time;
        statelist.push_back(state_one);
        yaw_s.push_back(state_onpiece.yaw);

     }
    for(const auto state : statelist)
    {
        Eigen::Vector2d pos = state.head(2);
        double yaw = -state(2);
        Eigen::Matrix2d ego_R;
        ego_R << cos(yaw), -sin(yaw),
                 sin(yaw), cos(yaw);
        
        Eigen::Vector4d distance2center;
        // distance2center << Constants::ackerWidth/ 2.0, Constants::ackerLF, Constants::ackerWidth / 2.0,Constants::ackerLB;
        distance2center << Constants::ackerWidth/ 2.0, Constants::ackerWidth/ 2.0, Constants::ackerWidth / 2.0,Constants::ackerWidth/ 2.0;

        // distance2center << Constants::ackerLF*2.5, Constants::ackerLF*2.5, Constants::ackerLF*2.5,Constants::ackerLF*2.5;

        Eigen::Vector4d have_stopped_expanding;
        have_stopped_expanding << 1.0, 1.0, 1.0, 1.0;

        // Eigen::MatrixXd hPoly,hPoly_t;
        Eigen::MatrixXd hPoly;
        Eigen::MatrixXd one_smallesthPoly;

        Eigen::Vector2d expend_direction;
        one_smallesthPoly.resize(4,4);

        if(count_ !=statelist.size()-1) 
        {
     
            // expend_direction= (statelist[count_+1]-statelist[count_]).head(2);

            one_smallesthPoly.col(0).tail(2)=statelist[count_+1].head(2);
            one_smallesthPoly.col(2).tail(2)=statelist[count_].head(2);

            one_smallesthPoly.col(1)(2)= one_smallesthPoly.col(0)(2);
            one_smallesthPoly.col(1)(3)=one_smallesthPoly.col(2)(3);
            one_smallesthPoly.col(3)(2)= one_smallesthPoly.col(2)(2);
            one_smallesthPoly.col(3)(3)=one_smallesthPoly.col(0)(3);
            one_smallesthPoly.col(0).head(2)<<0,1;
            one_smallesthPoly.col(1).head(2)<<1,0;
            one_smallesthPoly.col(2).head(2)<<0,-1;
            one_smallesthPoly.col(3).head(2)<<-1,0;
        }
        // if(count_!=statelist.size()-1)
        // {
            

        // }
        else
        {
              one_smallesthPoly=hPolys_smallest.back();
             
        }
        hPolys_smallest.push_back(one_smallesthPoly);
        hPoly.resize(4, 4);
        // hPoly_t.resize(6,4);
        while(have_stopped_expanding.norm() > 1e-6)
        {
            for(int i = 0; i < 4; i++)
            {
            
                Eigen::Vector2d point1, point2, newpoint1, newpoint2; 
                State s1,s2,snew1,snew2;
                // bool isocc = false;               
                switch(i)
                {
                case 0: //上
                
                    point1 = pos +   Eigen::Vector2d(-distance2center(3), distance2center(0));
                    point2 = pos +  Eigen::Vector2d(distance2center(1), distance2center(0));
                    newpoint1 = pos +Eigen::Vector2d(-distance2center(3), distance2center(0) + step);
                    newpoint2 = pos +  Eigen::Vector2d(distance2center(1), distance2center(0) + step);
                    s1.x=point1(0);
                    s1.y=point1(1);
                    s2.x=point2(0);
                    s2.y=point2(1);
                    snew1.x=newpoint1(0);
                    snew1.y=newpoint1(1);
                    snew2.x=newpoint2(0);
                    snew2.y=newpoint2(1);
                    if(!m_env.struct_corrid_check_line(s1, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(s2, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(snew1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s2, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    distance2center(i) += step;
                    // if(expend_direction(1)>0)
                    // {
                        if(distance2center(i) > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    // }
                    // else
                    // {
                    // if(distance2center(i) > limitBound*0.5)
                    // {
                    //     have_stopped_expanding(i) = 0.0;
                    //     break;
                    // }
                    // }
                   
                    break;

                case 1:// x正
                    point1 = pos +Eigen::Vector2d(distance2center(1), distance2center(0));
                    point2 = pos +Eigen::Vector2d(distance2center(1), -distance2center(2));
                    newpoint1 = pos + Eigen::Vector2d(distance2center(1) + step, distance2center(0));
                    newpoint2 = pos + Eigen::Vector2d(distance2center(1) + step, -distance2center(2));
                    s1.x=point1(0);
                    s1.y=point1(1);
                    s2.x=point2(0);
                    s2.y=point2(1);
                    snew1.x=newpoint1(0);
                    snew1.y=newpoint1(1);
                    snew2.x=newpoint2(0);
                    snew2.y=newpoint2(1);
                     if(!m_env.struct_corrid_check_line(s1, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(s2, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(snew1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s2, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                //    if(expend_direction(0)>0)
                //     {
                        if(distance2center(i) > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    // }
                    // else
                    // {
                    // if(distance2center(i) > limitBound*0.5)
                    // {
                    //     have_stopped_expanding(i) = 0.0;
                    //     break;
                    // }
                    // }
                    break;

                case 2:// 下
                    point1 = pos +   Eigen::Vector2d(distance2center(1), -distance2center(2));
                    point2 = pos + Eigen::Vector2d(-distance2center(3), -distance2center(2));
                    newpoint1 = pos + Eigen::Vector2d(distance2center(1), -distance2center(2) - step);
                    newpoint2 = pos + Eigen::Vector2d(-distance2center(3), -distance2center(2) - step);
                    s1.x=point1(0);
                    s1.y=point1(1);
                    s2.x=point2(0);
                    s2.y=point2(1);
                    snew1.x=newpoint1(0);
                    snew1.y=newpoint1(1);
                    snew2.x=newpoint2(0);
                    snew2.y=newpoint2(1);
                     if(!m_env.struct_corrid_check_line(s1, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(s2, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(snew1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s2, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                    // if(expend_direction(3)>0)
                    // {
                        if(distance2center(i) > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    // }
                    // else
                    // {
                    // if(distance2center(i) > limitBound*0.5)
                    // {
                    //     have_stopped_expanding(i) = 0.0;
                    //     break;
                    // }
                    // }
                    break;

                case 3: // x负
                    point1 = pos +  Eigen::Vector2d(-distance2center(3), -distance2center(2));
                    point2 = pos +  Eigen::Vector2d(-distance2center(3), distance2center(0));
                    newpoint1 = pos + Eigen::Vector2d(-distance2center(3) - step, -distance2center(2));
                    newpoint2 = pos +  Eigen::Vector2d(-distance2center(3) - step, distance2center(0));
                    s1.x=point1(0);
                    s1.y=point1(1);
                    s2.x=point2(0);
                    s2.y=point2(1);
                    snew1.x=newpoint1(0);
                    snew1.y=newpoint1(1);
                    snew2.x=newpoint2(0);
                    snew2.y=newpoint2(1);   
                     if(!m_env.struct_corrid_check_line(s1, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(s2, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(snew1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s2, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                //   if(expend_direction(2)>0)
                //     {
                        if(distance2center(i) > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    // }
                    // else
                    // {
                    // if(distance2center(i) > limitBound*0.5)
                    // {
                    //     have_stopped_expanding(i) = 0.0;
                    //     break;
                    // }
                    // }
                    break;

                }
            }
        }
Eigen::Vector2d point1, norm1;
        point1 << pos +  Eigen::Vector2d(distance2center(1), distance2center(0));
        // norm1 << -sin(yaw), cos(yaw);
        norm1 <<0, 1;

        hPoly.col(0).head<2>() = norm1;
        hPoly.col(0).tail<2>() = point1;

        // hPoly_t.col(0).head<3>() = norm1;
        // hPoly_t.col(0).tail<3>() = point1;
      

        Eigen::Vector2d point2, norm2;
        point2 << pos + Eigen::Vector2d(distance2center(1), -distance2center(2));
        // norm2 << cos(yaw), sin(yaw);
        norm2 <<-1, 0;

        hPoly.col(1).head<2>() = norm2;
        hPoly.col(1).tail<2>() = point2;
        // hPoly_t.col(1).head<3>() = norm2;
        // hPoly_t.col(1).tail<3>() = point2;
        Eigen::Vector2d point3, norm3;
        point3 << pos +  Eigen::Vector2d(-distance2center(3), -distance2center(2));
        // norm3 << sin(yaw), -cos(yaw);
        norm3 <<0, -1;

        hPoly.col(2).head<2>() = norm3;
        hPoly.col(2).tail<2>() = point3;
        // hPoly_t.col(2).head<3>() = norm3;
        // hPoly_t.col(2).tail<3>() = point3;
        Eigen::Vector2d point4, norm4;
        point4 << pos +  Eigen::Vector2d(-distance2center(3), distance2center(0));
        // norm4 << -cos(yaw), -sin(yaw);
        norm4 << 1, 0;

        hPoly.col(3).head<2>() = norm4;
        hPoly.col(3).tail<2>() = point4;   
        // hPoly_t.col(3).head<3>() = norm4;
        // hPoly_t.col(3).tail<3>() = point4;
        // hPolys_.push_back(hPoly);    
        
        hPolys_t.push_back(hPoly);
        ego_expending.push_back(distance2center);
        t_t.push_back(state(3));
        count_++;

    }
    all_corrid_.push_back(hPolys_t);
    cars_expanding.push_back(ego_expending);
    smallest_all_corrid_.push_back(hPolys_smallest);
    t_ts.push_back(t_t);
    tnum_id.push_back(t_t.size());
    add_t_id.push_back(addt);
    addt=addt+t_t.size();


    yaws.push_back(yaw_s);
    statelists.push_back(statelist);
    }
    std::cout<<"all_corrid_ size "<<all_corrid_.size()<<std::endl;

    // std::vector<Rectangle_X> optimization_variables = initializeOptimizationVariables(statelists, Constants::ackerLB+Constants::ackerLF, Constants::ackerWidth);
    // SmartPtr<IpoptApplication> app = new IpoptApplication();
    // Optimizer optimizer(agents.size(),tnum_id,all_corrid_,t_ts,statelists,Constants::ackerLB+Constants::ackerLF, Constants::ackerWidth,Constants::maxVel);
    // optimizer.optimize_corridors();
    // bool if_ok=optimize_corridors(agents.size(),tnum_id,all_corrid_,t_ts,statelists,Constants::ackerLB+Constants::ackerLF, Constants::ackerWidth,Constants::maxVel);
    // cout<<"if_ok "<<if_ok<<endl;
   MyOptimizationProblemzhouxin cor_opt(agents.size(),tnum_id,all_corrid_,t_ts,statelists,Constants::ackerLB+Constants::ackerLF, Constants::ackerWidth,Constants::maxVel,smallest_all_corrid_);
    // // 创建问题实例并传递变量
    //     SmartPtr<TNLP> my_nlp = new MyOptimizationProblem(optimization_variables,agents.size(),tnum_id,add_t_id,all_corrid_,t_ts,yaws,statelists,Constants::ackerLB+Constants::ackerLF, Constants::ackerWidth);
    cor_opt.opt_corrid();
    all_corrid_.swap(cor_opt.opt_after_all_corrid_);



    // all_corrid_.swap(optimizer.opt_after_all_corrid_);
    
}
std::vector<int> calculateCumulativeSum(const std::vector<int>& tnum_id) {
    // 初始化一个用于存储累加和的vector
    std::vector<int> cumulative_sum(tnum_id.size(), 0);

    // 累加变量
    int total_cumulative = 0;

    // 遍历每辆车
    for (size_t i = 0; i < tnum_id.size(); ++i) {
        // 存储前i辆车的走廊优化变量累加和
        cumulative_sum[i] = total_cumulative;
        
        // 更新累加和：当前车的走廊数量 * 每个走廊有6个优化变量
        total_cumulative += tnum_id[i]*6;
    }

    return cumulative_sum;
}

bool optimize_corridors(int num_agents, std::vector<int> tnum_id, 
              std::vector<std::vector<Eigen::MatrixXd>> all_corrid,
              std::vector<std::vector<double>> t_ts, 
              std::vector<std::vector<Eigen::Vector4d>> statelists,
              double vehicle_length, double vehicle_width, double max_velocity) {
    typedef CPPAD_TESTVECTOR(double) Dvector;
//     // 初始化优化变量，填入初始走廊数据
    
    
    Optimizer optimizer(num_agents,tnum_id,all_corrid,t_ts,statelists,vehicle_length, vehicle_width,max_velocity);
    std::vector<int> sum_tnum_id=calculateCumulativeSum(tnum_id);
// Define a max size (e.g., based on system limits)
    size_t max_size = std::numeric_limits<size_t>::max() / 2;
    Dvector x_init;
    if (optimizer.num_vars_ <= max_size) {
        // Safe to create a Dvector of this size
    x_init.resize(optimizer.num_vars_);

    } else {
        ROS_WARN("Attempting to create a vector too large! Reducing size.");
        // Consider reducing the size or splitting the data into smaller parts
    }

    ROS_WARN("optimizer struct done.");


    for (size_t i = 0; i < num_agents; ++i) {
        for (size_t j = 0; j < tnum_id[i]; ++j) {
                size_t idx_i =sum_tnum_id[i] + (j)*6;
                Eigen::Vector2d point1 = all_corrid[i][j].col(0).tail(2); // 提取每个点的坐标
                Eigen::Vector2d point2 = all_corrid[i][j].col(2).tail(2); // 提取每个点的坐标
                double x_length=std::abs(point1.x()-point2.x());
                double y_length=std::abs(point1.y()-point2.y());

                x_init[idx_i] = statelists[i][j](0);  // x坐标
                x_init[idx_i+1] = statelists[i][j](1);  // y坐标
                x_init[idx_i+2] = x_length/20;  // y坐标
                x_init[idx_i+3] = x_length/20;  // y坐标
                x_init[idx_i+4] = y_length/20;  // y坐标
                x_init[idx_i+5] = y_length/20;  // y坐标


            
        }
    }
   int num_vars_cons=0; 
    // ROS_WARN("optimizer struct done_1.");

// for (size_t i = 0; i < num_agents; i++) {
//     for (size_t j = i + 1; j < num_agents; j++) {
//         for (size_t t = 0; t<std::min(tnum_id[i],tnum_id[j]); t++) {
//             // 相交约束
//             num_vars_cons++;  // 相交约束，1个
//         }
//     }
// }

    // ROS_WARN("optimizer struct done_2.");

for (size_t i = 0; i < num_agents; i++) {
    for (size_t j = 0; j < tnum_id[i]; j++) {
        // 走廊约束
        num_vars_cons += 1;  // 
        
    }
}
    // ROS_WARN("optimizer struct done_3.");

for (size_t i = 0; i < num_agents; i++) {
    for (size_t t = 0; t < tnum_id[i] - 1; t++) {
        // 行驶范围和时间约束
        num_vars_cons += 3;  // 每对 (t, t+1) 产生 10 个约束
    }
}

 ROS_INFO_STREAM("Number of constraints: " << num_vars_cons);

    // 设置求解器配置
    std::string options;
    // turn off any printing
    options += "Integer print_level  5\n";
    options += "String sb            yes\n";
    // maximum iterations
    options += "Integer max_iter     50\n";
    //approximate accuracy in first order necessary conditions;
    // see Mathematical Programming, Volume 106, Number 1,
    // Pages 25-57, Equation (6)
    options += "Numeric tol          1e-3\n";//-4
    //derivative tesing
    options += "String derivative_test   second-order\n";
    // maximum amount of random pertubation; e.g.,
    // when evaluation finite diff
    options += "Numeric point_perturbation_radius   1e-6\n";
    options += "Numeric derivative_test_tol   1e-3\n";

    // options += "String linear_solver mumps\n";  // 使用 maui 求解器
    // options += "Integer linear_solver 1\n";    // 开启稀疏线性求解
    // options += "String hessian_approximation limited-memory\n";  // 使用有限记忆Hessian计算



      Dvector xl(optimizer.num_vars_), xu(optimizer.num_vars_);
    for(size_t i = 0; i < optimizer.num_vars_; i++)
    {
        if(i%6==0||i%6==1)
        {
        xl[i] = 0.0; // x_i >= 0
        xu[i] = 600.0; // 无上界     
        }
        else
        {
        xl[i] = 0.0; // x_i >= 0
        xu[i] = 40.0; // 无上界   
        }

    }
  
    Dvector gl(num_vars_cons), gu(num_vars_cons);
   for(size_t i = 0; i < num_vars_cons; i++)
    {
        gl[i] =1e-4; // x_i >= 0
        gu[i] = 1000; 
    }
//     Dvector gl(1), gu(1);
//    for(size_t i = 0; i < 1; i++)
//     {
//         gl[i] =0; // x_i >= 0
//         gu[i] = 200; // 无上界
//     }

    // 创建一个 CppAD::vector 来存储优化结果
    CppAD::ipopt::solve_result<Dvector>  solution;
    ROS_WARN("chushihua done");
    

    // 调用 CppAD 的求解器进行优化
    CppAD::ipopt::solve<Dvector, Optimizer>(
        options,         // 配置选项
        x_init,          // 初始优化变量
        xl, xu, gl, gu,
        optimizer,   
        solution       // 存储结果
             // 目标函数和约束函数（Optimizer 实现了 FG_eval）
    );
    bool ok = true;
     // 输出求解器的状态和迭代次数

    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    cout<<"ok is "<<ok<<endl;

     ROS_INFO("Final objective value: %f", solution.obj_value);
    // 将转换后的结果传递给转换函数
    std::vector<std::vector<Eigen::MatrixXd>> converted_corridors = optimizer.convert_solution_to_corridors(solution);
    if(ok)
    {
        all_corrid_.swap(converted_corridors);
        std::cout<<"all_corrid_"<<all_corrid_[0].size()<<std::endl;

    }
    // optimizer.opt_after_all_corrid_ = converted_corridors;
    return true;
}

void getRectangleConst_ts_OBB()
{
    // hPolys_.clear();
    double resolution = Constants::mapResolution;
    double step = resolution ;
    double limitBound = 16.0;
    std::vector<std::vector<Eigen::Vector4d>> statelists;
    std::vector<int> tnum_id;
    std::vector<int> add_t_id;
    std::vector<std::vector<double>> yaws;
    int addt=0;
    for(int i=0;i<agents.size();i++)
    {
     std::vector<Eigen::Vector4d> statelist;  
     std::vector<Eigen::MatrixXd> hPolys_t;


     std::vector<double> t_t;
     std::vector<double> yaw_s;
     std::vector<Eigen::Vector4d> ego_expending;
     int count_=0;
     for(const auto state_onpiece : agents[i].solution.states)
     {
        Eigen::Vector4d state_one;
        state_one(0)=state_onpiece.x;
        state_one(1)=state_onpiece.y;
        state_one(2)=state_onpiece.yaw;
        state_one(3)=state_onpiece.time;
        statelist.push_back(state_one);
        yaw_s.push_back(state_onpiece.yaw);

     }
    for(const auto state : statelist)
    {
        Eigen::Vector2d pos = state.head(2);
        double yaw = -state(2);
        Eigen::Matrix2d ego_R;
        ego_R << cos(yaw), -sin(yaw),
                 sin(yaw), cos(yaw);
        
        Eigen::Vector4d distance2center;
        // distance2center << Constants::ackerWidth/ 2.0, Constants::ackerLF, Constants::ackerWidth / 2.0,Constants::ackerLB;
        distance2center << Constants::ackerWidth/ 2.0, Constants::ackerWidth/ 2.0, Constants::ackerWidth / 2.0,Constants::ackerWidth/ 2.0;

        // distance2center << Constants::ackerLF*2.5, Constants::ackerLF*2.5, Constants::ackerLF*2.5,Constants::ackerLF*2.5;

        Eigen::Vector4d have_stopped_expanding;
        have_stopped_expanding << 1.0, 1.0, 1.0, 1.0;

        // Eigen::MatrixXd hPoly,hPoly_t;
        Eigen::MatrixXd hPoly;
        Eigen::MatrixXd one_smallesthPoly;

        Eigen::Vector2d expend_direction;
        one_smallesthPoly.resize(4,4);

        hPoly.resize(4, 4);
        // hPoly_t.resize(6,4);
        while(have_stopped_expanding.norm() > 1e-6)
        {
            for(int i = 0; i < 4; i++)
            {
            
                Eigen::Vector2d point1, point2, newpoint1, newpoint2; 
                State s1,s2,snew1,snew2;
                // bool isocc = false;               
                switch(i)
                {
                case 0: //上
                
                    point1 = pos + ego_R*Eigen::Vector2d(-distance2center(3), distance2center(0));
                    point2 = pos + ego_R*Eigen::Vector2d(distance2center(1), distance2center(0));
                    newpoint1 = pos + ego_R*Eigen::Vector2d(-distance2center(3), distance2center(0) + step);
                    newpoint2 = pos + ego_R*Eigen::Vector2d(distance2center(1), distance2center(0) + step);
                    s1.x=point1(0);
                    s1.y=point1(1);
                    s2.x=point2(0);
                    s2.y=point2(1);
                    snew1.x=newpoint1(0);
                    snew1.y=newpoint1(1);
                    snew2.x=newpoint2(0);
                    snew2.y=newpoint2(1);
                    if(!m_env.struct_corrid_check_line(s1, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(s2, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(snew1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s2, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    distance2center(i) += step;
                    // if(expend_direction(1)>0)
                    // {
                        if(distance2center(i) > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    // }
                    // else
                    // {
                    // if(distance2center(i) > limitBound*0.5)
                    // {
                    //     have_stopped_expanding(i) = 0.0;
                    //     break;
                    // }
                    // }
                   
                    break;

                case 1:// x正
                    point1 = pos +ego_R*Eigen::Vector2d(distance2center(1), distance2center(0));
                    point2 = pos +ego_R*Eigen::Vector2d(distance2center(1), -distance2center(2));
                    newpoint1 = pos + ego_R*Eigen::Vector2d(distance2center(1) + step, distance2center(0));
                    newpoint2 = pos + ego_R*Eigen::Vector2d(distance2center(1) + step, -distance2center(2));
                    s1.x=point1(0);
                    s1.y=point1(1);
                    s2.x=point2(0);
                    s2.y=point2(1);
                    snew1.x=newpoint1(0);
                    snew1.y=newpoint1(1);
                    snew2.x=newpoint2(0);
                    snew2.y=newpoint2(1);
                     if(!m_env.struct_corrid_check_line(s1, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(s2, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(snew1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s2, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                //    if(expend_direction(0)>0)
                //     {
                        if(distance2center(i) > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    // }
                    // else
                    // {
                    // if(distance2center(i) > limitBound*0.5)
                    // {
                    //     have_stopped_expanding(i) = 0.0;
                    //     break;
                    // }
                    // }
                    break;

                case 2:// 下
                    point1 = pos + ego_R*Eigen::Vector2d(distance2center(1), -distance2center(2));
                    point2 = pos + ego_R*Eigen::Vector2d(-distance2center(3), -distance2center(2));
                    newpoint1 = pos + ego_R*Eigen::Vector2d(distance2center(1), -distance2center(2) - step);
                    newpoint2 = pos + ego_R*Eigen::Vector2d(-distance2center(3), -distance2center(2) - step);
                    s1.x=point1(0);
                    s1.y=point1(1);
                    s2.x=point2(0);
                    s2.y=point2(1);
                    snew1.x=newpoint1(0);
                    snew1.y=newpoint1(1);
                    snew2.x=newpoint2(0);
                    snew2.y=newpoint2(1);
                     if(!m_env.struct_corrid_check_line(s1, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(s2, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(snew1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s2, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                    // if(expend_direction(3)>0)
                    // {
                        if(distance2center(i) > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    // }
                    // else
                    // {
                    // if(distance2center(i) > limitBound*0.5)
                    // {
                    //     have_stopped_expanding(i) = 0.0;
                    //     break;
                    // }
                    // }
                    break;

                case 3: // x负
                    point1 = pos + ego_R*Eigen::Vector2d(-distance2center(3), -distance2center(2));
                    point2 = pos + ego_R*Eigen::Vector2d(-distance2center(3), distance2center(0));
                    newpoint1 = pos +ego_R*Eigen::Vector2d(-distance2center(3) - step, -distance2center(2));
                    newpoint2 = pos + ego_R*Eigen::Vector2d(-distance2center(3) - step, distance2center(0));
                    s1.x=point1(0);
                    s1.y=point1(1);
                    s2.x=point2(0);
                    s2.y=point2(1);
                    snew1.x=newpoint1(0);
                    snew1.y=newpoint1(1);
                    snew2.x=newpoint2(0);
                    snew2.y=newpoint2(1);   
                     if(!m_env.struct_corrid_check_line(s1, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(s2, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    
                    if(!m_env.struct_corrid_check_line(snew1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s1, snew2))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    if(!m_env.struct_corrid_check_line(s2, snew1))
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                //   if(expend_direction(2)>0)
                //     {
                        if(distance2center(i) > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    // }
                    // else
                    // {
                    // if(distance2center(i) > limitBound*0.5)
                    // {
                    //     have_stopped_expanding(i) = 0.0;
                    //     break;
                    // }
                    // }
                    break;

                }
            }
        }
Eigen::Vector2d point1, norm1;
        point1 << pos +ego_R*Eigen::Vector2d(distance2center(1), distance2center(0));
        norm1 << -sin(yaw), cos(yaw);
        // norm1 <<0, 1;

        hPoly.col(0).head<2>() = norm1;
        hPoly.col(0).tail<2>() = point1;

        // hPoly_t.col(0).head<3>() = norm1;
        // hPoly_t.col(0).tail<3>() = point1;
      

        Eigen::Vector2d point2, norm2;
        point2 << pos + ego_R*Eigen::Vector2d(distance2center(1), -distance2center(2));
        norm2 << cos(yaw), sin(yaw);
        // norm2 <<-1, 0;

        hPoly.col(1).head<2>() = norm2;
        hPoly.col(1).tail<2>() = point2;
        // hPoly_t.col(1).head<3>() = norm2;
        // hPoly_t.col(1).tail<3>() = point2;
        Eigen::Vector2d point3, norm3;
        point3 << pos + ego_R*Eigen::Vector2d(-distance2center(3), -distance2center(2));
        norm3 << sin(yaw), -cos(yaw);
        // norm3 <<0, -1;

        hPoly.col(2).head<2>() = norm3;
        hPoly.col(2).tail<2>() = point3;
        // hPoly_t.col(2).head<3>() = norm3;
        // hPoly_t.col(2).tail<3>() = point3;
        Eigen::Vector2d point4, norm4;
        point4 << pos +ego_R*Eigen::Vector2d(-distance2center(3), distance2center(0));
        norm4 << -cos(yaw), -sin(yaw);
        // norm4 << 1, 0;

        hPoly.col(3).head<2>() = norm4;
        hPoly.col(3).tail<2>() = point4;   
        // hPoly_t.col(3).head<3>() = norm4;
        // hPoly_t.col(3).tail<3>() = point4;
        // hPolys_.push_back(hPoly);    
        
        hPolys_t.push_back(hPoly);
        ego_expending.push_back(distance2center);
        t_t.push_back(state(3));
        count_++;

    }
    all_corrid_.push_back(hPolys_t);
    cars_expanding.push_back(ego_expending);

    t_ts.push_back(t_t);
    tnum_id.push_back(t_t.size());
    add_t_id.push_back(addt);
    addt=addt+t_t.size();


    yaws.push_back(yaw_s);
    statelists.push_back(statelist);
    }
    std::cout<<"all_corrid_ size "<<all_corrid_.size()<<std::endl;

    // std::vector<Rectangle_X> optimization_variables = initializeOptimizationVariables(statelists, Constants::ackerLB+Constants::ackerLF, Constants::ackerWidth);
    // SmartPtr<IpoptApplication> app = new IpoptApplication();
    // MyOptimizationProblem cor_opt(agents.size(),tnum_id,all_corrid_,t_ts,statelists,Constants::ackerLB+Constants::ackerLF, Constants::ackerWidth,Constants::maxVel);
    // // 创建问题实例并传递变量
    //     SmartPtr<TNLP> my_nlp = new MyOptimizationProblem(optimization_variables,agents.size(),tnum_id,add_t_id,all_corrid_,t_ts,yaws,statelists,Constants::ackerLB+Constants::ackerLF, Constants::ackerWidth);
    // cor_opt.opt_corrid();
    // all_corrid_.swap(cor_opt.opt_after_all_corrid_);
    // Optimizer optimizer(agents.size(),tnum_id,all_corrid_,t_ts,statelists,Constants::ackerLB+Constants::ackerLF, Constants::ackerWidth,Constants::maxVel);
    // (num_agents_,tnum_id_,all_corrid_,t_ts_,statelists_,vehicle_length_, vehicle_width_,max_velocity_);
}




void CheckRectangleAndPointCollide(const Eigen::MatrixXd Rectangle,const Eigen::Vector2d point,bool &ifcollide)
{
ifcollide=false;

Eigen::Vector2d v1=point-Rectangle.col(0).tail<2>();
Eigen::Vector2d v2=point-Rectangle.col(1).tail<2>();
Eigen::Vector2d v3=point-Rectangle.col(2).tail<2>();
Eigen::Vector2d v4=point-Rectangle.col(3).tail<2>();

double dot1=v1.dot(Rectangle.col(0).head<2>());
double dot2=v2.dot(Rectangle.col(1).head<2>());
double dot3=v3.dot(Rectangle.col(2).head<2>());
double dot4=v4.dot(Rectangle.col(3).head<2>());

if(dot1>0 && dot2>0 && dot3>0 && dot4>0 )
{
   ifcollide=true;
}


}
State getState(size_t agentIdx,
       const std::vector<Agent> agents,size_t t) {
    assert(agentIdx < agents[agentIdx].solution.states.size());
    if (t < agents[agentIdx].solution.states.size() && t>=0) {
      int t_int=floor(t/Constants::timeResolution);
      return agents[agentIdx].solution.states[t_int];
    }
    else if (t<0) {
        return agents[agentIdx].solution.states.front();
    }
    assert(!agents[agentIdx].solution.states.empty());
    return agents[agentIdx].solution.states.back();
  }
void CheckRectangleConst_ts()
{

    int max_t = 0;
    int min_t = 9999;

      // 检查t时刻 agent之间有无碰撞
      for (size_t i = 0; i < agents.size(); i++) {
        for (size_t j = i + 1; j < agents.size(); ++j) {
            max_t = std::max<int>(agents[i].start.time , agents[j].start.time);
            min_t = std::max<int>(agents[i].start.time + agents[i].solution.states.size()*Constants::timeResolution,
                                  agents[j].start.time + agents[j].solution.states.size()*Constants::timeResolution);//把这个时间改成id去找，反正存的就是单位时间resolution
            if(max_t > min_t) continue;
            // std::cout<<"i "<<i<<std::endl;
            // std::cout<<"j "<<j<<std::endl;
            // std::cout<<"max_t "<<max_t<<std::endl;
            // std::cout<<"min_t "<<min_t<<std::endl;
            for (double t = max_t+Constants::timeResolution; t < min_t; t=t+Constants::timeResolution)
            {
             State state1 = getState(i, agents, t - agents[i].start.time);
              State state2 = getState(j, agents, t - agents[j].start.time);
                double distance;
                distance=sqrt(pow(state1.x-state2.x,2)+pow(state1.y-state2.y,2));
                if (distance<30 && i!=j)
                {
                    Candidate_collideinfo one_collide;
                    one_collide.id1=i;
                    one_collide.id2=j;
                    one_collide.time=t;
                if(candidate_collideinfo_.size()==0)
                    {candidate_collideinfo_.push_back(one_collide); break;}
                    for(const auto checkprio:candidate_collideinfo_)
                    {
                        if(one_collide.id1==checkprio.id1 && one_collide.id2==checkprio.id2 )
                        {
                            break;
                        }
                        else
                        {
                        candidate_collideinfo_.push_back(one_collide);
                        }

                    }


                }
                }
            }
        }

        std::cout<<"candidate_collideinfo_.size() "<<candidate_collideinfo_.size()<<std::endl;
        for(const auto  onethe_candidate_collideinfo :candidate_collideinfo_)
        {
        int di_1= onethe_candidate_collideinfo.id1;
        int di_2= onethe_candidate_collideinfo.id2;
        int count_id,count;
        int adjust_id,other_car_id_;
        max_t = std::max<int>(agents[di_1].start.time , agents[di_2].start.time);
        min_t = std::max<int>(agents[di_1].start.time + agents[di_1].solution.states.size()*Constants::timeResolution,
                                  agents[di_2].start.time + agents[di_2].solution.states.size()*Constants::timeResolution);//把这个时间改成id去找，反正存的就是单位时间resolution
           
           
            // count_id=std::min<int>(all_corrid_[di_1].size(),all_corrid_[di_2].size());
            // count=0;


        for (double t = max_t+Constants::timeResolution; t < min_t; t=t+Constants::timeResolution)
            {
            State state1 = getState(di_1, agents, t - agents[di_1].start.time);
            State state2 = getState(di_2, agents, t - agents[di_2].start.time);
            int t1=floor(t/Constants::timeResolution);
            bool ifcollide_=false;
            if(t1<all_corrid_[di_1].size() && t1<all_corrid_[di_2].size())
{            CheckRectangleCollide(all_corrid_[di_1][t1],all_corrid_[di_2][t1],ifcollide_);  
            if(ifcollide_) 
            {
            if(state1.v<state2.v)
            {
                adjust_id=di_1;
                other_car_id_=di_2;
            }
            if(state1.v>state2.v)
            {
                adjust_id=di_2;
                other_car_id_=di_1;
            }
            else//先按速度分，小的调整，相等则看id等级，数大调整
            {
            if(di_1>di_2)
            {
            adjust_id=di_1;
            other_car_id_=di_2;

            }
            else
            {
            adjust_id=di_2;
            other_car_id_=di_1;

            }  
            }
            
            Candidate_Adjust_collideinfo one_adjustinfo;
            one_adjustinfo.adjust_car_id=adjust_id;
            one_adjustinfo.corid_id=t1;
            one_adjustinfo.other_car_id=other_car_id_;

            candidate_adjust_collideinfo_.push_back(one_adjustinfo);
            
            }}
                }

        // for(const auto corrid_1:all_corrid_[di_1])
        // {
            
        //   if(count<count_id)
        //   {
        //     bool ifcollide_=false;
        //     CheckRectangleCollide(corrid_1,all_corrid_[di_2][count],ifcollide_);  
        //     if(ifcollide_) 
        //     {
        //     if(agents[di_1].solution.states[count].v<agents[di_2].solution.states[count].v)
        //     {
        //         adjust_id=di_1;
        //         other_car_id_=di_2;
        //     }
        //     if(agents[di_1].solution.states[count].v>agents[di_2].solution.states[count].v)
        //     {
        //         adjust_id=di_2;
        //         other_car_id_=di_1;
        //     }
        //     else//先按速度分，小的调整，相等则看id等级，数大调整
        //     {
        //     if(di_1>di_2)
        //     {
        //     adjust_id=di_1;
        //     other_car_id_=di_2;

        //     }
        //     else
        //     {
        //     adjust_id=di_2;
        //     other_car_id_=di_1;

        //     }  
        //     }
          
        //     Candidate_Adjust_collideinfo one_adjustinfo;
        //     one_adjustinfo.adjust_car_id=adjust_id;
        //     one_adjustinfo.corid_id=count;
        //     one_adjustinfo.other_car_id=other_car_id_;

        //     candidate_adjust_collideinfo_.push_back(one_adjustinfo);
            
        //     }
        //      count++;
        //   }
        //     else{break;}
           
        // }
        

        }

        std::cout<<"candidate_adjust_collideinfo_.size() "<<candidate_adjust_collideinfo_.size()<<std::endl;


AdjustRectangleConst_ts(candidate_adjust_collideinfo_);
}


void AdjustRectangleConst_ts(std::vector<Candidate_Adjust_collideinfo> adjust_collideinfo_candidate)
{
int count=1;
int last_cor_id1,last_cor_id2;
last_cor_id1=0;
last_cor_id2=0;
// double retreat_dist1=0;
// double retreat_dist2=0;
for(const auto one_adjustinfo : adjust_collideinfo_candidate)
{
double vel=agents[one_adjustinfo.adjust_car_id].solution.states[one_adjustinfo.corid_id].v;
Eigen::Vector2d pos_;
pos_<< agents[one_adjustinfo.adjust_car_id].solution.states[one_adjustinfo.corid_id].x,agents[one_adjustinfo.adjust_car_id].solution.states[one_adjustinfo.corid_id].y;
double yaw_=agents[one_adjustinfo.adjust_car_id].solution.states[one_adjustinfo.corid_id].yaw;
Eigen::Vector4d overflow,overflow_2;
overflow<<1.0,1.0,1.0,1.0;
overflow_2<<1.0,1.0,1.0,1.0;
int adjust_count=1; 
bool check_once=true;//还撞
while(overflow.norm()>1e-6 && check_once && overflow_2.norm()>1e-6)  //不能减小或者不撞了
{
std::vector<int> adjust_id_cor;
adjust_id_cor=CheckRectangleCollide_id(all_corrid_[one_adjustinfo.adjust_car_id][one_adjustinfo.corid_id],all_corrid_[one_adjustinfo.other_car_id][one_adjustinfo.corid_id],check_once);
getSmaller_Rectangle(all_corrid_[one_adjustinfo.adjust_car_id][one_adjustinfo.corid_id],pos_,yaw_,vel,one_adjustinfo.corid_id,one_adjustinfo.adjust_car_id,overflow,adjust_id_cor);     

if(adjust_count%2==0)
{
double vel2=agents[one_adjustinfo.other_car_id].solution.states[one_adjustinfo.corid_id].v;
Eigen::Vector2d pos_2;
pos_2<< agents[one_adjustinfo.other_car_id].solution.states[one_adjustinfo.corid_id].x,agents[one_adjustinfo.other_car_id].solution.states[one_adjustinfo.corid_id].y;
double yaw_2=agents[one_adjustinfo.other_car_id].solution.states[one_adjustinfo.corid_id].yaw;

getSmaller_Rectangle(all_corrid_[one_adjustinfo.other_car_id][one_adjustinfo.corid_id],pos_2,yaw_2,vel2,one_adjustinfo.corid_id,one_adjustinfo.other_car_id,overflow_2,adjust_id_cor); 



}

CheckRectangleCollide(all_corrid_[one_adjustinfo.adjust_car_id][one_adjustinfo.corid_id],all_corrid_[one_adjustinfo.other_car_id][one_adjustinfo.corid_id],check_once);
adjust_count++;

}
last_cor_id1=one_adjustinfo.corid_id;
last_cor_id2=one_adjustinfo.corid_id;
std::cout<<"adjust "<<count<<std::endl;
count++;
}



}


void getSmaller_Rectangle(Eigen::MatrixXd &hPoly_the,const Eigen::Vector2d pos,const double yaw,const double vel_,int id_corrid,int car_id,Eigen::Vector4d &overflow,std::vector<int> adjust_id_cor)
{
    
    double resolution = Constants::mapResolution;
    double step = resolution;
    double fuyaw=-yaw;
      Eigen::Matrix2d ego_R;
      ego_R << cos(fuyaw), -sin(fuyaw),
                sin(fuyaw), cos(fuyaw);
      
      // Eigen::MatrixXd hPoly,hPoly_t;
      Eigen::MatrixXd hPoly;
      hPoly.resize(4, 4);
      // hPoly_t.resize(6,4);
      Eigen::Vector4d distance2center;
      distance2center=cars_expanding[car_id][id_corrid];
          for(int i = 0; i < 4; i++)
          {

              Eigen::Vector2d point1, point2, newpoint1, newpoint2; 
            //   bool isocc = false;               
              switch(i)
              {
              case 0: // 
                  if(std::find(adjust_id_cor.begin(),adjust_id_cor.end(),0)!=adjust_id_cor.end())
                  {
                    if(distance2center(0)<=(Constants::ackerLF*2.5))
                  {overflow(0)=0.0;
                    break;}
                  distance2center(i) -= step;
                  break;


                  }
                  else
                  {
                    overflow(0)=0.0;
                    break;
                  }
                 

              case 1: // dx
                   if(std::find(adjust_id_cor.begin(),adjust_id_cor.end(),1)!=adjust_id_cor.end())
                    {
                        if(distance2center(1)<=(Constants::ackerLF+vel_/Constants::maxAcc)*2.5)
                        {overflow(1)=0.0;
                        break;
                        }
                        distance2center(i) -= step;
                        break;
                    }
                    else
                    {
                        overflow(1)=0.0;
                        break;
                    }
                  

              case 2: // -dy
              if(std::find(adjust_id_cor.begin(),adjust_id_cor.end(),2)!=adjust_id_cor.end())
                {
                    if(distance2center(2)<=(Constants::ackerLF+vel_/Constants::maxAcc)*2.5)
                    {overflow(2)=0.0;break;}
                    distance2center(i) -= step;
                    break;
                }
                else
                {
                overflow(2)=0.0;break;
                }
                 

              case 3: // 
              if(std::find(adjust_id_cor.begin(),adjust_id_cor.end(),3)!=adjust_id_cor.end())
               { 
                  if(distance2center(3)<=(Constants::ackerLF)*2.5)
                  {overflow(3)=0.0;break;}
                  distance2center(i) -= step;
                  break;
                  }
              else
              {
                overflow(3)=0.0;break;
              }

              }
          }
      
      Eigen::Vector2d point1, norm1;
      point1 << pos + ego_R * Eigen::Vector2d(distance2center(1), distance2center(0));
      norm1 << -sin(fuyaw), cos(fuyaw);
      hPoly_the.col(0).head<2>() = norm1;
      hPoly_the.col(0).tail<2>() = point1;

      // hPoly_t.col(0).head<3>() = norm1;
      // hPoly_t.col(0).tail<3>() = point1;
    

      Eigen::Vector2d point2, norm2;
      point2 << pos + ego_R * Eigen::Vector2d(distance2center(1), -distance2center(2));
      norm2 << cos(fuyaw), sin(fuyaw);
      hPoly_the.col(1).head<2>() = norm2;
      hPoly_the.col(1).tail<2>() = point2;
      // hPoly_t.col(1).head<3>() = norm2;
      // hPoly_t.col(1).tail<3>() = point2;
      Eigen::Vector2d point3, norm3;
      point3 << pos + ego_R * Eigen::Vector2d(-distance2center(3), -distance2center(2));
      norm3 << sin(fuyaw), -cos(fuyaw);
      hPoly_the.col(2).head<2>() = norm3;
      hPoly_the.col(2).tail<2>() = point3;
      // hPoly_t.col(2).head<3>() = norm3;
      // hPoly_t.col(2).tail<3>() = point3;
      Eigen::Vector2d point4, norm4;
      point4 << pos + ego_R * Eigen::Vector2d(-distance2center(3), distance2center(0));
      norm4 << -cos(fuyaw), -sin(fuyaw);
      hPoly_the.col(3).head<2>() = norm4;
      hPoly_the.col(3).tail<2>() = point4;   
      // hPoly_t.col(3).head<3>() = norm4;
      // hPoly_t.col(3).tail<3>() = point4;
        // hPolys_.push_back(hPoly);    
    cars_expanding[car_id][id_corrid]=distance2center;
    


    
}

void getSmaller_Rectangle_forretreat(Eigen::MatrixXd &hPoly_the,const Eigen::Vector2d pos,const double yaw,const double vel_,int id_corrid,int car_id,Eigen::Vector4d &overflow,double &retreat_dist)
{
    
    double resolution = Constants::mapResolution;
    double step = resolution;
   
      Eigen::Matrix2d ego_R;
      ego_R << cos(yaw), -sin(yaw),
                sin(yaw), cos(yaw);
      
      // Eigen::MatrixXd hPoly,hPoly_t;
      Eigen::MatrixXd hPoly;
      hPoly.resize(4, 4);
      // hPoly_t.resize(6,4);
      Eigen::Vector4d distance2center;
      distance2center=cars_expanding[car_id][id_corrid];
          for(int i = 0; i < 4; i++)
          {

              Eigen::Vector2d point1, point2, newpoint1, newpoint2; 
            //   bool isocc = false;               
              switch(i)
              {
              case 0: // 
                  if(distance2center(0)<=(Constants::ackerWidth / 2.0))
                  {overflow(0)=0.0;break;}
                  distance2center(i) -= step;
                  break;

              case 1: // dx
                  if(distance2center(1)<=(Constants::ackerLF+vel_/Constants::maxAcc))
                  {overflow(1)=0.0;break;}
                  distance2center(i) -= step;
                  break;

              case 2: // -dy
                  if(distance2center(2)<=(Constants::ackerLF+vel_/Constants::maxAcc))
                  {overflow(2)=0.0;break;}
                  distance2center(i) -= step;
                //   retreat_dist+=step;
                  break;

              case 3: // 
                if(distance2center(3)<=(Constants::ackerWidth / 2.0))
                  {overflow(3)=0.0;break;}
                  distance2center(i) -= step;
                  break;

              }
          }
      
      Eigen::Vector2d point1, norm1;
      point1 << pos + ego_R * Eigen::Vector2d(distance2center(1), distance2center(0));
      norm1 << -sin(yaw), cos(yaw);
      hPoly_the.col(0).head<2>() = norm1;
      hPoly_the.col(0).tail<2>() = point1;

      // hPoly_t.col(0).head<3>() = norm1;
      // hPoly_t.col(0).tail<3>() = point1;
    

      Eigen::Vector2d point2, norm2;
      point2 << pos + ego_R * Eigen::Vector2d(distance2center(1), -distance2center(2));
      norm2 << cos(yaw), sin(yaw);
      hPoly_the.col(1).head<2>() = norm2;
      hPoly_the.col(1).tail<2>() = point2;
      // hPoly_t.col(1).head<3>() = norm2;
      // hPoly_t.col(1).tail<3>() = point2;
      Eigen::Vector2d point3, norm3;
      point3 << pos + ego_R * Eigen::Vector2d(-distance2center(3), -distance2center(2));
      norm3 << sin(yaw), -cos(yaw);
      hPoly_the.col(2).head<2>() = norm3;
      hPoly_the.col(2).tail<2>() = point3;
      // hPoly_t.col(2).head<3>() = norm3;
      // hPoly_t.col(2).tail<3>() = point3;
      Eigen::Vector2d point4, norm4;
      point4 << pos + ego_R * Eigen::Vector2d(-distance2center(3), distance2center(0));
      norm4 << -cos(yaw), -sin(yaw);
      hPoly_the.col(3).head<2>() = norm4;
      hPoly_the.col(3).tail<2>() = point4;   
      // hPoly_t.col(3).head<3>() = norm4;
      // hPoly_t.col(3).tail<3>() = point4;
        // hPolys_.push_back(hPoly);    
    cars_expanding[car_id][id_corrid]=distance2center;
    


    
}

double ProjectPointOnAxis(const Eigen::Vector2d vertices,const Eigen::Vector2d normals)
{

return(vertices(0)*normals(0)+vertices(1)*normals(1));

}
void CheckRectangleCollide(const Eigen::MatrixXd Rectangle1,const Eigen::MatrixXd Rectangle2,bool &ifcollide)
{
ifcollide=true;
for(int i=0;i<4;i++)
{
   double min1,max1,min2,max2;
   min1=dbl_max;max1=-dbl_max;min2=dbl_max;max2=-dbl_max;
   for(int j=0;j<4;j++)
   {
      double projection=ProjectPointOnAxis(Rectangle1.col(j).tail<2>(),Rectangle1.col(i).head<2>());
      min1=std::min(min1,projection);
      max1=std::max(max1,projection);
   }
   for(int k=0;k<4;k++)
   {
      double projection=ProjectPointOnAxis(Rectangle2.col(k).tail<2>(),Rectangle1.col(i).head<2>());
      min2=std::min(min2,projection);
      max2=std::max(max2,projection);
   }
if(max1<min2||max2<min1)
{
   ifcollide=false;
}


}


}

bool overlap(const Eigen::VectorXd& proj1, const Eigen::VectorXd& proj2) {

    double minProj1 = proj1.minCoeff();

    double maxProj1 = proj1.maxCoeff();

    double minProj2 = proj2.minCoeff();

    double maxProj2 = proj2.maxCoeff();

    return !(minProj1 > maxProj2 || minProj2 > maxProj1);

}


// 对矩形投影到指定轴

Eigen::VectorXd project(const Eigen::MatrixXd& rect, const Eigen::Vector2d& axis) {

    Eigen::VectorXd result(rect.cols());

    for (int i = 0; i < rect.cols(); i++) {

        Eigen::Vector2d point(rect(2, i), rect(3, i));

        result(i) = point.dot(axis);

    }

    return result;

}



std::vector<int> CheckRectangleCollide_id(const Eigen::MatrixXd Rectangle1,const Eigen::MatrixXd Rectangle2,bool &ifcollide)
{
ifcollide=true;
std::vector<int> colide_col;
for(int i=0;i<Rectangle1.cols();i++)
{
  
Eigen::Vector2d axis(Rectangle1(0, i), Rectangle1(1, i));

        Eigen::VectorXd proj1 = project(Rectangle1, axis);

        Eigen::VectorXd proj2 = project(Rectangle2, axis);

        if (overlap(proj1, proj2)) {

            ifcollide=true;
            colide_col.push_back(i);

        }

}
return colide_col;

}




void visualization_corridor()
{
    
   // if (corrids_pub_markerarray.getNumSubscribers() == 0)
   //  {
   //    return;
   //  }
   // cout<<"222222222222222222"<<endl;
    // corrids_pub_markerarray = nh.advertise<visualization_msgs::MarkerArray>("/corrids", 10);
    // corrids_pub_clearmarkerarray = nh.advertise<visualization_msgs::MarkerArray>("/corridsclean", 10);
    // numid=1;
    
    
//    visualization_msgs::Marker marker_Clear;
//    marker_Clear.action=visualization_msgs::Marker::DELETEALL;
//    marker_ClearArray.markers.push_back(marker_Clear);
//    corrids_pub_clearmarkerarray.publish(marker_ClearArray);
//    double prio_time=0;

  
 for (int j=0;j<all_corrid_.size();j++)
 {
   // cout<<"33333333333333333"<<endl;
    std::cout<<"all_corrid_[j].size() "<<all_corrid_[j].size()<<std::endl;
    int count_time=1;
    for(const auto onpiece_state : all_corrid_[j])
    {
    
    visualization_msgs::Marker marker;
    marker.header.frame_id="world";
    marker.header.stamp = ros::Time::now();
    marker.type=visualization_msgs::Marker::CUBE;
    marker.action=visualization_msgs::Marker::ADD;
    marker.id=numid;
    Eigen::Vector2d _center=(onpiece_state.col(0).tail<2>()+onpiece_state.col(1).tail<2>()+onpiece_state.col(2).tail<2>()+onpiece_state.col(3).tail<2>())/4;
    double dy=std::sqrt(std::pow(onpiece_state.col(0)(2)-onpiece_state.col(1)(2),2)+std::pow(onpiece_state.col(0)(3)-onpiece_state.col(1)(3),2));
    double dx=std::sqrt(std::pow(onpiece_state.col(2)(2)-onpiece_state.col(1)(2),2)+std::pow(onpiece_state.col(2)(3)-onpiece_state.col(1)(3),2));
    marker.pose.position.x=_center(0);
    marker.pose.position.y=_center(1);
    marker.pose.position.z=count_time*Constants::timeResolution;
    Eigen::Vector3d direction1,direction2,direction;
    direction1<<onpiece_state.col(0).head<2>(),0;
    direction2<<onpiece_state.col(1).head<2>(),0;
    direction2.normalized();
    Eigen::Quaterniond orientation_;
    orientation_.setFromTwoVectors(Eigen::Vector3d(1,0,0),direction2);

    marker.pose.orientation.w=orientation_.w();
    marker.pose.orientation.x=orientation_.x();
    marker.pose.orientation.y=orientation_.y();
    marker.pose.orientation.z=orientation_.z();

    marker.scale.x=dx;
    marker.scale.y=dy;
 

    marker.scale.z=Constants::timeResolution;

    marker.color.r=j%2;
    marker.color.g=(j/2)%2;

    marker.color.b=(j/4)%2;
    marker.color.a=0.5;
    marker_array.markers.push_back(marker);
    numid=numid+1;
    count_time=count_time+1;
    // corrids_pub_markerarray.publish(marker_array);

    }

 }

// for (int kk=0;kk<100;kk++)
// {
//   corrids_pub_markerarray.publish(marker_array);  
// }


  ros::Timer ttimer = nh_.createTimer(ros::Duration(1.0), 
                        boost::bind(&Safe_Corridor::timer_Callback,this,_1));



}
void timer_Callback(const ros::TimerEvent& e)
{
ROS_INFO("TIMER_CALLBACKININ");
corrids_pub_markerarray.publish(marker_array);


}




};

