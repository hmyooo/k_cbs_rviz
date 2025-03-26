#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <traj_utils/DataDisp.h>
#include <traj_utils/Bspline.h>
#include <traj_utils/MultiBsplines.h>
#include <traj_utils/planning_visualization.h>
// #include <plan_env/grid_map.h>
#include <plan_env/obj_predictor.h>
#include <traj_utils/plan_container.hpp>
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>
#include "nav_msgs/Path.h"
#include <dy_avoidobs/localtraj.h>
#include <dy_avoidobs/Multilocaltrajs.h>

#include<string>
namespace dy_obs
{
    enum FSM_EXEC_STATE
    {
      INIT,//初始状态
      WAIT_TARGET,//等待触发
      GEN_NEW_TRAJ,//生成轨迹
      REPLAN_TRAJ,//重规划轨迹
      EXEC_TRAJ,//执行轨迹
      EMERGENCY_STOP,//紧急停止
      SEQUENTIAL_START//顺序启动
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };
  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager
  {
    // SECTION stable
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool planGlobalTraj_(const nav_msgs::Path path);
    void getLocalTarget();
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method

    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    void initPlanModules(ros::NodeHandle &nh,PlanningVisualization::Ptr vis);

    void deliverTrajToOptimizer(void) { bspline_optimizer_->setSwarmTrajs(&swarm_trajs_buf_); };

    void setDroneIdtoOpt(void) { bspline_optimizer_->setDroneId(pp_.drone_id); }

    double getSwarmClearance(void) { return bspline_optimizer_->getSwarmClearance(); }

    bool checkCollision(int drone_id);
    void set_gloabltrajanddata(const PolynomialTraj &traj, const ros::Time &time);
   
    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    // GridMap::Ptr grid_map_;
    fast_planner::ObjPredictor::Ptr obj_predictor_;    
    SwarmTrajData swarm_trajs_buf_;
    ros::Publisher  bspline_pub_;
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;    
    PlanningVisualization::Ptr visualization_;
    bool have_new_target_;
    BsplineOptimizer::Ptr bspline_optimizer_;


    // bool have_odom_ ;

  private:
    /* main planning algorithms & modules */
    
    // ros::Publisher obj_pub_; //zx-todo 

    dy_avoidobs::Multilocaltrajs multi_localtraj_msgs_buf_;

    // void publishSwarmTrajs(bool startup_pub);
    bool flag_step_2_success ;
    double ratio;
    int continous_failures_count_{0};
    double the_t_step_,progress_time;
    void updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now);

    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);
    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);
    // !SECTION stable

    // SECTION developing

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };


  
} //


#endif