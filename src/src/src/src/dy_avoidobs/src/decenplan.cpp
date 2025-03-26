#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include "nav_msgs/Path.h"
#include <pcl/filters/passthrough.h>                 //直通滤波器头文件
//承担fsm功能
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include<fstream>
#include<math.h>   
#include <visualization_msgs/Marker.h>
// #include "opencv2/opencv.hpp"
#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <dy_avoidobs/planner_manager.h>
#include "std_msgs/Bool.h"



#include<path_searching/dyn_a_star.h>

using namespace std;
using namespace dy_obs;

ros::Subscriber path_sub;
ros::Subscriber Dpath_sub;
ros::Subscriber curpose_sub;
// ros::Subscriber trigger_theswarm;


ros::Subscriber localmap_sub;
ros::Subscriber swarm_trajs_sub_,broadcast_bspline_sub_;
// std::string trac_yaml;
// std::string stablemap;
// std::string pathy;
ros::Publisher _grid_path_vis_pub;
ros::Publisher _grid_localmap_vis_pub;
ros::Publisher _grid_globalseg_vis_pub;

ros::Publisher  local_a_star_path;

ros::Publisher  swarm_trajs_pub_, broadcast_bspline_pub_;
vector<vector<Vector3d>> node_Apath;
vector<vector<Vector3d>> node_map;
vector<vector<Vector3d>> global_seg;
ros::Timer exec_timer_, safety_timer_,curren_pos_timer;
double t_start_all;
// geometry_msgs::Pose::Ptr curpose_=boost::make_shared <geometry_msgs::Pose> ();
geometry_msgs::PoseStamped pathpose;
visualization_msgs::Marker marker;
nav_msgs::Path globalPath;
bool globcaltrajstate=false;
bool cur_a_star=false;
bool flag_escape_emergency_;

PlanningVisualization::Ptr visualization_;
int car_id_;
int car_num;
// int waypoint_num_, wp_id_;
int continously_called_times_{0};
// int timecurr;
// int posecount;

Eigen::Vector3d globalgoal;
// pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
std::vector<tracpara> vi;
// std::vector<tracpara> loacalvi(200);
// std::vector<Eigen::Vector3d> loacalvi(200);
bool localgolestate=false;
Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
bool have_odom_=false;
bool have_target_=false;
// bool have_recv_pre_agent_ = false;
bool have_trigger_=true;
// bool the_trigger_=false;

double emergency_time_;//如果距离碰撞的时间小于该值，立刻切换到停止模式
double no_replan_thresh_, replan_thresh_;
std::string global_path;
// std::vector<Eigen::Vector3d> wps_;
traj_utils::MultiBsplines multi_bspline_msgs_buf_;
// bool have_recv_pre_agent_=false;
// AStar::Ptr a_star_;
double timemax=0;

EGOPlannerManager::Ptr planner_manager_;
FSM_EXEC_STATE exec_state_;



std::string pcd_file_back;
std::string pcd_file_trac;
std::string file_directory;
std::string file_name_back;
std::string file_name_trac;
// Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
// Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
// Eigen::Vector3d local_target_pt_, local_target_vel_;   
// int countvi=0;
 bool planGlobalTraj(const nav_msgs::Path path)
  {
  
  int pt_num = globalPath.poses.size();
  
  Eigen::MatrixXd pos(3, pt_num);
  for (int i = 0; i < pt_num; ++i)
  {
    pos(0,i) = globalPath.poses[i].pose.position.x;
    pos(1,i) = globalPath.poses[i].pose.position.y;
    pos(2,i) = globalPath.poses[i].pose.position.z;
  }
  Eigen::Vector3d start_pos,end_pos;
  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd time(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i)
  {
    time(i) = 1;//间隔1s的点
  }

  // time(0) *= 2.0;
  // time(time.rows() - 1) *= 2.0;//首尾拉长因为要加减速度
  PolynomialTraj gl_traj;
  start_pos(0) = globalPath.poses[0].pose.position.x;
  start_pos(1) = globalPath.poses[0].pose.position.y;
  start_pos(2) = globalPath.poses[0].pose.position.z;
  end_pos(0)=path.poses.back().pose.position.x;
  end_pos(1)=path.poses.back().pose.position.y;
  end_pos(2)=path.poses.back().pose.position.z;


  if (pos.cols() >= 3)
    gl_traj = PolynomialTraj::minSnapTraj(pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), time);
  else if (pos.cols() == 2)
    gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos,Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), end_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), time(0));
  else
    return false;

  auto time_now = ros::Time::now();
  planner_manager_->set_gloabltrajanddata(gl_traj, time_now);
  planner_manager_->global_data_.setGlobalTraj(gl_traj, time_now);
  planner_manager_->bspline_optimizer_->Setgloabaldata(planner_manager_->global_data_);
    // cout<<"iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii"<<endl;
    // cout<<"global_data_.global_traj_.evaluate(0)"<<planner_manager_->global_data_.global_traj_.evaluate(0)<<endl;
    // cout<<"global_data_.global_traj_.evaluate(1)"<<planner_manager_->global_data_.global_traj_.evaluate(1)<<endl;
    // cout<<"global_data_.global_traj_.evaluate(2)"<<planner_manager_->global_data_.global_traj_.evaluate(2)<<endl;
    // cout<<"global_data_.global_traj_.evaluate(3)"<<planner_manager_->global_data_.global_traj_.evaluate(3)<<endl;
    // cout<<"global_data_.global_traj_.evaluate(4)"<<planner_manager_->global_data_.global_traj_.evaluate(4)<<endl;
    // cout<<"global_data_.global_traj_.evaluate(0)"<<gl_traj.evaluate(0)<<endl;
    // cout<<"global_data_.global_traj_.evaluate(1)"<<gl_traj.evaluate(1)<<endl;
    // cout<<"global_data_.global_traj_.evaluate(2)"<<gl_traj.evaluate(2)<<endl;
    // cout<<"global_data_.global_traj_.evaluate(3)"<<gl_traj.evaluate(3)<<endl;
    // cout<<"global_data_.global_traj_.evaluate(4)"<<gl_traj.evaluate(4)<<endl;
  return true;

  }
void visGridPath( vector<vector<Vector3d>> nodes)
{   
    
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    std::string nsstr = "car_"+std::to_string(car_id_)+"_decenplan/grid_path_vis";
    node_vis.ns = nsstr.c_str();

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

   
    node_vis.color.a = 1.0;
    node_vis.color.r = 0.0;
    node_vis.color.g = 1.0;
    node_vis.color.b = 0.0;
    

    node_vis.scale.x = 2.0;
    node_vis.scale.y = 2.0;
    node_vis.scale.z = 2.0;

    geometry_msgs::Point pt;
  for(int i = 0; i < int(nodes.size()); i++)
{
    for(int j = 0; j < int(nodes[i].size()); j++)
    {
        Vector3d coord = nodes[i][j];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }
}
    _grid_path_vis_pub.publish(node_vis);
}

void visGridloaclmap( vector<vector<Vector3d>> nodes)
{   
   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    std::string nsstr = "car_"+std::to_string(car_id_)+"_decenplan/grid_loalmap_vis";
    node_vis.ns = nsstr.c_str();

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

   
    node_vis.color.a = 1.0;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;
    

    node_vis.scale.x = 2.0;
    node_vis.scale.y = 2.0;
    node_vis.scale.z = 2.0;

    geometry_msgs::Point pt;
 for(int i = 0; i < int(nodes.size()); i++)
{
    for(int j = 0; j < int(nodes[i].size()); j++)
    {
        Vector3d coord = nodes[i][j];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }
}

    _grid_localmap_vis_pub.publish(node_vis);
}
void visGridglobal_seg(vector<vector<Vector3d>> nodes)
{
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    std::string nsstr = "car_"+std::to_string(car_id_)+"_decenplan/grid_globalseg_vis";
    node_vis.ns = nsstr.c_str();

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

   
    node_vis.color.a = 1.0;
    node_vis.color.r = 1.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;
    

    node_vis.scale.x = 2.0;
    node_vis.scale.y = 2.0;
    node_vis.scale.z = 2.0;

    geometry_msgs::Point pt;
 for(int i = 0; i < int(nodes.size()); i++)
{
    for(int j = 0; j < int(nodes[i].size()); j++)
    {
        Vector3d coord = nodes[i][j];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }
}

    _grid_globalseg_vis_pub.publish(node_vis);
}

  std::pair<int,FSM_EXEC_STATE> timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }
 
void planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    bool success = false;
    if (have_odom_)
    {
    cout<<"odom_pos_ is "<<odom_pos_(0)<<odom_pos_(1)<<odom_pos_(2)<<endl;
    cout<<"next_wp is "<<next_wp(0)<<next_wp(1)<<next_wp(2)<<endl;

    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), next_wp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    cout<<"success one is "<<success<<endl;
    visualization_->displayGoalPoint(Eigen::Vector3d(next_wp(0),next_wp(1),0), Eigen::Vector4d(0, 0.5, 0.5, 1), 5, 0);
    // visualization_->displayGoalPoint(Eigen::Vector3d(0,0,0), Eigen::Vector4d(0, 0.5, 0.5, 1), 10, 0);


    if (success)
    {
      planner_manager_->end_pt_ = next_wp;

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      planner_manager_->end_vel_.setZero();
      have_target_ = true;
      planner_manager_->have_new_target_ = true;
      cout<<"exec_state_xxxxxxxxxx"<<exec_state_<<endl;
      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          ros::spinOnce();
          ros::Duration(0.001).sleep();
        }
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      // visualization_->displayGoalPoint(planner_manager_->end_pt_, Eigen::Vector4d(1, 0, 0, 1), 1, 0);
      cout<<"gloabl_traj size "<<gloabl_traj.size()<<endl;
      cout<<"gloabl_traj 1 "<<gloabl_traj[1]<<endl;
      cout<<"gloabl_traj end "<<gloabl_traj.back()<<endl;


      visualization_->displayGlobalPathList(gloabl_traj, 1.0, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }

    }
   
  }

// void pubthe_astar_path(vector<Vector3d> nodes)
// {
// nav_msgs::Path localPaths;
// geometry_msgs::PoseStamped current_pose;
// for (auto a:nodes)
// {
// current_pose.header.stamp = ros::Time::now();
// current_pose.header.frame_id = "world";
// current_pose.pose.position.x = a(0);
// current_pose.pose.position.y = a(1);
// current_pose.pose.position.z = a(2);//时间
// current_pose.pose.orientation.x= car_id_;//时间

// localPaths.poses.push_back(current_pose);
// }
// local_a_star_path.publish(localPaths);


// }

 bool planFromGlobalTraj(const int trial_times /*=1*/) //zx-todo
  {
    cout<<"planFromGlobalTraj ininin-------------"<<endl;
    planner_manager_->start_pt_ = odom_pos_;
    planner_manager_->start_vel_ = odom_vel_;
    planner_manager_->start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
    flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (planner_manager_->callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }
  bool planFromCurrentTraj(const int trial_times=1 /*=1*/)
  {
    cout<<"planFromCurrentTraj ininin----------------------------"<<endl;

    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    planner_manager_->start_pt_ = info->position_traj_.evaluateDeBoorT(0.3);
    planner_manager_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(0.3);
    planner_manager_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(0.3);
    // planner_manager_->start_pt_ = info->position_traj_.evaluateDeBoorT(info->duration_/2);
    // planner_manager_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(info->duration_/2);
    // planner_manager_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(info->duration_/2);

    bool success = planner_manager_->callReboundReplan(false, false);
    ofstream ofs;
    std::string str11 = "/home/rancho/1hmy/ego-planner-swarm/record/planFromCurrentTraj";
    std::string str1txt=str11+std::to_string(car_id_);
    str11=str1txt+".txt";
    ofs.open(str11,std::ios::app);
    ofs<<"t_cur "<<t_cur<<endl;
    ofs<<"success "<<success<<endl;
    
    if (!success)
    {
      success = planner_manager_->callReboundReplan(true, false);
    ofs<<"success "<<success<<endl;

      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = planner_manager_->callReboundReplan(true, true);
          ofs<<"success "<<success<<endl;

          if (success)
            break;
        }
        if (!success)
        {
          ofs.close();
          return false;
        }
      }
    }
    ofs.close();
    return true;
  }
    bool planFromCurrentTraj_(const int trial_times=1 /*=1*/)
  {
    cout<<"planFromCurrentTraj_ ininin----------------------------"<<endl;

    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();
    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    planner_manager_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    planner_manager_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    planner_manager_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    // planner_manager_->start_pt_ = info->position_traj_.evaluateDeBoorT(info->duration_);
    // planner_manager_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(info->duration_);
    // planner_manager_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(info->duration_);

    bool success = planner_manager_->callReboundReplan(false, false);

    if (!success)
    {
      // success = planner_manager_->callReboundReplan(true, false);
      success = planner_manager_->callReboundReplan(false, false);

      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = planner_manager_->callReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }
// void odometryCallback(const nav_msgs::OdometryConstPtr &msg)
//   {
//     odom_pos_(0) = msg->pose.pose.position.x;
//     odom_pos_(1) = msg->pose.pose.position.y;
//     odom_pos_(2) = msg->pose.pose.position.z;

//     odom_vel_(0) = msg->twist.twist.linear.x;
//     odom_vel_(1) = msg->twist.twist.linear.y;
//     odom_vel_(2) = msg->twist.twist.linear.z;

//     //odom_acc_ = estimateAcc( msg );

//     have_odom_ = true;
//   }



  // void updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
  // {
  //   planner_manager_->local_data_.start_time_ = time_now;
  //   planner_manager_->local_data_.position_traj_ = position_traj;
  //   // getDerivative(position_traj,the_local_data_.velocity_traj_ );
  //   planner_manager_->local_data_.start_pos_ = planner_manager_->local_data_.position_traj_.evaluateDeBoorT(0.0);
  //   planner_manager_->local_data_.duration_ = planner_manager_->local_data_.position_traj_.getTimeSum();
  //   planner_manager_->local_data_.traj_id_ += 1;
  //   planner_manager_->local_data_.car_id = car_id_;
  //   cur_a_star=true;
  // }
  




  void publishSwarmTrajs(bool startup_pub)
  {
    auto info = &planner_manager_->local_data_;

    traj_utils::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.drone_id = planner_manager_->pp_.drone_id;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    ofstream ofs;
    ofs.open("/home/rancho/1hmy/ego-planner-swarm/record/test_pubswarm.txt",std::ios::app);
    ofs<<"startup_pub is " <<startup_pub<< endl;
    ofs<<"pos_pts.cols() is " <<pos_pts.cols()<< endl;//修改
    ofs<<"traj_id_ is " <<info->traj_id_<< endl;//修改
    ofs<<"drone_id is " << bspline.drone_id<< endl;//修改


   
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    // cout << knots.transpose() << endl;
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    if (startup_pub)
    {
      multi_bspline_msgs_buf_.drone_id_from = planner_manager_->pp_.drone_id; // zx-todo
      if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id + 1)
      {
        multi_bspline_msgs_buf_.traj.back() = bspline;
      }
      else if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id)
      {
        multi_bspline_msgs_buf_.traj.push_back(bspline);
      }
      else
      {
        ROS_ERROR("Wrong traj nums and drone_id pair!!! traj.size()=%d, drone_id=%d", (int)multi_bspline_msgs_buf_.traj.size(), planner_manager_->pp_.drone_id);
        // return plan_and_refine_success;
      }
      ofs<<"swarm_trajs_pub_" << endl;//修改

      swarm_trajs_pub_.publish(multi_bspline_msgs_buf_);
    }

    broadcast_bspline_pub_.publish(bspline);
    ofs.close();
  }


// void pathcallback(const nav_msgs::Path path) {

//   // cout<<"globcaltrajstate--------"<<globcaltrajstate<<endl;
//    if(globcaltrajstate)
//   {return;} 
//   globalPath=path; 
//   timemax=path.poses.back().pose.position.z;
//   // cout<<"path.poses.back().pose.position.z--------"<<car_id_<<endl;
//   if(planGlobalTraj(path))
//   {
//     // cout<<"the   success ????????????  "<<endl;

//     globcaltrajstate=true;
    
//     globalgoal(0)=path.poses.back().pose.position.x;
//     globalgoal(1)=path.poses.back().pose.position.y;
//     globalgoal(2)=timemax;
//     // if(have_odom_)
    

//   }

//   }
   

  // void showomdCallback(const ros::TimerEvent &e)
  // {

  // visualization_->displaycurrPoint(odom_pos_, Eigen::Vector4d(1, 0, 0, 1), 2, 0);

  // }
  void checkCollisionCallback(const ros::TimerEvent &e)
  {
    // if(the_trigger_)
    // {
    LocalTrajData *info = &planner_manager_->local_data_;
    // auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)//可以改为加载玩地图信息和收到odo现在位置信息
      return;

    /* ---------- check lost of depth ---------- */
    // if (map->getOdomDepthTimeout())
    // {
    //   ROS_ERROR("Depth Lost! EMERGENCY_STOP");
    //   enable_fail_safe_ = false;
    //   changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    // }

    /* ---------- check trajectory ---------- */
    // constexpr double time_step = 0.01;
    constexpr double time_step = 0.1;

    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    Eigen::Vector3d p_cur = info->position_traj_.evaluateDeBoorT(t_cur);
    odom_pos_=p_cur;

    const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();//0.5(xiangsu)改为2.0
    double t_cur_global = ros::Time::now().toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      bool occ = false;
      occ |= planner_manager_->bspline_optimizer_->a_star_->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t));

      for (size_t id = 0; id < planner_manager_->swarm_trajs_buf_.size(); id++)
      {
        if ((planner_manager_->swarm_trajs_buf_.at(id).drone_id != (int)id) || (planner_manager_->swarm_trajs_buf_.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

        double t_X = t_cur_global - planner_manager_->swarm_trajs_buf_.at(id).start_time_.toSec();
        Eigen::Vector3d swarm_pridicted = planner_manager_->swarm_trajs_buf_.at(id).position_traj_.evaluateDeBoorT(t_X);
        double dist = (p_cur - swarm_pridicted).norm();

        if (dist < CLEARANCE)
        {
          occ = true;
          break;
        }
      }
     
    
      // odom_pos_=p_cur;
      if (occ)
      {

        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          publishSwarmTrajs(false);
          return;
        }
        else
        {
          // if (t - t_cur < emergency_time_) // 0.8s of emergency time
          // {
          //   ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
          //   changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          // }
          // else
          // {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          // }
          return;
        }
        break;
      }
    }




    }
    
  
   void printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }
  bool callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    traj_utils::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    planner_manager_->bspline_pub_.publish(bspline);

    return true;
  }


 void getpose(const ros::TimerEvent &e)
 {
    // exec_timer_.stop(); 
    LocalTrajData *info = &planner_manager_->local_data_;
    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)//可以改为加载玩地图信息和收到odo现在位置信息
      return;
    constexpr double time_step = 0.3;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    cout<<"t_cur is "<<t_cur<<endl;
    cout<<"duration_ is "<<info->duration_<<endl;

    for (double t = t_cur; t < info->duration_; t += time_step)
    // if(t_cur<=timemax)
    // {
    {
    odom_pos_= info->position_traj_.evaluateDeBoorT(t_cur);
    odom_pos_(2) =ros::Time::now().toSec()-t_start_all;

    odom_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    have_odom_ = true;
   
    // visualization_->displaycurrPoint(odom_pos_, Eigen::Vector4d(1, 0, 1, 1), 2, 0);
    }
    // }
    // exec_timer_.start();
    

 }
void execFSMCallback(const ros::TimerEvent &e)
  {
    // if(the_trigger_)
    // {
    exec_timer_.stop(); // To avoid blockage
    // if(have_odom_)
    // {
    //   visualization_->displaycurrPoint(odom_pos_, Eigen::Vector4d(1, 0, 0, 1), 2, 0);

    // }
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!have_target_)
        cout << "wait for goal or trigger." << endl;
      fsm_num = 0;
    }
    cout << "exec_state_"<<exec_state_ << endl;
    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return;
        // return;
      }

      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return;
      // return;
      else
      {
        // if ( planner_manager_->pp_.drone_id <= 0 )
        // {
        //   changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        // }
        // else
        // {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
        // }
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm
    {
      // cout << "id=" << planner_manager_->pp_.drone_id << " have_recv_pre_agent_=" << have_recv_pre_agent_ << endl;
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 ))
      {
        if (have_odom_ && have_target_ && have_trigger_)
        {
          bool success = planFromGlobalTraj(10); // zx-todo
          if (success)
          {
            changeFSMExecState(EXEC_TRAJ, "FSM");

            publishSwarmTrajs(true);
          }
          else
          {
            ROS_ERROR("Failed to generate the first trajectory!!!");
            changeFSMExecState(SEQUENTIAL_START, "FSM");
          }
        }
        else
        {
          ROS_ERROR("No odom or no target! have_odom_=%d, have_target_=%d", have_odom_, have_target_);
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;
     
   
      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        publishSwarmTrajs(false);
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      // if (planFromCurrentTraj(1))
      if (planFromCurrentTraj_(1))

      
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        publishSwarmTrajs(false);
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);
      // odom_pos_=pos;
      /* && (end_pt_ - pos).norm() < 0.5 */
      // if ((wp_id_ < waypoint_num_ - 1) &&
      //     (planner_manager_->end_pt_ - pos).norm() < no_replan_thresh_)
      // {
      //   wp_id_++;
      //   planNextWaypoint(globalgoal);
      // }
      // else if ((planner_manager_->bspline_optimizer_->local_target_pt_ - planner_manager_->end_pt_).norm() < 1e-3) // close to the global target
      if ((planner_manager_->bspline_optimizer_->local_target_pt_ - planner_manager_->end_pt_).norm() < 1e-3) // close to the global target

      // {
        if (t_cur > info->duration_ - 1e-2)
        {
          have_target_ = false;
          have_trigger_ = false;

         
            // wp_id_ = 0;
            planNextWaypoint(globalgoal);

          changeFSMExecState(WAIT_TARGET, "FSM");
          goto force_return;
          // return;
        }
        else if ((planner_manager_->end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      // }
      else if (t_cur > replan_thresh_)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    // data_disp_.header.stamp = ros::Time::now();
    // data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
    // odom_pos_=
    planNextWaypoint(globalgoal);


    // }
    
  }

  void BroadcastBsplineCallback(const traj_utils::BsplinePtr &msg) //可能要加
  {
    ofstream ofs;
    ofs.open("/home/rancho/1hmy/ego-planner-swarm/record/test_pubboast.txt",std::ios::app);
    ofs<<"inin " <<endl;
    size_t id = msg->drone_id;
    ofs<<"id is "<<id<<endl;

    if ((int)id == planner_manager_->pp_.drone_id)
    {
      ofs<<"1return "<<endl;
      
      ofs.close();
      return;

    }
      

    if (fabs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
    {
      ROS_ERROR("Time difference is too large! Local - Remote Agent %d = %fs",
                msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
      ofs<<"Time difference is too large! Local  "<<endl;  
      ofs.close();    
      return;
    }

    /* Fill up the buffer */
    if (planner_manager_->swarm_trajs_buf_.size() <= id)
    {
      for (size_t i = planner_manager_->swarm_trajs_buf_.size(); i <= id; i++)
      {
        OneTrajDataOfSwarm blank;
        blank.drone_id = -1;
        planner_manager_->swarm_trajs_buf_.push_back(blank);
      }
    ofs<<"blank"<< endl;//修改

    }

    /* Test distance to the agent */
    Eigen::Vector3d cp0(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
    Eigen::Vector3d cp1(msg->pos_pts[1].x, msg->pos_pts[1].y, msg->pos_pts[1].z);
    Eigen::Vector3d cp2(msg->pos_pts[2].x, msg->pos_pts[2].y, msg->pos_pts[2].z);
    Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    if ((swarm_start_pt - odom_pos_).norm() > planner_manager_->pp_.planning_horizen_ * 4.0f / 3.0f)
    {
      planner_manager_->swarm_trajs_buf_[id].drone_id = -1;
      ofs<<"too far "<< endl;//修改
      ofs.close();
      return; // if the current drone is too far to the received agent.
    }

    /* Store data */
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t j = 0; j < msg->knots.size(); ++j)
    {
      knots(j) = msg->knots[j];
    }
    for (size_t j = 0; j < msg->pos_pts.size(); ++j)
    {
      pos_pts(0, j) = msg->pos_pts[j].x;
      pos_pts(1, j) = msg->pos_pts[j].y;
      pos_pts(2, j) = msg->pos_pts[j].z;
    }

    planner_manager_->swarm_trajs_buf_[id].drone_id = id;

    if (msg->order % 2)
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = msg->knots[msg->knots.size() - ceil(cutback)];
    }
    else
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = (msg->knots[msg->knots.size() - floor(cutback)] + msg->knots[msg->knots.size() - ceil(cutback)]) / 2;
    }

    UniformBspline pos_traj(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
    pos_traj.setKnot(knots);
    planner_manager_->swarm_trajs_buf_[id].position_traj_ = pos_traj;

    planner_manager_->swarm_trajs_buf_[id].start_pos_ = planner_manager_->swarm_trajs_buf_[id].position_traj_.evaluateDeBoorT(0);

    planner_manager_->swarm_trajs_buf_[id].start_time_ = msg->start_time;
    // planner_manager_->swarm_trajs_buf_[id].start_time_ = ros::Time::now(); // Un-reliable time sync

    /* Check Collision */
    if (planner_manager_->checkCollision(id))
    {
      changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK");
    }
   
    
    ofs<<"store is start_time is " <<msg->start_time<< endl;//修改
     ofs.close();
  }

  void swarmTrajsCallback(const traj_utils::MultiBsplinesPtr &msg)
  {
    ofstream ofs;
    ofs.open("/home/rancho/1hmy/ego-planner-swarm/record/test_subswarm.txt",std::ios::app);
    ofs<<"inin " <<endl;
    size_t id = msg->drone_id_from;
    ofs<<"id is "<<id<<endl;
    ofs.close();

    // multi_localtraj_msgs_buf_.traj.clear();
    // multi_localtraj_msgs_buf_ = *msg;
    multi_bspline_msgs_buf_.traj.clear();
    multi_bspline_msgs_buf_ = *msg;
    // cout << "\033[45;33mmulti_bspline_msgs_buf.drone_id_from=" << multi_bspline_msgs_buf_.drone_id_from << " multi_bspline_msgs_buf_.traj.size()=" << multi_bspline_msgs_buf_.traj.size() << "\033[0m" << endl;

    if (!have_odom_)
    {
      ROS_ERROR("swarmTrajsCallback(): no odom!, return.");
      return;
    }

    if ((int)msg->traj.size() != msg->drone_id_from + 1) // drone_id must start from 0
    {
      ROS_ERROR("Wrong trajectory size! msg->traj.size()=%d, msg->drone_id_from+1=%d", (int)msg->traj.size(), msg->drone_id_from + 1);
      return;
    }

    if (msg->traj[0].order != 3) // only support B-spline order equals 3.
    {
      ROS_ERROR("Only support B-spline order equals 3.");
      return;
    }

    // Step 1. receive the trajectories
    planner_manager_->swarm_trajs_buf_.clear();
    planner_manager_->swarm_trajs_buf_.resize(msg->traj.size());

    for (size_t i = 0; i < msg->traj.size(); i++)
    {

      Eigen::Vector3d cp0(msg->traj[i].pos_pts[0].x, msg->traj[i].pos_pts[0].y, msg->traj[i].pos_pts[0].z);
      Eigen::Vector3d cp1(msg->traj[i].pos_pts[1].x, msg->traj[i].pos_pts[1].y, msg->traj[i].pos_pts[1].z);
      Eigen::Vector3d cp2(msg->traj[i].pos_pts[2].x, msg->traj[i].pos_pts[2].y, msg->traj[i].pos_pts[2].z);
      Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
      if ((swarm_start_pt - odom_pos_).norm() > planner_manager_->pp_.planning_horizen_ * 4.0f / 3.0f)
      {
        planner_manager_->swarm_trajs_buf_[i].drone_id = -1;
        continue;
      }

      Eigen::MatrixXd pos_pts(3, msg->traj[i].pos_pts.size());
      Eigen::VectorXd knots(msg->traj[i].knots.size());
      for (size_t j = 0; j < msg->traj[i].knots.size(); ++j)
      {
        knots(j) = msg->traj[i].knots[j];
      }
      for (size_t j = 0; j < msg->traj[i].pos_pts.size(); ++j)
      {
        pos_pts(0, j) = msg->traj[i].pos_pts[j].x;
        pos_pts(1, j) = msg->traj[i].pos_pts[j].y;
        pos_pts(2, j) = msg->traj[i].pos_pts[j].z;
      }

      planner_manager_->swarm_trajs_buf_[i].drone_id = i;

      if (msg->traj[i].order % 2)
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)];
      }
      else
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = (msg->traj[i].knots[msg->traj[i].knots.size() - floor(cutback)] + msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)]) / 2;
      }

      // planner_manager_->swarm_trajs_buf_[i].position_traj_ =
      UniformBspline pos_traj(pos_pts, msg->traj[i].order, msg->traj[i].knots[1] - msg->traj[i].knots[0]);
      pos_traj.setKnot(knots);
      planner_manager_->swarm_trajs_buf_[i].position_traj_ = pos_traj;

      planner_manager_->swarm_trajs_buf_[i].start_pos_ = planner_manager_->swarm_trajs_buf_[i].position_traj_.evaluateDeBoorT(0);

      planner_manager_->swarm_trajs_buf_[i].start_time_ = msg->traj[i].start_time;
    }

    // have_recv_pre_agent_ = true;
  }

void yaml_read(std::string filename,int car_id_)
{
    YAML::Node config = YAML::LoadFile(filename);
  //   std::string agentname="agent";
  //   agentname=agentname+std::to_string(car_id_);
  // //  for(auto node: config["schedule"])
  //   // YAML::Node 
   if(config["schedule"][car_id_])
   {
        for (auto it = config["schedule"][car_id_].begin(); it != config["schedule"][car_id_].end(); ++it) {
            // 获取key
            std::string key = it->first.as<std::string>();
            // 获取value，假设value还是YAML::Node
            YAML::Node value = it->second;
            // 对value进一步处理    
            for(auto s: value){
          geometry_msgs::PoseStamped current_pose;
          current_pose.header.stamp = ros::Time::now();
          current_pose.header.frame_id = "world";
          current_pose.pose.position.x = s["x"].as<double>();
          current_pose.pose.position.y = s["y"].as<double>();
          current_pose.pose.position.z = s["t"].as<int>();//
          current_pose.pose.orientation.x= s["yaw"].as<double>();//yaw
          globalPath.poses.push_back(current_pose);

                
            }
          
        }
        
      
    }
  int pt_num = globalPath.poses.size();
  Eigen::MatrixXd pos(3, pt_num);
  for (int i = 0; i < pt_num; ++i)
  {
    pos(0,i) = globalPath.poses[i].pose.position.x;
    pos(1,i) = globalPath.poses[i].pose.position.y;
    if(i==pt_num-1)
      pos(2,i) = globalPath.poses[i].pose.position.z;
  }
  Eigen::Vector3d start_pos,end_pos;
  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd time(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i)
  {
    // time(i) = 1;//间隔1s的点
    time(i) = (pos.col(i + 1).head(2) - pos.col(i).head(2)).norm() *0.689/ (planner_manager_->pp_.max_vel_);
    if(i==0)
      pos(2,i)=0;
    else
    pos(2,i)=time(i);
  }

  // time(0) *= 2.0;
  // time(time.rows() - 1) *= 2.0;//首尾拉长因为要加减速度
  PolynomialTraj gl_traj;
  start_pos(0) = globalPath.poses[0].pose.position.x;
  start_pos(1) = globalPath.poses[0].pose.position.y;
  start_pos(2) = globalPath.poses[0].pose.position.z;
  end_pos(0)=globalPath.poses.back().pose.position.x;
  end_pos(1)=globalPath.poses.back().pose.position.y;
  end_pos(2)=globalPath.poses.back().pose.position.z;
  globalgoal=end_pos;


  if (pos.cols() >= 3)
    gl_traj = PolynomialTraj::minSnapTraj(pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), time);
  else if (pos.cols() == 2)
    gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos,Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), end_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), time(0));
  // else
  //   return false;

  auto time_now = ros::Time::now();
  planner_manager_->set_gloabltrajanddata(gl_traj, time_now);
  planner_manager_->global_data_.setGlobalTraj(gl_traj, time_now);
  planner_manager_->bspline_optimizer_->Setgloabaldata(planner_manager_->global_data_);
  timemax=end_pos(2);

  odom_pos_=start_pos;
  odom_vel_(0) = planner_manager_->global_data_.global_traj_.evaluateVel(0)(0);
  odom_vel_(1) = planner_manager_->global_data_.global_traj_.evaluateVel(0)(1);
  odom_vel_(2) = 0.3;

    
  have_odom_ = true;
  planner_manager_->init_pt_ = odom_pos_;
   
     cout<<"pt_num"<<pt_num<<endl;
}

int main(int argc, char **argv)
    {
      ros::init(argc, argv, "decenplan");//启动该节点并设置其名称，名称必须唯一  
      ros::NodeHandle nh("~");
    //ros::Publisher pub = n.advertise<std_msgs::String>("message", 1000);//将节点设置成发布者，并将所发布>主题道类型和名称告知节点管理器。第一个参数是消息的名称：message，第二个是缓冲区道大小。如果主题发布数据速度较快，那么将缓冲区设置为1000个消息。  
    //ros::Rate loop_rate(10);//设置发送数据道频率为10Hz  

      nh.param("car_id", car_id_, 10);
      nh.param("car_num", car_num, 10);
      nh.param("fsm/emergency_time", emergency_time_, 1.0);
      nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
      nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
      // nh.param("fsm/globalpath", global_path, "-1.0");

      // nh.param("trac_yaml", trac_yaml, std::string("/home/1.yaml"));
      // nh.param("mappng", stablemap, std::string("/home/1.yaml"));
      nh.param("fsm/path_yaml", global_path, std::string("/home/1.yaml"));
      nh.param("fsm/file_directory", file_directory, std::string("/home/"));
      nh.param("fsm/file_name_back", file_name_back, std::string("map0"));
      nh.param("fsm/file_name_trac", file_name_trac, std::string("map"));
      std::string str1 = "/path/agent";
      std::string str2 = "/Dpath/agent";
      std::string str3 = "/dynamic/pose_";
      // std::string str4 = "/grid_path_vis_";
      // std::string str5 = "/grid_localmap_vis_";
      // // std::string str6 = "/planning/local_target";
      // std::string str6 = "/grid_globalseg_vis_";
      

    const std::string pcd_format = ".pcd";
    pcd_file_back = file_directory + file_name_back + pcd_format;
    pcd_file_trac = file_directory + file_name_trac + pcd_format;

      std::string sub_topic_pathname=str1+std::to_string(car_id_);
      std::string sub_topic_Dpathname=str2+std::to_string(car_id_);
      std::string sub_topic_curpose=str3+std::to_string(car_id_);
      // std::string pub_topic_grid_path=str4+std::to_string(car_id_);
      // std::string pub_topic_localmap_path=str5+std::to_string(car_id_);
      // std::string pub_topic_globalseg_path=str6+std::to_string(car_id_);
    
    cout<<"11111111111111111111  "<<car_id_<<endl;

      visualization_.reset(new PlanningVisualization(nh,car_id_,pcd_file_back,pcd_file_trac));
    cout<<"222222222222222222222222  "<<car_id_<<endl;
      planner_manager_.reset(new EGOPlannerManager);
      // planner_manager_->reset(new EGOPlannerManager);
      planner_manager_->initPlanModules(nh,visualization_);
    cout<<"333333333333333333333333333  "<<endl;

      planner_manager_->deliverTrajToOptimizer(); // store trajectories
    cout<<"444444444444444444444444444444444444444444  "<<car_id_<<endl;

      planner_manager_->setDroneIdtoOpt();
    cout<<"555555555555555555555555555555555555  "<<car_id_<<endl;

      yaml_read(global_path,car_id_);
      planner_manager_->local_data_.traj_id_ = 0;
      t_start_all = ros::Time::now().toSec();
      exec_state_ = FSM_EXEC_STATE::INIT;
      exec_timer_ = nh.createTimer(ros::Duration(0.01), &execFSMCallback);
      safety_timer_ = nh.createTimer(ros::Duration(0.5), &checkCollisionCallback);
      // curren_pos_timer = nh.createTimer(ros::Duration(0.2), &getpose);

      // curren_pos_timer = nh.createTimer(ros::Duration(0.01), &showomdCallback);

      

      // curpose_sub=nh.subscribe<geometry_msgs::PoseStamped>(sub_topic_curpose.c_str(), 100,curposecallback); 
      // trigger_theswarm=nh.subscribe("/trigger_swarm", 1,trigger_callback); 


      // localmap_sub = nh.subscribe<sensor_msgs::PointCloud2>("/local_cloud_map", 1,localmapcallback);   
      // read_trac_yaml(trac_yaml);
      // read_map_(stablemap);
      // path_sub = nh.subscribe<nav_msgs::Path>(sub_topic_pathname.c_str(), 1,pathcallback);  

    if (car_id_ >= 1)
    {
      string sub_topic_name = string("/car_") + std::to_string(car_id_ - 1) + string("_planning/swarm_trajs");
      swarm_trajs_sub_ = nh.subscribe(sub_topic_name.c_str(), 10, swarmTrajsCallback, ros::TransportHints().tcpNoDelay());
    }
    string pub_topic_name = string("/car_") + std::to_string(car_id_) + string("_planning/swarm_trajs");
    swarm_trajs_pub_ = nh.advertise<traj_utils::MultiBsplines>(pub_topic_name.c_str(), 10);
    broadcast_bspline_pub_ = nh.advertise<traj_utils::Bspline>("planning/broadcast_bspline_from_planner", 10);
    broadcast_bspline_sub_ = nh.subscribe("planning/broadcast_bspline_to_planner", 10, &BroadcastBsplineCallback, ros::TransportHints().tcpNoDelay());


      // _grid_path_vis_pub = nh.advertise<visualization_msgs::Marker>(pub_topic_grid_path.c_str(), 1);
      // _grid_localmap_vis_pub= nh.advertise<visualization_msgs::Marker>(pub_topic_localmap_path.c_str(), 1);
      // _grid_globalseg_vis_pub= nh.advertise<visualization_msgs::Marker>(pub_topic_globalseg_path.c_str(), 1);
      // // local_a_star_path=nh.advertise<nav_msgs::Path>(pub_topic_localpath.c_str(), 2);
      // local_a_star_path=nh.advertise<nav_msgs::Path>("/planning/local_target", 1);
      // ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);//定时器，cmdcallback根据得到的轨迹发送该时刻的期望位置、速度、加速度和偏航角，如果已经执行完轨迹就旋停。周期0.01秒
      // ros::Timer Ttimer = nh.createTimer(ros::Duration(0.5), 
      //                   boost::bind(publishSwarmTrajs, _1,cur_a_star,local_data_));
        
    // cout<<"11111111111111111111  "<<car_id_<<endl;

      ROS_INFO("Wait for 1 second.");
      int count = 0;
      while (ros::ok() && count++ < 6000)
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      ROS_WARN("Waiting for trigger from [n3ctrl] from RC");


      while (ros::ok() && (!have_odom_ || !have_trigger_))
      {
        // cout<<"ru 2 "<<have_odom_<<endl;      
      //   if(the_trigger_)
      // {
      // exec_timer_ = nh.createTimer(ros::Duration(0.01), &execFSMCallback);
      // safety_timer_ = nh.createTimer(ros::Duration(0.05), &checkCollisionCallback);
      // the_trigger_=false;
      // }
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }


      //  ros::Rate r(2);   
      //   while (ros::ok()){
      //       // if (listener.num >= agv_count * 2){
      //       //     std::cout<< "starting " <<std::endl;
      //       //     break;
      //       // }
      //       ros::spinOnce();                
      //       r.sleep(); }
    // ros::spin();
  //  while (ros::ok()){
  //  ros::Duration(0.1).sleep();
  //   spinner.start();
  //  }

  return 0;
    }