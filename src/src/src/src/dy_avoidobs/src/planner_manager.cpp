// #include <fstream>
#include <dy_avoidobs/planner_manager.h>
#include <thread>
#include "visualization_msgs/Marker.h" // zx-todo
#include <iostream>

#include<fstream>

namespace dy_obs
{
static double sum_time = 0;
static double sum_opttime = 0;
static double count_success = 0;
bool globcaltrajstate=false;
nav_msgs::Path globalPath;
EGOPlannerManager planglobal;
  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() {}

  // void pathcallback(const nav_msgs::Path path) {

  // // cout<<"globcaltrajstate--------"<<globcaltrajstate<<endl;
  //  if(globcaltrajstate)
  // {return;} 
  // globalPath=path;
  // double timemax;
  // timemax=path.poses.back().pose.position.z;
  // // cout<<"path.poses.back().pose.position.z--------"<<car_id_<<endl;
  // if(planglobal.planGlobalTraj_(path))
  // {globcaltrajstate=true;}

  // }
  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh,PlanningVisualization::Ptr vis )
  {
    /* read algorithm parameters */
    std::string trac_yaml;
    // double max_vec_;
    cout<<"aaaaaaaaaaaaaaaaaaaaa"<<endl;
    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_vel_yaw", pp_.max_vel_yaw, -1.0);
    pp_.max_vel_yaw=pp_.max_vel_yaw/180.f* M_PI;
    // nh.param("manager/max_vel", max_vec_, -1.0);

    
    cout<<"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"<<endl;

    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("manager/use_distinctive_trajs", pp_.use_distinctive_trajs, false);
    nh.param("manager/drone_id", pp_.drone_id, -1);
    nh.param("manager/trac_yaml", trac_yaml, std::string("/home/1.yaml"));


    std::string str1 = "/path/agent";
    std::string sub_topic_pathname=str1+std::to_string(pp_.drone_id);
    ros::Subscriber path_sub;
    // path_sub = nh.subscribe<nav_msgs::Path>(sub_topic_pathname.c_str(), 1,pathcallback);  
    // have_odom_ = false;

    local_data_.traj_id_ = 0;
    // grid_map_.reset(new GridMap);
    // grid_map_->initMap(nh);

    // obj_predictor_.reset(new fast_planner::ObjPredictor(nh));
    // obj_predictor_->init();
    // obj_pub_ = nh.advertise<visualization_msgs::Marker>("/dynamic/obj_prdi", 10); // zx-todo

    bspline_optimizer_.reset(new BsplineOptimizer);
    cout<<"ccccccccccccccccccccccccccccccc"<<endl;

    bspline_optimizer_->setParam(nh);
    cout<<"dddddddddddddddddddddddddddddddd"<<endl;

    bspline_optimizer_->read_trac_yaml(trac_yaml);
    cout<<"eeeeeeeeeeeeeeeeeeeeee"<<endl;

    
    bspline_optimizer_->setEnvironment();
    cout<<"ffffffffffffffffffffffffff"<<endl;

    bspline_optimizer_->setEnvironment_(obj_predictor_);
    cout<<"gggggggggggggggggggggg"<<endl;

    // bspline_optimizer_->a_star_.reset(new AStar);
    // cout<<"hhhhhhhhhhhhhhhhh"<<endl;

    // bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));
    // bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(150, 150, 150));//修改


    visualization_ = vis;
    bspline_pub_ = nh.advertise<traj_utils::Bspline>("planning/bspline", 10);
  }
 bool EGOPlannerManager::planGlobalTraj_(const nav_msgs::Path path)
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
  global_data_.setGlobalTraj(gl_traj, time_now);

  return true;

  }
  // !SECTION


// void EGOPlannerManager::publishSwarmTrajs(bool startup_pub)
// {
   
//   dy_avoidobs::localtraj localtraj_;
//   localtraj_.start_time =local_data_.start_time_;
//   localtraj_.traj_id= local_data_.traj_id_;
//   // localtraj_.duration_ = planner_manager_->local_data_.duration_;
//   localtraj_.car_id = local_data_.car_id;
//   if(local_data_.position_traj_.getControlPoint().cols()>0)
//   {
//     Eigen::MatrixXd pos_pts = local_data_.position_traj_.getControlPoint();

//     for (int i = 0; i < pos_pts.cols(); i++)
//     {
//       geometry_msgs::Point pt;
//       pt.x = pos_pts(0, i);
//       pt.y = pos_pts(1, i);
//       pt.z = pos_pts(2, i);
//       localtraj_.pos_pts.push_back(pt);
//     }
//   }
   
//     if (startup_pub)
//     {
//       multi_localtraj_msgs_buf_.car_id_from = local_data_.car_id; // zx-todo
//       // if ((int)multi_localtraj_msgs_buf_.traj.size() == planner_manager_->local_data_.car_id + 1)
//       // {
//       //   multi_localtraj_msgs_buf_.traj.back() = localtraj_;
//       // }
//       // else if ((int)multi_localtraj_msgs_buf_.traj.size() == planner_manager_->local_data_.car_id)
//       // {
//         multi_localtraj_msgs_buf_.traj.push_back(localtraj_);
//       // }
//       // else
//       // {
//       //   ROS_ERROR("Wrong traj nums and drone_id pair!!! traj.size()=%d, drone_id=%d", (int)multi_localtraj_msgs_buf_.traj.size(),the_local_data_.car_id);
//       //   // return plan_and_refine_success;
//       // }
//       swarm_trajs_pub_.publish(multi_localtraj_msgs_buf_);

//     }


// }
//   // SECTION rebond replanning

bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {
    static int count = 0;
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n", pp_.drone_id, count++);
    // cout.precision(3);
    // cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
    //      << endl;

    // if ((start_pt - local_target_pt).norm() < 0.2)//修改
    if ((sqrt(pow(start_pt[0]-local_target_pt[0],2)+pow(start_pt[1]-local_target_pt[1],2)) < 0.2))//修改
    {
      cout << "Close to goal" << endl;
      continous_failures_count_++;
      return false;
    }

    bspline_optimizer_->setLocalTargetPt(local_target_pt);

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt, t_refine;

    /*** STEP 1: INIT ***/
    // double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.5 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits修改
    // double ts = sqrt(pow(start_pt[0]-local_target_pt[0],2)+pow(start_pt[1]-local_target_pt[1],2)) > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.5 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    // double ts = 1;
    // double ts = (local_target_pt[2]-start_pt[2])*0.1;//修改
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    double ts=the_t_step_;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.//则规划初始点到局部重点的多项式轨迹。分配的时间为以最大加速度和速度飞行所需的时间。
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        // PolynomialTraj gl_traj;

        // double dist = (start_pt - local_target_pt).norm();//修改
        // double dist = sqrt(pow(start_pt[0]-local_target_pt[0],2)+pow(start_pt[1]-local_target_pt[1],2));//修改
        // cout<<"dist"<<dist<<endl;
        cout<<"start_pt"<<start_pt[0]<<" "<<start_pt[1]<<" "<<start_pt[2]<<endl;
        cout<<"local_target_pt"<<local_target_pt[0]<<" "<<local_target_pt[1]<<" "<<local_target_pt[2]<<endl;

        // cout<<"dist"<<dist<<endl;

        // double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;
        // if(time>0.5)//修改
        // {
        // cout << "one_segment_traj_gen exceeds 0.5s"<< time << endl;//修改
        // }
        // if(flag_step_2_success)
        //   local_target_pt[2]=local_target_pt[2]*ratio;
        double time = progress_time;
        // if (!flag_randomPolyTraj)//计算出始末两点的多项式轨迹
        // {
        //   local_target_pt[2]=(start_pt[2]+time)*0.1;//修改
        //   gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        // }
        // else//在始末位置之间随机插入一个点，调用PolynomialTraj::minSnapTraj() 计算出轨迹
        // {
        //   local_target_pt[2]=(start_pt[2]+time)/0.2;//修改
        //   Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
        //   Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
        //   Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
        //                                        (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
        //                                        (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
        //   Eigen::MatrixXd pos(3, 3);
        //   pos.col(0) = start_pt;
        //   pos.col(1) = random_inserted_pt;
        //   pos.col(2) = local_target_pt;
        //   Eigen::VectorXd t(2);
        //   t(0) = t(1) = time / 2;
        //   gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        // }

        double t;
        bool flag_too_far;
        // ts *= 1.5; // ts will be divided by 1.5 in the next
        //将得到的轨迹以ts=ctrl_pt_dist/max_vel_ 为时间间隔分段，如果分段的距离大于1.5倍定义的控制点距离ctrl_pt_dist 
        //则ts/=1.5 再继续分，直到分段的距离都小于1.5倍定义的控制点距离ctrl_pt_dist 并且分段点的数量大于7。最后将轨迹的始末速度和加速度赋值给start_end_derivatives
        cout<<"time "<< time <<endl;
        // cout<<"global_data_.global_traj_.evaluate(0)"<<global_data_.global_traj_.evaluate(0)<<endl;
        // cout<<"global_data_.global_traj_.evaluate(1)"<<global_data_.global_traj_.evaluate(1)<<endl;
        // cout<<"global_data_.global_traj_.evaluate(2)"<<global_data_.global_traj_.evaluate(2)<<endl;
        // cout<<"global_data_.global_traj_.evaluate(3)"<<global_data_.global_traj_.evaluate(3)<<endl;
        // cout<<"global_data_.global_traj_.evaluate(4)"<<global_data_.global_traj_.evaluate(4)<<endl;
        
        do
        {
          // ts /= 1.5;
          // ts=0.3;
          point_set.clear();
          flag_too_far = false;
          // Eigen::Vector3d last_pt = global_data_.global_traj_.evaluate(0);
          Eigen::Vector3d last_pt = start_pt;

          // cout<<"last_pt "<< last_pt <<endl;

          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = global_data_.global_traj_.evaluate(t+start_pt(2));
            // Eigen::Vector3d pt = global_data_.global_traj_.evaluate(t);
           
          // cout<<"pt "<< pt <<endl;
          // if(flag_step_2_success)
          //   {
          //   pt = global_data_.global_traj_.evaluate(t+start_pt[2]);
          //    pt[2]=pt[2]*ratio;
          //   }
          cout<<"last_pt - pt "<<(last_pt.head(2) - pt.head(2)).norm() <<endl;
          cout<<"ctrl_pt_dist*1.5 "<<pp_.ctrl_pt_dist*1.5 <<endl;
          cout<<"time "<< time <<endl;
          cout<<"ts "<< ts <<endl;
          cout<<"t "<< t <<endl;




            // if ((last_pt.head(2) - pt.head(2)).norm() > pp_.ctrl_pt_dist * 1.5)
            // {
            //   flag_too_far = true;
            //   break;
            // }
            last_pt = pt;
            point_set.push_back(pt);
          }
          // cout<<"flag_too_far "<<flag_too_far <<endl;
          cout<<"point_set.size "<<point_set.size() <<endl;

        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(global_data_.global_traj_.evaluateVel(start_pt(2)));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(global_data_.global_traj_.evaluateAcc(start_pt(2)));
        start_end_derivatives.push_back(global_data_.global_traj_.evaluateAcc(t+start_pt(2)));

        cout<<"no.1 part "<<endl;
      }
      else // Initial path generated from previous trajectory.
      {
      ofstream ofs;
      std::string str11 = "/home/rancho/1hmy/ego-planner-swarm/record/test_else.txt";
      std::string str1txt=str11+std::to_string(bspline_optimizer_->drone_id_);
      str11=str1txt+".txt";
      ofs.open(str11,std::ios::app);

      // ofs<<"1" <<endl;
        double t;
        double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();
        ofs<<"start_pt is "<<start_pt(0)<<" "<<start_pt(1)<<" "<<start_pt(2)<<endl;
        ofs<<"local_target_pt is "<<local_target_pt(0)<<" "<<local_target_pt(1)<<" "<<local_target_pt(2)<<endl;

        ofs<<"t_cur is "<<t_cur<<endl;
        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        // for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        // {
        //   segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
        //   if (t > t_cur)
        //   {
        //     pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
        //   }
        // }
        Eigen::Vector3d pt;
        pt(2)=start_pt(2);
        if(t_cur<local_data_.duration_)
        {
          for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          double the_t=pt(2)+ts;
          Eigen::Vector3d pt=local_data_.position_traj_.evaluateDeBoorT(t);
          pt(2)=the_t;
          ofs<<"pt is "<<pt(0)<<" "<<pt(1)<<" "<<pt(2)<<endl;

          segment_point.push_back(pt);
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;
        cout<<"segment_point size "<<segment_point.size()<<endl;
        cout<<"segment_point back "<<segment_point.back()(2)<<endl;

        double poly_time = local_target_pt[2]-segment_point.back()(2);//修改

        // double poly_time =local_data_.position_traj_.evaluateDeBoorT(t)[2]-local_target_pt[2];//修改
        ofs<<"poly_time is "<<poly_time<<endl;

        if (poly_time > ts)
        {
          Eigen::Vector3d ptt=local_data_.position_traj_.evaluateDeBoorT(t);
          ptt(2)=t;
          // local_target_pt[2]=(t+poly_time);//修改
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(ptt,
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);
          // Eigen::Vector3d pts;
          ptt(2)=segment_point.back()(2);
          ofs<<"ptt(2) is "<<ptt(2)<<endl;

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              double ptt_t=ptt(2)+ts;
              ptt=gl_traj.evaluate(t);
              ptt(2)=ptt_t;
              segment_point.push_back(ptt);
              ofs<<"t is "<<t<<endl;

              ofs<<"gl_traj.evaluat is "<<ptt(0)<<" "<<ptt(1)<<" "<<ptt(2)<<endl;

              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              ROS_ERROR("pseudo_arc_length is empty, return!");
              continous_failures_count_++;
              return false;
            }
          }
        }
        }
        else
        {
          if(abs(t_cur-local_data_.duration_)<1e-6)
          {
          Eigen::Vector3d ptt=local_data_.position_traj_.evaluateDeBoorT(local_data_.duration_);
          ptt(2)=start_pt(2);
          // local_target_pt[2]=(t+poly_time);//修改
          segment_point.push_back(ptt);
          double the_duan_time=local_target_pt[2]-ptt(2);
          cout<<"the_duan_time "<<the_duan_time<<endl;
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(ptt,
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(local_data_.duration_),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(local_data_.duration_),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), the_duan_time);
          for (t = ts; t < the_duan_time; t += ts)
          {
              double ptt_t=ptt(2)+ts;
              ptt=gl_traj.evaluate(t);
              ptt(2)=ptt_t;
              segment_point.push_back(ptt);
              ofs<<"t_ is "<<t<<endl;

              ofs<<"gl_traj.evaluat  "<<ptt(0)<<" "<<ptt(1)<<" "<<ptt(2)<<endl;

              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            

          }


          }
          else
          {
          for (t = 0; t < progress_time; t += ts)
          {
          Eigen::Vector3d pt = global_data_.global_traj_.evaluate(t+start_pt(2));

          segment_point.push_back(pt);
          }
          }

          cout<<"segment_point.size "<<segment_point.size()<<endl;
        }
        

      // double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;//修改
      // double poly_time = sqrt(pow(local_data_.position_traj_.evaluateDeBoorT(t)[0]-local_target_pt[0],2)+pow(local_data_.position_traj_.evaluateDeBoorT(t)[1]-local_target_pt[1],2))/ pp_.max_vel_ * 2;//修改

      // if(segment_point.size()<= bspline_optimizer_->getOrder())
        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())//pseudo_arc_length分段轨迹的累加和距离
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])//sample_length采样长度
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          // local_target_pt[2]=t+ts;//修改；
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help
        //修改2
        point_set.swap(segment_point);
        Eigen::Vector3d changetwo;
        changetwo=local_data_.velocity_traj_.evaluateDeBoorT(t_cur);//修改2
        changetwo[2]=0.3;//修改2
        start_end_derivatives.push_back(changetwo);
        start_end_derivatives.push_back(local_target_vel);
        changetwo=local_data_.acceleration_traj_.evaluateDeBoorT(t_cur);
        changetwo[2]=0;
        start_end_derivatives.push_back(changetwo);
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());
        // start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        // start_end_derivatives.push_back(local_target_vel);
        // start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        // start_end_derivatives.push_back(Eigen::Vector3d::Zero());
        //修改2
        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
        cout<<"no.2 part "<<endl;
      ofs.close();

      }
      
    } while (flag_regenerate);

    Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);//houyu:这里用得到controlpoints
    cout<<"ctrl_pts size"<<ctrl_pts.cols()<<endl;
    vector<std::pair<int, int>> segments;
    bspline_optimizer_->a_star_->initdymap=false;
    segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);//

    t_init = ros::Time::now() - t_start;
    t_start = ros::Time::now();
    double t_current= (ros::Time::now() - local_data_.start_time_).toSec();

    point_set.clear();
    for (int j = 0; j < ctrl_pts.cols(); j++)
    // for (int j = 0; j < ctrl_pts.cols(); j++)

    {
      point_set.push_back(ctrl_pts.col(j));
      // point_set.push_back(ctrl_pts.col(j));
      visualization_->displayInitPathList(point_set, 0.8, 0);//修改visual

    }
    // if(bspline_optimizer_->occupath.size()>0)
    // if(bspline_optimizer_->a_star_pathesa.size()>0)
    //   visualization_->displayMultiInitPathList(bspline_optimizer_->a_star_pathesa, 0.2); // This visuallization will take up several milliseconds.
        
      {visualization_->occuPathList(bspline_optimizer_->occupath, 0.9, 0);}

    visualization_->displaylocalstartPoint(start_pt, Eigen::Vector4d(1, 0, 0, 1), 2, 0);
    visualization_->displaylocaltargetPoint(local_target_pt, Eigen::Vector4d(1, 0, 0, 1), 2, 0);
    // if(start_pt(2)>1)
    visualization_->pubSensedPoints(start_pt);


    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = false;
    // vector<vector<Eigen::Vector3d>> vis_trajs;
    // double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

    if (pp_.use_distinctive_trajs)
    {
      // cout << "enter" << endl;
      std::vector<ControlPoints> trajs = bspline_optimizer_->distinctiveTrajs(segments);//houyu:考虑动态障碍物

      // cout << "\033[1;33m"
      //      << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;
      bspline_optimizer_->occupoint={0,0,0};
      double final_cost, min_cost = 999999.0;
      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        // if (bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, trajs[i], ts))//进入优化，还需要动态障碍物信息
        if (bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, ts,trajs[i],t_current))//进入优化，还需要动态障碍物信息
        // if (bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, ts,point_set,t_current))//进入优化，还需要动态障碍物信息



        {

          // cout << "traj " << trajs.size() - i << " success." << endl;

          flag_step_1_success = true;
          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            ctrl_pts = ctrl_pts_temp;
          }

          // visualization
          point_set.clear();
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          // for (int j = 0; j < ctrl_pts.cols(); j++)

          {
            point_set.push_back(ctrl_pts_temp.col(j));
            // point_set.push_back(ctrl_pts.col(j));

          }
          // vis_trajs.push_back(point_set);
        }
        // else
        // {
        //   cout << "traj " << trajs.size() - i << " failed." << endl;
        // }
      }

      t_opt = ros::Time::now() - t_start;

      // visualization_->displayMultiInitPathList(vis_trajs, 0.2); // This visuallization will take up several milliseconds.
      // visualization_->displayInitPathList(point_set, 0.8, 0);//修改visual
      // if(vis_trajs.size()>0)
      // {
      // }
      

      // visualization_->displayMultiInitPathList(vis_trajs, 1); // This visuallization will take up several milliseconds.修改visual
      cout<<"cover visualization 1"<<endl;
    }
    else
    {
      flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
      t_opt = ros::Time::now() - t_start;
      // static int vis_id = 0;
      // visualization_->displayInitPathList(point_set, 0.2, 0);//修改visual
      visualization_->displayInitPathList(point_set, 0.8, 0);//修改visual

    }
    cout << "plan_step1_success=\033[42m" << flag_step_1_success <<"\033[0m"<< endl;

    if (!flag_step_1_success)
    {
      visualization_->displayOptimalList(ctrl_pts, 0);
      cout<<"cover visualization 2"<<endl;

      continous_failures_count_++;
      return false;
    }

    t_start = ros::Time::now();

    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_,pp_.max_vel_yaw);//?？动力学约束
    if(bspline_optimizer_->predict_dyobs.size()>0)
    {
    visualization_->displaypredictpath(bspline_optimizer_->predict_dyobs,1.0,0);
    }
        bspline_optimizer_->predict_dyobs.clear();
    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    // Note: Only adjust time in single drone mode. But we still allow drone_0 to adjust its time profile.
    if (pp_.drone_id <= -1)
    {

      
      flag_step_2_success = true;
      if (!pos.checkFeasibility(ratio, false))
      {
        cout << "Need to reallocate time." << endl;

        Eigen::MatrixXd optimal_control_points;
        flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);//
        if (flag_step_2_success)
         {pos = UniformBspline(optimal_control_points, 3, ts);
          visualization_->displayOptimalList(optimal_control_points, 0);} 
      }

      if (!flag_step_2_success)
      {
        visualization_->displayoccuPoint(bspline_optimizer_->occupoint, Eigen::Vector4d(0, 0.5, 0.5, 1), 10, 0);
        printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
        continous_failures_count_++;
        return false;
      }
   
      
      cout<<"flag_step_2_success is \033[42m "<<flag_step_2_success<<"\033[0m"<<endl;
    }
    else
    {
      static bool print_once = true;
      if (print_once)
      {
        print_once = false;
        ROS_ERROR("IN SWARM MODE, REFINE DISABLED!");
      }
    }

    t_refine = ros::Time::now() - t_start;

    // save planned results
    updateTrajInfo(pos, ros::Time::now());
    // ofstream ofs;
    // publishSwarmTrajs(true);
    
      // ofstream ofs;
      // std::string str11 = "/home/rancho/1hmy/ego-planner-swarm/record/updateTrajInfo";
      // std::string str1txt=str11+std::to_string(pp_.drone_id);
      // str11=str1txt+".txt";
      // ofs.open(str11,std::ios::app);
      // for (int j = 0; j < ctrl_pts.cols(); j++)
      //     ofs<<ctrl_pts(0,j)<<"  "<<ctrl_pts(1,j)<<"  "<<ctrl_pts(2,j)<< endl;//修改
      // ofs<< endl;//修改
      // ofs.close();
    sum_time += (t_init + t_opt + t_refine).toSec();
    sum_opttime += t_opt.toSec();
    count_success++;
    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << ",avg_time=" << sum_time / count_success << endl;
    cout << "global time is " << sum_time << endl;
    cout << "globalopt time is " << sum_opttime << endl;
    cout << "flag_step_1_success is " << flag_step_1_success << endl;



    // ofs.open("/home/houmingyu/桌面/record/test.txt",std::ios::app);
    // ofs<<pp_.drone_id<<endl; 
    // ofs<<"global time is " << sum_time << endl;c_yaml
    // ofs<<"globalopt time is " << sum_opttime << endl;
    // ofs.close();
    // success. YoY
    continous_failures_count_ = 0;
    return true;
  
  }
void EGOPlannerManager::getLocalTarget()
{
    double t;
    //planning_horizen_局部规划范围
    // double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;//修改修改（0.1875）
    // double t_step =0.3;//
    the_t_step_=0.3;
    progress_time=3.0;
    // cout<<"t is "<<t<<endl;
    double dist_min = 9999, dist_min_t = 0.0;
    // cout<<"last_progress_time_ is "<<global_data_.last_progress_time_<<endl;
    // cout<<"global_duration_ is "<<global_data_.global_duration_<<endl;
    // ofstream ofs;
    // std::string str11 = "/home/rancho/1hmy/ego-planner-swarm/record/getLocalTarget";
    // std::string str1txt=str11+std::to_string(pp_.drone_id);
    // str11=str1txt+".txt";
    // ofs.open(str11,std::ios::app);
    // ofs<<"last_progress_time_ " <<global_data_.last_progress_time_<<endl;
    // ofs<<"global_duration_ is is " <<global_data_.global_duration_<<endl;

   if (start_pt_[2]+progress_time > global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      // global_data_.last_progress_time_ = global_data_.global_duration_;
      cout<<"t > global_data_.global_duration_"<<endl;
    }
    else
    {
      local_target_pt_=global_data_.getPosition(start_pt_[2]+progress_time);

    }
    // global_data_.last_progress_time_+=progress_time;

    // for (t = global_data_.last_progress_time_; t < global_data_.global_duration_; t += the_t_step_)
    // {
    //   Eigen::Vector3d pos_t = global_data_.getPosition(t);
    //   // double dist = (pos_t - start_pt_).norm();
    //   double dist =sqrt(pow(pos_t[0]-start_pt_[0],2)+pow(pos_t[1]-start_pt_[1],2));//修改
    //  ofs<<"t is "<<t<<endl;
    //  ofs<<"pos_t "<<pos_t(0)<<pos_t(1)<<pos_t(2)<<endl;
    //  ofs<<"dist is "<<dist<<endl;

    //  //修改，t肯定是增加的，把t的值赋值到z上
    //   if (t < global_data_.last_progress_time_ + 1e-5 && dist > pp_.planning_horizen_)
    //   {
    //     // Important cornor case!
    //     for (; t < global_data_.global_duration_; t += the_t_step_)
    //     {
    //       Eigen::Vector3d pos_t_temp = global_data_.getPosition(t);//修改
          
    //       // double dist_temp = (pos_t_temp - start_pt_).norm();//
    //       double dist_temp =sqrt(pow(pos_t_temp[0]-start_pt_[0],2)+pow(pos_t_temp[1]-start_pt_[1],2));//修改
    //       if (dist_temp < pp_.planning_horizen_) //修改
    //       {
    //         pos_t = pos_t_temp;
    //         // dist = (pos_t - start_pt_).norm();//修改
    //         dist = sqrt(pow(pos_t[0]-start_pt_[0],2)+pow(pos_t[1]-start_pt_[1],2));//修改
    //         cout << "Escape cornor case \"getLocalTarget\"" << endl;
    //         break;
    //       }
    //     }
    //   }

    //   if (dist < dist_min)
    //   {
    //     dist_min = dist;
    //     dist_min_t = t;
    //   }

    //   if (dist >= pp_.planning_horizen_)
    //   {
    //     if(pos_t[2]>start_pt_[2])
    //     {
    //       local_target_pt_ = pos_t;
    //       local_target_pt_[2]=t;
    //     }
    //     else
    //     {
    //       ofs<<"t dosen't go"<<endl;
    //     }
    //     ofs<< "local_target_pt_ t"<<t-global_data_.last_progress_time_<< endl;//修改
    //     //修改t-z
    //     if(t-global_data_.last_progress_time_>5)
    //     {
    //     ofs<< "prediction exceeds 5s"<<t-global_data_.last_progress_time_<< endl;//修改
    //     }
    //     // local_target_pt_[2]=(global_data_.last_progress_time_+3)*0.1;//修改2
    //     //修改2

    //     //修改
    //     global_data_.last_progress_time_ = dist_min_t;
    //     break;

    //   }
    // ofs<< endl;//修改

    // }
    // ofs.close();
 

    // if ((end_pt_ - local_target_pt_).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))//修改
    if (sqrt(pow(end_pt_[0]-local_target_pt_[0],2)+pow(end_pt_[1]-local_target_pt_[1],2))< (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))//修改
    {
      local_target_vel_ = Eigen::Vector3d::Zero();
      // have_new_target_ = true;

    }
    else
    {
      local_target_vel_ = global_data_.getVelocity(start_pt_[2]+progress_time);
      local_target_vel_[2]=0.3;//修改2
      // have_new_target_ = true;

    }
  }
bool EGOPlannerManager::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();
    // nav_msgs::Path path;//修改
    // path.header.frame_id = "world";//修改
    bool plan_and_refine_success =
        reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;
    bspline_optimizer_->occupath.clear();
    bspline_optimizer_->a_star_pathesa.clear();

    
    cout << "reboundReplan=" << plan_and_refine_success << endl;
    // cout << "exec_timer is "<<sum_exec_time<<endl;
    // ofstream ofs;
    // ofs.open("/home/houmingyu/桌面/record/test.txt",std::ios::app);
    // ofs<<"exec_timer is " <<sum_exec_time<< endl;
    // ofs<<"target is " <<local_target_pt_[0]<<"  "<<local_target_pt_[1]<<"  "<<local_target_pt_[2]<< endl;//修改
    // ofs.close();

    if (plan_and_refine_success)
    {

      auto info = &local_data_;

      traj_utils::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;
      //修改 
      // geometry_msgs::PoseStamped pose_stamped;
      // pose_stamped.header.frame_id = "world";
      // pose_stamped.pose.position.x = local_target_pt_[0];
      // pose_stamped.pose.position.y = local_target_pt_[1];
      // pose_stamped.pose.position.z = local_target_pt_[2];
      // path.poses.push_back(pose_stamped);

     //修改
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
      // cout << knots.transpose() << endl;
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      /* 1. publish traj to traj_server */
      bspline_pub_.publish(bspline);

      /* 2. publish traj to the next drone of swarm */

      /* 3. publish traj for visualization */
      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
      // ofstream ofs;
      // ofs.open("/home/rancho/1hmy/ego-planner-swarm/record/test.txt",std::ios::app);
      // ofs<<"targetx is " <<local_target_pt_[0]<<endl;
      // ofs<<"targety is " <<local_target_pt_[1]<<endl;
      // ofs<<"targetz is " <<local_target_pt_[2]<< endl;//修改
      // ofs<<"id is " <<info->car_id<< endl;//修改

      // ofs.close();
    }

    return plan_and_refine_success;
  }



  

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0),ros::Time::now());

    return true;
  }
  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    if (local_data_.start_time_.toSec() < 1e9) // It means my first planning has not started
      return false;
    double my_traj_start_time = local_data_.start_time_.toSec();
    double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.toSec();

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + local_data_.duration_ * 2 / 3, other_traj_start_time + swarm_trajs_buf_[drone_id].duration_);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((local_data_.position_traj_.evaluateDeBoorT(t - my_traj_start_time).head(2) - swarm_trajs_buf_[drone_id].position_traj_.evaluateDeBoorT(t - other_traj_start_time).head(2) ).norm() < bspline_optimizer_->getSwarmClearance())//修改
      {
        return true;
      }
    }

    return false;
  }
  void EGOPlannerManager::set_gloabltrajanddata(const PolynomialTraj &traj, const ros::Time &time)
  {

  global_data_.setGlobalTraj(traj, time);


  }
  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    // global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  
  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_->ref_pts_.clear();
    // std::ofstream ofs;
    // std::string str11 = "/home/rancho/1hmy/ego-planner-swarm/record/refineTrajAlgo";
    // str11=str11+".txt";
    // ofs.open(str11,std::ios::app);
    // ofs<<"t_step is "<<t_step<<endl;
    // ofs<<"traj.getTimeSum() is "<<traj.getTimeSum()<<endl;
    // ofs.close();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t));
    for (long unsigned int i =1;i<bspline_optimizer_->ref_pts_.size();i++)
      bspline_optimizer_->ref_pts_[i](2)=bspline_optimizer_->ref_pts_[i-1](2)+t_step;
    bool success = bspline_optimizer_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    // local_data_.end_pos_=local_data_.position_traj_.evaluateDeBoorT(duration_);
    local_data_.traj_id_ += 1;
    ofstream ofs;
    std::string str11 = "/home/rancho/1hmy/ego-planner-swarm/record/updateTrajInfo";
    std::string str1txt=str11+std::to_string(pp_.drone_id);
    str11=str1txt+".txt";
    ofs.open(str11,std::ios::app);
    // for (int j = 0; j < ctrl_pts.cols(); j++)
    //     ofs<<ctrl_pts(0,j)<<"  "<<ctrl_pts(1,j)<<"  "<<ctrl_pts(2,j)<< endl;//修改
    // ofs<< endl;//修改
    ofs<<"duration_ "<<local_data_.duration_<<endl;
    ofs.close();
  }

  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {//轨迹的始末速度和加速度start_end_derivative
    // std::ofstream ofs;
    // std::string str11 = "/home/rancho/1hmy/ego-planner-swarm/record/reparamBspline";
    // str11=str11+".txt";
    // ofs.open(str11,std::ios::app);
       
    double time_origin = bspline.getTimeSum();//得到轨迹时长
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);
    // ofs<<"time_origin is " <<time_origin<<endl;
    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    

    dt = duration / double(seg_num);
    // ofs<<"dt is " <<dt<<endl;
    
    time_inc = duration - time_origin;//增加时长

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      Eigen::Vector3d pos_the;
      pos_the=bspline.evaluateDeBoorT(time);
      pos_the(2)=time+start_pt_[2];
      // point_set.push_back(bspline.evaluateDeBoorT(time));
      point_set.push_back(pos_the);


    }
    //修改
    // for (size_t i = 0; i < point_set.size()-1; i++)
    // {
    //   ofs<<" point_set[i](2) 1 is " << point_set[i](2)<<endl;
    //   point_set[i](2)=point_set[i](2)*ratio;
    //   ofs<<" point_set[i](2) 2  is " << point_set[i](2)<<endl;

    // }
    // // point_set.back()(2)=duration;
    // ofs<<"duration is " <<duration<<endl;
    // ofs.close();
    // for (size_t i = 1; i < point_set.size(); i++)
    // {
    //   point_set[i](2)=point_set[i-1](2)+dt;
    // }
    //修改
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);//使用拟合的方法将一段轨迹点拟合成均匀B样条函数，得到控制点。
  }

} // namespace ego_planner
