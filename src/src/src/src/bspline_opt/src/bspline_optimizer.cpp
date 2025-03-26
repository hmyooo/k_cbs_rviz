#include "bspline_opt/bspline_optimizer.h"
#include "bspline_opt/gradient_descent_optimizer.h"
// using namespace std;
#include <cmath>
namespace dy_obs
{

  void BsplineOptimizer::setParam(ros::NodeHandle &nh)
  {
    nh.param("optimization/lambda_smooth", lambda1_, -1.0);
    nh.param("optimization/lambda_collision", lambda2_, -1.0);
    nh.param("optimization/lambda_feasibility", lambda3_, -1.0);
    nh.param("optimization/lambda_fitness", lambda4_, -1.0);

    nh.param("optimization/lambda_Distancestatic", lambda5_, -1.0);
    nh.param("optimization/lambda_gloabl", lambda6_, -1.0);
    nh.param("optimization/lambda_f_dyobs", lambda7_, -1.0);


    nh.param("optimization/dist0", dist0_, -1.0);
    nh.param("optimization/swarm_clearance", swarm_clearance_, -1.0);
    nh.param("optimization/max_vel", max_vel_, -1.0);
    nh.param("optimization/max_acc", max_acc_, -1.0);
    nh.param("optimization/mappng", stablemap, std::string("/home/1.png"));
    nh.param("optimization/maprelolution", mapresolution,1.0);


    nh.param("optimization/order", order_, 3);
  }
  void BsplineOptimizer::read_trac_yaml(std::string trac_yaml)
  {

     YAML::Node config = YAML::LoadFile(trac_yaml);
      if (config["schedule"]) {
      vi = config["schedule"].as<std::vector<tracpara>>();}
      timemax=vi.back().t;
      // for (std::vector<tracpara>::iterator it = vi.begin(); it != vi.end(); ++it) {
      // std::cout << "vector: x: " << it->x << " y: " << it->y << " t: " << it->t<< std::endl;
      //   }
  }
      
  // void BsplineOptimizer::setEnvironment(const GridMap::Ptr &map)
  // {
  //   this->grid_map_ = map;
  // }
    void BsplineOptimizer::setEnvironment()
  {
    this->a_star_.reset(new AStar);
    this->a_star_->initGridMap(stablemap);
  }

  void BsplineOptimizer::setEnvironment_(const fast_planner::ObjPredictor::Ptr mov_obj)
  {
    this->a_star_.reset(new AStar);
    this->a_star_->initGridMap(stablemap);
    this->moving_objs_ = mov_obj;
    // ros::Time t_now = ros::Time::now();
    // this->moving_objs_->predict_dyobs_trajs_.setGlobalStartTime(t_now);
  }

  void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.points = points;
  }

  void BsplineOptimizer::setBsplineInterval(const double &ts) { bspline_interval_ = ts; }

  void BsplineOptimizer::setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr) { swarm_trajs_ = swarm_trajs_ptr; }

  void BsplineOptimizer::setDroneId(const int drone_id) { drone_id_ = drone_id; }

  std::vector<ControlPoints> BsplineOptimizer::distinctiveTrajs(vector<std::pair<int, int>> segments)//传入initControlPoints中得到的调整后的障碍物控制点对
  {
    if (segments.size() == 0) // will be invoked again later.
    {
      std::vector<ControlPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    constexpr int MAX_TRAJS = 8;
    constexpr int VARIS = 2;
    int seg_upbound = std::min((int)segments.size(), static_cast<int>(floor(log(MAX_TRAJS) / log(VARIS))));
    std::vector<ControlPoints> control_pts_buf;
    control_pts_buf.reserve(MAX_TRAJS);
    const double RESOLUTION = mapresolution;
    const double CTRL_PT_DIST = (cps_.points.col(0) - cps_.points.col(cps_.size - 1)).norm() / (cps_.size - 1);

    // Step 1. Find the opposite vectors and base points for every segment.
    std::vector<std::pair<ControlPoints, ControlPoints>> RichInfoSegs;
    for (int i = 0; i < seg_upbound; i++)
    {
      std::pair<ControlPoints, ControlPoints> RichInfoOneSeg;
      ControlPoints RichInfoOneSeg_temp;
      cps_.segment(RichInfoOneSeg_temp, segments[i].first, segments[i].second);
      RichInfoOneSeg.first = RichInfoOneSeg_temp;
      RichInfoOneSeg.second = RichInfoOneSeg_temp;
      RichInfoSegs.push_back(RichInfoOneSeg);

      // cout << "RichInfoOneSeg_temp, out" << endl;
      // cout << "RichInfoSegs[" << i << "].first" << endl;
      // for ( int k=0; k<RichInfoOneSeg_temp.size; k++ )
      //   if ( RichInfoOneSeg_temp.base_point[k].size() > 0 )
      //   {
      //     cout << "###" << RichInfoOneSeg_temp.points.col(k).transpose() << endl;
      //     for (int k2 = 0; k2 < RichInfoOneSeg_temp.base_point[k].size(); k2++)
      //     {
      //       cout << "      " << RichInfoOneSeg_temp.base_point[k][k2].transpose() << " @ " << RichInfoOneSeg_temp.direction[k][k2].transpose() << endl;
      //     }
      //   }
    }

    for (int i = 0; i < seg_upbound; i++)
    {

      // 1.1 Find the start occupied point id and the last occupied point id
      //找到离障碍物最近的控制点对(occ_start_id,occ_end_id)及其对应的坐标(occ_start_pt,occ_end_pt)（也就是2.2.2中没有调整时的控制点对）
      if (RichInfoSegs[i].first.size > 1)
      {
        int occ_start_id = -1, occ_end_id = -1;
        Eigen::Vector3d occ_start_pt, occ_end_pt;
        for (int j = 0; j < RichInfoSegs[i].first.size - 1; j++)
        {
          //cout << "A *" << j << "*" << endl;
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j + 1)).norm() / 2;
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j + 1));
            cout << " " << a_star_->getInflateOccupancy(pt) << " pt=" << pt.transpose() << endl;
            if (a_star_->getInflateOccupancy(pt))
            {
              occ_start_id = j;
              occ_start_pt = pt;
              goto exit_multi_loop1;
            }
          }
        }
      exit_multi_loop1:;
        for (int j = RichInfoSegs[i].first.size - 1; j >= 1; j--)
        {
          //cout << "j=" << j << endl;
          //cout << "B *" << j << "*" << endl;
          ;
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j - 1)).norm();
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j - 1));
            cout << " " << a_star_->getInflateOccupancy(pt) << " pt=" << pt.transpose() << endl;
            ;
            if (a_star_->getInflateOccupancy(pt))
            {
              occ_end_id = j;
              occ_end_pt = pt;
              goto exit_multi_loop2;
            }
          }
        }
      exit_multi_loop2:;

        // double check
        if (occ_start_id == -1 || occ_end_id == -1)
        {
          // It means that the first or the last control points of one segment are in obstacles, which is not allowed.
          // ROS_WARN("What? occ_start_id=%d, occ_end_id=%d", occ_start_id, occ_end_id);

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;

          continue;

          // cout << "RichInfoSegs[" << i << "].first" << endl;
          // for (int k = 0; k < RichInfoSegs[i].first.size; k++)
          // {
          //   if (RichInfoSegs[i].first.base_point.size() > 0)
          //   {
          //     cout << "###" << RichInfoSegs[i].first.points.col(k).transpose() << endl;
          //     for (int k2 = 0; k2 < RichInfoSegs[i].first.base_point[k].size(); k2++)
          //     {
          //       cout << "      " << RichInfoSegs[i].first.base_point[k][k2].transpose() << " @ " << RichInfoSegs[i].first.direction[k][k2].transpose() << endl;
          //     }
          //   }
          // }
        }

        // 1.2 Reverse the vector and find new base points from occ_start_id to occ_end_id.
        for (int j = occ_start_id; j <= occ_end_id; j++)
        {
          Eigen::Vector3d base_pt_reverse, base_vec_reverse;
          if (RichInfoSegs[i].first.base_point[j].size() != 1)
          {
            cout << "RichInfoSegs[" << i << "].first.base_point[" << j << "].size()=" << RichInfoSegs[i].first.base_point[j].size() << endl;
            ROS_ERROR("Wrong number of base_points!!! Should not be happen!.");

            cout << setprecision(5);
            cout << "cps_" << endl;
            cout << " clearance=" << cps_.clearance << " cps.size=" << cps_.size << endl;
            for (int temp_i = 0; temp_i < cps_.size; temp_i++)
            {
              if (cps_.base_point[temp_i].size() > 1 && cps_.base_point[temp_i].size() < 1000)
              {
                ROS_ERROR("Should not happen!!!");
                cout << "######" << cps_.points.col(temp_i).transpose() << endl;
                for (size_t temp_j = 0; temp_j < cps_.base_point[temp_i].size(); temp_j++)
                  cout << "      " << cps_.base_point[temp_i][temp_j].transpose() << " @ " << cps_.direction[temp_i][temp_j].transpose() << endl;
              }
            }

            std::vector<ControlPoints> blank;
            return blank;
          }

          base_vec_reverse = -RichInfoSegs[i].first.direction[j][0];

          // The start and the end case must get taken special care of.
          if (j == occ_start_id)
          {
            base_pt_reverse = occ_start_pt;
          }
          else if (j == occ_end_id)
          {
            base_pt_reverse = occ_end_pt;
          }
          else
          {
            base_pt_reverse = RichInfoSegs[i].first.points.col(j) + base_vec_reverse * (RichInfoSegs[i].first.base_point[j][0] - RichInfoSegs[i].first.points.col(j)).norm();
          }

          if (a_star_->getInflateOccupancy(base_pt_reverse)) // Search outward.
          {
            double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
            double l = RESOLUTION;
            for (; l <= l_upbound; l += RESOLUTION)
            {
              Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
              cout << base_pt_temp.transpose() << endl;
              if (!a_star_->getInflateOccupancy(base_pt_temp))
              {
                RichInfoSegs[i].second.base_point[j][0] = base_pt_temp;
                RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
                break;
              }
            }
            if (l > l_upbound)
            {
              ROS_WARN("Can't find the new base points at the opposite within the threshold. i=%d, j=%d", i, j);

              segments.erase(segments.begin() + i);
              RichInfoSegs.erase(RichInfoSegs.begin() + i);
              seg_upbound--;
              i--;

              goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
            }
          }
          else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(j)).norm() >= RESOLUTION) // Unnecessary to search.
          {
            RichInfoSegs[i].second.base_point[j][0] = base_pt_reverse;
            RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
          }
          else
          {
            ROS_WARN("base_point and control point are too close!");
            cout << "base_point=" << RichInfoSegs[i].first.base_point[j][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(j).transpose() << endl;

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;

            goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
          }
        }

        // 1.3 Assign the base points to control points within [0, occ_start_id) and (occ_end_id, RichInfoSegs[i].first.size()-1].
        if (RichInfoSegs[i].second.size)
        {
          for (int j = occ_start_id - 1; j >= 0; j--)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_start_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_start_id][0];
          }
          for (int j = occ_end_id + 1; j < RichInfoSegs[i].second.size; j++)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_end_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_end_id][0];
          }
        }

      exit_multi_loop3:;
      }
      else if (RichInfoSegs[i].first.size == 1)
      {
        cout << "i=" << i << " RichInfoSegs.size()=" << RichInfoSegs.size() << endl;
        cout << "RichInfoSegs[i].first.size=" << RichInfoSegs[i].first.size << endl;
        cout << "RichInfoSegs[i].first.direction.size()=" << RichInfoSegs[i].first.direction.size() << endl;
        cout << "RichInfoSegs[i].first.direction[0].size()=" << RichInfoSegs[i].first.direction[0].size() << endl;
        cout << "RichInfoSegs[i].first.points.cols()=" << RichInfoSegs[i].first.points.cols() << endl;
        cout << "RichInfoSegs[i].first.base_point.size()=" << RichInfoSegs[i].first.base_point.size() << endl;
        cout << "RichInfoSegs[i].first.base_point[0].size()=" << RichInfoSegs[i].first.base_point[0].size() << endl;
        Eigen::Vector3d base_vec_reverse = -RichInfoSegs[i].first.direction[0][0];
        Eigen::Vector3d base_pt_reverse = RichInfoSegs[i].first.points.col(0) + base_vec_reverse * (RichInfoSegs[i].first.base_point[0][0] - RichInfoSegs[i].first.points.col(0)).norm();

        if (a_star_->getInflateOccupancy(base_pt_reverse)) // Search outward.
        {
          double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
          double l = RESOLUTION;
          for (; l <= l_upbound; l += RESOLUTION)
          {
            Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
            //cout << base_pt_temp.transpose() << endl;
            if (!a_star_->getInflateOccupancy(base_pt_temp))
            {
              RichInfoSegs[i].second.base_point[0][0] = base_pt_temp;
              RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
              break;
            }
          }
          if (l > l_upbound)
          {
            ROS_WARN("Can't find the new base points at the opposite within the threshold, 2. i=%d", i);

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;
          }
        }
        else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(0)).norm() >= RESOLUTION) // Unnecessary to search.
        {
          RichInfoSegs[i].second.base_point[0][0] = base_pt_reverse;
          RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
        }
        else
        {
          ROS_WARN("base_point and control point are too close!, 2");
          cout << "base_point=" << RichInfoSegs[i].first.base_point[0][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(0).transpose() << endl;

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;
        }
      }
      else
      {
        segments.erase(segments.begin() + i);
        RichInfoSegs.erase(RichInfoSegs.begin() + i);
        seg_upbound--;
        i--;
      }
    }
    // cout << "A3" << endl;

    // Step 2. Assemble each segment to make up the new control point sequence.
    if (seg_upbound == 0) // After the erase operation above, segment legth will decrease to 0 again.
    {
      std::vector<ControlPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    std::vector<int> selection(seg_upbound);
    std::fill(selection.begin(), selection.end(), 0);
    selection[0] = -1; // init
    int max_traj_nums = static_cast<int>(pow(VARIS, seg_upbound));
    for (int i = 0; i < max_traj_nums; i++)
    {
      // 2.1 Calculate the selection table.
      int digit_id = 0;
      selection[digit_id]++;
      while (digit_id < seg_upbound && selection[digit_id] >= VARIS)
      {
        selection[digit_id] = 0;
        digit_id++;
        if (digit_id >= seg_upbound)
        {
          ROS_ERROR("Should not happen!!! digit_id=%d, seg_upbound=%d", digit_id, seg_upbound);
        }
        selection[digit_id]++;
      }

      // 2.2 Assign params according to the selection table.
      ControlPoints cpsOneSample;
      cpsOneSample.resize(cps_.size);
      cpsOneSample.clearance = cps_.clearance;
      int cp_id = 0, seg_id = 0, cp_of_seg_id = 0;
      while (/*seg_id < RichInfoSegs.size() ||*/ cp_id < cps_.size)
      {
        //cout << "A ";
        // if ( seg_id >= RichInfoSegs.size() )
        // {
        //   cout << "seg_id=" << seg_id << " RichInfoSegs.size()=" << RichInfoSegs.size() << endl;
        // }
        // if ( cp_id >= cps_.base_point.size() )
        // {
        //   cout << "cp_id=" << cp_id << " cps_.base_point.size()=" << cps_.base_point.size() << endl;
        // }
        // if ( cp_of_seg_id >= RichInfoSegs[seg_id].first.base_point.size() )
        // {
        //   cout << "cp_of_seg_id=" << cp_of_seg_id << " RichInfoSegs[seg_id].first.base_point.size()=" << RichInfoSegs[seg_id].first.base_point.size() << endl;
        // }

        if (seg_id >= seg_upbound || cp_id < segments[seg_id].first || cp_id > segments[seg_id].second)
        {
          cpsOneSample.points.col(cp_id) = cps_.points.col(cp_id);
          cpsOneSample.base_point[cp_id] = cps_.base_point[cp_id];
          cpsOneSample.direction[cp_id] = cps_.direction[cp_id];
        }
        else if (cp_id >= segments[seg_id].first && cp_id <= segments[seg_id].second)
        {
          if (!selection[seg_id]) // zx-todo
          {
            cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].first.points.col(cp_of_seg_id);
            cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].first.base_point[cp_of_seg_id];
            cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].first.direction[cp_of_seg_id];
            cp_of_seg_id++;
          }
          else
          {
            if (RichInfoSegs[seg_id].second.size)
            {
              cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].second.points.col(cp_of_seg_id);
              cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].second.base_point[cp_of_seg_id];
              cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].second.direction[cp_of_seg_id];
              cp_of_seg_id++;
            }
            else
            {
              // Abandon this trajectory.
              goto abandon_this_trajectory;
            }
          }

          if (cp_id == segments[seg_id].second)
          {
            cp_of_seg_id = 0;
            seg_id++;
          }
        }
        else
        {
          ROS_ERROR("Shold not happen!!!!, cp_id=%d, seg_id=%d, segments.front().first=%d, segments.back().second=%d, segments[seg_id].first=%d, segments[seg_id].second=%d",
                    cp_id, seg_id, segments.front().first, segments.back().second, segments[seg_id].first, segments[seg_id].second);
        }

        cp_id++;
      }

      control_pts_buf.push_back(cpsOneSample);

    abandon_this_trajectory:;
    }

    return control_pts_buf;
  } // namespace ego_planner
  // bool BsplineOptimizer::checkdyoccu(const Eigen::Vector3d pos)
  // {
  //   for (std::vector<tracpara>::iterator it = vi.begin(); it != vi.end(); ++it) //该位置同时间层找障碍物
  //     {

  //       double disttime=it->t-pos(2);
   

  //       if(disttime>0.3)//即it是未来
  //       {
  //         break;
  //       }
  //       // if(disttime==0.3 ||(disttime<0.3&&disttime>-0.1))
  //       if(fabs(disttime)<0.3)

  //       {
  //         double distance_the=sqrt(pow(pos(0)-it->x,2)+pow(pos(1)-it->y,2));

  //          if(distance_the<7.5)//horizen范围内51cm
  //         {
  //         // ofs<<"i is " <<i<<endl;
  //         // the_i.push_back(i);
  //         dy_ob(0)=it->x;
  //         dy_ob(1)=it->y;
  //         dy_ob(2)=it->t;
        
  //         loacalvi.push_back(dy_ob);
  //         occ=true;


  //         }
  //       }

  //     }
  // }
  /* This function is very similar to check_collision_and_rebound(). 
   * It was written separately, just because I did it once and it has been running stably since March 2020.
   * But I will merge then someday.*/
  /*
    get angle ACB, point C is the center point
    A(x1,y1)
    B(x2,y2)
    C(x3,y3)
*/
double BsplineOptimizer::get_angle_with_points(double x1, double y1, double x2, double y2, double x3, double y3)
{
    double theta = atan2(x1 - x3, y1 - y3) - atan2(x2 - x3, y2 - y3);
    if (theta > M_PI)
        theta -= 2 * M_PI;
    if (theta < -M_PI)
        theta += 2 * M_PI;

    theta = abs(theta * 180.0 / M_PI);

    if(y2<=y3) {
        theta = 360.0 - theta;
    }

    return theta;
}


  std::vector<std::pair<int, int>> BsplineOptimizer::initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init /*= true*/)
  {

    // if (flag_first_init)
    // {
    //   cps_.clearance = dist0_;
    //   cps_.resize(init_points.cols());
    //   cps_.points = init_points;
    // }
    ofstream ofs;
    std::string str11 = "/home/rancho/1hmy/ego-planner-swarm/record/astar";
    std::string str1txt=str11+std::to_string(drone_id_);
    str11=str1txt+".txt";
    ofs.open(str11,std::ios::app);
//     // /*** Segment the initial trajectory according to obstacles to dynamic***/  
//     // constexpr int ENOUGH_INTERVAL = 2;
    
//     // double step_size = mapresolution/ ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1)) / 1.5;//步长step_size设为栅格大小除以1.5倍的控制点的平均距离
//     // double step_size = mapresolution;//步长step_size设为栅格大小除以1.5倍的控制点的平均距离
    
    // // double step_size = grid_map_->getResolution() / ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1)) / 1.5;//步长step_size设为栅格大小除以1.5倍的控制点的平均距离
    // int in_id = -1, out_id = -1;
    // vector<std::pair<int, int>> segment_ids;
    // int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ, last_occ = false;
    // bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    int i_end = init_points.cols() - order_ ; // only check closed 2/3 points
    int i_idx=init_points.cols() - 1;
    Eigen::Vector3d dy_ob,start_pt,local_goalpt;
    // Eigen::Vector3d dy_ob;

    double disttime,distance_the;
    vector<Eigen::Vector3d> a_star_pathes;
    std::vector<Eigen::Vector3d> loacalvi;
    std::vector<int> the_i;
    double dist_maxposx=0.0;
    double dist_maxposy=0.0;
    double maxposx=0.0;
    double maxposy=0.0;
    bool maxposx_bool=false;
    bool maxposy_bool=false;


    for (int i = order_; i <= i_end; ++i)
    {
    //   //cout << " *" << i-1 << "*" ;
     for (std::vector<tracpara>::iterator it = vi.begin(); it != vi.end(); ++it) //该位置同时间层找障碍物
      {

        disttime=it->t-init_points.col(i)(2);
   

        if(disttime>0.3)//即it是未来
        {
          break;
        }
        // if(disttime==0.3 ||(disttime<0.3&&disttime>-0.1))
        if(fabs(disttime)<0.3)

        {
          distance_the=sqrt(pow(init_points.col(order_)(0)-it->x,2)+pow(init_points.col(order_)(1)-it->y,2));

           if(distance_the<15 && abs(init_points.col(order_)(0)-it->x)<15 && abs(init_points.col(order_)(1)-it->y)<15 )//horizen范围内51cm
          {
          ofs<<"distance_the is " <<distance_the<<endl;
          the_i.push_back(i);
          dy_ob(0)=it->x;
          dy_ob(1)=it->y;
          dy_ob(2)=it->t;
          ofs<<"dy_ob(0) " <<dy_ob(0)<<endl;
          ofs<<"dy_ob(1) " <<dy_ob(1)<<endl;
          ofs<<"dy_ob(2) " <<dy_ob(2)<<endl;
        
          loacalvi.push_back(dy_ob);
          occ=true;
          if(!maxposx_bool)
            maxposx=dy_ob(0);
          if(!maxposy_bool)
            maxposy=dy_ob(1);
  
          if((abs(dy_ob(0)-maxposx)<15)&& abs(dy_ob(0)-maxposx)>abs(dist_maxposx))
              maxposx=dy_ob(0);
              maxposx_bool=true;
              dist_maxposx=dy_ob(0)-maxposx;

          if((abs(dy_ob(1)-maxposy)<15)&& abs(dy_ob(1)-maxposy)>abs(dist_maxposy))
              maxposy=dy_ob(1);
              maxposy_bool=true;
              dist_maxposy=dy_ob(1)-maxposy;
          // this->occupath.push_back(init_points.col(i));

          }
        }

      }
  
    }
    // if(the_i.size()==1)
    // {
    //   for(int i = -1; i <= 1; i++)
		// 	for(int j = -1; j<=1; j++){
		// 		if(i == 0&&j ==0) continue;
		// 		Vector3d nei = Vector3d(init_points.col(0)(0)+i, init_points.col(0)(1)+j, init_points.col(0)(2));
    //     if((nei.head(2)-loacalvi[0].head(2)).norm()>7.5)
    //     {
    //       cout<<"1 ok"<<endl;
    //     }
        
        
        
    //     }
    // }
    if(occ)
    {
    a_star_->initDyGridMap(loacalvi,drone_id_);
    // for (std::vector<Eigen::Vector3d>::iterator it = loacalvi.begin(); it != loacalvi.end(); ++it) //该位置同时间层找障碍物
    // {
    //   if(*it[0]>maxpos && *it[0]>*it[1])
    //         maxpos=*it[0];
    //   if(*it[1]>maxpos && *it[1]>*it[0])
    //         maxpos=*it[1];
    // }
    if(loacalvi.size()>0 && a_star_->initdymap)
    {
      start_pt=init_points.col(order_);
      // start_pt(1)=init_points.col(order_+1)(1);
      // start_pt(2)=init_points.col(order_+1)(2);
      local_goalpt=init_points.col(i_end);
      Eigen::Vector3d global_pos= the_global_data.global_traj_.evaluate(local_goalpt(2));
      for(int i=-1.0;i<=1;i++)
        for(int j=-1.0;j<=1;j++)
          {
            if(i==0&&j==0)
              continue;
            if(abs(get_angle_with_points(global_pos(0),global_pos(1),maxposx+i*dist_maxposx,maxposy+j*dist_maxposy,start_pt(0),start_pt(1)))<50)
            {
            local_goalpt(0)=maxposx+i*dist_maxposx;
            local_goalpt(1)=maxposy+j*dist_maxposy;

            }
          
          }
      // for(double i=-3.0;i<=3.0;i++)
        // for(double j=-3.0;j<=3.0;i++)
      // {
      // local_goalpt(0)=maxposx-dist_maxposx;
      // local_goalpt(1)=maxposy-dist_maxposy;
      // if(a_star_->AstarSearch(start_pt,local_goalpt,a_star_pathes,drone_id_))
          // a_star_pathesa.push_back(a_star_pathes);

// }
      // local_goalpt(1)=init_points.col(i_end-1)(1);
      // local_goalpt(2)=init_points.col(i_end-1)(2);

      a_star_->AstarSearch(start_pt,local_goalpt,a_star_pathes,drone_id_);
    }
    if(!a_star_pathes.empty())
   {
//     for (int i = order_; i <= i_end; ++i)
// {
//   init_points.col(i)(0)=
// }
    ofs<<"init_points path is: " <<init_points.cols()<<endl;

     for (int i = 0; i < init_points.cols(); i++)
{
    ofs<<init_points.col(i)(0)<<" "<<init_points.col(i)(1)<<" "<<init_points.col(i)(2)<<endl;
}   

    ofs<<"a_star_pathes is: " <<a_star_pathes.size()<<endl;

    for (int i = 0; i < a_star_pathes.size(); i++)
{
    ofs<<a_star_pathes[i](0)<<" "<<a_star_pathes[i](1)<<" "<<a_star_pathes[i](2)<<endl;
}
      this->occupath.swap(a_star_pathes);

      for(int i=1;i<a_star_pathes.size();i++)
      {
       

        init_points.col(order_+i)(0)=a_star_pathes[i](0);
        init_points.col(order_+i)(1)=a_star_pathes[i](1);

      }


   }


    }
   
    // if (a_star_pathes.size()>0)
    // {
      // this->occupath.swap(a_star_pathes);

      // for(int i=1;i<a_star_pathes.size();i++)
      // {
      //   // double dist_betw_=(init_points.col(order_+i-1)-a_star_pathes[i-1]).norm();
      //   // double x1,y1;
      //   // // ofs<<"dist_betw_ is: " <<dist_betw_<<endl;
      //   // if (init_points.col(order_+i-1)(0)-a_star_pathes[i-1](0)==0)
      //   // {
      //   //  x1=a_star_pathes[i-1](0)+dist_betw_;
      //   //  y1=a_star_pathes[i-1](1);

      //   // }
      //   // else
      //   // {
      //   //  x1=a_star_pathes[i-1](0)+dist_betw_*cos(atan((init_points.col(order_+i-1)(1)-a_star_pathes[i-1](1))/(init_points.col(order_+i-1)(0)-a_star_pathes[i-1](0))));
      //   //  y1=a_star_pathes[i-1](1)+dist_betw_*sin(atan((init_points.col(order_+i-1)(1)-a_star_pathes[i-1](1))/(init_points.col(order_+i-1)(0)-a_star_pathes[i-1](0))));
      //   // }

      //   init_points.col(order_+i)(0)=a_star_pathes[i](0);
      //   init_points.col(order_+i)(1)=a_star_pathes[i](1);

      //   // init_points.col(order_+i)(0)=x1;
      //   // init_points.col(order_+i)(1)=y1;

      // }
      // cps_.points = init_points;
    // }
     ofs.close();

      if (flag_first_init)
    {
      cps_.clearance = dist0_;
      cps_.resize(init_points.cols());
      cps_.points = init_points;
    }
    if(loacalvi.size()>0)
    {loacalvi.clear(); a_star_pathes.clear();}
    vector<std::pair<int, int>> blank_ret;
    return blank_ret;
  //   constexpr int ENOUGH_INTERVAL = 2;
  //   double step_size = mapresolution / ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1)) / 1.5;
  //   int in_id = -1, out_id = -1;
  //   vector<std::pair<int, int>> segment_ids;
  //   int same_occ_state_times = ENOUGH_INTERVAL + 1;
  //   bool occ,occ1,last_occ = false;
  //   bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
  //   int i_end = (int)init_points.cols() - order_ - ((int)init_points.cols() - 2 * order_) / 3; // only check closed 2/3 points.
  //   for (int i = order_; i <= i_end; ++i)
  //   {
  //     //cout << " *" << i-1 << "*" ;
  //     for (double a = 1.0; a > 0.0; a -= step_size)
  //     {
  //       Eigen::Vector3d the_one=a * init_points.col(i - 1) + (1 - a) * init_points.col(i);
       
  //       occ1 = a_star_->getInflateOccupancy(a * init_points.col(i - 1) + (1 - a) * init_points.col(i));
  //       //cout << " " << occ;
  //       // cout << setprecision(5);
  //       // cout << (a * init_points.col(i-1) + (1-a) * init_points.col(i)).transpose() << " occ1=" << occ << endl;
  //       for (std::vector<tracpara>::iterator it = vi.begin(); it != vi.end(); ++it) //该位置同时间层找障碍物
  //     {

  //       double disttime=it->t-the_one(2);
   

  //       if(disttime>0.3)//即it是未来
  //       {
  //         break;
  //       }
  //       // if(disttime==0.3 ||(disttime<0.3&&disttime>-0.1))
  //       if(fabs(disttime)<0.3)

  //       {
  //         double distance_the=sqrt(pow(the_one(0)-it->x,2)+pow(the_one(1)-it->y,2));

  //          if(distance_the<15)//horizen范围内51cm
  //         {
  //         // ofs<<"i is " <<i<<endl;
  //         // the_i.push_back(i);
  //         dy_ob(0)=it->x;
  //         dy_ob(1)=it->y;
  //         dy_ob(2)=it->t;
        
  //         loacalvi.push_back(dy_ob);
  //         occ=true;


  //         }
  //       }

  //     }
  //       if ((occ ||occ1) && !last_occ)
  //       {
  //         if (same_occ_state_times > ENOUGH_INTERVAL || i == order_)
  //         {
  //           in_id = i - 1;
  //           flag_got_start = true;
  //         }
  //         same_occ_state_times = 0;
  //         flag_got_end_maybe = false; // terminate in advance
  //       }
  //       else if (!(occ ||occ1) && last_occ)
  //       {
  //         out_id = i;
  //         flag_got_end_maybe = true;
  //         same_occ_state_times = 0;
  //       }
  //       else
  //       {
  //         ++same_occ_state_times;
  //       }

  //       if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == (int)init_points.cols() - order_)))
  //       {
  //         flag_got_end_maybe = false;
  //         flag_got_end = true;
  //       }

  //       last_occ = (occ ||occ1);

  //       if (flag_got_start && flag_got_end)
  //       {
  //         flag_got_start = false;
  //         flag_got_end = false;
  //         segment_ids.push_back(std::pair<int, int>(in_id, out_id));
  //       }
  //     }
  //   }
  //   // cout << endl;

  //   // for (size_t i = 0; i < segment_ids.size(); i++)
  //   // {
  //   //   cout << "segment_ids=" << segment_ids[i].first << " ~ " << segment_ids[i].second << endl;
  //   // }

  //   // return in advance
  //   if (segment_ids.size() == 0)
  //   {
  //     vector<std::pair<int, int>> blank_ret;
  //     return blank_ret;
  //   }
     
  //   if (loacalvi.size()>0)
  //   {
  //     a_star_->initDyGridMap(loacalvi);

  //   }
    
  //   /*** a star search ***/
  //   vector<vector<Eigen::Vector3d>> a_star_pathes;
  //   vector<Eigen::Vector3d> a_star_pathe;
  //   for (size_t i = 0; i < segment_ids.size(); ++i)
  //   {
  //     Eigen::Vector3d in(init_points.col(segment_ids[i].first)), out(init_points.col(segment_ids[i].second));
  //     cout << "in=" << in.transpose() << " out=" << out.transpose() << endl;

  //     if (a_star_->AstarSearch(in,out,a_star_pathe,drone_id_))
  //     {
  //       a_star_pathes.push_back(a_star_pathe);
  //     }
  //     else
  //     {
  //       ROS_ERROR("a star error, force return!");
  //       vector<std::pair<int, int>> blank_ret;
  //       return blank_ret;
  //     }
  //   }

  //   /*** calculate bounds ***/
  //   int id_low_bound, id_up_bound;
  //   vector<std::pair<int, int>> bounds(segment_ids.size());
  //   for (size_t i = 0; i < segment_ids.size(); i++)
  //   {

  //     if (i == 0) // first segment
  //     {
  //       id_low_bound = order_;
  //       if (segment_ids.size() > 1)
  //       {
  //         id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
  //       }
  //       else
  //       {
  //         id_up_bound = init_points.cols() - order_ - 1;
  //       }
  //     }
  //     else if (i == segment_ids.size() - 1) // last segment, i != 0 here
  //     {
  //       id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
  //       id_up_bound = init_points.cols() - order_ - 1;
  //     }
  //     else
  //     {
  //       id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
  //       id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  // id_up_bound : -1.0f fix()
  //     }

  //     bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
  //   }

  //   // cout << "+++++++++" << endl;
  //   // for ( int j=0; j<bounds.size(); ++j )
  //   // {
  //   //   cout << bounds[j].first << "  " << bounds[j].second << endl;
  //   // }

  //   /*** Adjust segment length ***/
  //   vector<std::pair<int, int>> adjusted_segment_ids(segment_ids.size());
  //   constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient force
  //   int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
  //   for (size_t i = 0; i < segment_ids.size(); i++)
  //   {
  //     /*** Adjust segment length ***/
  //     num_points = segment_ids[i].second - segment_ids[i].first + 1;
  //     //cout << "i = " << i << " first = " << segment_ids[i].first << " second = " << segment_ids[i].second << endl;
  //     if (num_points < minimum_points)
  //     {
  //       double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

  //       adjusted_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first ? segment_ids[i].first - add_points_each_side : bounds[i].first;

  //       adjusted_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second ? segment_ids[i].second + add_points_each_side : bounds[i].second;
  //     }
  //     else
  //     {
  //       adjusted_segment_ids[i].first = segment_ids[i].first;
  //       adjusted_segment_ids[i].second = segment_ids[i].second;
  //     }

  //     //cout << "final:" << "i = " << i << " first = " << adjusted_segment_ids[i].first << " second = " << adjusted_segment_ids[i].second << endl;
  //   }
  //   for (size_t i = 1; i < adjusted_segment_ids.size(); i++) // Avoid overlap
  //   {
  //     if (adjusted_segment_ids[i - 1].second >= adjusted_segment_ids[i].first)
  //     {
  //       double middle = (double)(adjusted_segment_ids[i - 1].second + adjusted_segment_ids[i].first) / 2.0;
  //       adjusted_segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
  //       adjusted_segment_ids[i].first = static_cast<int>(middle + 1.1);
  //     }
  //   }

  //   // Used for return
  //   vector<std::pair<int, int>> final_segment_ids;

  //   /*** Assign data to each segment ***/
  //   for (size_t i = 0; i < segment_ids.size(); i++)
  //   {
  //     // step 1
  //     for (int j = adjusted_segment_ids[i].first; j <= adjusted_segment_ids[i].second; ++j)
  //       cps_.flag_temp[j] = false;

  //     // step 2
  //     int got_intersection_id = -1;
  //     for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
  //     {
  //       Eigen::Vector3d ctrl_pts_law(init_points.col(j + 1) - init_points.col(j - 1)), intersection_point;
  //       int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
  //       double val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law), last_val = val;
  //       while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
  //       {
  //         last_Astar_id = Astar_id;

  //         if (val >= 0)
  //           --Astar_id;
  //         else
  //           ++Astar_id;

  //         val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law);

  //         if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
  //         {
  //           intersection_point =
  //               a_star_pathes[i][Astar_id] +
  //               ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
  //                (ctrl_pts_law.dot(init_points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
  //               );

  //           //cout << "i=" << i << " j=" << j << " Astar_id=" << Astar_id << " last_Astar_id=" << last_Astar_id << " intersection_point = " << intersection_point.transpose() << endl;

  //           got_intersection_id = j;
  //           break;
  //         }
  //       }

  //       if (got_intersection_id >= 0)
  //       {
  //         double length = (intersection_point - init_points.col(j)).norm();
  //         if (length > 1e-5)
  //         {
  //           cps_.flag_temp[j] = true;
  //           for (double a = length; a >= 0.0; a -= mapresolution)
  //           {
  //             occ = a_star_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * init_points.col(j));

  //             if (occ || a < mapresolution)
  //             {
  //               if (occ)
  //                 a += mapresolution;
  //               cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * init_points.col(j));
  //               cps_.direction[j].push_back((intersection_point - init_points.col(j)).normalized());
  //               // cout << "A " << j << endl;
  //               break;
  //             }
  //           }
  //         }
  //         else
  //         {
  //           got_intersection_id = -1;
  //         }
  //       }
  //     }

  //     /* Corner case: the segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
  //     if (segment_ids[i].second - segment_ids[i].first == 1)
  //     {
  //       Eigen::Vector3d ctrl_pts_law(init_points.col(segment_ids[i].second) - init_points.col(segment_ids[i].first)), intersection_point;
  //       Eigen::Vector3d middle_point = (init_points.col(segment_ids[i].second) + init_points.col(segment_ids[i].first)) / 2;
  //       int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
  //       double val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law), last_val = val;
  //       while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
  //       {
  //         last_Astar_id = Astar_id;

  //         if (val >= 0)
  //           --Astar_id;
  //         else
  //           ++Astar_id;

  //         val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);

  //         if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
  //         {
  //           intersection_point =
  //               a_star_pathes[i][Astar_id] +
  //               ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
  //                (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
  //               );

  //           if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
  //           {
  //             cps_.flag_temp[segment_ids[i].first] = true;
  //             cps_.base_point[segment_ids[i].first].push_back(init_points.col(segment_ids[i].first));
  //             cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

  //             got_intersection_id = segment_ids[i].first;
  //           }
  //           break;
  //         }
  //       }
  //     }

  //     //step 3
  //     if (got_intersection_id >= 0)
  //     {
  //       for (int j = got_intersection_id + 1; j <= adjusted_segment_ids[i].second; ++j)
  //         if (!cps_.flag_temp[j])
  //         {
  //           cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
  //           cps_.direction[j].push_back(cps_.direction[j - 1].back());
  //           // cout << "AAA " << j << endl;
  //         }

  //       for (int j = got_intersection_id - 1; j >= adjusted_segment_ids[i].first; --j)
  //         if (!cps_.flag_temp[j])
  //         {
  //           cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
  //           cps_.direction[j].push_back(cps_.direction[j + 1].back());
  //           // cout << "AAAA " << j << endl;
  //         }

  //       final_segment_ids.push_back(adjusted_segment_ids[i]);
  //     }
  //     else
  //     {
  //       // Just ignore, it does not matter ^_^.
  //       // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
  //     }
  //   }

  //   return final_segment_ids;
  }

  int BsplineOptimizer::earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
    // cout << "k=" << k << endl;
    // cout << "opt->flag_continue_to_optimize_=" << opt->flag_continue_to_optimize_ << endl;
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  double BsplineOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->combineCostRebound(x, grad, cost, n);

    opt->iter_num_ += 1;
    return cost;
  }

  double BsplineOptimizer::costFunctionRefine(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->combineCostRefine(x, grad, cost, n);

    opt->iter_num_ += 1;
    return cost;
  }

  void BsplineOptimizer::calcSwarmCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {
    cost = 0.0;
    int end_idx = q.cols() - order_ - (double)(q.cols() - 2 * order_) * 1.0 / 3.0; // Only check the first 2/3 points
    const double CLEARANCE = swarm_clearance_ * 2;
    double t_now = ros::Time::now().toSec();
    constexpr double a = 2.0, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;
    // ofstream ofs;
    // std::string str11 = "/home/rancho/1hmy/ego-planner-swarm/record/opticost_swarm";
    // std::string str1txt=str11+std::to_string(drone_id_);
    // str11=str1txt+".txt";
    // ofs.open(str11,std::ios::app);
    // ofs<<"swarm_trajs_size is " <<swarm_trajs_->size()<<endl;
    for (int i = order_; i < end_idx; i++)
    {
      double glb_time = t_now + ((double)(order_ - 1) / 2 + (i - order_ + 1)) * bspline_interval_;

      for (size_t id = 0; id < swarm_trajs_->size(); id++)
      {
        if ((swarm_trajs_->at(id).drone_id != (int)id) || swarm_trajs_->at(id).drone_id == drone_id_)
        {
          continue;
        }

        double traj_i_satrt_time = swarm_trajs_->at(id).start_time_.toSec();
        if (glb_time < traj_i_satrt_time + swarm_trajs_->at(id).duration_ - 0.1)
        {
          /* def cost=(c-sqrt([Q-O]'D[Q-O]))^2, D=[1/b^2,0,0;0,1/b^2,0;0,0,1/a^2] */
          Eigen::Vector3d swarm_prid = swarm_trajs_->at(id).position_traj_.evaluateDeBoorT(glb_time - traj_i_satrt_time);
          // Eigen::Vector3d dist_vec = cps_.points.col(i) - swarm_prid;//修改
          Eigen::Vector2d dist_vec = cps_.points.col(i).head(2) - swarm_prid.head(2) ;//修改

          // double ellip_dist = sqrt(dist_vec(2) * dist_vec(2) * inv_a2 + (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2);//修改
          double ellip_dist = sqrt((dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2);//修改
          

          double dist_err = CLEARANCE - ellip_dist;

          Eigen::Vector3d dist_grad = cps_.points.col(i) - swarm_prid;
          Eigen::Vector3d Coeff;
          Coeff(0) = -2 * (CLEARANCE / ellip_dist - 1) * inv_b2;
          Coeff(1) = Coeff(0);
          Coeff(2) = -2 * (CLEARANCE / ellip_dist - 1) * inv_a2;
          

          // ofs<<"ellip_dist is " <<ellip_dist<<endl;
          // ofs<<"dist_err is " <<dist_err<<endl;
          // ofs<<"CLEARANCE is " <<CLEARANCE<<endl;
          // ofs<<"dist_grad is " <<dist_grad<<endl;


          // ofs<<"end "<< endl;//修改
          // ofs<< endl;//修改
          // ofs.close();
          if (dist_err < 0)
          {
            /* do nothing */
          }
          else
          {
            cost += pow(dist_err, 2);
            gradient.col(i) += (Coeff.array() * dist_grad.array()).matrix();
          }

          if (min_ellip_dist_ > dist_err)
          {
            min_ellip_dist_ = dist_err;
          }
        }
      }
    }

  }

  void BsplineOptimizer::calcMovingObjCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {
    cost = 0.0;
    int end_idx = q.cols() - order_;
    constexpr double CLEARANCE = 1.5;
    double t_now = ros::Time::now().toSec();

    for (int i = order_; i < end_idx; i++)
    {
      double time = ((double)(order_ - 1) / 2 + (i - order_ + 1)) * bspline_interval_;

      for (int id = 0; id < moving_objs_->getObjNums(); id++)
      {
        Eigen::Vector3d obj_prid = moving_objs_->evaluateConstVel(id, t_now + time);
        // double dist = (cps_.points.col(i) - obj_prid).norm();//修改
        double dist = (cps_.points.col(i).head(2) - obj_prid.head(2)).norm();//修改

        //cout /*<< "cps_.points.col(i)=" << cps_.points.col(i).transpose()*/ << " moving_objs_=" << obj_prid.transpose() << " dist=" << dist << endl;
        double dist_err = CLEARANCE - dist;
        Eigen::Vector3d dist_grad = (cps_.points.col(i) - obj_prid).normalized();

        if (dist_err < 0)
        {
          /* do nothing */
        }
        else
        {
          cost += pow(dist_err, 2);
          gradient.col(i) += -2.0 * dist_err * dist_grad;
        }
      }
      // cout << "time=" << time << " i=" << i << " order_=" << order_ << " end_idx=" << end_idx << endl;
      // cout << "--" << endl;
    }
    // cout << "---------------" << endl;
  }
  void BsplineOptimizer::calcDistancestatic_CostRebound(const Eigen::MatrixXd &q, double &cost,
                                                 Eigen::MatrixXd &gradient)
 {

    
    cost = 0.0;
    int end_idx = q.cols() - order_;
    // double demarcation = cps_.clearance;//控制点的安全距离sf
    double demarcation = 2;//控制点的安全距离sf

    // double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);//a=3倍的安全距离//a=3倍的安全距离b=-3*sf的平方c=sf的平方
    Eigen::Vector3d static_obs;
    force_stop_type_ = DONT_STOP;
    double dist;
    // ofstream ofs;
    // std::string str11 = "/home/rancho/1hmy/ego-planner-swarm/record/opticost_sstatic";
    // std::string str1txt=str11+std::to_string(drone_id_);
    // str11=str1txt+".txt";
    // ofs.open(str11,std::ios::app);
    //  ofs<<"clearance is " <<demarcation<<endl;
    /*** calculate distance cost and gradient ***///计算距离成本和梯度
    for (auto i = order_; i < end_idx; ++i)//遍历控制点
{
  static_obs=cps_.points.col(i);
   for (int deltx = 0; deltx < 8; deltx++)//遍历所有基点
  {
      for (int delty = 0; delty < 8; delty++)//遍历所有基点
    {
        static_obs(0)+=deltx;
        static_obs(1)+=delty;
        if(static_obs(0)<a_star_->map_voxel_num_(0)&&static_obs(1)<a_star_->map_voxel_num_(1))
        {
        if(a_star_->getInflateOccupancy(static_obs))
        {
          dist = 0;
          cost +=9999;
        }
        else
        {
        dist = (cps_.points.col(i) - static_obs).norm();
        }
        double dist_err =dist-cps_.clearance  ;//安全距离-距离，对应论文上的cij,即cij=sf-dij
        Eigen::Vector3d dist_grad = (cps_.points.col(i) - static_obs).normalized();//距离梯度为第i个控制点与对应的第j个基点的方向
        
        // ofs<<"dist_err is " <<dist_err<<endl;
        // ofs<<"dist_grad is " <<dist_grad<<endl;

        
        if (dist_err > 0)
        {
          /* do nothing */
        }
        else//如果cij小于安全距离（0<cij<sf）
        {
          cost = cost-dist_err;//代价为cij的三次方
          // cost +=a * dist_err * dist_err + b * dist_err + c;
          gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;//该控制点的梯度为=控制点的梯度+(-3*cij*cij*梯度方向)，对应论文计算梯度的第二个公式
        }  
        }

       
      }
    }
  }
  // ofs<<"end "<< endl;//修改
  // ofs<< endl;//修改
  // ofs.close();
 }
  void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                 Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost)//与静态障碍物
  {
    cost = 0.0;
    int end_idx = q.cols() - order_;
    double demarcation = cps_.clearance;//控制点的安全距离sf
    double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);//a=3倍的安全距离//a=3倍的安全距离b=-3*sf的平方c=sf的平方

    force_stop_type_ = DONT_STOP;
    if (iter_num > 3 && smoothness_cost / (cps_.size - 2 * order_) < 0.1) // 0.1 is an experimental value that indicates the trajectory is smooth enough.
    {
      check_collision_and_rebound();//检查碰撞和对应轨迹弹出
    }

    /*** calculate distance cost and gradient ***///计算距离成本和梯度
    for (auto i = order_; i < end_idx; ++i)//遍历控制点
    {
      for (size_t j = 0; j < cps_.direction[i].size(); ++j)//遍历所有基点
      {
        double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);//修改距离=(第i个控制点-第i个控制点对应的第j个基点（即控制点到第j个障碍物的距离）)*该方向的方向向量
        // double dist = (cps_.points.col(i).head(2) - cps_.base_point[i][j].head(2)).dot(cps_.direction[i][j].head(2));//修改
        double dist_err = cps_.clearance - dist;//安全距离-距离，对应论文上的cij,即cij=sf-dij
        Eigen::Vector3d dist_grad = cps_.direction[i][j];//距离梯度为第i个控制点与对应的第j个基点的方向

        if (dist_err < 0)
        {
          /* do nothing */
        }
        else if (dist_err < demarcation)//如果cij小于安全距离（0<cij<sf）
        {
          cost += pow(dist_err, 3);//代价为cij的三次方
          gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;//该控制点的梯度为=控制点的梯度+(-3*cij*cij*梯度方向)，对应论文计算梯度的第二个公式
        }
        else
        {
          cost += a * dist_err * dist_err + b * dist_err + c;//代价=代价+a*cij*cij+b*cij+c，其中，a=3倍的sf，b=-3*sf的平方,c=sf的平方
          gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;//该控制点的梯度为=控制点的梯度-(2.0 * a * cij + b) * 梯度方向
        }
      }
    }
  }
  void BsplineOptimizer::calcbetwen_gloabl(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {
    cost = 0.0;
    int end_idx = q.cols() - order_;
    constexpr double CLEARANCE = 0.5;
    // double t_start = the_global_data.last_progress_time_;
    // ofstream ofs;
    // std::string str11 = "/home/rancho/1hmy/ego-planner-swarm/record/opticost_golobal";
    // std::string str1txt=str11+std::to_string(drone_id_);
    // str11=str1txt+".txt";
    for (int i = order_; i < end_idx; i++)
    {
      double time = ((double)(order_ - 1) / 2 + (i - order_ + 1)) * bspline_interval_;
      cout<<"cps_.points.col(i)(2) is "<<cps_.points.col(i)(2)<<endl;
      double dist = (cps_.points.col(i) - the_global_data.global_traj_.evaluate(cps_.points.col(i)(2))).norm();//全局的t 
      
      double dist_err =dist-CLEARANCE  ;
      Eigen::Vector3d dist_grad = (cps_.points.col(i) - the_global_data.global_traj_.evaluate(cps_.points.col(i)(2))).normalized();
      // ofs.open(str11,std::ios::app);
      // ofs<<"dist is " <<dist<<endl;
      // ofs<<"dist_grad is " <<dist_grad<<endl;
      // ofs<<"time is " <<time<<endl;
      // // ofs<<"t_start is " <<t_start<<endl;
      // ofs<<"end "<< endl;//修改
      // ofs<< endl;//修改
      // ofs.close();
      if (dist_err < 0)
      {
        /* do nothing */
      }
      else
      {
        cost += pow(dist_err, 2);
        gradient.col(i) += -2.0 * dist_err * dist_grad;
      }
    
        }
     

    }
      
     
  //计算每步时间的全局位置与b样条在此位置的距离


  
  void BsplineOptimizer::calcbetwen_f_dyobs(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {
//计算预测的每步时间的动态障碍物位置与b样条在此位置的距离，需要注意不同层有不同的权重值
    cost = 0.0;
    int end_idx = q.cols() - order_;
    constexpr double CLEARANCE = 80;//
    double t_now = ros::Time::now().toSec();
    int counnt=1;
    // bool flag_first_his;
    Eigen::Vector3d hisobs_pos,dy_ob;
    double disttime,distance_the;
    list<Eigen::Vector3d> history_traj;
    vector<Eigen::Vector3d> dy_obs;
    // ofstream ofs;
    // std::string str11 = "/home/rancho/1hmy/ego-planner-swarm/record/opticost_dyobs";
    // std::string str1txt=str11+std::to_string(drone_id_);
    // str11=str1txt+".txt";
    // ofs.open(str11,std::ios::app);
    // ofs<<"end_idx is " <<end_idx<<endl;
    // ofs<<"order_ is " <<order_<<endl;
    // ofs<<"vi size is " <<vi.size()<<endl;
    // ofs<<endl;


     
        for (int i = order_; i < end_idx; i++)//与未来控制点进行cost计算
      {
        counnt =counnt+0.1;
      // double time = ((double)(order_ - 1) / 2 + (i - order_ + 1)) * bspline_interval_;
        // ofs<<"cps_.points.col(i)(2) is " <<cps_.points.col(i)(2)<<endl;

        // Eigen::Vector3d dyobj_prid = moving_objs_->evaluateConstVel_dyobs(history_traj,cps_.points.col(i)(2));
        // Eigen::Vector3d dyobj_prid = moving_objs_->evaluateConstVel_dyobs(history_traj, time);



      for (std::vector<tracpara>::iterator it = vi.begin(); it != vi.end(); ++it) //该位置同时间层找障碍物
      {
        // advance(it, countV);
        // if(it->t==0)
        // {continue;;}
        disttime=it->t-cps_.points.col(i)(2);
        // ofs<<"disttime are " <<disttime<<endl;

        if(disttime>0.3)//即it是未来
        {
          break;
        }
        // if(disttime==0.3 ||(disttime<0.3&&disttime>-0.1))
        if(fabs(disttime)<1e-6)

        {
          distance_the=sqrt(pow(cps_.points.col(i)(0)-it->x,2)+pow(cps_.points.col(i)(1)-it->y,2));
           if(distance_the<15)//horizen范围内51cm
          {
          dy_ob(0)=it->x;
          dy_ob(1)=it->y;
          dy_ob(2)=it->t;
        
          double dist = (cps_.points.col(i) - dy_ob).norm();//修改
          predict_dyobs.push_back(dy_ob);
          double dist_err = (CLEARANCE - dist)*counnt;
          Eigen::Vector3d dist_grad = (cps_.points.col(i) - dy_ob).normalized();

            if (dist_err < 0)
          {
            /* do nothing */
          }
          
          else
          {
            if(dist<1)
            {
              cost +=99999;
            }
            else
            {
              cost += pow(dist_err, 2);
            }
            
            gradient.col(i) += -2.0 * dist_err * dist_grad;
          }
      // ofs<<"dyobj_prid is " <<dyobj_prid<<endl;
          }
        }

      }


       
        // double dist = (cps_.points.col(0) - dyobj_prid).norm();//修改

        //cout /*<< "cps_.points.col(i)=" << cps_.points.col(i).transpose()*/ << " moving_objs_=" << obj_prid.transpose() << " dist=" << dist << endl;
        

      // ofs<<"cost is " <<cost<<endl;

      // ofs<<"dist_err is " <<dist_err<<endl;
      // ofs<<"dist_grad is " <<dist_grad<< endl;//修改
      // ofs<<"end "<< endl;//修改
      // ofs<< endl;//修改


      }
      // break;
    // }

        // ofs.close();
  }
  void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {

    cost = 0.0;

    int end_idx = q.cols() - order_;

    // def: f = |x*v|^2/a^2 + |x×v|^2/b^2
    // double a2 = 25, b2 = 1;
    double a2 = 25, b2 = 25;

    for (auto i = order_ - 1; i < end_idx + 1; ++i)
    {
      Eigen::Vector3d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];
      Eigen::Vector3d v = (ref_pts_[i] - ref_pts_[i - 2]).normalized();
      // x(2)=0;
      // v(2)=0;
      double xdotv = x.dot(v);
      Eigen::Vector3d xcrossv = x.cross(v);
      double f = pow((xdotv), 2) / a2 + pow(xcrossv.norm(), 2) / b2;
      cost += f;
      Eigen::Matrix3d m;
      m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
      Eigen::Vector3d df_dx = 2 * xdotv / a2 * v + 2 / b2 * m * xcrossv;

      gradient.col(i - 1) += df_dx / 6;
      gradient.col(i) += 4 * df_dx / 6;
      gradient.col(i + 1) += df_dx / 6;
    }
  }

  void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
  {

    cost = 0.0;

    if (falg_use_jerk)
    {
      Eigen::Vector3d jerk, temp_j;

      for (int i = 0; i < q.cols() - 3; i++)
      {
        /* evaluate jerk */
        jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
        // jerk = q.col(i + 3).head(2); - 3 * q.col(i + 2) .head(2)+ 3 * q.col(i + 1).head(2) - q.col(i).head(2);//修改，加了.head(2)
        cost += jerk.squaredNorm();
        temp_j = 2.0 * jerk;
        /* jerk gradient */
        gradient.col(i + 0) += -temp_j;
        gradient.col(i + 1) += 3.0 * temp_j;
        gradient.col(i + 2) += -3.0 * temp_j;
        gradient.col(i + 3) += temp_j;
      }
    }
    else
    {
      Eigen::Vector3d acc, temp_acc;

      for (int i = 0; i < q.cols() - 2; i++)
      {
        /* evaluate acc */
        acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
        // acc = q.col(i + 2).head(2) - 2 * q.col(i + 1).head(2) + q.col(i).head(2);//修改，加了.head(2)
        cost += acc.squaredNorm();
        temp_acc = 2.0 * acc;
        /* acc gradient */
        gradient.col(i + 0) += temp_acc;
        gradient.col(i + 1) += -2.0 * temp_acc;
        gradient.col(i + 2) += temp_acc;
      }
    }
  }

  void BsplineOptimizer::calcTerminalCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {
    cost = 0.0;

    // zero cost and gradient in hard constraints
    Eigen::Vector3d q_3, q_2, q_1, dq;
    q_3 = q.col(q.cols() - 3);
    q_2 = q.col(q.cols() - 2);
    q_1 = q.col(q.cols() - 1);

    dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - local_target_pt_;
    cost += dq.squaredNorm();

    gradient.col(q.cols() - 3) += 2 * dq * (1 / 6.0);
    gradient.col(q.cols() - 2) += 2 * dq * (4 / 6.0);
    gradient.col(q.cols() - 1) += 2 * dq * (1 / 6.0);
  }

  void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                             Eigen::MatrixXd &gradient)//计算可行性代价
  {

    //#define SECOND_DERIVATIVE_CONTINOUS

#ifdef SECOND_DERIVATIVE_CONTINOUS
    ROS_WARN("SECOND_DERIVATIVE_CONTINOUS!!!!!!!!!!");

    cost = 0.0;
    double demarcation = 1.0; // 1m/s, 1m/s/s
    double ar = 3 * demarcation, br = -3 * pow(demarcation, 2), cr = pow(demarcation, 3);
    double al = ar, bl = -br, cl = cr;

    /* abbreviation */
    double ts, ts_inv2, ts_inv3;
    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;
    ts_inv3 = 1 / ts / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      // for (int j = 0; j < 3; j++)
      for (int j = 0; j < 2; j++)

      {
        if (vi(j) > max_vel_ + demarcation)
        {
          double diff = vi(j) - max_vel_;
          cost += (ar * diff * diff + br * diff + cr) * ts_inv3; // multiply ts_inv3 to make vel and acc has similar magnitude

          double grad = (2.0 * ar * diff + br) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) > max_vel_)
        {
          double diff = vi(j) - max_vel_;
          cost += pow(diff, 3) * ts_inv3;
          ;

          double grad = 3 * diff * diff / ts * ts_inv3;
          ;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -(max_vel_ + demarcation))
        {
          double diff = vi(j) + max_vel_;
          cost += (al * diff * diff + bl * diff + cl) * ts_inv3;

          double grad = (2.0 * al * diff + bl) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -max_vel_)
        {
          double diff = vi(j) + max_vel_;
          cost += -pow(diff, 3) * ts_inv3;

          double grad = -3 * diff * diff / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      // for (int j = 0; j < 3; j++)
      for (int j = 0; j < 2; j++)

      {
        if (ai(j) > max_acc_ + demarcation)
        {
          double diff = ai(j) - max_acc_;
          cost += ar * diff * diff + br * diff + cr;

          double grad = (2.0 * ar * diff + br) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) > max_acc_)
        {
          double diff = ai(j) - max_acc_;
          cost += pow(diff, 3);

          double grad = 3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -(max_acc_ + demarcation))
        {
          double diff = ai(j) + max_acc_;
          cost += al * diff * diff + bl * diff + cl;

          double grad = (2.0 * al * diff + bl) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -max_acc_)
        {
          double diff = ai(j) + max_acc_;
          cost += -pow(diff, 3);

          double grad = -3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

#else

    cost = 0.0;
    /* abbreviation */
    double ts, /*vm2, am2, */ ts_inv2;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;//车辆轴向速度和加速度，转角速度和加速度
    // static float maxAcc = 2;
    // static float maxVel = 3;
    // static float minVel = 0.8;
    // static float accResolution = 0.8;
    // static float timeResolution = 1.0;
    // //前轮最大转向角（rad/s）
    static float maxSteerphi = 35.f/180.f* M_PI;
    std::vector<float> angles;
    std::vector<float> voloc;

    ts = bspline_interval_;//ts为b样条的间隔时间
    ts_inv2 = 1 / ts / ts;//加速度
    for (int i = 0; i < q.cols() - 1; i++)
    {
    float angle = atan2(q.col(i + 1)(1)-q.col(i)(1),q.col(i + 1)(0)-q.col(i)(0));//输出角度在[-PI, PI]
    float voli=sqrt(pow(q.col(i + 1)(0)-q.col(i)(0),2)+pow(q.col(i + 1)(1)-q.col(i)(1),2));
    angles.push_back(angle);
    voloc.push_back(voli);
    }
    for(int j=1;j<angles.size();j++)
    {
      if(fabs(angles[j]-angles[j-1]/ts)>maxSteerphi)
      {
        cost += pow(angles[j]-angles[j-1]/ts, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude////累加从i=1到Nc，计算WvF(Vi)
          // * ts_inv2使vel和acc具有相似的幅值,即乘以一个权重Wv
        gradient(0, j ) +=(fabs(angles[j]-angles[j-1]/ts) - maxSteerphi) / ts * ts_inv2;//第0行是角速度梯度，第一行是速度梯度
        
      }
      if(fabs(voloc[j]-voloc[j-1]/ts)>max_vel_)
      {
        cost += pow(voloc[j]-voloc[j-1]/ts, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude////累加从i=1到Nc，计算WvF(Vi)
          // * ts_inv2使vel和acc具有相似的幅值,即乘以一个权重Wv
        gradient(1, j) += (fabs(voloc[j]-voloc[j-1]/ts) - max_vel_) / ts * ts_inv2;//第0行是角速度梯度，第一行是速度梯度
        
      }
       if(fabs(voloc[j]-voloc[j-1] )* ts_inv2>max_acc_)
      {
        cost += pow((voloc[j]-voloc[j-1] )* ts_inv2, 2) ; // multiply ts_inv3 to make vel and acc has similar magnitude////累加从i=1到Nc，计算WvF(Vi)
          // * ts_inv2使vel和acc具有相似的幅值,即乘以一个权重Wv
        gradient(2, j) += (fabs(voloc[j]-voloc[j-1]/ts) - max_vel_) / ts * ts_inv2;//第0行是角速度梯度，第一行是速度梯度，第二行是加速度
        
      }
      
    }


    // /* velocity feasibility */
    // for (int i = 0; i < q.cols() - 1; i++)//i=1到Nc,对应论文中Feasibility Penalty部分公式(8)
    // {
      
    //   Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;///对应论文中公式(2)

    //   //cout << "temp_v * vi=" ;
    //   // for (int j = 0; j < 3; j++)//修改//xyz三轴上分别计算速度
    //   for (int j = 0; j < 2; j++)//修改
    //   {
    //     if (vi(j) > max_vel_)//每个轴上的速度>最大速度
    //     {
    //       // cout << "zx-todo VEL" << endl;
    //       // cout << vi(j) << endl;
    //       cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude////累加从i=1到Nc，计算WvF(Vi)
    //       // * ts_inv2使vel和acc具有相似的幅值,即乘以一个权重Wv
    //       gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
    //       gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
    //     }
    //     else if (vi(j) < -max_vel_)
    //     {
    //       cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

    //       gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
    //       gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
    //     }
    //     else
    //     {
    //       /* code */
    //     }
    //   }
    // }

    // /* acceleration feasibility */
    // for (int i = 0; i < q.cols() - 2; i++)//i=1到Nc-1,对应论文中Feasibility Penalty部分公式(8)
    // {
    //   Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;//加速度=(第三个控制点-2*第二个控制点+当前控制点）* 1 / ts / ts

    //   //cout << "temp_a * ai=" ;
    //   // for (int j = 0; j < 3; j++)//修改//计算xyz每个轴上的加速度
    //   for (int j = 0; j < 2; j++)//修改
    //   {
    //     if (ai(j) > max_acc_)//加速度>最大加速度
    //     {
    //       // cout << "zx-todo ACC" << endl;
    //       // cout << ai(j) << endl;
    //       cost += pow(ai(j) - max_acc_, 2);;//累加从i=1到Nc-1，计算WaF(Ai)

    //       gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
    //       gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
    //       gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
    //     }
    //     else if (ai(j) < -max_acc_)
    //     {
    //       cost += pow(ai(j) + max_acc_, 2);

    //       gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
    //       gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
    //       gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
    //     }
    //     else
    //     {
    //       /* code */
    //     }
    //   }
    //   //cout << endl;
    // }

#endif
  }

  bool BsplineOptimizer::check_collision_and_rebound(void)
  {

    int end_idx = cps_.size - order_;
    cout<<"cps_.size "<<cps_.size<<endl;
    cout<<"order_ "<<order_<<endl;

    /*** Check and segment the initial trajectory according to obstacles ***/
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    int i_end = end_idx - (end_idx - order_) / 3;
    for (int i = order_ - 1; i <= i_end; ++i)
    {

      bool occ = a_star_->getInflateOccupancy(cps_.points.col(i));

      /*** check if the new collision will be valid ***/
      if (occ)
      {
        for (size_t k = 0; k < cps_.direction[i].size(); ++k)
        {
          cout.precision(2);
          if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 10 * mapresolution) // current point is outside all the collision_points.
          {
            occ = false; // Not really takes effect, just for better hunman understanding.
            break;
          }
        }
      }

      if (occ)
      {
        flag_new_obs_valid = true;

        int j;
        for (j = i - 1; j >= 0; --j)
        {
          occ = a_star_->getInflateOccupancy(cps_.points.col(j));
          if (!occ)
          {
            in_id = j;
            break;
          }
        }
        if (j < 0) // fail to get the obs free point
        {
          ROS_ERROR("ERROR! the drone is in obstacle. This should not happen.");
          in_id = 0;
          // ofstream ofs;
          // ofs.open("/home/houmingyu/桌面/record/test.txt",std::ios::app);
          // // ofs<<"ERROR! the drone is in obstacle. This should not happen."<<endl;
          // ofs.close();
        }

        for (j = i + 1; j < cps_.size; ++j)
        {
          occ = a_star_->getInflateOccupancy(cps_.points.col(j));

          if (!occ)
          {
            out_id = j;
            break;
          }
        }
        if (j >= cps_.size) // fail to get the obs free point
        {
          ROS_WARN("WARN! terminal point of the current trajectory is in obstacle, skip this planning.");

          force_stop_type_ = STOP_FOR_ERROR;
          return false;
        }

        i = j + 1;

        segment_ids.push_back(std::pair<int, int>(in_id, out_id));
      }
    }




    if (flag_new_obs_valid)
    {
      vector<Eigen::Vector3d> a_star_pathe;

      vector<vector<Eigen::Vector3d>> a_star_pathes;
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        /*** a star search ***/
        Eigen::Vector3d in(cps_.points.col(segment_ids[i].first)), out(cps_.points.col(segment_ids[i].second));
        // if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ 0.1, in, out))
        // {
        //   a_star_pathes.push_back(a_star_->getPath());
        // }
      if (a_star_->AstarSearch(in,out,a_star_pathe,drone_id_))
      {
        a_star_pathes.push_back(a_star_pathe);
      }
        else
        {
          ROS_ERROR("a star error");
          segment_ids.erase(segment_ids.begin() + i);
          i--;
        }
      }

      for (size_t i = 1; i < segment_ids.size(); i++) // Avoid overlap
      {
        if (segment_ids[i - 1].second >= segment_ids[i].first)
        {
          double middle = (double)(segment_ids[i - 1].second + segment_ids[i].first) / 2.0;
          segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
          segment_ids[i].first = static_cast<int>(middle + 1.1);
        }
      }

      /*** Assign parameters to each segment ***/
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        // step 1
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
          cps_.flag_temp[j] = false;

        // step 2
        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
          Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
          int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
          double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
          while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
          {
            last_Astar_id = Astar_id;

            if (val >= 0)
              --Astar_id;
            else
              ++Astar_id;

            val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

            // cout << val << endl;

            if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
            {
              intersection_point =
                  a_star_pathes[i][Astar_id] +
                  ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                   (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                  );

              got_intersection_id = j;
              break;
            }
          }

          if (got_intersection_id >= 0)
          {
            double length = (intersection_point - cps_.points.col(j)).norm();
            if (length > 1e-5)
            {
              cps_.flag_temp[j] = true;
              for (double a = length; a >= 0.0; a -= mapresolution)
              {
                bool occ = a_star_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

                if (occ || a < mapresolution)
                {
                  if (occ)
                    a += mapresolution;
                  cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                  cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                  break;
                }
              }
            }
            else
            {
              got_intersection_id = -1;
            }
          }
        }

        //step 3
        if (got_intersection_id >= 0)
        {
          for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
              cps_.direction[j].push_back(cps_.direction[j - 1].back());
            }

          for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
              cps_.direction[j].push_back(cps_.direction[j + 1].back());
            }
        }
        else
          ROS_WARN("Failed to generate direction. It doesn't matter.");
      }

      force_stop_type_ = STOP_FOR_REBOUND;
      return true;
    }

    return false;

  }

  bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts)
  {
    setBsplineInterval(ts);
    cout<<"ts is "<<ts<<endl;

    double final_cost;
    bool flag_success = rebound_optimize(final_cost);

    optimal_points = cps_.points;

    return flag_success;
  }

  bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double &final_cost, const ControlPoints &control_points, double ts)//
  {
    setBsplineInterval(ts);

    cps_ = control_points;

    bool flag_success = rebound_optimize(final_cost);

    optimal_points = cps_.points;

    return flag_success;
  }
    bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double &final_cost,double ts,const ControlPoints &control_points,double t_current)//houyu:用的这个
  {
    setBsplineInterval(ts);
    cps_ = control_points;
    cout<<"b_spline ts is "<<ts<<endl;
    bool flag_success = rebound_optimize_(final_cost,t_current);
    t_current_=t_current;
    optimal_points = cps_.points;

    return flag_success;
  }

  bool BsplineOptimizer::BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points)
  {
    //设置初始控制点init_points、时间间隔ts，调用refine_optimize()重新分配时间，得到最优的动力学可行轨迹，并将其控制点赋值给optimal_points。
    setControlPoints(init_points);
    setBsplineInterval(ts);

    bool flag_success = refine_optimize();

    optimal_points = cps_.points;

    return flag_success;
  }
  bool BsplineOptimizer::rebound_optimize_(double &final_cost,double t_current)
  {
    iter_num_ = 0;
    int start_id = order_;
    // int end_id = this->cps_.size - order_; //Fixed end
    int end_id = this->cps_.size; // Free end
    variable_num_ = 3 * (end_id - start_id);
    cout<<"end_id is "<<end_id<<endl;
    cout<<"start_id is "<<start_id<<endl;

    ros::Time t0 = ros::Time::now(), t1, t2;
    int restart_nums = 0, rebound_times = 0;
    ;
    bool flag_force_return, flag_occ, success;
    new_lambda2_ = lambda2_;
    constexpr int MAX_RESART_NUMS_SET = 3;
    do
    {
      /* ---------- prepare ---------- */
      min_cost_ = std::numeric_limits<double>::max();
      min_ellip_dist_ = INIT_min_ellip_dist_;
      iter_num_ = 0;
      flag_force_return = false;
      flag_occ = false;
      success = false;
      // force_stop_type_ = DONT_STOP;

      double q[variable_num_];
      memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      // lbfgs_params.mem_size = 16;
      // lbfgs_params.max_iterations = 200;
      lbfgs_params.g_epsilon = 0.01;
      lbfgs_params.mem_size = 64;//128
      lbfgs_params.max_iterations = 1000;
      // lbfgs_params.g_epsilon = 0.0;

      /* ---------- optimize ---------- */
      t1 = ros::Time::now();
      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
      t2 = ros::Time::now();
      double time_ms = (t2 - t1).toSec() * 1000;
      double total_time_ms = (t2 - t0).toSec() * 1000;

      /* ---------- success temporary, check collision again ---------- */
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        //ROS_WARN("Solver error in planning!, return = %s", lbfgs::lbfgs_strerror(result));
        flag_force_return = false;

        /*** collision check, phase 1 ***/
        if ((min_ellip_dist_ != INIT_min_ellip_dist_) && (min_ellip_dist_ > swarm_clearance_))
        {
          success = false;
          restart_nums++;
          initControlPoints(cps_.points, false);
          new_lambda2_ *=2;

          printf("\033[32miter(+1)=%d,time(ms)=%5.3f, swarm too close, keep optimizing\n\033[0m", iter_num_, time_ms);

          continue;
        }

      //   /*** collision check, phase 2 ***/
      //   UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
      //   double tm, tmp;
      //   traj.getTimeSpan(tm, tmp);
      //   cout<<"tm is "<<tm<<endl;
      //   cout<<"tmp is "<<tmp<<endl;

      //   double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / mapresolution);
      //   cout<<"t_step is "<<t_step<<endl;
      //   if((traj.evaluateDeBoorT(tm)-traj.evaluateDeBoorT(tmp*2/3)).norm()>5.0)
      // {
      //    for (double t = tm; t < tmp * 2 / 3; t += t_step) // Only check the closest 2/3 partition of the whole trajectory.
      //   {
      //   cout<<"traj.evaluateDeBoorT(t) is "<<traj.evaluateDeBoorT(t)<<endl;
        
      //     flag_occ = a_star_->getInflateOccupancy(traj.evaluateDeBoorT(t));
      //     if (flag_occ)
      //     {
      //       //cout << "hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;
            
      //       occupoint=traj.evaluateDeBoorT(t);
      //       occupoint(2)=0;
      //       ofstream ofs;
      //       std::string str11 = "/home/rancho/1hmy/ego-planner-swarm/record/first_3";
      //       std::string str1txt=str11+std::to_string(drone_id_);
      //       str11=str1txt+".txt";
      //       ofs.open(str11,std::ios::app);
      //       ofs<<"occupoint is " <<occupoint(0)<<"  "<<occupoint(1)<<endl;
      //       ofs.close();    

      //       if (t <= bspline_interval_) // First 3 control points in obstacles!
      //       {
      //         // cout << cps_.points.col(1).transpose() << "\n"
      //         //      << cps_.points.col(2).transpose() << "\n"
      //         //      << cps_.points.col(3).transpose() << "\n"
      //         //      << cps_.points.col(4).transpose() << endl;
      //         ROS_WARN("First 3 control points in obstacles! return false, t=%f", t);
      //         return false;
      //       }

      //       break;
      //     }
      //   }
      // }
       

        // cout << "XXXXXX" << ((cps_.points.col(cps_.points.cols()-1) + 4*cps_.points.col(cps_.points.cols()-2) + cps_.points.col(cps_.points.cols()-3))/6 - local_target_pt_).norm() << endl;

        /*** collision check, phase 3 ***/
//#define USE_SECOND_CLEARENCE_CHECK
#ifdef USE_SECOND_CLEARENCE_CHECK
        bool flag_cls_xyp, flag_cls_xyn, flag_cls_zp, flag_cls_zn;
        Eigen::Vector3d start_end_vec = traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm);
        Eigen::Vector3d offset_xy(-start_end_vec(0), start_end_vec(1), 0);
        offset_xy.normalize();
        Eigen::Vector3d offset_z = start_end_vec.cross(offset_xy);
        offset_z.normalize();
        offset_xy *= cps_.clearance / 2;
        offset_z *= cps_.clearance / 2;

        Eigen::MatrixXd check_pts(cps_.points.rows(), cps_.points.cols());
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts.col(i) = cps_.points.col(i);
          check_pts(0, i) += offset_xy(0);
          check_pts(1, i) += offset_xy(1);
          check_pts(2, i) += offset_xy(2);
        }
        flag_cls_xyp = initControlPoints(check_pts, false).size() > 0;
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts(0, i) -= 2 * offset_xy(0);
          check_pts(1, i) -= 2 * offset_xy(1);
          check_pts(2, i) -= 2 * offset_xy(2);
        }
        flag_cls_xyn = initControlPoints(check_pts, false).size() > 0;
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts(0, i) += offset_xy(0) + offset_z(0);
          check_pts(1, i) += offset_xy(1) + offset_z(1);
          check_pts(2, i) += offset_xy(2) + offset_z(2);
        }
        flag_cls_zp = initControlPoints(check_pts, false).size() > 0;
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts(0, i) -= 2 * offset_z(0);
          check_pts(1, i) -= 2 * offset_z(1);
          check_pts(2, i) -= 2 * offset_z(2);
        }
        flag_cls_zn = initControlPoints(check_pts, false).size() > 0;
        if ((flag_cls_xyp ^ flag_cls_xyn) || (flag_cls_zp ^ flag_cls_zn))
          flag_occ = true;
#endif

        if (!flag_occ)
        {
          printf("\033[32miter(+1)=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
          success = true;
        }
        else // restart
        {
          restart_nums++;
          initControlPoints(cps_.points, false);
          new_lambda2_ *= 2;

          printf("\033[32miter(+1)=%d,time(ms)=%5.3f, collided, keep optimizing\n\033[0m", iter_num_, time_ms);
        }
      }
      else if (result == lbfgs::LBFGSERR_CANCELED)
      {
        flag_force_return = true;
        rebound_times++;
        cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",rebound." << endl;
      }
      else
      {
        ROS_WARN("Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
        ROS_WARN("Solver error. LBFGSERR_CANCELED = %d. Skip this planning.", lbfgs::LBFGSERR_CANCELED);

        // while (ros::ok());
      }

    } while (
        ((flag_occ || ((min_ellip_dist_ != INIT_min_ellip_dist_) && (min_ellip_dist_ > swarm_clearance_))) && restart_nums < MAX_RESART_NUMS_SET) ||
        (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

    return success;
  }
  bool BsplineOptimizer::rebound_optimize(double &final_cost)//houyu:优化（连续可导，与原轨迹距离，与动态障碍物的距离，dist小于多少则为无穷，每层加权，因为时间越长越不可信，加速度速度边界条件）
  {
    iter_num_ = 0;
    int start_id = order_;
    // int end_id = this->cps_.size - order_; //Fixed end
    int end_id = this->cps_.size; // Free end
    variable_num_ = 3 * (end_id - start_id);
    cout<<"start_id is "<<start_id<<endl;
    cout<<"end_id is "<<end_id<<endl;

    ros::Time t0 = ros::Time::now(), t1, t2;
    int restart_nums = 0, rebound_times = 0;
    ;
    bool flag_force_return, flag_occ, success;
    new_lambda2_ = lambda2_;
    constexpr int MAX_RESART_NUMS_SET = 3;
    do
    {
      /* ---------- prepare ---------- */
      min_cost_ = std::numeric_limits<double>::max();
      min_ellip_dist_ = INIT_min_ellip_dist_;
      iter_num_ = 0;
      flag_force_return = false;
      flag_occ = false;
      success = false;

      double q[variable_num_];
      memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;
      lbfgs_params.max_iterations = 200;
      lbfgs_params.g_epsilon = 0.01;

      /* ---------- optimize ---------- */
      t1 = ros::Time::now();
      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
      t2 = ros::Time::now();
      double time_ms = (t2 - t1).toSec() * 1000;
      double total_time_ms = (t2 - t0).toSec() * 1000;

      /* ---------- success temporary, check collision again ---------- */
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        //ROS_WARN("Solver error in planning!, return = %s", lbfgs::lbfgs_strerror(result));
        flag_force_return = false;

        /*** collision check, phase 1 ***/
        if ((min_ellip_dist_ != INIT_min_ellip_dist_) && (min_ellip_dist_ > swarm_clearance_))
        {
          success = false;
          restart_nums++;
          initControlPoints(cps_.points, false);
          new_lambda2_ *= 2;

          printf("\033[32miter(+1)=%d,time(ms)=%5.3f, swarm too close, keep optimizing\n\033[0m", iter_num_, time_ms);

          continue;
        }

        /*** collision check, phase 2 ***/
        UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
        double tm, tmp;
        traj.getTimeSpan(tm, tmp);
        double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / mapresolution);
        for (double t = tm; t < tmp * 2 / 3; t += t_step) // Only check the closest 2/3 partition of the whole trajectory.
        {
          flag_occ = a_star_->getInflateOccupancy(traj.evaluateDeBoorT(t));
          if (flag_occ)
          {

            //cout << "hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

            if (t <= bspline_interval_) // First 3 control points in obstacles!
            {
              // cout << cps_.points.col(1).transpose() << "\n"
              //      << cps_.points.col(2).transpose() << "\n"
              //      << cps_.points.col(3).transpose() << "\n"
              //      << cps_.points.col(4).transpose() << endl;
              ROS_WARN("First 3 control points in obstacles! return false, t=%f", t);
              return false;
            }

            break;
          }
        }

        // cout << "XXXXXX" << ((cps_.points.col(cps_.points.cols()-1) + 4*cps_.points.col(cps_.points.cols()-2) + cps_.points.col(cps_.points.cols()-3))/6 - local_target_pt_).norm() << endl;

        /*** collision check, phase 3 ***/
//#define USE_SECOND_CLEARENCE_CHECK
#ifdef USE_SECOND_CLEARENCE_CHECK
        bool flag_cls_xyp, flag_cls_xyn, flag_cls_zp, flag_cls_zn;
        Eigen::Vector3d start_end_vec = traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm);
        Eigen::Vector3d offset_xy(-start_end_vec(0), start_end_vec(1), 0);
        offset_xy.normalize();
        Eigen::Vector3d offset_z = start_end_vec.cross(offset_xy);
        offset_z.normalize();
        offset_xy *= cps_.clearance / 2;
        offset_z *= cps_.clearance / 2;

        Eigen::MatrixXd check_pts(cps_.points.rows(), cps_.points.cols());
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts.col(i) = cps_.points.col(i);
          check_pts(0, i) += offset_xy(0);
          check_pts(1, i) += offset_xy(1);
          check_pts(2, i) += offset_xy(2);
        }
        flag_cls_xyp = initControlPoints(check_pts, false).size() > 0;
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts(0, i) -= 2 * offset_xy(0);
          check_pts(1, i) -= 2 * offset_xy(1);
          check_pts(2, i) -= 2 * offset_xy(2);
        }
        flag_cls_xyn = initControlPoints(check_pts, false).size() > 0;
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts(0, i) += offset_xy(0) + offset_z(0);
          check_pts(1, i) += offset_xy(1) + offset_z(1);
          check_pts(2, i) += offset_xy(2) + offset_z(2);
        }
        flag_cls_zp = initControlPoints(check_pts, false).size() > 0;
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts(0, i) -= 2 * offset_z(0);
          check_pts(1, i) -= 2 * offset_z(1);
          check_pts(2, i) -= 2 * offset_z(2);
        }
        flag_cls_zn = initControlPoints(check_pts, false).size() > 0;
        if ((flag_cls_xyp ^ flag_cls_xyn) || (flag_cls_zp ^ flag_cls_zn))
          flag_occ = true;
#endif

        if (!flag_occ)
        {
          printf("\033[32miter(+1)=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
          success = true;
        }
        else // restart
        {
          restart_nums++;
          initControlPoints(cps_.points, false);
          new_lambda2_ *= 2;

          printf("\033[32miter(+1)=%d,time(ms)=%5.3f, collided, keep optimizing\n\033[0m", iter_num_, time_ms);
        }
      }
      else if (result == lbfgs::LBFGSERR_CANCELED)
      {
        flag_force_return = true;
        rebound_times++;
        cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",rebound." << endl;
      }
      else
      {
        ROS_WARN("Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
        ROS_WARN("Solver error. LBFGSERR_CANCELED = %d. Skip this planning.", lbfgs::LBFGSERR_CANCELED);
        
        // while (ros::ok());
      }

    } while (
        ((flag_occ || ((min_ellip_dist_ != INIT_min_ellip_dist_) && (min_ellip_dist_ > swarm_clearance_))) && restart_nums < MAX_RESART_NUMS_SET) ||
        (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

    return success;
  }
  //再优化
  //使用L-BFGS方法对目标函数进行优化
  //得到重新分配时间后，光滑、拟合较好、动力学可行的轨迹。
  bool BsplineOptimizer::refine_optimize()
  {
    iter_num_ = 0;
    int start_id = order_;
    int end_id = this->cps_.points.cols() - order_;
    variable_num_ = 3 * (end_id - start_id);

    double q[variable_num_];
    double final_cost;

    memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

    double origin_lambda4 = lambda4_;
    bool flag_safe = true;
    int iter_count = 0;
    do
    {
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;
      lbfgs_params.max_iterations = 200;
      // lbfgs_params.g_epsilon = 0.001;
      lbfgs_params.g_epsilon = 0.01;


      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRefine, NULL, NULL, this, &lbfgs_params);
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        //pass
      }
      else
      {
        ROS_ERROR("Solver error in refining!, return = %d, %s", result, lbfgs::lbfgs_strerror(result));
      }

      UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
      double tm, tmp;
      traj.getTimeSpan(tm, tmp);
      double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / mapresolution); // Step size is defined as the maximum size that can passes throgth every gird.
      for (double t = tm; t < tmp * 2 / 3; t += t_step)
      {
        // if (a_star_->getInflateOccupancy(traj.evaluateDeBoorT(t)))
        // {
        //   // cout << "Refined traj hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;
        //   occupoint=traj.evaluateDeBoorT(t);
        //   occupoint(2)=0;
        //     ofstream ofs;
        //     std::string str11 = "/home/rancho/1hmy/ego-planner-swarm/record/occupoint";
        //     std::string str1txt=str11+std::to_string(drone_id_);
        //     str11=str1txt+".txt";
        //     ofs.open(str11,std::ios::app);
        //     ofs<<"occupoint is " <<occupoint(0)<<"  "<<occupoint(1)<<endl;
        //     ofs.close();  
        //   Eigen::MatrixXd ref_pts(ref_pts_.size(), 3);
        //   for (size_t i = 0; i < ref_pts_.size(); i++)
        //   {
        //     ref_pts.row(i) = ref_pts_[i].transpose();
        //   }

        //   flag_safe = false;
        //   break;
        // }
      }

      if (!flag_safe)
        lambda4_ *= 8;

      iter_count++;
    } while (!flag_safe && iter_count <= 0);

    lambda4_ = origin_lambda4;

    //cout << "iter_num_=" << iter_num_ << endl;

    return flag_safe;
  }
   void BsplineOptimizer::Setgloabaldata(GlobalTrajData global_data)
   {this->the_global_data=global_data;}
//代价计算
  void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
  {
    // cout << "drone_id_=" << drone_id_ << endl;
    // cout << "cps_.points.size()=" << cps_.points.size() << endl;
    // cout << "n=" << n << endl;
    // cout << "sizeof(x[0])=" << sizeof(x[0]) << endl;

    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_distance, f_feasibility /*, f_mov_objs*/, f_swarm, f_terminal;
    double f_distbetwen_gloabl,f_dyobs,f_static_distance;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_swarm = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_terminal = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_distbetwen_gloabl = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_dyobs = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_static_distance = Eigen::MatrixXd::Zero(3, cps_.size);


    calcSwarmCost(cps_.points, f_swarm, g_swarm);
    calcTerminalCost(cps_.points, f_terminal, g_terminal);
    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);
    
    calcDistancestatic_CostRebound(cps_.points, f_static_distance, g_static_distance);
    calcbetwen_gloabl(cps_.points, f_distbetwen_gloabl, g_distbetwen_gloabl);
    calcbetwen_f_dyobs(cps_.points, f_dyobs, g_dyobs);
    // calcMovingObjCost(cps_.points, f_mov_objs, g_mov_objs);

      ofstream ofs;
      std::string str11 = "/home/rancho/1hmy/ego-planner-swarm/record/opticost";
      std::string str1txt=str11+std::to_string(drone_id_);
      str11=str1txt+".txt";
      ofs.open(str11,std::ios::app);
      ofs<<"f_swarm is " <<f_swarm<<endl;
      ofs<<"f_terminal is " <<f_terminal<<endl;
      ofs<<"f_smoothness is " <<f_smoothness<< endl;//修改
      ofs<<"f_distance is " <<f_distance<< endl;//修改
      ofs<<"f_feasibility is " <<f_feasibility<< endl;//修改

      ofs<<"f_static_distance is " <<f_static_distance<< endl;//修改
      ofs<<"f_distbetwen_gloabl is " <<f_distbetwen_gloabl<< endl;//修改
      ofs<<"f_dyobs is " <<f_dyobs<< endl;//修改

      ofs<<"end "<< endl;//修改
      ofs<< endl;//修改
      ofs.close();

    //lambda1_is f_smoothness;new 2 is f_distance,f_swarm,2f_terminal100,3 isf_feasibility /100
    // f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility + new_lambda2_ * f_swarm + lambda2_ * f_terminal;
    f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility + new_lambda2_ * f_swarm + lambda2_ * f_terminal+lambda5_*f_static_distance+lambda6_*f_distbetwen_gloabl+lambda7_*f_dyobs;

    //f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility + new_lambda2_ * f_mov_objs;
    //printf("origin %f %f %f %f\n", f_smoothness, f_distance, f_feasibility, f_combine);

    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility + new_lambda2_ * g_swarm + lambda2_ * g_terminal+lambda5_*g_static_distance+lambda6_*g_distbetwen_gloabl+lambda7_*g_dyobs;
    //Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility + new_lambda2_ * g_mov_objs;
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

  void BsplineOptimizer:: combineCostRefine(const double *x, double *grad, double &f_combine, const int n)
  {

    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_fitness, f_feasibility;
    double f_dyobs;
    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());

    // Eigen::MatrixXd g_distbetwen_gloabl = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_dyobs = Eigen::MatrixXd::Zero(3, cps_.size);
    //time_satrt = ros::Time::now();

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcFitnessCost(cps_.points, f_fitness, g_fitness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);
    // calcbetwen_gloabl(cps_.points, f_distbetwen_gloabl, g_distbetwen_gloabl);
    calcbetwen_f_dyobs(cps_.points, f_dyobs, g_dyobs);
    // lambda1_=lambda1_/100;
    lambda4_=lambda4_*10;
    lambda3_=lambda3_*100;
    /* ---------- convert to solver format...---------- */
    // f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility;
    f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility+lambda7_*f_dyobs;

    // printf("origin %f %f %f %f\n", f_smoothness, f_fitness, f_feasibility, f_combine);

    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + lambda4_ * g_fitness + lambda3_ * g_feasibility+lambda7_*g_dyobs;
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
    //  ofstream ofs;
    //   std::string str11 = "/home/rancho/1hmy/ego-planner-swarm/record/refine_optcost";
    //   std::string str1txt=str11+std::to_string(drone_id_);
    //   str11=str1txt+".txt";
    //   ofs.open(str11,std::ios::app);
    //   ofs<<"f_smoothness is " <<f_smoothness<<endl;
    //   ofs<<"f_fitness is " <<f_fitness<<endl;
    //   ofs<<"f_feasibility is " <<f_feasibility<< endl;//修改
    //   // ofs<<"g_distbetwen_gloabl is " <<g_distbetwen_gloabl<< endl;//修改
    //   ofs<<"g_dyobs is " <<f_dyobs<< endl;//修改


    //   ofs<<"lambda1_ is " <<lambda1_<< endl;//修改
    //   ofs<<"lambda4_ is " <<lambda4_<< endl;//修改
    //   ofs<<"lambda3_ is " <<lambda3_<< endl;//修改
    //   ofs<<"lambda6_ is " <<lambda6_<< endl;//修改
    //   ofs<<"lambda7_ is " <<lambda7_<< endl;//修改
      

     
    //   ofs<< endl;//修改
    //   ofs.close();
    lambda3_=lambda3_/100;
    lambda4_=lambda4_/10;

  }

} // namespace ego_planner