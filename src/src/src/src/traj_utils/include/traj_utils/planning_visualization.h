#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
//#include <bspline_opt/uniform_bspline.h>
#include <iostream>
//#include <bspline_opt/polynomial_traj.h>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdlib.h>
#include<fstream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include "nav_msgs/Path.h"
#include <pcl/filters/passthrough.h>                 //直通滤波器头文
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
using std::vector;
namespace dy_obs
{
  class PlanningVisualization
  {
  private:
    ros::NodeHandle node;

    ros::Publisher goal_point_pub,global_list_pub,init_list_pub;
    ros::Publisher curren_point_pub,localstart_point_pub;
    ros::Publisher localtarget_point_pub;
    ros::Publisher occu_point_pub,pred_pub,occu_path;

    ros::Publisher optimal_list_pub;
    ros::Publisher a_star_list_pub;
    ros::Publisher guide_vector_pub;
    ros::Publisher intermediate_state_pub;
    ros::Publisher _all_map_pub;
    ros::Publisher _all_map_pub_trac;
    ros::Publisher _all_map_pub_local;
    sensor_msgs::PointCloud2 localMap_pcd;//相当于outupt
    sensor_msgs::PointCloud2 localMap_pcd_trac;//相当于outupt
    sensor_msgs::PointCloud2 localMap_;//相当于outupt

    pcl::PointXYZ pointmin;//用于存放三个轴的最小值
    pcl::PointXYZ pointmax;//用于存放三个轴的最大值
  public:
    PlanningVisualization(/* args */) {}
    ~PlanningVisualization() {}
    PlanningVisualization(ros::NodeHandle &nh,int car_id_,std::string mapback,std::string maptrac);

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id,  bool show_sphere = true);
    void generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                  const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                   const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
    void displayoccuPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
    void displaycurrPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
    void displaylocaltargetPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
    void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id);
    void displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id);
    void displayMultiInitPathList(vector<vector<Eigen::Vector3d>> init_trajs, const double scale);
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
    // void mapdiaplay(std::string filemap);
    void displaypredictpath(vector<Eigen::Vector3d> init_pts, const double scale, int id);
    void occuPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id);
    void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);
    void displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void pubSensedPoints(Eigen::Vector3d curpos);
    void displaylocalstartPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);

    // void displayIntermediateState(ros::Publisher& intermediate_pub, ego_planner::BsplineOptimizer::Ptr optimizer, double sleep_time, const int start_iteration);
    // void displayNewArrow(ros::Publisher& guide_vector_pub, ego_planner::BsplineOptimizer::Ptr optimizer);
  };
} // namespace ego_planner
#endif