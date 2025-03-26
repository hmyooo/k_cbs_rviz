#pragma once
#include "environment.hpp"
#include "cl_cbs.hpp"
#include "opencv2/opencv.hpp"
#include "yaml-cpp/yaml.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "traj_utils/planning_visualization.h"
#include "visualization_msgs/Marker.h"
#include <Eigen/Eigen>
#include <time.h>
#include <pcl/filters/passthrough.h> 
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>                 //直通滤波器头文件
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include "std_msgs/Bool.h"
#include <ros/ros.h>
#include "safe_corridor.hpp"
using Eigen::VectorXd;

using Eigen::Vector3i;
std::vector<std::deque<VectorXd>> PoseBag;
clock_t startTime,endTime;
double conflict_num;
std::vector<Vector3i> colors = {Vector3i (163, 17, 71) ,Vector3i (134, 33, 179) ,Vector3i (90, 207, 51) ,Vector3i (59, 137, 129) ,Vector3i (241, 242, 125) ,
    Vector3i (154, 181, 67) ,Vector3i (21, 109, 63) ,Vector3i (107, 116, 193) ,Vector3i (172, 45, 109) ,Vector3i (242, 87, 167) ,Vector3i (151, 140, 166) ,
    Vector3i (122, 119, 89) ,Vector3i (159, 89, 50) ,Vector3i (67, 136, 163) ,Vector3i (239, 201, 76) ,Vector3i (227, 180, 159) ,Vector3i (214, 169, 129) ,
    Vector3i (174, 29, 107) ,Vector3i (145, 22, 82) ,Vector3i (230, 168, 178)};
nav_msgs::Path* DAgentPaths;
nav_msgs::Path* AgentPaths;

struct Timepub{
    bool success;
    ros::Publisher* Car_Mesh_pub;
    ros::Publisher* Dynamic_paths_pub;
    ros::Publisher* paths_pub;
};
ros::Publisher _all_map_pub;
ros::Publisher _all_map_pub_trac;
ros::Publisher _all_map_pub_local;

sensor_msgs::PointCloud2 localMap_pcd;//相当于outupt
sensor_msgs::PointCloud2 localMap_pcd_trac;//相当于outupt
sensor_msgs::PointCloud2 localMap_;//相当于outupt
ros::Publisher Currentpose_pub;
vector<ros::Publisher> pose_pubs; 
int posebagmaxsize=0;
int posebagid=0;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud_back(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud_back_curr(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud_trac(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud_trac_new(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointXYZ pointmin;//用于存放三个轴的最小值
pcl::PointXYZ pointmax;//用于存放三个轴的最大值
// Safe_Corridor these_safe_corridor;
std::vector<std::vector<Eigen::MatrixXd>> all_corrid_,smallest_corrid_;
std::vector<std::vector<double>> t__ts;
ros::Publisher corrids_pub_markerarray,small_corrids_pub_markerarray,marker_pub_OBB;
visualization_msgs::MarkerArray marker_array,small_marker_array;

double normalizeHeadingRad(float t) {
    t =0.5*M_PI - t;
    double theta;
    if (t < 0) {
        t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
        theta= 2.f * M_PI + t;
    }
    theta= t - 2.f * M_PI * (int)(t / (2.f * M_PI));
    return theta;
}
/*
* 四元数转欧拉角
*/
double ToEulerAngles(geometry_msgs::Quaternion q) {
    double roll,pitch,yaw;
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    roll = atan2(sinr_cosp, cosr_cosp);
 
    // pitch (y-axis rotation)
    double sinp = sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

 
    // yaw (z-axis rotation)
    double siny_cosp = +2.0  * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
 		
		
    if (yaw < 0) {
        yaw += 2 * M_PI;
    }

    
	
    return yaw;
}

/*
* 欧拉角转四元数
*/
geometry_msgs::Quaternion ToQuaternion(double yaw, double pitch = 0, double roll = 0) {// yaw (Z), pitch (Y), roll (X)
// Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    geometry_msgs::Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    double norm = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;

    return q;
}


/*
* 定时显示无人车位置
*/
void mesh_gen( visualization_msgs::Marker &meshROS,int i,VectorXd tm_pose){
    meshROS.header.stamp = ros::Time::now();
    meshROS.id = i;  
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD;
    meshROS.scale.x = 2.0;
    meshROS.scale.y = 2.0;
    meshROS.scale.z = 2.0;
    meshROS.color.a = 0.8 ;
    meshROS.color.r = colors[i][0]/255.0;
    meshROS.color.g = colors[i][1]/255.0;
    meshROS.color.b = colors[i][2]/255.0;
    meshROS.mesh_resource = "package://path_search/meshes/car.dae";
    meshROS.header.frame_id = "world";
    meshROS.ns = "mesh";
    meshROS.pose.position.x = tm_pose(1);
    meshROS.pose.position.y = tm_pose(2);
    meshROS.pose.position.z = 0;
    meshROS.pose.orientation = ToQuaternion(tm_pose(3));
}
void Dynamic_path_gen(nav_msgs::Path &path,int i,VectorXd tm_pose){
    geometry_msgs::PoseStamped current_pose;
    current_pose.header.stamp = ros::Time::now();
    current_pose.header.frame_id = "world";
    current_pose.pose.position.x = tm_pose(1);
    current_pose.pose.position.y = tm_pose(2);
    current_pose.pose.position.z = 0.1;//时间*10tm_pose(0)
    current_pose.pose.orientation.x= tm_pose(3);//yaw
    current_pose.pose.orientation.y= tm_pose(4);//速度
    current_pose.pose.orientation.z= i;//



    path.poses.push_back(current_pose);
}
    void pubSensedPoints(double curposez) {

      if(curposez<pointmax.z)
    {
      //局部点云
      pcl::PassThrough<pcl::PointXYZ> passthrough;
      //输入点云1
      passthrough.setInputCloud(pcd_cloud_trac);
      //设置对z轴进行操作
      passthrough.setFilterFieldName("z");
      //设置滤波范围
      passthrough.setFilterLimits(curposez*10,curposez*10+60);//当前位置6s预测地图，20层3Bei即为60

    //   passthrough.setFilterLimits(DAgentPaths[posebagid].poses.back().pose.position.z*3, DAgentPaths[posebagid].poses.back().pose.position.z*3+60);//当前位置6s预测地图，20层3Bei即为60
      // true表示保留滤波范围外，false表示保留范围内
      passthrough.setFilterLimitsNegative(false);
      //执行滤波并存储
      passthrough.filter(*cloud_after_PassThrough);
      int j=0;
      pcd_cloud_back_curr->clear();
      for(int ipoint=0;ipoint<pcd_cloud_back->points.size();ipoint++)
      {
            pcl::PointXYZ p;
            {
            p.x = pcd_cloud_back->points[ipoint].x;

            p.y = pcd_cloud_back->points[ipoint].y;

            p.z = pcd_cloud_back->points[ipoint].z+curposez*10;//3s一取

            pcd_cloud_back_curr->push_back(p);
            j++;
            }
   

           
      }
    //         for(int ipoint=0;ipoint<cloud_after_PassThrough->points.size();ipoint++)
    //   {
           

    //     cloud_after_PassThrough->points[ipoint].z=cloud_after_PassThrough->points[ipoint].z/10;//3s一取
    //   }
      pcd_cloud_back_curr->height = 1;//对于无序点云hight默认是1

      pcd_cloud_back_curr->width =j;//cloud_extract点云文件中push_back了j个点，故width=j
      (*local_cloud)=(*cloud_after_PassThrough)+(*pcd_cloud_back_curr);
    //   countcurr++;
    //   cout<<"countcurr"<<countcurr<<endl;
      // cout<<"localcloud size is"<<local_cloud->points.size()<<endl;

      // kdtreeLocalMap.setInputCloud(local_cloud->makeShared());



      }

  if (pcd_cloud_back_curr->points.size()>0) 
  {
    pcl::toROSMsg(*pcd_cloud_back_curr, localMap_pcd);
  // globalMap_pcd.header.frame_id = "world";
  // _all_map_pub.publish(globalMap_pcd);
    localMap_pcd.header.frame_id = "world";
    _all_map_pub.publish(localMap_pcd);
    // cout<<"if nor"<<endl;
    }

    if (cloud_after_PassThrough->points.size()>0) 
  {
    pcl::toROSMsg(*cloud_after_PassThrough, localMap_pcd_trac);
  // globalMap_pcd.header.frame_id = "world";
  // _all_map_pub.publish(globalMap_pcd);
    localMap_pcd_trac.header.frame_id = "world";
    _all_map_pub_trac.publish(localMap_pcd_trac);
    // cout<<"if nor"<<endl;
     }
     if (local_cloud->points.size()>0) 
  {
    pcl::toROSMsg(*local_cloud, localMap_);
  // globalMap_pcd.header.frame_id = "world";
  // _all_map_pub.publish(globalMap_pcd);
    localMap_.header.frame_id = "world";
    _all_map_pub_local.publish(localMap_);
    // cout<<"if nor"<<endl;
     }

}
void pub_corrid()
{

 int numid=1;
 for (int j=0;j<all_corrid_.size();j++)
 {
   // cout<<"33333333333333333"<<endl;
    // std::cout<<"all_corrid_[j].size() "<<all_corrid_[j].size()<<std::endl;
    int count_time=0;
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
    // marker.pose.position.z=count_time*Constants::timeResolution;
    // marker.pose.position.z=0;
    marker.pose.position.z=t__ts[j][count_time];
    

    Eigen::Vector3d direction1,direction2,direction;
    direction1<<onpiece_state.col(0).head<2>(),0;
    direction2<<onpiece_state.col(1).head<2>(),0;
    direction2.normalized();
    Eigen::Quaterniond orientation_;
    orientation_.setFromTwoVectors(Eigen::Vector3d(1,0,0),direction2);

    // double yaw = std::atan2(direction2(1), direction2(0));
 		
		
    // if (yaw < 0) {
    //     yaw += 2 * M_PI;
    // }
    
    // marker.pose.orientation=ToQuaternion(yaw);

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
  corrids_pub_markerarray.publish(marker_array);  
// }
 int numid_=1000;
 for (int j=0;j<smallest_corrid_.size();j++)
 {
   // cout<<"33333333333333333"<<endl;

    int count_time=0;
    for(const auto onpiece_state : smallest_corrid_[j])
    {
    
    visualization_msgs::Marker marker;
    marker.header.frame_id="world";
    marker.header.stamp = ros::Time::now();
    marker.type=visualization_msgs::Marker::CUBE;
    // marker.type=visualization_msgs::Marker::SPHERE;

    marker.action=visualization_msgs::Marker::ADD;
    marker.id=numid_;
    Eigen::Vector2d _center=(onpiece_state.col(0).tail<2>()+onpiece_state.col(1).tail<2>()+onpiece_state.col(2).tail<2>()+onpiece_state.col(3).tail<2>())/4;
    double dy=std::sqrt(std::pow(onpiece_state.col(0)(2)-onpiece_state.col(1)(2),2)+std::pow(onpiece_state.col(0)(3)-onpiece_state.col(1)(3),2));
    double dx=std::sqrt(std::pow(onpiece_state.col(2)(2)-onpiece_state.col(1)(2),2)+std::pow(onpiece_state.col(2)(3)-onpiece_state.col(1)(3),2));
    // double dy=std::abs(onpiece_state.col(0)(3)-onpiece_state.col(1)(3));
    // double dx=std::abs(onpiece_state.col(2)(2)-onpiece_state.col(1)(2));
    marker.pose.position.x=_center(0);
    marker.pose.position.y=_center(1);
    marker.pose.position.z=count_time*Constants::timeResolution;
    // marker.pose.position.z=0;
    marker.pose.position.z=t__ts[j][count_time];
    

    Eigen::Vector3d direction1,direction2,direction;
    direction1<<onpiece_state.col(0).head<2>(),0;
    direction2<<onpiece_state.col(1).head<2>(),0;
    direction2.normalized();
    Eigen::Quaterniond orientation_;
    orientation_.setFromTwoVectors(Eigen::Vector3d(1,0,0),direction2);

    // double yaw = std::atan2(direction2(1), direction2(0));
 		
		
    // if (yaw < 0) {
    //     yaw += 2 * M_PI;
    // }
    
    // marker.pose.orientation=ToQuaternion(yaw);

    marker.pose.orientation.w=orientation_.w();
    marker.pose.orientation.x=orientation_.x();
    marker.pose.orientation.y=orientation_.y();
    marker.pose.orientation.z=orientation_.z();
    // marker.pose.orientation.w=1.0;
    // marker.pose.orientation.x=0.0;
    // marker.pose.orientation.y=0.0;
    // marker.pose.orientation.z=0.0;
    if(dx==0)
    {
        dx=0.01;
    }
    if(dy==0)
    {
        dy=0.01;
    }
    marker.scale.x=dx;
    marker.scale.y=dy;
 


    marker.scale.z=Constants::timeResolution;

    marker.color.r=j%3;
    marker.color.g=(j/2)%2;

    marker.color.b=(j/4)%3;
    marker.color.a=0.5;
    small_marker_array.markers.push_back(marker);
    numid_=numid_+1;
    count_time=count_time+1;
    // corrids_pub_markerarray.publish(marker_array);

    }

 }

// for (int kk=0;kk<100;kk++)
// {
  small_corrids_pub_markerarray.publish(small_marker_array);  
}
geometry_msgs::Point toGeometryMsgPoint(const Eigen::Vector3d& vec) {
    geometry_msgs::Point p;
    p.x = vec.x();
    p.y = vec.y();
    p.z = vec.z();
    return p;
}
void pub_corrid_OBB()
{

    visualization_msgs::MarkerArray marker_array;  // 用于存储所有安全走廊的标记
    // std::cout<<"pub all_corrid_.size() "<<all_corrid_.size()<<std::endl;
    // std::cout<<"pub all_corrid_[0].size() "<<all_corrid_[0].size()<<std::endl;

    int numid = 0;  // 用于唯一标识每个安全走廊
    // 遍历所有车辆的安全走廊
    for (int j = 0; j < all_corrid_.size(); j++) {
         int count_time=0;
        for (const auto& corridor : all_corrid_[j]) {
            // 创建Marker消息
            visualization_msgs::Marker corridor_marker;
            corridor_marker.header.frame_id = "world";  // 或者你使用的其他坐标系
            corridor_marker.header.stamp = ros::Time::now();
            corridor_marker.ns = "safe_corridors";
            corridor_marker.id = numid++;  // 为每个安全走廊分配一个唯一的ID
            corridor_marker.type = visualization_msgs::Marker::LINE_LIST;
            corridor_marker.action = visualization_msgs::Marker::ADD;
            corridor_marker.scale.x = 1.0;  // 设置线段的宽度
            corridor_marker.color.r=j%2;
            corridor_marker.color.g=(j/2)%2;

            corridor_marker.color.b=(j/4)%2;
            corridor_marker.color.a=0.5;
            Eigen::Vector3d p1 ;
            Eigen::Vector3d p2 ;
            Eigen::Vector3d p3 ;
            Eigen::Vector3d p4 ;
            // 假设每个矩形是4个点，依次连接这些点
            p1.head(2)=corridor.col(0).tail<2>();
            p2.head(2)=corridor.col(1).tail<2>();
            p3.head(2)=corridor.col(2).tail<2>();
            p4.head(2)=corridor.col(3).tail<2>();
            p1(2)=t__ts[j][count_time];
            p2(2)=t__ts[j][count_time];
            p3(2)=t__ts[j][count_time];
            p4(2)=t__ts[j][count_time];
            corridor_marker.pose.orientation.x = 0.0;
            corridor_marker.pose.orientation.y = 0.0;
            corridor_marker.pose.orientation.z = 0.0;
            corridor_marker.pose.orientation.w = 1.0;  // 单位四元数


            // 连接四个点：p1 -> p2, p2 -> p3, p3 -> p4, p4 -> p1
            corridor_marker.points.push_back(toGeometryMsgPoint(p1));  // p1 -> p2
            corridor_marker.points.push_back(toGeometryMsgPoint(p2));

            corridor_marker.points.push_back(toGeometryMsgPoint(p2));  // p2 -> p3
            corridor_marker.points.push_back(toGeometryMsgPoint(p3));

            corridor_marker.points.push_back(toGeometryMsgPoint(p3));  // p3 -> p4
            corridor_marker.points.push_back(toGeometryMsgPoint(p4));

            corridor_marker.points.push_back(toGeometryMsgPoint(p4));  // p4 -> p1
            corridor_marker.points.push_back(toGeometryMsgPoint(p1));

            // 将当前的走廊标记添加到MarkerArray中
            marker_array.markers.push_back(corridor_marker);
            count_time=count_time+1;
        }
    }

    // 发布整个MarkerArray
    marker_pub_OBB.publish(marker_array);
}
 

    // marker.scale.z=Constants::timeResolution;


    

    // corrids_pub_markerarray.publish(marker_array);



void timerCallback(const ros::TimerEvent &e, std::vector<Agent>& agents, Timepub& pub){
    
    if (!pub.success)
        return;
    VectorXd tm_pose(5);
    // 当前节点位置
    // for (int i = 0; i <4; i++)
    //     poly_pubs[i].publish(myPolygons[i]);
    // std_msgs::Bool flag_succ;
    // flag_succ.data=pub.success;

    int num = 0;
    for (int i = 0; i < agents.size(); i++){
        pub.paths_pub[i].publish(AgentPaths[i]);
        if (PoseBag[i].empty()) 
            continue;
        tm_pose = PoseBag[i].front();
        // 发布无人车模型
        visualization_msgs::Marker meshROS;
        mesh_gen(meshROS, i, tm_pose);
        Dynamic_path_gen(DAgentPaths[i], i, tm_pose);
        pub.Car_Mesh_pub[i].publish(meshROS);
        pub.Dynamic_paths_pub[i].publish(DAgentPaths[i]);
        PoseBag[i].pop_front();
        geometry_msgs::PoseStamped curpose;
        curpose.header.frame_id = "world";
        curpose.header.seq = i;
        curpose.header.stamp = ros::Time::now();
        curpose.pose.position.x = tm_pose(1);
        curpose.pose.position.y = tm_pose(2);
        curpose.pose.position.z = 0.1;//时间tm_pose(0)
        curpose.pose.orientation.x= tm_pose(3);//yaw
        curpose.pose.orientation.y= tm_pose(4);//速度
        curpose.pose.orientation.z= i;//
        pose_pubs[i].publish(curpose);
    }
    // pubflag.publish(flag_succ);

    pubSensedPoints(tm_pose(0));
    // pub_corrid();
    pub_corrid_OBB();
    // these_safe_corridor.visualization_corridor();
}


/*
* 设置地图图片
* func：picturedeal
* param file：文件路径
*/
void picturedeal(std::string file, std::shared_ptr<Constants::map>& map){
    if(!file.empty()){
        cv::Mat iamge,gray,dst;
        iamge = cv::imread(file);
        cv::cvtColor(iamge, gray, cv::COLOR_BGR2GRAY);//先转为灰度图
        cv::threshold(gray, dst, 0, 255, cv::THRESH_OTSU);
        Constants::map map_temp = Constants::map(Constants::size(dst.cols,dst.rows));
        for(int i = 0; i< map_temp.info.width; i++)
            for(int j = 0; j< map_temp.info.height; j++)
                map_temp.data[j * map_temp.info.width + i] = (int)dst.at<uchar>(j, i);
        map =std::make_shared<Constants::map>(map_temp);
    }
}
void path_yaml(YAML::Node config,std::vector<Agent>& agents){
    for(auto node: config["schedule"]){
        for (auto it = node.begin(); it != node.end(); ++it) {
            // 获取key
            std::string key = it->first.as<std::string>();
            // 获取value，假设value还是YAML::Node
            YAML::Node value = it->second;
            // 对value进一步处理
            Agent  agent;         
            for(auto s: value){
                State  state = State(s["x"].as<double>(),s["y"].as<double>(),s["yaw"].as<double>()
                ,s["v"].as<double>(),s["t"].as<int>());
                agent.solution.states.emplace_back(state);
            }
            agents.emplace_back(agent);
        }
    }
}
void show_yaml_read(std::string filename,std::vector<Agent> &agents){
    YAML::Node config = YAML::LoadFile(filename);
    path_yaml(config,agents);
}
void auto_input(std::string input_file,std::vector<Agent>& agents, std::string& mapfile, int& agv_count){
    // 初始化所有agent
     
    YAML::Node config = YAML::LoadFile(input_file);
    for (auto a:config["agents"]) {
       
        auto single_model = std::make_shared<AckermannModel>(a["model"]["lb"].as<double>(), a["model"]["lf"].as<double>(),
                                a["model"]["width"].as<double>());
        const auto& goal = a["goal"];
        const auto& start = a["start"];
        Agent single_anget;
        single_anget = Agent(State(start[0].as<double>(), start[1].as<double>(),
            start[2].as<double>()),State(goal[0].as<double>(), goal[1].as<double>(),
            goal[2].as<double>()),single_model);
        single_anget.start.time = 0;
        single_anget.goal.time = 0;
        single_anget.model->index = a["index"].as<int>();
        agents.emplace_back(single_anget);
    }
    agv_count = agents.size();
    // mapfile = config["map"].as<std::string>();
}
/*
* 读取yaml 自动输入
*/




template <typename T1, typename T2>
class Listener{
public:
    Listener(ros::NodeHandle nh,std::string name):
                car_name(name), car_handle(nh){
        car_subscriber=car_handle.subscribe(car_name,1,&Listener::msgCallback,this);
    }
    std::string car_name;
    ros::NodeHandle car_handle;
    ros::Subscriber car_subscriber;
    T1 p;
    virtual void msgCallback(const T2 &msg)  = 0;
    T1 data(){
        return p;
    }

};
// output to file
class CBS_Listener: public Listener<std::vector<Agent>,geometry_msgs::PoseStamped::ConstPtr>{
public:
    CBS_Listener(ros::NodeHandle nh_,std::vector<Agent> &agents_):Listener(nh_,"/move_base_simple/goal"),
    agents(agents_){};
    void msgCallback(const geometry_msgs::PoseStampedConstPtr &point){
        if (num % 2 == 0){
             if(num == 0){
                inputout = std::ofstream("/home/rancho/1hmy/kinematic-cbs/output/input.yaml");
                inputout << "agents:" << std::endl;};
            
            auto single_model = std::make_shared<AckermannModel>(1.3, 4,3.5);
            
            Agent single_anget(single_model);
            single_anget.model->index = int(num/2);
            
            double yaw = normalizeHeadingRad(ToEulerAngles(point->pose.orientation)+M_PI/2);
            inputout <<"- start: [" <<point->pose.position.x<<" , "<<point->pose.position.y<<" , "
             <<yaw << "]" << std::endl;
            single_anget.start = State(point->pose.position.x,point->pose.position.y,yaw);
            agents.emplace_back(single_anget);
            inputout <<"  index: " <<int(num/2) << std::endl;
            inputout <<"  model:" << std::endl;
            inputout <<"    type: Ackerman" << std::endl;
            inputout <<"    lf: 4" << std::endl;
            inputout <<"    lb: 1.3" << std::endl;
            inputout <<"    width: 3.5" << std::endl;
            inputout <<"    delta: 0.785398" << std::endl;
        }
        else{
            double yaw = normalizeHeadingRad(ToEulerAngles(point->pose.orientation)+M_PI/2);
            agents[int(num/2)].goal = State(point->pose.position.x,point->pose.position.y,
                            yaw);
            inputout <<"  goal: [" <<point->pose.position.x<<" , "<<point->pose.position.y<<" , "
             <<yaw << "]" << std::endl;
            std::cout << "goal" << agents[int(num/2)].goal<<std::endl;
        }
        num++;
    }
    std::vector<Agent> &agents;
    int num = 0;
    std::ofstream inputout;
    
};


/*
* 输出到yaml文件
*/
void output_toyaml(std::string outputfile, std::string outputfile_1,std::string outputfile_2,std::vector<Agent>  &agents,std::string outputfile_corid,double mapresolution,double origin_x,double origin_y){
    std::ofstream out(outputfile);
    // output to file
    out << "statistics:" << std::endl;
    out <<"Total_Plan_Time: " <<(double)(endTime - startTime) / 10e6 << "ms" << std::endl;
    out << "  Total Expand high-level nodes: " << conflict_num << std::endl;
    out << "  Average Low-level-search time: " <<  (double)(endTime - startTime) / 10e6/((double)(conflict_num + agents.size())) << std::endl;
    out << "schedule:" << std::endl;  
    for (size_t a = 0; a < agents.size(); ++a) 
    {
        out << "- agent" << a << ":" << std::endl;
        // int count=0;
        for (const auto& state : agents[a].solution.states) {
        //     if(count%5==0)
        //    {
                out << "    - x: " << state.x<< std::endl
                << "      y: " << state.y << std::endl
                << "      yaw: " << state.yaw << std::endl
                << "      v: " << state.v << std::endl
                << "      t: " << state.time << std::endl;
                // << "      t: " << state.time*0.0689 << std::endl;


        //    } 
                // count++;
        }
    
       
    }
  
    out.close(); 

    std::ofstream out2(outputfile_corid);
    // output to file
    out2 << "schedule:" << std::endl;  
    for (size_t a = 0; a < all_corrid_.size(); ++a) 
    {
        out2 << "- agent" << a << ":" << std::endl;
        int count=0;
        for (const auto& corrid_state : all_corrid_[a]) {
        //     if(count%5==0)
        //    {
                out2 << "    - cor00: " << corrid_state.col(0)(0)<< std::endl
                << "      cor01: " << corrid_state.col(0)(1) << std::endl
                << "      cor02: " << corrid_state.col(0)(2) << std::endl
                << "      cor03: " << corrid_state.col(0)(3) << std::endl
                << "      cor10: " << corrid_state.col(1)(0) << std::endl
                << "      cor11: " << corrid_state.col(1)(1) << std::endl
                << "      cor12: " << corrid_state.col(1)(2) << std::endl
                << "      cor13: " << corrid_state.col(1)(3) << std::endl
                << "      cor20: " << corrid_state.col(2)(0) << std::endl
                << "      cor21: " << corrid_state.col(2)(1) << std::endl
                << "      cor22: " << corrid_state.col(2)(2) << std::endl
                << "      cor23: " << corrid_state.col(2)(3) << std::endl
                << "      cor30: " << corrid_state.col(3)(0) << std::endl
                << "      cor31: " << corrid_state.col(3)(1) << std::endl
                << "      cor32: " << corrid_state.col(3)(2) << std::endl
                << "      cor33: " << corrid_state.col(3)(3) << std::endl
                << "      t: " << t__ts[a][count] << std::endl;
                // << "      t: " << state.time*0.0689 << std::endl;


        //    } 
                count++;
        }
    
       
    }
  
    out2.close(); 
    
    std::ofstream out1(outputfile_1);

    out1<< "agents:" << std::endl; 
       for (size_t a = 0; a < agents.size(); ++a) 
    {
        out1 << "- start: [" << agents[a].solution.states[0].x *mapresolution+origin_x<< " , "<< agents[a].solution.states[0].y*mapresolution+origin_y<<" , "<< agents[a].solution.states[0].yaw <<"]" << std::endl;
        // int count=0;
            out1 <<"  index: " <<a<< std::endl;
            out1 <<"  model:" << std::endl;
            out1 <<"    type: Ackerman" << std::endl;
            out1 <<"    lf: " <<Constants::ackerLF<< std::endl;
            out1 <<"    lb: " <<Constants::ackerLB<< std::endl;
            out1 <<"    width: " <<Constants::ackerWidth<< std::endl;
            out1 <<"    delta: "<<Constants::deltat << std::endl;
        out1 << "  goal: [" << agents[a].solution.states.back().x*mapresolution+origin_x << " , "<< agents[a].solution.states.back().y*mapresolution+origin_y<<" , "<< agents[a].solution.states.back().yaw <<"]" << std::endl;
       
    }
    out1.close(); 
}





/*
* 将结果pub出去
*/

void output_toshow(std::vector<Agent> agents){
    geometry_msgs::PoseStamped curremt_pose;
    curremt_pose.header.stamp = ros::Time::now();
    curremt_pose.header.frame_id = "world";
    for (int i = 0; i < agents.size(); i++){
        std::deque<VectorXd> Posebag;
        AgentPaths[i].header.stamp = ros::Time::now();
        AgentPaths[i].header.frame_id = "world";
        DAgentPaths[i].header.stamp = ros::Time::now();
        DAgentPaths[i].header.frame_id = "world";
        for (const auto& state : agents[i].solution.states) {
            VectorXd tm_vec(5);
            tm_vec(0) = state.time;
            tm_vec(1) = state.x;
            tm_vec(2) = state.y;
            tm_vec(3) = normalizeHeadingRad(state.yaw);
            tm_vec(4) = state.v;

            Posebag.push_back(tm_vec);
            curremt_pose.pose.position.x = state.x;
            curremt_pose.pose.position.y = state.y;
            curremt_pose.pose.position.z = 0;//state.time
            curremt_pose.pose.orientation= ToQuaternion(tm_vec(3));
            AgentPaths[i].poses.push_back(curremt_pose);
        }
        PoseBag.push_back(Posebag);
    }
    

}



/*
* 将结果pub出去
*/
void output_topub(ros::NodeHandle &nh,std::vector<Agent>  &agents, Timepub &pub,std::string pcd_file0,std::string pcd_file1,std::vector<std::vector<Eigen::MatrixXd>> all_corrid,std::vector<std::vector<double>> t_ts,std::vector<std::vector<Eigen::MatrixXd>> smallest_corrid){
    DAgentPaths = new nav_msgs::Path[agents.size()];
    AgentPaths = new nav_msgs::Path[agents.size()];
    _all_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/local_cloud_back", 10);
    _all_map_pub_trac = nh.advertise<sensor_msgs::PointCloud2>("/local_cloud_trac", 10);
    _all_map_pub_local = nh.advertise<sensor_msgs::PointCloud2>("/local_cloud_map", 10);
    corrids_pub_markerarray = nh.advertise<visualization_msgs::MarkerArray>("/corrids", 10);
    marker_pub_OBB = nh.advertise<visualization_msgs::MarkerArray>("/corridor_markers_OBB", 10);

    small_corrids_pub_markerarray=nh.advertise<visualization_msgs::MarkerArray>("/small_corrids", 10);
//     if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file0, *pcd_cloud_back) == -1) {
//   PCL_ERROR("Couldn't read file0: %s \n", pcd_file0.c_str());
  
// }
//   if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file1, *pcd_cloud_trac) == -1) {
//   PCL_ERROR("Couldn't read file1: %s \n", pcd_file1.c_str());
// // Environment  mapf_box(grid, agents);
// // these_safe_corridor(mapf_box,agents,nh);
// // safe_corridor.getRectangleConst_ts();
// }
  pcl::getMinMax3D(*pcd_cloud_trac,pointmin,pointmax);
  all_corrid_=all_corrid;
  smallest_corrid_=smallest_corrid;
  t__ts=t_ts;

    for (size_t a = 0; a < agents.size(); ++a) {
        std::string str1 = "/path/agent";
        std::string str2 = "/Dpath/agent";
        std::string str3 = "/CarMesh/agent";
        pub.paths_pub[a] = nh.advertise<nav_msgs::Path>(str1.append(std::to_string(a)), 2);  
        pub.Dynamic_paths_pub[a] = nh.advertise<nav_msgs::Path>(str2.append(std::to_string(a)), 2);    
        pub.Car_Mesh_pub[a] = nh.advertise<visualization_msgs::Marker>(str3.append(std::to_string(a)), 2);        
    }
    output_toshow(agents);

    for(int posebagcount=0;posebagcount<PoseBag.size();posebagcount++)
    {
       if(PoseBag[posebagcount].size()>posebagmaxsize) 
       {
        posebagmaxsize=PoseBag[posebagcount].size();
        posebagid=posebagcount;
       }
    }
}






/*
* search 主要程序
*/
bool pathfinding(std::shared_ptr<Constants::map> grid, std::vector<Agent>  &agents, int batchSize){
    bool success;
    std::multimap<int, std::pair<State, std::shared_ptr<Model>>> dynamic_obstacles;
    
    startTime = ros::Time::now().toNSec();
    clock_t batch_start_time  = ros::Time::now().toNSec();
    for (size_t iter = 0; iter < (double)(agents.size() )/ batchSize; iter++) {
        clock_t batch_start_time  = ros::Time::now().toNSec();
        size_t first = iter * batchSize;
        size_t last = first + batchSize;
        if (last >= agents.size()) last = agents.size();
        std::vector<Agent> m_agents(agents.begin() + first, agents.begin() + last);
        // 把剩下的未计算机器人的终点加入到dynamic_obstacles中
        for (auto a = agents.begin() + last; a != agents.end(); a++) {
        dynamic_obstacles.insert(
            std::pair<int, std::pair<State, std::shared_ptr<Model>>>(
                -1, std::make_pair(State(a->goal.x, a->goal.y, a->goal.yaw),
                                    a->model)));
        }
        Environment  mapf(dynamic_obstacles,grid, m_agents);
        CL_CBS<State, Action, double, Conflict, Constraints,Environment>
                        cbsHybrid(mapf,1.5);
        success = cbsHybrid.search();

        if (!success) {
            std::cout << "\033[1m\033[31m No." << iter << "iter fail to find a solution \033[0m\n";
            break;
        }
        // 若success
        else {
            endTime = ros::Time::now().toNSec();
            std::cout << "[Plan]{sucess}  Time in plan is " <<(double)(endTime - batch_start_time) / 10e6 << "ms" <<std::endl;
            std::cout << "\n Complete No. " << iter + 1
                      << "\n batch. Runtime:" << (double)(endTime - batch_start_time) / 10e6 
                      << "\n Expand high-level nodes:" << mapf.highLevelExpanded()
                      << "\n Average Low-level-search time:"
                      << (double)(endTime - batch_start_time) / 10e6 /
                             (mapf.highLevelExpanded() + agents.size())
                      << std::endl;
            conflict_num = conflict_num + mapf.highLevelExpanded() - 1;
        for (auto a:m_agents) {
            for (const auto& state : a.solution.states)
            dynamic_obstacles.insert(
                std::pair<int, std::pair<State, std::shared_ptr<Model>>>(
                    state.time,
                    std::make_pair(State(state.x, state.y, state.yaw),
                        a.model)));
            State lastState = a.solution.states.back();
            dynamic_obstacles.insert(std::pair<int, std::pair<State, std::shared_ptr<Model>>>(
                    -lastState.time,
                    std::make_pair(State(lastState.x, lastState.y, lastState.yaw),
                                    a.model)));

        }
        for (size_t i = 0; i < agents.size(); i++){
            for (auto m:m_agents) {
                if(m.model->index == agents[i].model->index)
                    agents[i] = m;
            }
        }

        }
        dynamic_obstacles.erase(-1);
    }
    /*
    主体程序运行
    */
    endTime = ros::Time::now().toNSec();
    
    return success;
}

