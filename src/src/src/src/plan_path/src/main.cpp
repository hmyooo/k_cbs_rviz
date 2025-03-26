#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <string>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "visualization_msgs/Marker.h"
#include <vector>
#include <Eigen/Eigen>
#include <deque>
#include <sstream>
#include "yaml-cpp/yaml.h"
#include <fstream>
#include "state.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <tf/tf.h>

typedef std::vector<std::pair<double, double>> CandidateSol;


std::vector<std::vector<CandidateSol>>  candidatesolutions;

using Eigen::VectorXd;
using Eigen::Vector4d;

using Eigen::Vector3i;
using Eigen::Vector3d;
using std::string;
//*visual relative
static string car_mesh_resource;
int color_r = 0, color_g = 0, color_b = 1.0;
static double color_a = 0.8;
bool stop_ = true;
std::vector<visualization_msgs::Marker> Agentmesh;
std::vector<visualization_msgs::Marker> obstaclemesh;
visualization_msgs::Marker meshROS,obstacleROS;

geometry_msgs::PoseStamped current_pose;
double showspin;

string _frame_id;
string recover_yaml,avoid_yaml;
string inputFile,candidate_yaml;

// 动态路径数据包
std::vector<std::deque<VectorXd>> PoseBag;
std::deque<VectorXd> Posebag;

std::vector<nav_msgs::Path> AGVPath, RecoverPath, AvoidPath;
std::vector<nav_msgs::Path> candidatePath;

nav_msgs::Path AGVpath;

Vector3d ackmodel;
std::vector<nav_msgs::Path> AGVTraj;

std::vector<nav_msgs::Path> InitialPaths;
nav_msgs::Path InitialPath;
nav_msgs::Path path_tmp;
std::vector<ros::Publisher> poly_pubs;
// 定义多边形对象
std::vector<geometry_msgs::PolygonStamped> myPolygons;

//添加无人机个数
// candidate patg pub
std::vector<ros::Publisher> candidate_Pub;

ros::Publisher candidate_pub;

std::vector<ros::Publisher> Spath_Pub;
ros::Publisher Spath_pub;


std::vector<ros::Publisher> D_recoverpath_Pub;
ros::Publisher D_recoverpath_pub;

std::vector<ros::Publisher> D_avoidpath_Pub;
ros::Publisher D_avoidpath_pub;

std::vector<ros::Publisher> Dpath_Pub;
ros::Publisher Dpath_pub;

std::vector<ros::Publisher> mesh_Pub;
ros::Publisher mesh_pub;
std::vector<Vector3i> colors = {Vector3i(255,85,0), Vector3i(0,85,0), Vector3i(0,85,127), Vector3i(0,0,255), Vector3i(85,0,255)};

// pub 障碍物
std::vector<ros::Publisher> obs_Pub;
ros::Publisher obs_pub;


std::vector<std::vector<State>> agents;
std::vector<Vector4d> obstacles;
std::map<int,std::vector<State>> recover_agents,avoid_agents;
std::map<int,State>  recover_starts,avoid_starts;

int agv_count = 0;
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

    return q;
}

//*发布rviz可视化信息*//
void timerCallback(const ros::TimerEvent &e){
    
    if (stop_)
        return;
    VectorXd tm_pose(5);
    color_r = 0, color_g = 0, color_b = 0;
    tm_pose = PoseBag[0].front();
    for (int i = 0; i <4; i++)
        poly_pubs[i].publish(myPolygons[i]);
    if(tm_pose(0) >= obstacles[0](3)){
        for (int j = 0; j < obstacles.size(); j++){
            obstacleROS.header.stamp = ros::Time::now();
            obstacleROS.id = j;  
            obstacleROS.type = visualization_msgs::Marker::CYLINDER;
            obstacleROS.action = visualization_msgs::Marker::ADD;
            obstacleROS.scale.x = obstacles[j](2)*2.5;
            obstacleROS.scale.y = obstacles[j](2)*2.5;
            obstacleROS.scale.z = 5;
            obstacleROS.color.a = 1;
            obstacleROS.color.r = 1.0;
            obstacleROS.color.g = 0;
            obstacleROS.color.b = 0;
            obstacleROS.header.frame_id = _frame_id;
            obstacleROS.ns = "shapes";
            obstaclemesh.push_back(obstacleROS);
            obstaclemesh[j].pose.position.x = obstacles[j](0);
            obstaclemesh[j].pose.position.y = obstacles[j](1);
            obstaclemesh[j].pose.position.z = 0;
            obstaclemesh[j].pose.orientation.w = 1;
            obstaclemesh[j].pose.orientation.x = 0;
            obstaclemesh[j].pose.orientation.y = 0;
            obstaclemesh[j].pose.orientation.z = 0;
            obs_Pub[j].publish(obstaclemesh[j]);
        }
        
    }
    int num = 0;
    // for(int i = 0; i < candidatesolutions.size();i++) {  
    //     for(int j = 0; j < candidatesolutions[i].size();j++){
    //         candidate_Pub[num].publish(candidatePath[num]);
    //         num++;
    //     } 
    // }
    for (int i = 0; i < agv_count; i++){
        if (PoseBag[i].empty()) 
            continue;
        Dpath_Pub[i].publish(AGVPath[i]);
        
        tm_pose = PoseBag[i].front();
        current_pose.header.stamp = ros::Time::now();
        current_pose.header.frame_id = "world";
        path_tmp.header.frame_id = _frame_id;
        path_tmp.header.stamp = ros::Time::now();
        AGVTraj.push_back(path_tmp);
        current_pose.pose.position.x = tm_pose(1);
        current_pose.pose.position.y = tm_pose(2);
        current_pose.pose.position.z = 0;
        AGVTraj[i].poses.push_back(current_pose);

        // if(avoid_starts.find(i) != avoid_starts.end()){
        //     std::cout<<"recover_starts"<<recover_starts[i].time<<"  avoid  "<< avoid_starts[i].time
        //     << "current "<<tm_pose(0)<<std::endl;
        //    if(tm_pose(0) >= avoid_starts[i].time && tm_pose(0) <recover_starts[i].time)
        //         D_avoidpath_Pub[i].publish(AGVTraj[i]);
        //     else if (tm_pose(0) >= recover_starts[i].time)
        //         D_recoverpath_Pub[i].publish(AGVTraj[i]);
        //     else Spath_Pub[i].publish(AGVTraj[i]);
        // }   
        // else
        Spath_Pub[i].publish(AGVTraj[i]);


        
        meshROS.header.stamp = ros::Time::now();
        meshROS.id = i;  
        meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
        meshROS.action = visualization_msgs::Marker::ADD;
        // meshROS.pose.orientation = ToQuaternion(tm_pose(2));
        meshROS.scale.x = 2.5;
        meshROS.scale.y = 2.5;
        meshROS.scale.z = 2.5;
        meshROS.color.a = color_a ;
        meshROS.color.r = colors[i][0];
        meshROS.color.g = colors[i][1];
        meshROS.color.b = colors[i][2];
        meshROS.mesh_resource = car_mesh_resource;
        meshROS.header.frame_id = _frame_id;
        meshROS.ns = "mesh";
        Agentmesh.push_back(meshROS);
        Agentmesh[i].pose.position.x = tm_pose(1);
        Agentmesh[i].pose.position.y = tm_pose(2);
        Agentmesh[i].pose.position.z = 0;
        Agentmesh[i].pose.orientation = ToQuaternion(tm_pose(3));
        mesh_Pub[i].publish(Agentmesh[i]);

        PoseBag[i].pop_front();

    }
    for (int i = 0; i < agv_count; i++){
        D_avoidpath_Pub[i].publish(AvoidPath[i]);
        D_recoverpath_Pub[i].publish(RecoverPath[i]);
        
    }

}

void yaml_read(std::string filename,std::map<int,std::vector<State>> &agents,std::map<int,State>  &starts, bool flag_ = true){
    YAML::Node config = YAML::LoadFile(filename);
    std::string agentname = "";
    State start;
    string name;
    if(flag_ == true)  name = "recover statistics";
    else  name = "avoid statistics";
    for(auto node: config[name]){
        for (auto it = node.begin(); it != node.end(); ++it) {
            // 获取key
            std::string key = it->first.as<string>();
            // 获取value，假设value还是YAML::Node
            YAML::Node value = it->second;
            if(key.find("start") != std::string::npos){          
                agentname = key;
                for(auto s: value)
                    start = State(s["x"].as<double>(),s["y"].as<double>(),s["yaw"].as<double>(),
                        s["t"].as<int>(),s["v"].as<double>());
                int pos = agentname.find("start");
                agentname = agentname.erase(pos,5); 
                continue;
            }
            if(key.find(agentname) != std::string::npos && agentname != ""){   
                std::vector<State>  single_soluution;           
                for(auto s: value){
                    State  state = State(s["x"].as<double>(),s["y"].as<double>(),s["yaw"].as<double>(),
                        s["t"].as<int>(),s["v"].as<double>());
                    single_soluution.push_back(state);
                }
                agentname.erase(agentname.begin(),agentname.begin()+5);
                int number;
                std::stringstream ss;
                ss << agentname;//可以是其他数据类型
                ss >> number; //string -> int
                starts.emplace(std::make_pair(number, start));
                agents.emplace(std::make_pair(number, single_soluution));
            }
        }
    }

}

void yaml_read_candidate(std::string filename,std::vector<std::vector<CandidateSol>> &solutions){
    YAML::Node config = YAML::LoadFile(filename);
    for (auto it = config.begin(); it != config.end(); ++it) {
        // 获取key
        std::string key = it->first.as<string>();
        // std::cout << key << std::endl;
        // 获取value，假设value还是YAML::Node
        YAML::Node value = it->second;
        std::vector<CandidateSol> candidatessss; 
        for (auto ij = value.begin(); ij != value.end(); ++ij) {
            YAML::Node path_ss = ij->second;
            CandidateSol solution;
            for(auto s:path_ss){
                std::pair<double,double> state = std::make_pair(s[0].as<double>()/10,s[1].as<double>()/10);
                // std::cout << state.first << " " << state.second <<std::endl;
                solution.push_back(state);
            }
            candidatessss.push_back(solution);
        }
        solutions.push_back(candidatessss);
    }
}


void yaml_read_input(std::string filename,std::vector<std::vector<State>> & agents,std::vector<Vector4d> & obstacles){
    YAML::Node config = YAML::LoadFile(filename);
    for(auto node: config["schedule"]){
        for (auto it = node.begin(); it != node.end(); ++it) {
            // 获取key
            std::string key = it->first.as<string>();
            // 获取value，假设value还是YAML::Node
            YAML::Node value = it->second;
            // 对value进一步处理
            std::vector<State>  single_soluution;         
            for(auto s: value){
                State  state = State(s["x"].as<double>(),s["y"].as<double>(),s["yaw"].as<double>(),
                    s["t"].as<int>(),s["v"].as<double>());
                single_soluution.emplace_back(state);
            }
            agents.emplace_back(single_soluution);
        }
        
    }
    for (auto _obs:config["obs"]) {
            Vector4d obs;
            obs[0] = _obs[0].as<double>();
            obs[1] = _obs[1].as<double>();
            obs[2] = _obs[2].as<double>();
            obs[3] = _obs[3].as<int>();
            obstacles.emplace_back(obs);
        
        
    }
    for (auto model:config["agents"]) {
        ackmodel[0] = model["model"]["lf"].as<double>();
        ackmodel[1] = model["model"]["lb"].as<double>();
        ackmodel[2] = model["model"]["width"].as<double>() - 1;    
       
    }
    
}
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



geometry_msgs::Point32* calpoints(State s){
    float fAngle = -s.yaw;
    float h = ackmodel[2];
    float w = ackmodel[0] + ackmodel[1];
    float a = (float)sin(fAngle) * 0.5f;
    float b = (float)cos(fAngle) * 0.5f;
    geometry_msgs::Point32* pts = new geometry_msgs::Point32[4];
    double cx = (w/2 -  ackmodel[1])*cos(fAngle) + s.x;
    double cy = (w/2 -  ackmodel[1])*sin(fAngle) + s.y;
    pts[0].x = (cx + a * h - b * w);
    pts[0].y = (cy - b * h - a * w);
    pts[2].x = (2 * cx - pts[0].x);
    pts[2].y = (2 * cy - pts[0].y);
    
    pts[3].x = (cx - a * h - b * w);
    pts[3].y = (cy + b * h - a * w);
    pts[1].x = (2 * cx - pts[3].x);
    pts[1].y = (2 * cy - pts[3].y);
    return pts;
}

void rviz_show_car(std::vector<std::vector<State>> &solutions, std::vector<std::vector<State>> &initial_solutions){ 
    //*rviz可视化
    geometry_msgs::PoseStamped curremt_pose;
    curremt_pose.header.stamp = ros::Time::now();
    curremt_pose.header.frame_id = "world";

    VectorXd tm_vec(5);
    int num = 0;
    // for(int i = 0; i <candidatesolutions.size(); i++){
    //     AGVpath.header.stamp = ros::Time::now();
    //     AGVpath.header.frame_id = "world";
    //     for(int j= 0; j < candidatesolutions[i].size(); j++){
    //         candidatePath.push_back(AGVpath);
    //         for (int k = 0; k < candidatesolutions[i][j].size(); k++){         
    //             auto sol = candidatesolutions[i][j];   
    //             curremt_pose.pose.position.x = sol[k].first;
    //             curremt_pose.pose.position.y = sol[k].second;
    //             curremt_pose.pose.position.z = 0;
    //             candidatePath[num].poses.push_back(curremt_pose);
    //         }
    //         num++;
    //     }
    // }

    for (int i = 0; i < agv_count;i++){
        AGVpath.header.stamp = ros::Time::now();
        AGVpath.header.frame_id = "world";
        PoseBag.push_back(Posebag);
        AGVPath.push_back(AGVpath);
        AvoidPath.push_back(AGVpath);
        RecoverPath.push_back(AGVpath);
        // PoseBag[i].clear();
        // if(i == 0){
        //     for (int j = 0; j < avoid_starts[i].time;j++){
        //     tm_vec(0) = solutions[i][j].time;
        //     tm_vec(1) = solutions[i][j].x;
        //     tm_vec(2) = solutions[i][j].y;
        //     tm_vec(3) = normalizeHeadingRad(solutions[i][j].yaw);
        //     PoseBag[i].push_back(tm_vec);
        //     };
        // }
            
        // else
        //  if(avoid_starts.find(i) != avoid_starts.end()&& i!=3){
            
        //     for (int j = 0; j < avoid_starts[i].time + 10;j++){
        //         tm_vec(0) = solutions[i][j].time;
        //         tm_vec(1) = solutions[i][j].x;
        //         tm_vec(2) = solutions[i][j].y;
        //         tm_vec(3) = normalizeHeadingRad(solutions[i][j].yaw);
        //         PoseBag[i].push_back(tm_vec);
                
                
        //     };
        //  }
        //  else
            for (int j = 0; j < solutions[i].size();j++){
                tm_vec(0) = solutions[i][j].time;
                tm_vec(1) = solutions[i][j].x;
                tm_vec(2) = solutions[i][j].y;
                tm_vec(3) = normalizeHeadingRad(solutions[i][j].yaw);
                tm_vec(4) = solutions[i][j].v;

                PoseBag[i].push_back(tm_vec);
            };
        // PoseBag[i].clear();
        for (int j = 0; j < initial_solutions[i].size();j++){    
            curremt_pose.pose.position.x = initial_solutions[i][j].x;
            curremt_pose.pose.position.y = initial_solutions[i][j].y;
            curremt_pose.pose.position.z = 0;
            AGVPath[i].poses.push_back(curremt_pose);
        };
        


        if(avoid_starts.find(i) != avoid_starts.end() && i!=3){
            geometry_msgs::PolygonStamped myPolygon;
            for (int j = avoid_starts[i].time + 10; j < recover_starts[i].time;j++){  
                curremt_pose.pose.position.x = solutions[i][j].x;
                curremt_pose.pose.position.y = solutions[i][j].y;
                curremt_pose.pose.position.z = 0;
                AvoidPath[i].poses.push_back(curremt_pose);// 多边形数据 
                     
                myPolygon.header.frame_id = "world";
                geometry_msgs::Point32* points = calpoints(solutions[i][j]);
                myPolygon.polygon.points.push_back(points[3]);
                for (int i = 0; i < 4; i++){
                    // std::cout << points[i].x <<std::endl;
                     myPolygon.polygon.points.push_back(points[i]);
                     
                }
               
                
            }
            for (int j = recover_starts[i].time - 1; j >= avoid_starts[i].time+10;j--){  
                geometry_msgs::Point32* points = calpoints(solutions[i][j]);
                myPolygon.polygon.points.push_back(points[3]);
            }
                    
            myPolygons.push_back(myPolygon);
            geometry_msgs::PolygonStamped  myPolygon1;
            myPolygon1.header.frame_id = "world";
            auto end = solutions[i][recover_starts[i].time];
            geometry_msgs::Point32 point;
            point.x = end.x - 10;
            point.y = end.y - 10;
            myPolygon1.polygon.points.push_back(point);
            point.x = end.x - 10;
            point.y = end.y + 10;
            myPolygon1.polygon.points.push_back(point);
            point.x = end.x + 10;
            point.y = end.y + 10;
            myPolygon1.polygon.points.push_back(point);
            point.x = end.x + 10;
            point.y = end.y - 10;
            myPolygon1.polygon.points.push_back(point);
            myPolygons.push_back(myPolygon1);
            for (int j = recover_starts[i].time; j < solutions[i].size();j++){    
                curremt_pose.pose.position.x = solutions[i][j].x;
                curremt_pose.pose.position.y = solutions[i][j].y;
                curremt_pose.pose.position.z = 0;
                RecoverPath[i].poses.push_back(curremt_pose);
            }
        }
        

    };  
}




int main(int argc, char *argv[])
{

    //以下为添加部分，初始化ros节点
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh("~");

    //↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓以下为918日添加↓↓↓↓↓↓↓↓↓↓//

    nh.param("mesh_resource", car_mesh_resource, string("package://path_search/meshes/car.dae"));
    nh.param("frame_id", _frame_id, string("world"));
    nh.param("showspin", showspin, 0.05);
    //添加无人机个数
    nh.param("avoid_yaml", avoid_yaml, string("/home/rancho/triplez/datadeal/path_show/src/reading_pcd/pcd/mapdeal/output/hard_regular_30(10)_recover.yaml"));
    nh.param("recover_yaml", recover_yaml, string("/home/rancho/triplez/datadeal/path_show/src/reading_pcd/pcd/mapdeal/output/hard_regular_30(10)_recover.yaml"));
    nh.param("inputFile", inputFile, string("/home/rancho/triplez/datadeal/path_show/src/reading_pcd/pcd/mapdeal/input/hard_regular_30(10).yaml"));//ymlshuru
    nh.param("candidate_yaml", candidate_yaml, string("/home/rancho/triplez/datadeal/path_show/src/reading_pcd/pcd/mapdeal/input/hard_regular_30(10).yaml"));//ymlshuru
    //↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑以上为添加↑↑↑↑↑↑↑↑↑↑↑↑//

    for(int i=0; i <4; i++) {
        string str1 = "polygonpublisher";
        ros::Publisher poly_pub;
        poly_pub = nh.advertise<geometry_msgs::PolygonStamped>(str1.append(std::to_string(i)), 1);
        poly_pubs.push_back(poly_pub);

    }
    




    ros::Timer Ttimer = nh.createTimer(ros::Duration(showspin), timerCallback);

    ros::Rate rate(10);
    bool status = ros::ok();


    

    yaml_read_input(inputFile,agents,obstacles);
    // yaml_read_candidate(candidate_yaml,candidatesolutions);
    yaml_read(avoid_yaml,avoid_agents,avoid_starts,false);
    yaml_read(recover_yaml,recover_agents,recover_starts);
    agv_count = agents.size();
    // string str1 = "/candidate/agent";
    // for(int i = 0; i < candidatesolutions.size();i++) {
    //     string str2 = str1;
    //     str2.append(std::to_string(i));
    //     str2.append("/path");
    //     std::vector<ros::Publisher> can_pub;
    //     for(int j = 0; j < candidatesolutions[i].size();j++) {
    //         string str3 = str2;
    //         candidate_pub = nh.advertise<nav_msgs::Path>(str3.append(std::to_string(j)), 2);
    //         candidate_Pub.push_back(candidate_pub);

    //     }
    // }
   
    std::vector<std::vector<State>>  initial_solutions = agents;
    for(int i = 0; i < agents.size(); i++){  
        if(recover_agents.find(i) != recover_agents.end()){
            agents[i] = recover_agents[i];
        }
        string str1 = "/Spath/agent";
        string str2 = "/Dpath/agent";
        string str3 = "/Robot/agent";
        string str4 = "/Drecoverpath/agent";
        string str5 = "/Davoidpath/agent";

        Spath_pub = nh.advertise<nav_msgs::Path>(str2.append(std::to_string(i)), 2);
        Spath_Pub.push_back(Spath_pub);

        Dpath_pub = nh.advertise<nav_msgs::Path>(str1.append(std::to_string(i)), 2);
        Dpath_Pub.push_back(Dpath_pub);

        D_avoidpath_pub = nh.advertise<nav_msgs::Path>(str4.append(std::to_string(i)), 2);
        D_avoidpath_Pub.push_back(D_avoidpath_pub);

        D_recoverpath_pub = nh.advertise<nav_msgs::Path>(str5.append(std::to_string(i)), 2);
        D_recoverpath_Pub.push_back(D_recoverpath_pub);

        mesh_pub = nh.advertise<visualization_msgs::Marker>(str3.append(std::to_string(i)), 1);
        mesh_Pub.push_back(mesh_pub);
    }
    

    //添加无人机个数
    for (int i = 0; i < obstacles.size(); i++){
        string str1 = "/Obstacle";
        obs_pub = nh.advertise<visualization_msgs::Marker>(str1.append(std::to_string(i)), 1);
        obs_Pub.push_back(obs_pub);
    }
    rviz_show_car(agents,initial_solutions);

    
        
    stop_ = false;
    ros::spin();


      return 0;
    
}
