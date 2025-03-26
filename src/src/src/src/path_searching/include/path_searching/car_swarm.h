#ifndef _CAR_SWARM_H_
#define _CAR_SWARM_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "opencv2/opencv.hpp"
#include <boost/heap/fibonacci_heap.hpp>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include "nav_msgs/Path.h"


using namespace std;
using namespace Eigen;


 struct OneTrajDataOfSwarm
  {
    /* info of generated traj */

    int car_id;
    // double duration_;
	// double start_time_;
    // ros::Time start_time_;
    Eigen::Vector3d start_pos_;
    Eigen::Vector3d end_pos_;

    vector<Eigen::Vector3d> local_astar_traj_;
  };
typedef std::vector<OneTrajDataOfSwarm> SwarmTrajData;

class car_qun
{
private:
  ros::NodeHandle node_handle_;
public:
  SwarmTrajData swarm_a_path;
  OneTrajDataOfSwarm local_a_path;
  std::vector<nav_msgs::Path> car_local_trajpath;
public:
	typedef std::shared_ptr<car_qun> Ptr;
	car_qun(){};
  car_qun(ros::NodeHandle& node){this->node_handle_ = node;};
	~car_qun(){};
  void init();

};
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
    std::vector<T1> his_traj;
    
    virtual void msgCallback(const T2 &msg)  = 0;
    T1 data(){
        return p;
    }

    virtual void his_traj_clean()  = 0;
};


class Pose_Listener: public Listener<Eigen::Vector3d, geometry_msgs::PoseStamped::ConstPtr>{
public:
    Pose_Listener(ros::NodeHandle nh, std::string name):
      Listener(nh,name){};
    void msgCallback(const geometry_msgs::PoseStamped::ConstPtr &curpose){
        p(0) = curpose->pose.position.x;
        p(1) = curpose->pose.position.y;
        p(2) = curpose->pose.position.z;
        his_traj.push_back(p);
    }
    void his_traj_clean()
    {
      his_traj.clear();

    }
};
#endif