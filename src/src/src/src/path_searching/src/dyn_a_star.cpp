#include "path_searching/dyn_a_star.h"
#include "path_searching/car_swarm.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include "nav_msgs/Path.h"


// using namespace std;
// ros::Subscriber a_localpath;

int car_id_;
int car_num;

// car_qun::Ptr swarm_loacl_traj;
// AStar::Ptr a_star_;

// void local_pathcallback(const nav_msgs::Path path)
// {
// swarm_loacl_traj->car_local_trajpath.push_back(path);

// }

// void car_qun::init() {
//   /* get param */
//   // int queue_size, skip_nums;
//   // ros::Subscriber pose_sub = node_handle_.subscribe<geometry_msgs::PoseStamped>(
//   //       "/dynamic/pose_" + std::to_string(i), 10, &car_qun::poseCallback, obj_his.get());

//   // pose_subs_.push_back(pose_sub);

//   // predict_trajs_->at(i).setGlobalStartTime(t_now);


//   /* update prediction */
//   // predict_timer_ =
//   //     node_handle_.createTimer(ros::Duration(1 / predict_rate_), &ObjPredictor::predictCallback, this);
// }
void replan()
{
//a*与ego全局，a*与egoa*
cout<<"1111111111111111111"<<endl;
}
void chackego_swarm_collide(int carid, Pose_Listener *cars_position)
{

}
int main(int argc, char **argv)
    {
      ros::init(argc, argv, "decenplan");//启动该节点并设置其名称，名称必须唯一  
      ros::NodeHandle nh("~");

      nh.param("car_id", car_id_, 10);
      nh.param("car_num", car_num, 10);
     
      Pose_Listener *cars_position[car_num];
        // 开多线程接收
      ros::AsyncSpinner spinner(car_num*2); 
      for(int i = 0; i < car_num; i++){
        std::string str6 = "/dynamic/pose_";
        cars_position[i] = new Pose_Listener(nh,str6.append(std::to_string(i)));
      }


   while (ros::ok()){
   chackego_swarm_collide(car_id_,*cars_position);
   ros::Duration(0.2).sleep();
    spinner.start();
   }
     return 0;
    }
