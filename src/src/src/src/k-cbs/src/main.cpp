#include <ros/ros.h>
#include <ros/console.h>
#include "path_cal.hpp"
#include "std_msgs/Bool.h"

int main(int argc, char *argv[]){
    
    //以下为添加部分，初始化ros节点
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh("~");
    std::string mapfile,inputfile,outputfile,outputfile_1,outputfile_2,outputfile_corid;
    ros::Publisher trigger_the_swarm;

    int batchSize, agv_count;
    double showspin,mapresolution,origin_x,origin_y;
    bool manual_mode,out_mode;
    std::string pcd_file_back;
    std::string pcd_file_trac;
    std::string file_directory;
    std::string file_name_back;
    std::string file_name_trac;
    nh.param("map_file", mapfile, std::string("/home/triplez/experiment/kinematic-cbs/src/input/input.yaml"));
    nh.param("input_yaml", inputfile, std::string("/home/triplez/experiment/kinematic-cbs/src/input/input.yaml"));
    nh.param("output_yaml", outputfile, std::string("/home/triplez/experiment/1.yaml")); 
    nh.param("output_yaml_1", outputfile_1, std::string("/home/triplez/experiment/1.yaml"));  
    nh.param("output_yaml_2", outputfile_2, std::string("/home/triplez/experiment/1.yaml"));   
    nh.param("outputfile_corid", outputfile_corid, std::string("/home/triplez/experiment/1.yaml"));     

    nh.param("file_directory", file_directory, std::string("/home/"));
    nh.param("file_name_back", file_name_back, std::string("map0"));
    nh.param("file_name_trac", file_name_trac, std::string("map"));
    nh.param("batchSize", batchSize, 10);
    nh.param("agent_num", agv_count, 10);
    nh.param("manual_mode", manual_mode, true);
    nh.param("out_mode", out_mode, true);
    nh.param("showspin", showspin, 0.05);// 动画显示频率

    nh.param("mapresolution", mapresolution, 0.05);// 动画显示频率
    nh.param("origin_x", origin_x, 0.05);// 动画显示频率
    nh.param("origin_y", origin_y, 0.05);// 动画显示频率

    std::vector<Agent>  agents; 
    const std::string pcd_format = ".pcd";
    pcd_file_back = file_directory + file_name_back + pcd_format;
    pcd_file_trac = file_directory + file_name_trac + pcd_format;
    trigger_the_swarm=nh.advertise<std_msgs::Bool>("/trigger_swarm", 10);

    
    if(!manual_mode){
        // 自动读取yaml 输入起始点终点
        auto_input(inputfile,agents,mapfile, agv_count);
        agv_count = agents.size();
        std::cout<< "starting " <<std::endl;
    }
    
    else{
        CBS_Listener listener(nh,agents);
        // 手动创建一个Subscriber，订阅名为/turtle/pose的topic,注册回调函数poseCallback
        
        ros::Rate r(1);   
        while (ros::ok()){
            if (listener.num >= agv_count * 2){
                std::cout<< "starting " <<std::endl;
                listener.inputout.close();
                break;
            }
            ros::spinOnce();                
            r.sleep();
        }
    }
    
    bool success;
    Timepub pub;
    pub.success = false;
    
    std::shared_ptr<Constants::map> grid;
    picturedeal(mapfile, grid);
    success = pathfinding(grid, agents, batchSize);
    
   
    ros::Publisher paths_pub[agv_count],Car_Mesh_pub[agv_count],Dynamic_paths_pub[agv_count];

    for (int j = 0; j < agv_count; ++j) {
    ros::Publisher pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/dynamic/pose_" + std::to_string(j), 10);
    pose_pubs.push_back(pose_pub);
  }
    
    if(success){
        std::cout<<"cbs success"<<std::endl;
        // std_msgs::Bool flag_succ;
        // flag_succ.data=success;
        
        std::cout<<"start safe_corridor"<<std::endl;
        // SAFE BOX
            Environment  mapf_box(grid, agents);
            Safe_Corridor  safe_corridor(mapf_box,agents,nh);
            safe_corridor.getRectangleConst_ts();
            // safe_corridor.getRectangleConst_ts_OBB();
            // safe_corridor.getRectangleConst_ts_zhouxin();


            // safe_corridor.CheckRectangleConst_ts();

            // safe_corridor.visualization_corridor();
        //




        pub.Car_Mesh_pub = Car_Mesh_pub;
        pub.Dynamic_paths_pub = Dynamic_paths_pub;
        pub.paths_pub = paths_pub;
        output_topub(nh, agents, pub,pcd_file_back,pcd_file_trac,safe_corridor.all_corrid_,safe_corridor.t_ts,safe_corridor.smallest_all_corrid_);
        // trigger_the_swarm.publish(flag_succ);
        if(out_mode)
            output_toyaml(outputfile, outputfile_1,outputfile_2,agents,outputfile_corid,mapresolution,origin_x,origin_y);
            
        
        pub.success = success;

    }
    ros::Timer Ttimer = nh.createTimer(ros::Duration(showspin), 
                        boost::bind(&timerCallback, _1, agents, pub));

    
    // while(ros::ok())
        


        

    ros::spin();        
    return 0;
}