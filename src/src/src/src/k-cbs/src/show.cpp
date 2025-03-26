#include <ros/ros.h>
#include <ros/console.h>
#include "path_cal.hpp"

void yaml_read(std::string filename,std::vector<Agent> &agents){
    YAML::Node config = YAML::LoadFile(filename);
   for(auto node: config["schedule"]){
        for (auto it = node.begin(); it != node.end(); ++it) {
            // 获取key
            std::string key = it->first.as<std::string>();
            // 获取value，假设value还是YAML::Node
            YAML::Node value = it->second;
            // 对value进一步处理
            Agent  agent;         
            for(auto s: value){
                State  state = State(s["x"].as<double>(),s["y"].as<double>(),s["yaw"].as<double>(),
                    s["t"].as<int>(),s["v"].as<double>());
                agent.solution.states.emplace_back(state);
            }
            agents.emplace_back(agent);
        }
        
    }
}


int main(int argc, char *argv[]){
    
    //以下为添加部分，初始化ros节点
    ros::init(argc, argv, "show_node");
    ros::NodeHandle nh("~");
    std::string mapfile,inputfile;
    double showspin;
    std::string pcd_file_back;
    std::string pcd_file_trac;
    std::string file_directory;
    std::string file_name_back;
    std::string file_name_trac;
    nh.param("map_file", mapfile, std::string("/home/triplez/experiment/kinematic-cbs/src/input/input.yaml"));
    nh.param("input_yaml", inputfile, std::string("/home/rancho/0Triplez/experiment/k-cbs/output/output0.yaml"));
    nh.param("showspin", showspin, 0.05);// 动画显示频率
    nh.param("file_directory", file_directory, std::string("/home/"));
    nh.param("file_name_back", file_name_back, std::string("map0"));
    nh.param("file_name_trac", file_name_trac, std::string("map"));
    std::vector<Agent>  agents; 
    const std::string pcd_format = ".pcd";
    pcd_file_back = file_directory + file_name_back + pcd_format;
    pcd_file_trac = file_directory + file_name_trac + pcd_format;
    Timepub pub;
    yaml_read(inputfile,agents);
    ros::Publisher paths_pub[agents.size()],Car_Mesh_pub[agents.size()],Dynamic_paths_pub[agents.size()];
    std::vector<std::vector<Eigen::MatrixXd>> null_all_corrid_;
    std::vector<std::vector<Eigen::MatrixXd>> null_smaallall_corrid_;

    std::vector<std::vector<double>> null_all_t;


    pub.Car_Mesh_pub = Car_Mesh_pub;
    pub.Dynamic_paths_pub = Dynamic_paths_pub;
    pub.paths_pub = paths_pub;
    output_topub(nh, agents, pub,pcd_file_back,pcd_file_trac,null_all_corrid_,null_all_t,null_smaallall_corrid_);
    
    pub.success = true;
    ros::Timer Ttimer = nh.createTimer(ros::Duration(showspin), 
                        boost::bind(&timerCallback, _1, agents, pub));

    
    // while(ros::ok())
        


        

    ros::spin();        
    return 0;
}