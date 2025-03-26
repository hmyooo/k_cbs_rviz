#include <cppad/ipopt/solve.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <Eigen/Geometry>
#include <cstdlib>
#include <ros/ros.h>
#include <unordered_map>
#include <set>
#include <visualization_msgs/Marker.h>
#include <algorithm>
// #include "IpTNLP.hpp"
#include <cassert>
#include <limits>
#include <utility>
#include <coin/IpTNLP.hpp>  // 这是 Ipopt 的核心接口定义
# include <cppad/cppad.hpp> //

// 定义优化变量的数量
// size_t num_vehicles;
// size_t num_corridors_per_vehicle;
// size_t num_vars;  // 每个车有多个安全走廊，假设每个走廊由4个点表示

// 索引计算公式
// int base_index = sum_tnum_id_before_i;  // 所有前面车辆的走廊优化变量的累加和
// int corridor_offset = (t - 1) * 6;      // 该车第t个走廊的偏移量，每个走廊6个变量（从1开始）
// int variable_index = base_index + (t-1)*6 + k;  // 变量k的索引

// k从0到5，分别代表第t个走廊的6个优化变量

class Optimizer{
public:
    // typedef CppAD::vector<CppAD::AD<double>> ADvector;  // 定义 ADvector 类型
    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
    Optimizer(int num_agents, std::vector<int> tnum_id, 
              std::vector<std::vector<Eigen::MatrixXd>> all_corrid_,
              std::vector<std::vector<double>> t_ts, 
              std::vector<std::vector<Eigen::Vector4d>> statelists,
              double vehicle_length, double vehicle_width, double max_velocity)
        : num_agents_(num_agents), tnum_id_(tnum_id), 
          all_corrid_(all_corrid_), t_ts_(t_ts),
          statelists_(statelists), vehicle_length_(vehicle_length),
          vehicle_width_(vehicle_width), max_velocity_(max_velocity) {

        // std::cout<<"all_corrid_ OBB "<<all_corrid_[0].size()<<std::endl;
        sum_tnum_id=calculateCumulativeSum(tnum_id_);
        num_vars_ = std::accumulate(tnum_id.begin(), tnum_id.end(), 0)*6; // 每个车有多个走廊，每个走廊用4个点表示

        ROS_INFO_STREAM("Number of variables: " << num_vars_);

        // min_length=sqrt(vehicle_length*vehicle_length + vehicle_width*vehicle_width)*0.8;
        min_length=sqrt(vehicle_length*vehicle_length + vehicle_width*vehicle_width)*0.1;

    }
    std::vector<std::vector<Eigen::MatrixXd>> opt_after_all_corrid_;
    int num_vars_; 
    std::vector<std::vector<Eigen::MatrixXd>> convert_solution_to_corridors(
    const CppAD::ipopt::solve_result<CppAD::vector<double>> solution) {
   
    std::vector<std::vector<Eigen::MatrixXd>> optimized_corridors;
    for (size_t i = 0; i < num_agents_; ++i) {
        std::vector<Eigen::MatrixXd> agent_corridors;
        for (size_t j = 0; j < tnum_id_[i]; ++j) {
            size_t idx_i = sum_tnum_id[i] + (j)*6;
            // 获取车辆 i 在时刻 j 的六个优化变量
            double center_x = (solution.x[idx_i]);
            double center_y = (solution.x[idx_i+1]);
            double dx_neg   = (solution.x[idx_i+2]);
            double dx_pos   = (solution.x[idx_i+3]);
            double dy_neg   = (solution.x[idx_i+4]);
            double dy_pos   = (solution.x[idx_i+5]);

            // 计算四个角的坐标
            double min_x = center_x - dx_neg;
            double max_x = center_x + dx_pos;
            double min_y = center_y - dy_neg;
            double max_y = center_y + dy_pos;

            // 创建一个 4x4 的矩阵来表示走廊
            Eigen::MatrixXd corridor(4, 4);

            // 四个角坐标：这些是走廊的四个顶点
            corridor(2, 0) = max_x;
            corridor(3, 0) = max_y;
            corridor(2, 1) = max_x;
            corridor(3, 1) = min_y;
            corridor(2, 2) = min_x;
            corridor(3, 2) = min_y;
            corridor(2, 3) = min_x;
            corridor(3, 3) = max_y;

            // 法向量：假设每条边的法向量是与坐标轴平行的
            corridor(0, 0) = 0.0;  // 第一列 (min_x, min_y) 的法向量 (x正方向)
            corridor(1, 0) = 1.0;
            corridor(0, 1) = 0.0;  // 第二列 (max_x, min_y) 的法向量 (y负方向)
            corridor(1, 1) = -1.0;
            corridor(0, 2) = -1.0;  // 第三列 (max_x, max_y) 的法向量 (x负方向)
            corridor(1, 2) = 0.0;
            corridor(0, 3) = 0.0;  // 第四列 (min_x, max_y) 的法向量 (y正方向)
            corridor(1, 3) = 1.0;

            // 将计算出的矩阵添加到车辆的走廊列表中
            agent_corridors.push_back(corridor);
        }

        // 将车辆的所有走廊添加到总的走廊列表中
        optimized_corridors.push_back(agent_corridors);
    }

    return optimized_corridors;
}

    // 目标函数：最大化所有走廊的面积
  
    void operator()(ADvector& f,const ADvector& x)  {
        
        // 目标函数：最大化所有走廊的面积
        f[0] = 0;
        CppAD::AD<double> distance_penalty_weight = 10;
        CppAD::AD<double> area_penalty_weight = 0.5;
        CppAD::AD<double> constraints_penalty_weight = 10.0;

        for (size_t i = 0; i < num_agents_; ++i) {
            for (size_t j = 0; j < tnum_id_[i]; ++j) {
            size_t idx_i = sum_tnum_id[i] + (j)*6;

                  // 从优化变量中提取四个点的坐标
        // 从优化变量中提取立方体的参数
            CppAD::AD<double>  center_x = x[idx_i];
            CppAD::AD<double>  center_y = x[idx_i+1];
            CppAD::AD<double>  dx_neg = x[idx_i+2];
            CppAD::AD<double>  dx_pos = x[idx_i+3];
            CppAD::AD<double>  dy_neg = x[idx_i+4];
            CppAD::AD<double>  dy_pos = x[idx_i+5];   

            // 计算立方体的面积
            CppAD::AD<double> area = (dx_neg + dx_pos) +(dy_neg + dy_pos);  // 立方体的面积

            // 更新目标函数：最大化面积
            f[0] -= area_penalty_weight * area;

            // 获取车辆当前位置
            CppAD::AD<double> vehicle_x = statelists_[i][j](0) ;
            CppAD::AD<double> vehicle_y = statelists_[i][j](1);

            // 计算立方体距离
            CppAD::AD<double> dist = ((center_x - vehicle_x) * (center_x - vehicle_x) + (center_y - vehicle_y) * (center_y - vehicle_y));
            
            // 更新目标函数：最小化轴心与车辆位置的距离
            f[0] += distance_penalty_weight * dist;
           

            }
        }
 


    size_t idx = 1; 
    // 遍历每对不同车辆的相同时间走廊
    // for (size_t i = 0; i < num_agents_; i++) {
    //     for (size_t j = i + 1; j < num_agents_; j++) {
    //         for (size_t t = 0; t < std::min(tnum_id_[i],tnum_id_[j]); t++) {
    //             // 获取第i辆车和第j辆车在时刻t的走廊四个点的坐标
    //             size_t idx_i = sum_tnum_id[i] + (t)*6;
    //             size_t idx_j = sum_tnum_id[j] + (t)*6;

    //             CppAD::AD<double> center_x_i = x[idx_i];        
    //             CppAD::AD<double> center_y_i = x[idx_i + 1];    
    //             CppAD::AD<double> dx_neg_i = x[idx_i + 2];      
    //             CppAD::AD<double> dx_pos_i = x[idx_i + 3];      
    //             CppAD::AD<double> dy_neg_i = x[idx_i + 4];      
    //             CppAD::AD<double> dy_pos_i = x[idx_i + 5];      

    //             CppAD::AD<double> center_x_j = x[idx_j];        
    //             CppAD::AD<double> center_y_j = x[idx_j + 1];    
    //             CppAD::AD<double> dx_neg_j = x[idx_j + 2];      
    //             CppAD::AD<double> dx_pos_j = x[idx_j + 3];      
    //             CppAD::AD<double> dy_neg_j = x[idx_j + 4];      
    //             CppAD::AD<double> dy_pos_j = x[idx_j + 5];      

    //             // 判断两个立方体是否相交
    //             bool intersect = check_box_intersection(center_x_i, center_y_i, dx_neg_i, dx_pos_i, dy_neg_i, dy_pos_i,
    //                                                     center_x_j, center_y_j, dx_neg_j, dx_pos_j, dy_neg_j, dy_pos_j);

    //             // 如果相交，约束条件不满足，设置g[idx]为一个大的负值表示约束不满足
    //             if (intersect) {
    //                 f[idx] = -1.0;
    //                 // f[0] += constraints_penalty_weight;

    //             } else {
    //                 f[idx] = 1.0;  // 不相交，满足约束
    //             }
    //             // std::cout << "g[idx]: " << g[idx] << std::endl;
    //             idx++;
    //         }
    //     }
    // }


// //在大走廊内部
    for (size_t i = 0; i < num_agents_; i++) {
        for (size_t j = 0; j < tnum_id_[i]; j++) {
            // 获取当前车辆的安全走廊矩形的四个点和法向量
            Eigen::MatrixXd corridor_boundary = all_corrid_[i][j]; // 四行四列，前两行为法向量，后两行为坐标点
            size_t idx_i = sum_tnum_id[i] + (j)*6;
            
            // 提取点和法向量
            CppAD::AD<double> x0 = corridor_boundary.col(0)(2); CppAD::AD<double> y0 = corridor_boundary.col(0)(3);
            // CppAD::AD<double> x1 = corridor_boundary(2, 1); CppAD::AD<double> y1 = corridor_boundary(3, 1);
            CppAD::AD<double> x2 = corridor_boundary.col(2)(2); CppAD::AD<double> y2 = corridor_boundary.col(2)(3);
            // CppAD::AD<double> x3 = corridor_boundary(2, 3); CppAD::AD<double> y3 = corridor_boundary(3, 3);

            // 法向量
            // T nx0 = corridor_boundary(0, 0); T ny0 = corridor_boundary(1, 0); // 点0的法向量
            // T nx1 = corridor_boundary(0, 1); T ny1 = corridor_boundary(1, 1); // 点1的法向量
            // T nx2 = corridor_boundary(0, 2); T ny2 = corridor_boundary(1, 2); // 点2的法向量
            // T nx3 = corridor_boundary(0, 3); T ny3 = corridor_boundary(1, 3); // 点3的法向量

            CppAD::AD<double> center_x = x[idx_i];        
            CppAD::AD<double> center_y = x[idx_i + 1];    
            CppAD::AD<double> dx_neg = x[idx_i + 2];      
            CppAD::AD<double> dx_pos = x[idx_i + 3];      
            CppAD::AD<double> dy_neg = x[idx_i + 4];      
            CppAD::AD<double> dy_pos = x[idx_i + 5];      

            // 立方体的四个边界
            CppAD::AD<double> min_x = center_x - dx_neg;
            CppAD::AD<double> max_x = center_x + dx_pos;
            CppAD::AD<double> min_y = center_y - dy_neg;
            CppAD::AD<double> max_y = center_y + dy_pos;
                  // 对每个点与矩形边界进行约束：
            // 判断立方体的每个边是否位于矩形内（通过法向量内积判断）
            // 点0 (min_x, min_y) 必须在矩形的左下角
            // g[idx++] = (nx0 * (min_x - x0) + ny0 * (min_y - y0)) >= 0.0 ? -1.0 : 1.0; // 点0是否在矩形边界的右侧
            // // 点1 (max_x, min_y) 必须在矩形的右下角
            // g[idx++] = (nx1 * (max_x - x1) + ny1 * (min_y - y1)) >= 0.0 ? -1.0 : 1.0; // 点1是否在矩形边界的右侧
            // // 点2 (max_x, max_y) 必须在矩形的右上角
            // g[idx++] = (nx2 * (max_x - x2) + ny2 * (max_y - y2)) >= 0.0 ? -1.0 : 1.0; // 点2是否在矩形边界的右侧
            // // 点3 (min_x, max_y) 必须在矩形的左上角
            // g[idx++] = (nx3 * (min_x - x3) + ny3 * (max_y - y3)) >= 0.0 ? -1.0 : 1.0; // 点3是否在矩形边界的右侧



            // g[idx++] = min_x -x2 ; // 点0是否在矩形边界的右侧
            // // 点1 (max_x, min_y) 必须在矩形的右下角
            // g[idx++] =x0-max_x ;// 点1是否在矩形边界的右侧
            // // 点2 (max_x, max_y) 必须在矩形的右上角
            // g[idx++] = min_y-y2; // 点2是否在矩形边界的右侧
            // // 点3 (min_x, max_y) 必须在矩形的左上角
            // g[idx++] = y0-max_y; // 点3是否在矩形边界的右侧
            bool box_in = check_box_in_box(min_x,max_x,min_y,max_y,x2,x0,y2,y0) ; 

            // 如果相交，约束条件不满足，设置g[idx]为一个大的负值表示约束不满足
            if (box_in) {
                f[idx++] = x0-max_x+min_x-x2+y0-max_y+min_y-y2;
            } else {
                f[idx++] = -100.0*std::abs(std::abs(Value(min_x)-Value(x2))+std::abs(Value(x0)-Value(max_x))+std::abs(Value(max_y)-Value(y0))+std::abs(Value(y2)-Value(min_y)));  // 不相交，满足约束
                f[0] += constraints_penalty_weight*std::abs(std::abs(Value(min_x)-Value(x2))+std::abs(Value(x0)-Value(max_x))+std::abs(Value(max_y)-Value(y0))+std::abs(Value(y2)-Value(min_y)));

            }

            // f[idx++] = (x0-max_x+min_x-x2+y0-max_y+min_y-y2);


            // 点1 (max_x, min_y) 必须在矩形的右下角
            // g[idx++] =x0-max_x ;// 点1是否在矩形边界的右侧
            // // 点2 (max_x, max_y) 必须在矩形的右上角
            // g[idx++] = min_y-y2; // 点2是否在矩形边界的右侧
            // // 点3 (min_x, max_y) 必须在矩形的左上角
            // g[idx++] = y0-max_y; // 点3是否在矩形边界的右侧

            // CppAD::AD<double> min_x_length = min_length;  // 这里设置您期望的最小值
            // g[idx++] = (dx_neg + dx_pos) - min_x_length;  // x轴方向的最小长度约束
            // // y轴方向的长度：dy_neg + dy_pos
            // CppAD::AD<double> min_y_length = min_length;  // 这里设置您期望的最小值
            // g[idx++] = (dy_neg + dy_pos) - min_y_length;  // y轴方向的最小长度约束



        }
    }



// 遍历每个车辆相邻走廊重叠面积
    for (size_t i = 0; i < num_agents_; i++) {
        for (size_t t = 0; t < tnum_id_[i] - 1; t++) { // 遍历每个时刻，避免越界
            // 获取第i辆车在时刻t和t+1的走廊参数
            size_t idx_t = sum_tnum_id[i] + (t)*6;
            size_t idx_t_next = idx_t+6;
            // 提取当前时刻和下一个时刻的矩形参数
            double center_x_t = Value(x[idx_t]);        
            double center_y_t = Value(x[idx_t + 1]);    
            double dx_neg_t = Value(x[idx_t + 2]);      
            double dx_pos_t = Value(x[idx_t + 3]);      
            double dy_neg_t =Value(x[idx_t + 4]);      
            double dy_pos_t = Value(x[idx_t + 5]);     

            double center_x_t_next =Value(x[idx_t_next]);        
            double center_y_t_next = Value(x[idx_t_next + 1]);    
            double dx_neg_t_next = Value(x[idx_t_next + 2]);      
            double dx_pos_t_next = Value(x[idx_t_next + 3]);      
            double dy_neg_t_next = Value(x[idx_t_next + 4]);      
            double dy_pos_t_next = Value(x[idx_t_next + 5]);      

            // 计算两个矩形在x轴方向上的重叠长度
            CppAD::AD<double> overlap_x = std::max(0.0, std::min(center_x_t + dx_pos_t, center_x_t_next + dx_pos_t_next) -
                                      std::max(center_x_t - dx_neg_t, center_x_t_next - dx_neg_t_next));
            // 计算两个矩形在y轴方向上的重叠宽度
            CppAD::AD<double> overlap_y = std::max(0.0, std::min(center_y_t + dy_pos_t, center_y_t_next + dy_pos_t_next) -
                                      std::max(center_y_t - dy_neg_t, center_y_t_next - dy_neg_t_next));
            // 最小重叠面积要求
            CppAD::AD<double> min_overlap_length = min_length;  // 可以根据需要调整
            CppAD::AD<double> min_overlap_width = min_length;   // 可以根据需要调整

            // 新约束：重叠长度和宽度必须大于最小值
            f[idx++] = overlap_x+overlap_y ;  // x轴方向的重叠长度
            

               // y轴方向的重叠宽度

               
//可行驶区域包含
    
//             // 获取车辆i在时刻t的航向角和最大速度
            double theta_t = statelists_[i][t](2);
            double max_speed = max_velocity_; // 假设最大速度为1，根据实际需求替换

            // 计算行驶范围（最大速度 * 1秒）
            double gamma_s_minus = max_speed/2;  // 速度 * 1秒
            double gamma_s_plus = max_speed;  // 速度 * 1秒

            // 计算车辆在x轴和y轴方向上的最小和最大行驶边界
            double dix_min = (std::cos(theta_t) >= 0) 
                ? center_x_t + gamma_s_minus * std::cos(theta_t)
                : center_x_t + gamma_s_plus * std::cos(theta_t);

            double dix_max = (std::cos(theta_t) <= 0)
                ? center_x_t + gamma_s_minus * std::cos(theta_t)
                : center_x_t + gamma_s_plus * std::cos(theta_t);

            double diy_min = (std::sin(theta_t) >= 0) 
                ? center_y_t + gamma_s_minus * std::sin(theta_t)
                : center_y_t + gamma_s_plus * std::sin(theta_t);

            double diy_max = (std::sin(theta_t) <= 0) 
                ? center_y_t + gamma_s_minus * std::sin(theta_t)
                : center_y_t + gamma_s_plus * std::sin(theta_t);

            // // 计算当前时刻矩形的最小和最大边界
            // CppAD::AD<double> min_x = center_x_t - dx_neg_t;
            // CppAD::AD<double> max_x = center_x_t + dx_pos_t;
            // CppAD::AD<double> min_y = center_y_t - dy_neg_t;
            // CppAD::AD<double> max_y = center_y_t + dy_pos_t;

            // 车辆行驶范围的约束：确保当前时刻的走廊边界大于最大速度*1秒的距离
            // g[idx++] =  dix_min-min_x ;  // x轴最小边界
            // g[idx++] = max_x-dix_max;  // x轴最大边界
            // g[idx++] =   diy_min-min_y;  // y轴最小边界
            // g[idx++] =  max_y-diy_max ;  // y轴最大边界

            // 对下一个时刻的走廊进行相同的计算和约束
            CppAD::AD<double> min_x_next = center_x_t_next - dx_neg_t_next;
            CppAD::AD<double> max_x_next = center_x_t_next + dx_pos_t_next;
            CppAD::AD<double> min_y_next = center_y_t_next - dy_neg_t_next;
            CppAD::AD<double> max_y_next = center_y_t_next + dy_pos_t_next;



   
            
            bool box_in_1 = check_box_in_feasible(dix_min,min_x_next,max_x_next,dix_max,diy_min,min_y_next,max_y_next,diy_max) ; 

            // 如果相交，约束条件不满足，设置g[idx]为一个大的负值表示约束不满足
            if (box_in_1) {
                f[idx++] = min_x_next-dix_min+dix_max-max_x_next+min_y_next-diy_min+diy_max-max_y_next;
            } else {
                f[idx++] = -100.0*std::abs(std::abs(Value(min_x_next)-(dix_min))+std::abs((dix_max)-Value(max_x_next))+std::abs(Value(min_y_next)-(diy_min))+std::abs((diy_max)-Value(max_y_next)));;  
                // f[0] += constraints_penalty_weight;

            }

            // g[idx++] =  dix_min-min_x_next ;  // x轴最小边界
            // g[idx++] = max_x_next-dix_max;  // x轴最大边界
            // g[idx++] =   diy_min-min_y_next;  // y轴最小边界
            // g[idx++] =  max_y_next-diy_max ;  // y轴最大边界
            // 计算下一个时刻矩形的最小和最大边界
            // g[idx++] = min_x_next - dix_min;  // x轴最小边界
            // g[idx++] = dix_max - max_x_next;  // x轴最大边界
            // g[idx++] = min_y_next - diy_min;  // y轴最小边界
            // g[idx++] = diy_max - max_y_next;  // y轴最大边界

   // 计算γi_s+ 在x和y方向上的分量
            CppAD::AD<double> delta_x_max = max_speed * std::cos(theta_t);
            CppAD::AD<double> delta_y_max = max_speed * std::sin(theta_t);

            // 计算当前时刻的关键点和上一时刻的关键点之间的距离差
            CppAD::AD<double> delta_x = std::abs(center_x_t - center_x_t_next);
            CppAD::AD<double> delta_y = std::abs(center_y_t - center_y_t_next);


            bool box_in_2 = check_box_in_feasible_point(delta_x,delta_x_max,delta_y,delta_y_max) ; 

            // 如果相交，约束条件不满足，设置g[idx]为一个大的负值表示约束不满足
            if (box_in_2) {
                f[idx++] = std::abs(Value(delta_x_max))-std::abs(Value(delta_x))+std::abs(Value(delta_y_max))-std::abs(Value(delta_y));
            } else {
                f[idx++] = -100.0*std::abs(std::abs(Value(delta_x_max))-std::abs(Value(delta_x))+std::abs(Value(delta_y_max))-std::abs(Value(delta_y)));  // 不相交，满足约束
                // f[0] += constraints_penalty_weight;
            }

            // 车辆i在x轴和y轴方向的行驶距离不应超过允许的最大距离阈值
            // g[idx++] = delta_x-delta_x_max ;  // x轴方向的行驶距离约束
            // g[idx++] = delta_y-delta_y_max ;  // y轴方向的行驶距离约束


        }
    }





















        return ;
    }

    // 约束函数：确保走廊不相交
 
//     void g(ADvector&& g,const ADvector&& x) {

 
//     size_t idx = 0; 
//     // 遍历每对不同车辆的相同时间走廊
//     for (size_t i = 0; i < num_agents_; i++) {
//         for (size_t j = i + 1; j < num_agents_; j++) {
//             for (size_t t = 0; t < std::min(tnum_id_[i],tnum_id_[j]); t++) {
//                 // 获取第i辆车和第j辆车在时刻t的走廊四个点的坐标
//                 size_t idx_i = sum_tnum_id[i] + (t)*6;
//                 size_t idx_j = sum_tnum_id[j] + (t)*6;

//                 CppAD::AD<double> center_x_i = x[idx_i];        
//                 CppAD::AD<double> center_y_i = x[idx_i + 1];    
//                 CppAD::AD<double> dx_neg_i = x[idx_i + 2];      
//                 CppAD::AD<double> dx_pos_i = x[idx_i + 3];      
//                 CppAD::AD<double> dy_neg_i = x[idx_i + 4];      
//                 CppAD::AD<double> dy_pos_i = x[idx_i + 5];      

//                 CppAD::AD<double> center_x_j = x[idx_j];        
//                 CppAD::AD<double> center_y_j = x[idx_j + 1];    
//                 CppAD::AD<double> dx_neg_j = x[idx_j + 2];      
//                 CppAD::AD<double> dx_pos_j = x[idx_j + 3];      
//                 CppAD::AD<double> dy_neg_j = x[idx_j + 4];      
//                 CppAD::AD<double> dy_pos_j = x[idx_j + 5];      

//                 // 判断两个立方体是否相交
//                 bool intersect = check_box_intersection(center_x_i, center_y_i, dx_neg_i, dx_pos_i, dy_neg_i, dy_pos_i,
//                                                         center_x_j, center_y_j, dx_neg_j, dx_pos_j, dy_neg_j, dy_pos_j);

//                 // 如果相交，约束条件不满足，设置g[idx]为一个大的负值表示约束不满足
//                 if (intersect) {
//                     g[idx] = -1.0;
//                 } else {
//                     g[idx] = 1.0;  // 不相交，满足约束
//                 }
//                 std::cout << "g[idx]: " << g[idx] << std::endl;
//                 idx++;
//             }
//         }
//     }


// // //在大走廊内部
//     for (size_t i = 0; i < num_agents_; i++) {
//         for (size_t j = 0; j < tnum_id_[i]; j++) {
//             // 获取当前车辆的安全走廊矩形的四个点和法向量
//             Eigen::MatrixXd corridor_boundary = all_corrid_[i][j]; // 四行四列，前两行为法向量，后两行为坐标点
//             size_t idx_i = sum_tnum_id[i] + (j)*6;
            
//             // 提取点和法向量
//             CppAD::AD<double> x0 = corridor_boundary.col(0)(2); CppAD::AD<double> y0 = corridor_boundary.col(0)(3);
//             // CppAD::AD<double> x1 = corridor_boundary(2, 1); CppAD::AD<double> y1 = corridor_boundary(3, 1);
//             CppAD::AD<double> x2 = corridor_boundary.col(2)(2); CppAD::AD<double> y2 = corridor_boundary.col(2)(3);
//             // CppAD::AD<double> x3 = corridor_boundary(2, 3); CppAD::AD<double> y3 = corridor_boundary(3, 3);

//             // 法向量
//             // T nx0 = corridor_boundary(0, 0); T ny0 = corridor_boundary(1, 0); // 点0的法向量
//             // T nx1 = corridor_boundary(0, 1); T ny1 = corridor_boundary(1, 1); // 点1的法向量
//             // T nx2 = corridor_boundary(0, 2); T ny2 = corridor_boundary(1, 2); // 点2的法向量
//             // T nx3 = corridor_boundary(0, 3); T ny3 = corridor_boundary(1, 3); // 点3的法向量

//             CppAD::AD<double> center_x = x[idx_i];        
//             CppAD::AD<double> center_y = x[idx_i + 1];    
//             CppAD::AD<double> dx_neg = x[idx_i + 2];      
//             CppAD::AD<double> dx_pos = x[idx_i + 3];      
//             CppAD::AD<double> dy_neg = x[idx_i + 4];      
//             CppAD::AD<double> dy_pos = x[idx_i + 5];      

//             // 立方体的四个边界
//             CppAD::AD<double> min_x = center_x - dx_neg;
//             CppAD::AD<double> max_x = center_x + dx_pos;
//             CppAD::AD<double> min_y = center_y - dy_neg;
//             CppAD::AD<double> max_y = center_y + dy_pos;
//                   // 对每个点与矩形边界进行约束：
//             // 判断立方体的每个边是否位于矩形内（通过法向量内积判断）
//             // 点0 (min_x, min_y) 必须在矩形的左下角
//             // g[idx++] = (nx0 * (min_x - x0) + ny0 * (min_y - y0)) >= 0.0 ? -1.0 : 1.0; // 点0是否在矩形边界的右侧
//             // // 点1 (max_x, min_y) 必须在矩形的右下角
//             // g[idx++] = (nx1 * (max_x - x1) + ny1 * (min_y - y1)) >= 0.0 ? -1.0 : 1.0; // 点1是否在矩形边界的右侧
//             // // 点2 (max_x, max_y) 必须在矩形的右上角
//             // g[idx++] = (nx2 * (max_x - x2) + ny2 * (max_y - y2)) >= 0.0 ? -1.0 : 1.0; // 点2是否在矩形边界的右侧
//             // // 点3 (min_x, max_y) 必须在矩形的左上角
//             // g[idx++] = (nx3 * (min_x - x3) + ny3 * (max_y - y3)) >= 0.0 ? -1.0 : 1.0; // 点3是否在矩形边界的右侧



//             // g[idx++] = min_x -x2 ; // 点0是否在矩形边界的右侧
//             // // 点1 (max_x, min_y) 必须在矩形的右下角
//             // g[idx++] =x0-max_x ;// 点1是否在矩形边界的右侧
//             // // 点2 (max_x, max_y) 必须在矩形的右上角
//             // g[idx++] = min_y-y2; // 点2是否在矩形边界的右侧
//             // // 点3 (min_x, max_y) 必须在矩形的左上角
//             // g[idx++] = y0-max_y; // 点3是否在矩形边界的右侧
//             bool box_in = check_box_in_box(min_x,max_x,min_y,max_y,x2,x0,y2,y0) ; 

//             // 如果相交，约束条件不满足，设置g[idx]为一个大的负值表示约束不满足
//             if (box_in) {
//                 g[idx++] = 1.0;
//             } else {
//                 g[idx++] = -10.0;  // 不相交，满足约束
//             }
//             // 点1 (max_x, min_y) 必须在矩形的右下角
//             // g[idx++] =x0-max_x ;// 点1是否在矩形边界的右侧
//             // // 点2 (max_x, max_y) 必须在矩形的右上角
//             // g[idx++] = min_y-y2; // 点2是否在矩形边界的右侧
//             // // 点3 (min_x, max_y) 必须在矩形的左上角
//             // g[idx++] = y0-max_y; // 点3是否在矩形边界的右侧

//             // CppAD::AD<double> min_x_length = min_length;  // 这里设置您期望的最小值
//             // g[idx++] = (dx_neg + dx_pos) - min_x_length;  // x轴方向的最小长度约束
//             // // y轴方向的长度：dy_neg + dy_pos
//             // CppAD::AD<double> min_y_length = min_length;  // 这里设置您期望的最小值
//             // g[idx++] = (dy_neg + dy_pos) - min_y_length;  // y轴方向的最小长度约束



//         }
//     }



// // // 遍历每个车辆相邻走廊重叠面积
//     for (size_t i = 0; i < num_agents_; i++) {
//         for (size_t t = 0; t < tnum_id_[i] - 1; t++) { // 遍历每个时刻，避免越界
//             // 获取第i辆车在时刻t和t+1的走廊参数
//             size_t idx_t = sum_tnum_id[i] + (t)*6;
//             size_t idx_t_next = idx_t+6;
//             // 提取当前时刻和下一个时刻的矩形参数
//             double center_x_t = Value(x[idx_t]);        
//             double center_y_t = Value(x[idx_t + 1]);    
//             double dx_neg_t = Value(x[idx_t + 2]);      
//             double dx_pos_t = Value(x[idx_t + 3]);      
//             double dy_neg_t =Value(x[idx_t + 4]);      
//             double dy_pos_t = Value(x[idx_t + 5]);     

//             double center_x_t_next =Value(x[idx_t_next]);        
//             double center_y_t_next = Value(x[idx_t_next + 1]);    
//             double dx_neg_t_next = Value(x[idx_t_next + 2]);      
//             double dx_pos_t_next = Value(x[idx_t_next + 3]);      
//             double dy_neg_t_next = Value(x[idx_t_next + 4]);      
//             double dy_pos_t_next = Value(x[idx_t_next + 5]);      

//             // 计算两个矩形在x轴方向上的重叠长度
//             CppAD::AD<double> overlap_x = std::max(0.0, std::min(center_x_t + dx_pos_t, center_x_t_next + dx_pos_t_next) -
//                                       std::max(center_x_t - dx_neg_t, center_x_t_next - dx_neg_t_next));
//             // 计算两个矩形在y轴方向上的重叠宽度
//             CppAD::AD<double> overlap_y = std::max(0.0, std::min(center_y_t + dy_pos_t, center_y_t_next + dy_pos_t_next) -
//                                       std::max(center_y_t - dy_neg_t, center_y_t_next - dy_neg_t_next));
//             // 最小重叠面积要求
//             CppAD::AD<double> min_overlap_length = min_length;  // 可以根据需要调整
//             CppAD::AD<double> min_overlap_width = min_length;   // 可以根据需要调整

//             // 新约束：重叠长度和宽度必须大于最小值
//             g[idx++] = overlap_x+overlap_y ;  // x轴方向的重叠长度
//                // y轴方向的重叠宽度

               
// //可行驶区域包含
    
// //             // 获取车辆i在时刻t的航向角和最大速度
//             double theta_t = statelists_[i][t](2);
//             double max_speed = max_velocity_; // 假设最大速度为1，根据实际需求替换

//             // 计算行驶范围（最大速度 * 1秒）
//             double gamma_s_minus = max_speed;  // 速度 * 1秒
//             double gamma_s_plus = -max_speed;  // 速度 * 1秒

//             // 计算车辆在x轴和y轴方向上的最小和最大行驶边界
//             double dix_min = (std::cos(theta_t) >= 0) 
//                 ? center_x_t_next + gamma_s_minus * std::cos(theta_t)
//                 : center_x_t_next + gamma_s_plus * std::cos(theta_t);

//             double dix_max = (std::cos(theta_t) <= 0)
//                 ? center_x_t_next + gamma_s_minus * std::cos(theta_t)
//                 : center_x_t_next + gamma_s_plus * std::cos(theta_t);

//             double diy_min = (std::sin(theta_t) >= 0) 
//                 ? center_y_t_next + gamma_s_minus * std::sin(theta_t)
//                 : center_y_t_next + gamma_s_plus * std::sin(theta_t);

//             double diy_max = (std::sin(theta_t) <= 0) 
//                 ? center_y_t_next + gamma_s_minus * std::sin(theta_t)
//                 : center_y_t_next + gamma_s_plus * std::sin(theta_t);

//             // // 计算当前时刻矩形的最小和最大边界
//             // CppAD::AD<double> min_x = center_x_t - dx_neg_t;
//             // CppAD::AD<double> max_x = center_x_t + dx_pos_t;
//             // CppAD::AD<double> min_y = center_y_t - dy_neg_t;
//             // CppAD::AD<double> max_y = center_y_t + dy_pos_t;

//             // 车辆行驶范围的约束：确保当前时刻的走廊边界大于最大速度*1秒的距离
//             // g[idx++] =  dix_min-min_x ;  // x轴最小边界
//             // g[idx++] = max_x-dix_max;  // x轴最大边界
//             // g[idx++] =   diy_min-min_y;  // y轴最小边界
//             // g[idx++] =  max_y-diy_max ;  // y轴最大边界

//             // 对下一个时刻的走廊进行相同的计算和约束
//             CppAD::AD<double> min_x_next = center_x_t_next - dx_neg_t_next;
//             CppAD::AD<double> max_x_next = center_x_t_next + dx_pos_t_next;
//             CppAD::AD<double> min_y_next = center_y_t_next - dy_neg_t_next;
//             CppAD::AD<double> max_y_next = center_y_t_next + dy_pos_t_next;


//             bool box_in_1 = check_box_in_feasible(dix_min,min_x_next,max_x_next,dix_max,diy_min,min_y_next,max_y_next,diy_max) ; 

//             // 如果相交，约束条件不满足，设置g[idx]为一个大的负值表示约束不满足
//             if (box_in_1) {
//                 g[idx++] = 1.0;
//             } else {
//                 g[idx++] = -1.0;  // 不相交，满足约束
//             }

//             // g[idx++] =  dix_min-min_x_next ;  // x轴最小边界
//             // g[idx++] = max_x_next-dix_max;  // x轴最大边界
//             // g[idx++] =   diy_min-min_y_next;  // y轴最小边界
//             // g[idx++] =  max_y_next-diy_max ;  // y轴最大边界
//             // 计算下一个时刻矩形的最小和最大边界
//             // g[idx++] = min_x_next - dix_min;  // x轴最小边界
//             // g[idx++] = dix_max - max_x_next;  // x轴最大边界
//             // g[idx++] = min_y_next - diy_min;  // y轴最小边界
//             // g[idx++] = diy_max - max_y_next;  // y轴最大边界

//    // 计算γi_s+ 在x和y方向上的分量
//             CppAD::AD<double> delta_x_max = max_speed * std::cos(theta_t);
//             CppAD::AD<double> delta_y_max = max_speed * std::sin(theta_t);

//             // 计算当前时刻的关键点和上一时刻的关键点之间的距离差
//             CppAD::AD<double> delta_x = std::abs(center_x_t - center_x_t_next);
//             CppAD::AD<double> delta_y = std::abs(center_y_t - center_y_t_next);




//             bool box_in_2 = check_box_in_feasible_point(delta_x,delta_x_max,delta_y,delta_y_max) ; 

//             // 如果相交，约束条件不满足，设置g[idx]为一个大的负值表示约束不满足
//             if (box_in_2) {
//                 g[idx++] = 1.0;
//             } else {
//                 g[idx++] = -1.0;  // 不相交，满足约束
//             }

//             // 车辆i在x轴和y轴方向的行驶距离不应超过允许的最大距离阈值
//             // g[idx++] = delta_x-delta_x_max ;  // x轴方向的行驶距离约束
//             // g[idx++] = delta_y-delta_y_max ;  // y轴方向的行驶距离约束


//         }
//     }








//     return ;
// }

// 辅助函数：检查两条线段是否相交

bool check_box_intersection(CppAD::AD<double> center_x_i, CppAD::AD<double> center_y_i, CppAD::AD<double> dx_neg_i, CppAD::AD<double> dx_pos_i, CppAD::AD<double> dy_neg_i, CppAD::AD<double> dy_pos_i,
                            CppAD::AD<double> center_x_j, CppAD::AD<double> center_y_j, CppAD::AD<double> dx_neg_j, CppAD::AD<double> dx_pos_j, CppAD::AD<double> dy_neg_j, CppAD::AD<double> dy_pos_j) {
    // 检查x轴的重叠
    bool x_overlap = (center_x_i + dx_pos_i >= center_x_j - dx_neg_j) && (center_x_i - dx_neg_i <= center_x_j + dx_pos_j);
    // 检查y轴的重叠
    bool y_overlap = (center_y_i + dy_pos_i >= center_y_j - dy_neg_j) && (center_y_i - dy_neg_i <= center_y_j + dy_pos_j);

    // 如果x轴和y轴都重叠，则两个立方体相交
    return x_overlap && y_overlap;
}
           

bool check_box_in_box(CppAD::AD<double> min_x, CppAD::AD<double> max_x, CppAD::AD<double> min_y, CppAD::AD<double> max_y, CppAD::AD<double> x2, CppAD::AD<double> x0,
                            CppAD::AD<double> y2, CppAD::AD<double> y0) {
    // 检查矩形一的边界是否全部位于矩形二的边界内
    bool x_contained = (max_x <= x0-1e-6) && (min_x >= x2+1e-6);
    bool y_contained = (max_y <= y0-1e-6) && (min_y >= y2+1e-6);

    // 如果x轴和y轴方向上矩形一都被矩形二包含，则返回true
    return x_contained && y_contained;
}


bool check_box_in_feasible(CppAD::AD<double> dix_min, CppAD::AD<double> min_x_next, CppAD::AD<double> max_x_next, CppAD::AD<double> dix_max, CppAD::AD<double> diy_min, CppAD::AD<double> min_y_next,
                            CppAD::AD<double> max_y_next, CppAD::AD<double> diy_max) {
 
    bool x_1 = (dix_min-min_x_next <= 1e-4)&&(max_x_next-dix_max<=1e-4)&&(diy_min-min_y_next<=1e-4)&&(diy_max-max_y_next>=-1e-4);

    
    return x_1;
}


bool check_box_in_feasible_point(CppAD::AD<double> delta_x, CppAD::AD<double> delta_x_max, CppAD::AD<double> delta_y, CppAD::AD<double> delta_y_max) {

    bool x_1 = (std::abs(Value(delta_x))<= std::abs(Value(delta_x_max)))&&(std::abs(Value(delta_y))<= std::abs(Value(delta_y_max)));

    // 如果x轴和y轴方向上矩形一都被矩形二包含，则返回true
    return x_1;
}

// bool optimize_corridors() {
//     typedef CPPAD_TESTVECTOR(double) Dvector;
// //     // 初始化优化变量，填入初始走廊数据
//     Dvector x_init;
//     ROS_INFO_STREAM("Number of variables: " << num_vars_);


// // Define a max size (e.g., based on system limits)
//     size_t max_size = std::numeric_limits<size_t>::max() / 2;

//     if (num_vars_ <= max_size) {
//         // Safe to create a Dvector of this size
//         x_init.resize(num_vars_);
//     } else {
//         ROS_WARN("Attempting to create a vector too large! Reducing size.");
//         // Consider reducing the size or splitting the data into smaller parts
//     }



//     for (size_t i = 0; i < num_agents_; ++i) {
//         for (size_t j = 0; j < tnum_id_[i]; ++j) {
//                 size_t idx_i = sum_tnum_id[i] + (j)*6;
//                 Eigen::Vector2d point1 = all_corrid_[i][j].col(0).tail(2); // 提取每个点的坐标
//                 Eigen::Vector2d point2 = all_corrid_[i][j].col(2).tail(2); // 提取每个点的坐标
//                 double x_length=std::abs(point1.x()-point2.x());
//                 double y_length=std::abs(point1.y()-point2.y());

//                 x_init[idx_i] = statelists_[i][j](0);  // x坐标
//                 x_init[idx_i+1] = statelists_[i][j](1);  // y坐标
//                 x_init[idx_i+2] = x_length/2;  // y坐标
//                 x_init[idx_i+3] = x_length/2;  // y坐标
//                 x_init[idx_i+4] = y_length/2;  // y坐标
//                 x_init[idx_i+5] = y_length/2;  // y坐标


            
//         }
//     }

//     // 设置求解器配置
//     std::string options = "max_iter 100; tol 1e-6; print_level 5;";

//       Dvector xl(num_vars_), xu(num_vars_);
//     for(size_t i = 0; i < num_vars_; i++)
//     {
//         xl[i] = -1.0; // x_i >= 0
//         xu[i] = 1000.0; // 无上界
//     }
//     Dvector gl(num_vars_), gu(num_vars_);
//    for(size_t i = 0; i < num_vars_; i++)
//     {
//         gl[i] = 1; // x_i >= 0
//         gu[i] = 1; // 无上界
//     }
//     // 创建一个 CppAD::vector 来存储优化结果
//     CppAD::ipopt::solve_result<Dvector>  solution;
//     Optimizer optimizer(num_agents_,tnum_id_,all_corrid_,t_ts_,statelists_,vehicle_length_, vehicle_width_,max_velocity_);

//     // 调用 CppAD 的求解器进行优化
//     CppAD::ipopt::solve<Dvector, Optimizer>(
//         options,         // 配置选项
//         x_init,          // 初始优化变量
//         xl, xu, gl, gu,
//         optimizer,   
//         solution       // 存储结果
//              // 目标函数和约束函数（Optimizer 实现了 FG_eval）
//     );
//     bool ok = true;
//      // 输出求解器的状态和迭代次数
   
//     ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
//     cout<<"ok is "<<ok<<endl;
//      ROS_INFO("Final objective value: %f", solution.obj_value);
//     // 将转换后的结果传递给转换函数
//     std::vector<std::vector<Eigen::MatrixXd>> converted_corridors = convert_solution_to_corridors(solution);

//     opt_after_all_corrid_ = converted_corridors;
//     return true;
// }
private:
    int num_agents_;  // 车辆数量
    std::vector<int> tnum_id_;  // 标识符数量
     // 优化问题的变量数量
    double min_length;
     std::vector<int> sum_tnum_id;
    // 参数
    std::vector<std::vector<Eigen::MatrixXd>> all_corrid_;
    std::vector<std::vector<double>> t_ts_;
    std::vector<std::vector<Eigen::Vector4d>> statelists_;
    double vehicle_length_;
    double vehicle_width_;
    double max_velocity_;
    
    std::vector<int> calculateCumulativeSum(const std::vector<int>& tnum_id) 
    {
    // 初始化一个用于存储累加和的vector
    std::vector<int> cumulative_sum(tnum_id.size(), 0);

    // 累加变量
    int total_cumulative = 0;

    // 遍历每辆车
    for (size_t i = 0; i < tnum_id.size(); ++i) {
        // 存储前i辆车的走廊优化变量累加和
        cumulative_sum[i] = total_cumulative;
        
        // 更新累加和：当前车的走廊数量 * 每个走廊有6个优化变量
        total_cumulative += tnum_id[i]*6;
    }

    return cumulative_sum;
}
};
