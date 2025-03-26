#pragma once

// EIGEN
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cstdlib>

#include <ros/ros.h>
#include <unordered_map>
#include <set>
#include <visualization_msgs/Marker.h>
#include <algorithm>
// #include "IpTNLP.hpp"
#include <cassert>
#include <vector>
#include <cmath>
#include <limits>
#include <utility>
// #include <cppad/ipopt/solve.hpp>
#include <ilcplex/ilocplex.h>
ILOSTLBEGIN
using namespace std;
// struct Point {
//     double x, y;
// };

// struct Rectangle {
//     Point vertices[4];
// };
// struct Rectangle_X {
//     Eigen::Vector2d top_right;
//     Eigen::Vector2d bottom_left;
// };
class MyOptimizationProblemzhouxin {
private:
    int n_cars;
    std::vector<int>n_times;//每辆车的时间个数
    // std::vector<int>total_times;

    std::vector<std::vector<Eigen::MatrixXd>> all_corrid_;
    std::vector<std::vector<Eigen::MatrixXd>> smallest_all_corrid_;


    std::vector<std::vector<double>> t_ts;//每辆车的时间都是从0-多少
    // std::vector<std::vector<double>> yaws;
    std::vector<std::vector<Eigen::Vector4d>> statelist_;
    double car_length, car_width,max_vec_;
public:
    // 构造函数，传递初始化的优化变量
    MyOptimizationProblemzhouxin(int car_num,std::vector<int> tnum_id,std::vector<std::vector<Eigen::MatrixXd>> all_corrid,
    std::vector<std::vector<double>> t_ts_,std::vector<std::vector<Eigen::Vector4d>> statelist,
    double car_length_, double car_width_,double max_vec,std::vector<std::vector<Eigen::MatrixXd>> smallest_corrid_) : n_cars(car_num), n_times(std::move(tnum_id)),
    all_corrid_(std::move(all_corrid)),t_ts(std::move(t_ts_)),statelist_(std::move(statelist)),car_length(car_length_),car_width(car_width_),max_vec_(max_vec),smallest_all_corrid_(smallest_corrid_){}
    std::vector<std::vector<Eigen::MatrixXd>> opt_after_all_corrid_;
    bool opt_corrid()
    {
      auto max_it = std::max_element(n_times.begin(), n_times.end());
    // 解引用迭代器以获取最大值
    int maxNumPoints = *max_it;
    IloEnv env; // 环境变量，用于处理所有CPLEX对象
   try {
    IloModel model(env);

    std::vector<std::vector<IloNumVarArray>> rectangles(n_cars);
    for (int i = 0; i < n_cars; ++i) {
        rectangles[i] = std::vector<IloNumVarArray>(n_times[i]);

        for (int j = 0; j < n_times[i]; ++j) {
            rectangles[i][j] = IloNumVarArray(env, 6, 0, IloInfinity, ILOFLOAT); // 为每个矩形创建四个坐标变量
        }
    }
    

    IloExpr objective(env);
    for (int i = 0; i < n_cars; ++i) {
        for (int j = 0; j < n_times[i]; ++j) {
            // objective += -((rectangles[i][j][2] + rectangles[i][j][3]) + (rectangles[i][j][4] + rectangles[i][j][5]));
            // objective += 0.2*(IloAbs(rectangles[i][j][0]-statelist_[i][j](0))+IloAbs(rectangles[i][j][1]-statelist_[i][j](1)));
            
             // 定义辅助变量
        IloNumVar diff_x(env, -IloInfinity, IloInfinity);
        IloNumVar diff_y(env, -IloInfinity, IloInfinity);
        IloNumVar diff_x_abs(env, 0, IloInfinity);
        IloNumVar diff_y_abs(env, 0, IloInfinity);
        
        // 计算差值
        model.add(diff_x == rectangles[i][j][0] - statelist_[i][j](0));
        model.add(diff_y == rectangles[i][j][1] - statelist_[i][j](1));
        
        // 绝对值线性化
        model.add(diff_x <= diff_x_abs);
        model.add(-diff_x <= diff_x_abs);
        model.add(diff_y <= diff_y_abs);
        model.add(-diff_y <= diff_y_abs);
        
        // 更新目标函数
        objective += -0.5*((rectangles[i][j][2] + rectangles[i][j][3]) + 
                      (rectangles[i][j][4] + rectangles[i][j][5])) 
                    + 10*(diff_x_abs + diff_y_abs);

            
            

            
            // objective += -((rectangles[i][j][0] - rectangles[i][j][2]) + (rectangles[i][j][1] - rectangles[i][j][3]));
      
        // if(j>0)
        //     {
        //         objective += 0.2*(IloAbs((rectangles[i][j][0]+rectangles[i][j][2])/2-(rectangles[i][j-1][0]+rectangles[i][j-1][2])/2)+IloAbs((rectangles[i][j][1]+rectangles[i][j][3])/2-(rectangles[i][j-1][1]+rectangles[i][j-1][3])/2));
        //     }
        }
    }
//     for (int i = 0; i < n_cars; ++i) {
//     for (int j = 0; j < n_times[i]-1; ++j) {
//         // 计算相邻时间点矩形宽度的差异的绝对值
//         objective += 0.05*IloAbs((rectangles[i][j+1][0] - rectangles[i][j+1][2])
//                                   - (rectangles[i][j][0] - rectangles[i][j][2]));
//         // 计算相邻时间点矩形高度的差异的绝对值
//         objective += 0.05*IloAbs((rectangles[i][j+1][1] - rectangles[i][j+1][3])
//                                   - (rectangles[i][j][1] - rectangles[i][j][3]));
//     }
// }

    model.add(IloMinimize(env, objective));
 IloNum M = 100000; // M 是一个大的常数
 double  epsilon=0.1;

for (int t = 0; t < maxNumPoints; ++t) {
    for (int i = 0; i < n_cars; ++i) {
        for (int j = i + 1; j < n_cars; ++j) {
            if (t < n_times[i] && t < n_times[j]) {
                IloNumVarArray rectA = rectangles[i][t];
                IloNumVarArray rectB = rectangles[j][t];
                IloBoolVar zLeft(env,0,1), zRight(env,0,1), zTop(env,0,1), zBottom(env,0,1); // 二进制变量



                
                model.add((rectA[0]-rectA[2])-(rectB[0]+rectB[3])+ M * zLeft >=  epsilon );

                // A 在 B 的右边
                model.add((rectB[0]-rectB[2])- (rectA[0]+rectA[3])+ M * zRight>=  epsilon );

                // A 在 B 的上边
                model.add((rectA[1]-rectA[4])-(rectB[1]+rectB[5]) + M * zTop >=   epsilon);

                // A 在 B 的下边
                model.add((rectB[1]-rectB[4]) - (rectA[1]+rectA[5])+ M * zBottom >=   epsilon );
                // 至少一个方向上的约束必须被满足
              
                model.add(zLeft + zRight + zTop + zBottom <= 3);
    

            }
        }
    }
}


    for (int i = 0; i < n_cars; ++i) {   
        for (int t = 0; t < n_times[i]; ++t) {     
                IloNumVarArray rectA = rectangles[i][t];

              
                //x小
                model.add((rectA[0]-rectA[2]) >= all_corrid_[i][t].col(2)(2) );

                // x大
                model.add((rectA[0]+rectA[3]) <= all_corrid_[i][t].col(0)(2));

                // 上边
                model.add((rectA[1]+ rectA[5])<= all_corrid_[i][t].col(0)(3));

                // 下边
                model.add((rectA[1]- rectA[4]) >= all_corrid_[i][t].col(2)(3));
        


               
             
            
        
    }
}

  for (int i = 0; i < n_cars; i++) {   
        for (int t = 0; t < n_times[i]-1; t++) {        
                IloNumVarArray rectA = rectangles[i][t];
                IloNumVarArray rectB= rectangles[i][t+1];

                IloNumVar overlapX(env, 0, IloInfinity); // x方向重叠长度 ≥0
                IloNumVar overlapY(env, 0, IloInfinity); // y方向重叠长度 ≥0

                IloBoolVar xAMin(env), xBMin(env), xAMax(env), xBMax(env);
                IloBoolVar yAMin(env), yBMin(env), yAMax(env), yBMax(env);
// rectA[0]=(rectA[0]+rectA[3])
// rectA[1]=(rectA[1]+rectA[5])
// rectA[2]=(rectA[0]-rectA[2])
// rectA[3]=(rectA[1]-rectA[4])

// rectB[0]=(rectB[0]+rectB[3])
// rectB[1]=(rectB[1]+rectB[5])
// rectB[2]=(rectB[0]-rectB[2])
// rectB[3]=(rectB[1]-rectB[4])



                model.add(rectA[0]-rectA[2] <= rectB[0]+rectB[3]); 
                model.add(rectA[0]+rectA[3] >= rectB[0]-rectB[2]); 

               
                model.add(rectA[1]-rectA[4] <= rectB[1]+rectB[5]); 
                model.add(rectA[1]+rectA[5] >= rectB[1]-rectB[4]); 

               // 确定 min(rectA[0], rectB[0])
                model.add( rectA[0]+rectA[3] <= rectB[0]+rectB[3] + M*(1 - xAMin) );
                model.add( rectB[0]+rectB[3] <= rectA[0]+rectA[3] + M*(1 - xBMin) );
                model.add( xAMin + xBMin == 1 );
                
                // 确定 max(rectA[2], rectB[2])
                model.add( rectA[0]-rectA[2] >= rectB[0]-rectB[2] - M*(1 - xAMax) );
                model.add( rectB[0]-rectB[2] >= rectA[0]-rectA[2] - M*(1 - xBMax) );
                model.add( xAMax + xBMax == 1 );

                // 确定 min(rectA[1], rectB[1])
                model.add( rectA[1]+rectA[5] <= rectB[1]+rectB[5] + M*(1 - yAMin) );
                model.add( rectB[1]+rectB[5] <= rectA[1]+rectA[5] + M*(1 - yBMin) );
                model.add( yAMin + yBMin == 1 );

                // 确定 max(rectA[3], rectB[3])
                model.add( rectA[1]-rectA[4]>= rectB[1]-rectB[4] - M*(1 - yAMax) );
                model.add( rectB[1]-rectB[4] >= rectA[1]-rectA[4] - M*(1 - yBMax) );
                model.add( yAMax + yBMax == 1 );



                IloNumVar min_x(env, 0, IloInfinity);
                model.add(min_x <= rectA[0] + rectA[3]);
                model.add(min_x <= rectB[0] + rectB[3]);
                model.add(min_x >= rectA[0] + rectA[3] - M*(1 - xAMin));
                model.add(min_x >= rectB[0] + rectB[3] - M*(1 - xBMin));

                IloNumVar max_x(env, 0, IloInfinity);
                model.add(max_x >= rectA[0] - rectA[2]);
                model.add(max_x >= rectB[0] - rectB[2]);
                model.add(max_x <= rectA[0] - rectA[2] + M*(1 - xAMax));
                model.add(max_x <= rectB[0] - rectB[2] + M*(1 - xBMax));

                model.add(overlapX == min_x - max_x);

                IloNumVar min_y(env, 0, IloInfinity);
                model.add(min_y <= rectA[1] + rectA[5]);
                model.add(min_y <= rectB[1] + rectB[5]);
                model.add(min_y >= rectA[1] + rectA[5] - M*(1 - xAMin));
                model.add(min_y >= rectB[1] + rectB[5] - M*(1 - xBMin));

                IloNumVar max_y(env, 0, IloInfinity);
                model.add(max_y >= rectA[1] - rectA[4]);
                model.add(max_y >= rectB[1] - rectB[4]);
                model.add(max_y <= rectA[1] - rectA[4] + M*(1 - xAMax));
                model.add(max_y <= rectB[1] - rectB[4] + M*(1 - xBMax));

                model.add(overlapY == min_y - max_y);



                // 计算重叠长度
                // model.add( overlapX == xAMin*(rectA[0]+rectA[3]) + xBMin*(rectB[0]+rectB[3]) - xAMax*(rectA[0]-rectA[2]) - xBMax*(rectB[0]-rectB[2]) );
                // model.add( overlapY == yAMin*(rectA[1]+rectA[5]) + yBMin*(rectB[1]+rectB[5]) - yAMax*(rectA[1]-rectA[4]) - yBMax*(rectB[1]-rectB[4]) );

                // 约束最小重叠长度
                model.add( overlapX >= car_length );
                model.add( overlapY >= car_length );

                double theta_t = statelist_[i][t](2);
                // 修改后（放宽，例如增加20%余量）
                double relax_factor = 1.0; // 松弛系数
                double gamma_s_minus = max_vec_/2 * relax_factor;
                double gamma_s_plus = max_vec_ * relax_factor;
                // 预计算三角函数值（theta_t 为已知参数）
                double cos_theta = std::cos(theta_t);
                double sin_theta = std::sin(theta_t);

                // 确定 x 方向扩展系数
                double gamma_s_x_for_min = (cos_theta >= 0) ? gamma_s_minus : gamma_s_plus;
                double gamma_s_x_for_max = (cos_theta <= 0) ? gamma_s_minus : gamma_s_plus;

                // 确定 y 方向扩展系数
                double gamma_s_y_for_min = (sin_theta >= 0) ? gamma_s_minus : gamma_s_plus;
                double gamma_s_y_for_max = (sin_theta <= 0) ? gamma_s_minus : gamma_s_plus;


                // 定义变量（假设已提前声明）


// min_x_next=(rectB[0]-rectB[2])
// max_x_next=(rectB[0]+rectB[3])

// min_y_next=(rectB[1]-rectB[4])
// max_y_next=(rectB[1]+rectB[5])

                // 构建行驶范围边界表达式
                IloExpr dix_min(env); 
                dix_min = rectA[0] + gamma_s_x_for_min * cos_theta; // 线性表达式

                IloExpr dix_max(env);
                dix_max = rectA[0] + gamma_s_x_for_max * cos_theta; // 线性表达式

                IloExpr diy_min(env);
                diy_min = rectA[1] + gamma_s_y_for_min * sin_theta; // 线性表达式

                IloExpr diy_max(env);
                diy_max = rectA[1] + gamma_s_y_for_max * sin_theta; // 线性表达式

                IloNumVar relax_x_min(env, 0, IloInfinity);
                IloNumVar relax_x_max(env, 0, IloInfinity);
                IloNumVar relax_y_min(env, 0, IloInfinity);
                IloNumVar relax_y_max(env, 0, IloInfinity);

                // 添加约束：行驶范围必须包含下一时刻的矩形
                model.add(dix_min- relax_x_min <= rectB[0]-rectB[2] + epsilon);   // dix_min <= min_x_next + eps
                model.add(rectB[0]+rectB[3] <= dix_max + relax_x_max  + epsilon);   // max_x_next <= dix_max + eps
                model.add(diy_min - relax_y_min <= rectB[1]-rectB[4] + epsilon);   // diy_min <= min_y_next + eps
                model.add(rectB[1]+rectB[5] <= diy_max+ relax_y_max + epsilon);   // max_y_next <= diy_max + eps
                objective += 1000 * (relax_x_min + relax_x_max + relax_y_min + relax_y_max);
                // 清理表达式（防止内存泄漏）
                dix_min.end();
                dix_max.end();
                diy_min.end();
                diy_max.end();


                // 预计算方向投影的绝对值
                double delta_x_max_abs = std::abs(gamma_s_plus * cos_theta);
                double delta_y_max_abs = std::abs(gamma_s_plus * sin_theta);

                // 添加位移约束
                model.add(rectA[0] - rectB[0] <= delta_x_max_abs);
                model.add(rectB[0] - rectA[0] <= delta_x_max_abs);
                model.add(rectA[1] - rectB[1] <= delta_y_max_abs);
                model.add(rectB[1]  - rectA[1]  <= delta_y_max_abs);












               
            
        
    }
}

    IloCplex cplex(model);
    if (cplex.solve()) {
        // 输出结果
        // IloNumArray vals(env);
        // total_cost = cplex.getObjValue();
        // cplex.getValues(vals, var);
        std::cout<<"object "<<cplex.getObjValue()<<std::endl;

    opt_after_all_corrid_.resize(n_cars);
    for (int i = 0; i < n_cars; ++i) {
        opt_after_all_corrid_[i].resize(n_times[i]);
        for (int j = 0; j < n_times[i]; ++j) {
            opt_after_all_corrid_[i][j] = Eigen::MatrixXd(4, 4); // 假设每个矩形用一行四列的矩阵表示
            // for (int k = 2; k < 4; ++k) {
                opt_after_all_corrid_[i][j].col(0)(2) = cplex.getValue(rectangles[i][j][0]+rectangles[i][j][3]);
                opt_after_all_corrid_[i][j].col(0)(3) = cplex.getValue(rectangles[i][j][1]+rectangles[i][j][5]);
                opt_after_all_corrid_[i][j].col(0).head(2) <<0,1;

                opt_after_all_corrid_[i][j].col(2)(2) = cplex.getValue(rectangles[i][j][0]-rectangles[i][j][2]);
                opt_after_all_corrid_[i][j].col(2)(3) = cplex.getValue(rectangles[i][j][1]-rectangles[i][j][4]);
                opt_after_all_corrid_[i][j].col(2).head(2) <<0,-1;

                // double len_=cplex.getValue(rectangles[i][j][0])-cplex.getValue(rectangles[i][j][2]);
                // double width_=cplex.getValue(rectangles[i][j][1])-cplex.getValue(rectangles[i][j][3]);
                opt_after_all_corrid_[i][j].col(1)(2) = cplex.getValue(rectangles[i][j][0]+rectangles[i][j][3]);
                opt_after_all_corrid_[i][j].col(1)(3) = cplex.getValue(rectangles[i][j][1]-rectangles[i][j][4]);
                opt_after_all_corrid_[i][j].col(1).head(2) <<1,0;

                opt_after_all_corrid_[i][j].col(3)(2) = cplex.getValue(rectangles[i][j][0]-rectangles[i][j][2]);
                opt_after_all_corrid_[i][j].col(3)(3) = cplex.getValue(rectangles[i][j][1]+rectangles[i][j][5]);
                opt_after_all_corrid_[i][j].col(3).head(2) <<-1,0;


            // }
        }
    }

    } else {
        // 无解
        ROS_ERROR("RBPPlanner: Failed to optimize QP");


        //  if (cplex.refineConflict(model)) {
        //     IloCplex::ConflictStatusArray conflict(env);
        //     cplex.getConflict(conflict, model);
        //     std::cout << "Conflict refiner:" << std::endl;
        //     for (IloInt i = 0; i < conflict.getSize(); ++i) {
        //         if (conflict[i] == IloCplex::ConflictMember)
        //             std::cout << "Proved: " << model[i] << std::endl;
        //         else if (conflict[i] == IloCplex::ConflictPossibleMember)
        //             std::cout << "Possible: " << model[i] << std::endl;
        //     }
        // } else {
        //     std::cout << "Conflict refiner did not execute successfully." << std::endl;
        // }



    }
} catch (IloException& e) {
    std::cerr << "abnormal!!!!: " << e << std::endl;
} catch (...) {
    std::cerr << "unknown bnormal!!!!" << std::endl;
}
env.end();
    }
  

};








