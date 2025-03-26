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
class MyOptimizationProblem {
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
    MyOptimizationProblem(int car_num,std::vector<int> tnum_id,std::vector<std::vector<Eigen::MatrixXd>> all_corrid,
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
            rectangles[i][j] = IloNumVarArray(env, 4, 0, IloInfinity, ILOFLOAT); // 为每个矩形创建四个坐标变量
        }
    }
    IloExpr objective(env);
    for (int i = 0; i < n_cars; ++i) {
        for (int j = 0; j < n_times[i]; ++j) {
            objective += -((rectangles[i][j][0] - rectangles[i][j][2]) + (rectangles[i][j][1] - rectangles[i][j][3]))+0.2*(IloAbs((rectangles[i][j][0]+rectangles[i][j][2])/2-statelist_[i][j](0))+IloAbs((rectangles[i][j][1]+rectangles[i][j][3])/2-statelist_[i][j](1)));
            
            objective += 1.2*((all_corrid_[i][j].col(0)(2)-rectangles[i][j][0])+(all_corrid_[i][j].col(0)(3)-rectangles[i][j][1])+(rectangles[i][j][2]-all_corrid_[i][j].col(2)(2))+(rectangles[i][j][3]-all_corrid_[i][j].col(2)(3)));
            
            

            
            

            
            // objective += -((rectangles[i][j][0] - rectangles[i][j][2]) + (rectangles[i][j][1] - rectangles[i][j][3]));
      
        // if(j>0)
        //     {
        //         objective += 0.2*(IloAbs((rectangles[i][j][0]+rectangles[i][j][2])/2-(rectangles[i][j-1][0]+rectangles[i][j-1][2])/2)+IloAbs((rectangles[i][j][1]+rectangles[i][j][3])/2-(rectangles[i][j-1][1]+rectangles[i][j-1][3])/2));
        //     }
        }
    }
    for (int i = 0; i < n_cars; ++i) {
    for (int j = 0; j < n_times[i]-1; ++j) {
        // 计算相邻时间点矩形宽度的差异的绝对值
        objective += 0.05*IloAbs((rectangles[i][j+1][0] - rectangles[i][j+1][2])
                                  - (rectangles[i][j][0] - rectangles[i][j][2]));
        // 计算相邻时间点矩形高度的差异的绝对值
        objective += 0.05*IloAbs((rectangles[i][j+1][1] - rectangles[i][j+1][3])
                                  - (rectangles[i][j][1] - rectangles[i][j][3]));
    }
}

    model.add(IloMinimize(env, objective));
 IloNum M = 100000; // M 是一个大的常数
 double  epsilon=1e-6;

for (int t = 0; t < maxNumPoints; ++t) {
    for (int i = 0; i < n_cars; ++i) {
        for (int j = i + 1; j < n_cars; ++j) {
            if (t < n_times[i] && t < n_times[j]) {
                IloNumVarArray rectA = rectangles[i][t];
                IloNumVarArray rectB = rectangles[j][t];
                IloBoolVar zLeft(env,0,1), zRight(env,0,1), zTop(env,0,1), zBottom(env,0,1); // 二进制变量




                model.add(rectA[2] -rectB[0] + M * zLeft >=  epsilon );

                // A 在 B 的右边
                model.add(rectB[2] - rectA[0]+ M * zRight>=  epsilon );

                // A 在 B 的上边
                model.add(rectA[3] -rectB[1] + M * zTop >=   epsilon);

                // A 在 B 的下边
                model.add(rectB[3] - rectA[1]+ M * zBottom >=   epsilon );
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
                model.add(rectA[2] >= all_corrid_[i][t].col(2)(2) );

                // x大
                model.add(rectA[0] <= all_corrid_[i][t].col(0)(2));

                // 上边
                model.add(rectA[1] <= all_corrid_[i][t].col(0)(3));

                // 下边
                model.add(rectA[3] >= all_corrid_[i][t].col(2)(3));
        

                // model.add(rectA[0]-rectA[2] >= car_length*0.5);
                // model.add(rectA[1]-rectA[3] >= car_length*0.5);//狭窄时这里为*1，空旷*2/...
                
                if(smallest_all_corrid_[i][t].col(0)(2)>smallest_all_corrid_[i][t].col(2)(2) && smallest_all_corrid_[i][t].col(0)(3)>smallest_all_corrid_[i][t].col(2)(3)&&smallest_all_corrid_[i][t].col(0)(2)<=all_corrid_[i][t].col(0)(2)&& smallest_all_corrid_[i][t].col(2)(2) >=all_corrid_[i][t].col(2)(2)&&smallest_all_corrid_[i][t].col(0)(3)<=all_corrid_[i][t].col(0)(3)&&smallest_all_corrid_[i][t].col(2)(3) >=all_corrid_[i][t].col(2)(3))
                {
                    model.add(rectA[0]>=smallest_all_corrid_[i][t].col(0)(2) ); 
                    model.add(rectA[2]<=smallest_all_corrid_[i][t].col(2)(2) );  
                    model.add(rectA[1]>=smallest_all_corrid_[i][t].col(0)(3) ); 
                    model.add(rectA[3]<=smallest_all_corrid_[i][t].col(2)(3) );   

                }
              else  if(smallest_all_corrid_[i][t].col(0)(2)<smallest_all_corrid_[i][t].col(2)(2) && smallest_all_corrid_[i][t].col(0)(3)>smallest_all_corrid_[i][t].col(2)(3)&&smallest_all_corrid_[i][t].col(2)(2)<=all_corrid_[i][t].col(0)(2)&&smallest_all_corrid_[i][t].col(0)(2)>=all_corrid_[i][t].col(2)(2)&&smallest_all_corrid_[i][t].col(2)(3) >=all_corrid_[i][t].col(2)(3)&&smallest_all_corrid_[i][t].col(0)(3)<=all_corrid_[i][t].col(0)(3))
                {
                   model.add(rectA[0]>=smallest_all_corrid_[i][t].col(2)(2) ); 
                   model.add(rectA[2]<=smallest_all_corrid_[i][t].col(0)(2) );   
                   model.add(rectA[1]>=smallest_all_corrid_[i][t].col(0)(3) ); 
                   model.add(rectA[3]<=smallest_all_corrid_[i][t].col(2)(3) );   

                }
                else  if(smallest_all_corrid_[i][t].col(0)(2)<smallest_all_corrid_[i][t].col(2)(2) && smallest_all_corrid_[i][t].col(0)(3)<smallest_all_corrid_[i][t].col(2)(3)&&smallest_all_corrid_[i][t].col(2)(2)<=all_corrid_[i][t].col(0)(2)&&smallest_all_corrid_[i][t].col(0)(2)>=all_corrid_[i][t].col(2)(2)&&smallest_all_corrid_[i][t].col(2)(3)<=all_corrid_[i][t].col(0)(3)&&smallest_all_corrid_[i][t].col(0)(3)>=all_corrid_[i][t].col(2)(3))
                {
                   model.add(rectA[0]>=smallest_all_corrid_[i][t].col(2)(2) ); 
                   model.add(rectA[2]<=smallest_all_corrid_[i][t].col(0)(2) );   
                   model.add(rectA[1]>=smallest_all_corrid_[i][t].col(2)(3) ); 
                   model.add(rectA[3]<=smallest_all_corrid_[i][t].col(0)(3) );  

                }
                else  if(smallest_all_corrid_[i][t].col(0)(2)>smallest_all_corrid_[i][t].col(2)(2) && smallest_all_corrid_[i][t].col(0)(3)<smallest_all_corrid_[i][t].col(2)(3)&&smallest_all_corrid_[i][t].col(0)(2)<=all_corrid_[i][t].col(0)(2)&&smallest_all_corrid_[i][t].col(2)(2)>=all_corrid_[i][t].col(2)(2)&&smallest_all_corrid_[i][t].col(2)(3)<=all_corrid_[i][t].col(0)(3)&& smallest_all_corrid_[i][t].col(0)(3)>=all_corrid_[i][t].col(2)(3))
                {
                   model.add(rectA[0]>=smallest_all_corrid_[i][t].col(0)(2) ); 
                   model.add(rectA[2]<=smallest_all_corrid_[i][t].col(2)(2) );  
                   model.add(rectA[1]>=smallest_all_corrid_[i][t].col(2)(3) ); 
                   model.add(rectA[3]<=smallest_all_corrid_[i][t].col(0)(3) ); 

                }

                // if(smallest_all_corrid_[i][t].col(0)(2)>smallest_all_corrid_[i][t].col(2)(2))
                // {
                //      model.add(rectA[0]>=smallest_all_corrid_[i][t].col(0)(2) ); 
                //      model.add(rectA[2]<=smallest_all_corrid_[i][t].col(2)(2) );  
                // }
                // else
                // {
                //      model.add(rectA[0]>=smallest_all_corrid_[i][t].col(2)(2) ); 
                //      model.add(rectA[2]<=smallest_all_corrid_[i][t].col(0)(2) );  
                // }
                // if(smallest_all_corrid_[i][t].col(0)(3)>smallest_all_corrid_[i][t].col(2)(3))
                // {
                //     model.add(rectA[1]>=smallest_all_corrid_[i][t].col(0)(3) ); 
                //     model.add(rectA[3]<=smallest_all_corrid_[i][t].col(2)(3) );  
                // }
                // else
                // {
                //     model.add(rectA[1]>=smallest_all_corrid_[i][t].col(2)(3) ); 
                //     model.add(rectA[3]<=smallest_all_corrid_[i][t].col(0)(3) ); 
                // }

                // model.add(((rectA[0]+rectA[2]/2)-statelist_[i][t](0))<= car_length*3 || ((rectA[0]+rectA[2]/2)-statelist_[i][t](0))>= -car_length*3);
                // model.add((((rectA[1]+rectA[3]/2)-statelist_[i][t](0))<= car_length*3)||(((rectA[1]+rectA[3]/2)-statelist_[i][t](0))>= -car_length*3));    

             

                // 至少一个方向上的约束必须被满足
             
            
        
    }
}

  for (int i = 0; i < n_cars; i++) {   
        for (int t = 0; t < n_times[i]-1; t++) {        
                IloNumVarArray rectA = rectangles[i][t];
                IloNumVarArray rectB= rectangles[i][t+1];


                model.add(rectA[2] <= rectB[0] ); // A的左边界在B的右边界右侧
                model.add(rectA[0] >= rectB[2]); // A的右边界在B的左边界左侧

                // 在 y 轴上相交
                model.add(rectA[3] <= rectB[1]); // A的下边界在B的上边界上方
                model.add(rectA[1] >= rectB[3]); // A的上边界在B的下边界下方
                IloBoolVar z1(env,0,1), z2(env,0,1), z3(env,0,1), z4(env,0,1); // 二进制变量



                model.add((rectB[2]-rectA[0])+(rectA[3]-rectB[1])+M*z1>=car_width*1.2 ); 
            

                model.add((rectA[2]-rectB[0])+(rectB[3]-rectA[1])+M*z2>=car_width*1.2 ); 
            

                model.add((rectB[2]-rectA[0])+(rectB[3]-rectA[1])+M*z3>=car_width*1.2 ); 
            

                model.add((rectA[2]-rectB[0])+(rectA[3]-rectB[1])+M*z4>=car_width*1.2 ); 
                model.add(z1 + z2 + z3 + z4 <= 4);
                


                
                model.add( IloAbs((rectA[0]+rectA[2])/2-(rectB[0]+rectB[2])/2)<=car_length*6);
                model.add(IloAbs((rectA[1]+rectA[3])/2-(rectB[1]+rectB[3])/2)<=car_length*6); 
                model.add( IloAbs((rectA[0]+rectA[2])/2-(rectB[0]+rectB[2])/2)>=0);
                model.add(IloAbs((rectA[1]+rectA[3])/2-(rectB[1]+rectB[3])/2)>=0); 
                 


               
            
        
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
                opt_after_all_corrid_[i][j].col(0)(2) = cplex.getValue(rectangles[i][j][0]);
                opt_after_all_corrid_[i][j].col(0)(3) = cplex.getValue(rectangles[i][j][1]);
                opt_after_all_corrid_[i][j].col(0).head(2) <<0,1;

                opt_after_all_corrid_[i][j].col(2)(2) = cplex.getValue(rectangles[i][j][2]);
                opt_after_all_corrid_[i][j].col(2)(3) = cplex.getValue(rectangles[i][j][3]);
                opt_after_all_corrid_[i][j].col(2).head(2) <<0,-1;

                // double len_=cplex.getValue(rectangles[i][j][0])-cplex.getValue(rectangles[i][j][2]);
                double width_=cplex.getValue(rectangles[i][j][1])-cplex.getValue(rectangles[i][j][3]);
                opt_after_all_corrid_[i][j].col(1)(2) = cplex.getValue(rectangles[i][j][0]);
                opt_after_all_corrid_[i][j].col(1)(3) = cplex.getValue(rectangles[i][j][3]);
                opt_after_all_corrid_[i][j].col(1).head(2) <<1,0;

                opt_after_all_corrid_[i][j].col(3)(2) = cplex.getValue(rectangles[i][j][2]);
                opt_after_all_corrid_[i][j].col(3)(3) = cplex.getValue(rectangles[i][j][1]);
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









