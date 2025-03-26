#ifndef _UNIFORM_BSPLINE_H_
#define _UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include<fstream>

using namespace std;

namespace dy_obs
{
  // An implementation of non-uniform B-spline with different dimensions
  // It also represents uniform B-spline which is a special case of non-uniform
  class UniformBspline
  {
  private:
    // control points for B-spline with different dimensions.
    // Each row represents one single control point
    // The dimension is determined by column number
    // e.g. B-spline with N points in 3D space -> Nx3 matrix
    Eigen::MatrixXd control_points_;//控制点

    int p_, n_, m_;     // p degree, n+1 control points, m = n+p+1
    Eigen::VectorXd u_; // knots vector
    double interval_;   // knot span \delta t

    Eigen::MatrixXd getDerivativeControlPoints();//得到B样条曲线的导数的控制点

    double limit_vel_, limit_acc_, limit_ratio_, feasibility_tolerance_,limit_vel_yaw; // physical limits and time adjustment ratio

  public:
    UniformBspline() {}
    UniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);//控制点，阶数，时间间隔
    ~UniformBspline();

    Eigen::MatrixXd get_control_points(void) { return control_points_; }

    // initialize as an uniform B-spline
    void setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);

    // get / set basic bspline info

    void setKnot(const Eigen::VectorXd &knot);
    Eigen::VectorXd getKnot();
    Eigen::MatrixXd getControlPoint();
    double getInterval();
    bool getTimeSpan(double &um, double &um_p);

    // compute position / derivative

    Eigen::VectorXd evaluateDeBoor(const double &u);//首先定位到u在哪个区间内，然后使用deBoor-Cox递推公式求出u时刻B样条曲线的三维坐标。                                               // use u \in [up, u_mp]
    inline Eigen::VectorXd evaluateDeBoorT(const double &t) { return evaluateDeBoor(t + u_(p_)); } // use t \in [0, duration]
    UniformBspline getDerivative();//得到导数的控制点后，新初始化一个均匀B样条类，阶数为原来阶数-1，时间间隔一样，区间间隔为原来间隔的去头去尾，最终得到B样条曲线的导数曲线。

    // 3D B-spline interpolation of points in point_set, with boundary vel&acc
    // constraints
    // input : (K+2) points with boundary vel/acc; ts
    // output: (K+6) control_pts
    static void parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                      const vector<Eigen::Vector3d> &start_end_derivative,
                                      Eigen::MatrixXd &ctrl_pts);//使用拟合的方法将一段轨迹点拟合成均匀B样条函数，得到控制点。

    /* check feasibility, adjust time */

    void setPhysicalLimits(const double &vel, const double &acc, const double &tolerance,const double &vel_yaw);
    bool checkFeasibility(double &ratio, bool show = false);//验证轨迹的速度和加速度是否超过动力学限制（只检查各控制点处的大小）
    void lengthenTime(const double &ratio);//把轨迹的时间延长为原来的ratio倍

    /* for performance evaluation */

    double getTimeSum();
    double getLength(const double &res = 0.01);
    double getJerk();
    void getMeanAndMaxVel(double &mean_v, double &max_v);
    void getMeanAndMaxAcc(double &mean_a, double &max_a);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace ego_planner
#endif