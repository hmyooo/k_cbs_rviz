/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef _OBJ_PREDICTOR_H_
#define _OBJ_PREDICTOR_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <list>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using std::cout;
using std::endl;
using std::list;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class PolynomialPrediction;
typedef shared_ptr<vector<PolynomialPrediction>> ObjPrediction;
typedef shared_ptr<vector<Eigen::Vector3d>> ObjScale;

/* ========== prediction polynomial ========== */
class PolynomialPrediction {
private:
  vector<Eigen::Matrix<double, 6, 1>> polys;
  vector<Eigen::Matrix<double, 4, 1>> polyss;

  double t1, t2;  // start / end
  ros::Time global_start_time_;

public:
  PolynomialPrediction(/* args */) {
  }
  ~PolynomialPrediction() {
  }

  void setPolynomial(vector<Eigen::Matrix<double, 6, 1>>& pls) {
    polys = pls;
  }
  void setPolynomial_dyobs(vector<Eigen::Matrix<double, 4, 1>>& pls) {
    // polyss = pls;
    polyss.swap(pls);
  }
  void setTime(double t1, double t2) {
    this->t1 = t1;
    this->t2 = t2;
  }
  void setGlobalStartTime(ros::Time global_start_time) {
    global_start_time_ = global_start_time;
  }

  bool valid() {
    return polys.size() == 3;
  }

  /* note that t should be in [t1, t2] */
  Eigen::Vector3d evaluate(double t) {
    Eigen::Matrix<double, 6, 1> tv;
    tv << 1.0, pow(t, 1), pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);

    Eigen::Vector3d pt;
    pt(0) = tv.dot(polys[0]), pt(1) = tv.dot(polys[1]), pt(2) = tv.dot(polys[2]);

    return pt;
  }

  Eigen::Vector3d evaluateConstVel(double t) {
    Eigen::Matrix<double, 2, 1> tv;
    tv << 1.0, pow(t-global_start_time_.toSec(), 1);

    // cout << t-global_start_time_.toSec() << endl;

    Eigen::Vector3d pt;
    pt(0) = tv.dot(polys[0].head(2)), pt(1) = tv.dot(polys[1].head(2)), pt(2) = tv.dot(polys[2].head(2));
    

    return pt;
  }
    Eigen::Vector3d evaluateConstVel_dyobs(double t) {
    Eigen::Matrix<double, 2, 1> tv;
    tv << 1.0, pow(t, 1);

    cout <<"-global"<< t-global_start_time_.toSec() << endl;
    cout <<"not -"<< t << endl;

    Eigen::Vector3d pt;
    pt(0) = tv.dot(polyss[0].head(2)), pt(1) = tv.dot(polyss[1].head(2)), pt(2) = t;
    

    return pt;
  }
};

/* ========== subscribe and record object history ========== */
class ObjHistory {
public:
  int skip_num_;
  int queue_size_;
  ros::Time global_start_time_;

  ObjHistory() {
  }
  ~ObjHistory() {
  }

  void init(int id, int skip_num, int queue_size, ros::Time global_start_time);

  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  void clear() {
    history_.clear();
  }

  void getHistory(list<Eigen::Vector4d>& his) {
    his = history_;
  }

private:
  list<Eigen::Vector4d> history_;  // x,y,z;t
  int skip_;
  int obj_idx_;
  Eigen::Vector3d scale_;
};

/* ========== predict future trajectory using history ========== */
class ObjPredictor {
private:
  ros::NodeHandle node_handle_;

  int obj_num_;
  double lambda_;
  double predict_rate_;

  vector<ros::Subscriber> pose_subs_;
  ros::Subscriber marker_sub_;
  ros::Timer predict_timer_;
  vector<shared_ptr<ObjHistory>> obj_histories_;

  /* share data with planner */
  ObjPrediction predict_trajs_;
  
  ObjScale obj_scale_;
  vector<bool> scale_init_;

  void markerCallback(const visualization_msgs::MarkerConstPtr& msg);

  void predictCallback(const ros::TimerEvent& e);
  void predictPolyFit();
  void predictConstVel();

public:
  ObjPredictor(/* args */);
  ObjPredictor(ros::NodeHandle& node);
  ~ObjPredictor();

  void init();
  PolynomialPrediction predict_dyobs_trajs_;

  ObjPrediction getPredictionTraj();
  PolynomialPrediction get_dyobs_PredictionTraj();
  ObjScale getObjScale();
  int getObjNums() {return obj_num_;}

  Eigen::Vector3d evaluatePoly(int obs_id, double time);
  Eigen::Vector3d evaluateConstVel(int obs_id, double time);
  Eigen::Vector3d evaluateConstVel_dyobs(list<Eigen::Vector3d> his,double time);
  typedef shared_ptr<ObjPredictor> Ptr;
};

}  // namespace fast_planner

#endif