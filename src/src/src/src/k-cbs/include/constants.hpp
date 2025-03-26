#pragma once
#include <vector>
#include <cmath>
#include <string.h>
#include <iostream>
#include <Eigen/Eigen>

////###################################################
////                      INFO
////###################################################
//    HEADING => 0 - 359 degrees, 0 being north pointing towards positive Y
//    X-COORDINATE => designating the width of the grid
//    Y-COORDINATE => designating the height of the grid
/**
 * 约定: 
 *     HEADING： [0, 359]度，0表示朝向北(指向Y)
 *     X - 表示网格的宽度
 *     Y - 表示网格的高度
 */

namespace Constants {






//###################################################
//                    静态障碍物参数
//###################################################

// m
static float obs_r = 2.0;

//###################################################
//                    COMMON参数
//###################################################
/// [m] --- 最小转弯半径
static const float r = 1;//改

// [rad] 最小转向角
static float deltat = 6.75 / 180.0 * M_PI;


// least time to wait for constraint
static int constraintWaitTime = 2;
static int CALTime = 0;
static const bool show_path_flag = false;
static double obssafety_r = 40;
// [#] --- A movement cost penalty for turning (choosing non straight motion primitives)
//  只有这一个在holo和ackerman都用到了
static float penaltyTurning = 2.0;
static float penaltyWaiting = 4.0;

static bool same_time_falg = false;


//###################################################
//               在MODEL_LOOKUP中用到的参数
//###################################################
// [m] --- 车体朝向的离散数量
static const int headings = 72;
//朝向离散步长(以弧度表示)
static const float deltaHeadingRad = 2 * M_PI / (double)headings;


//每个位置的cell的分辨率，表示该cell下有多少个划分的小网格
/// [#] --- 每个cell里的离散位置数量的平方根
// 在collision_lookup中用到
static const int positionResolution = 10;
/// [#] --- 位置的数量
//实际就是 positionResolution * positionResolution，表示数量
static const int positions = positionResolution * positionResolution;
struct relPos {//相对于中心的位置：即以中心为坐标原点
  /// the x position relative to the center
  int x; 
  /// the y position relative to the center
  int y;
};
/// A structure capturing the lookup for each theta configuration
struct config {//用以获取每个theta的查找表的结构体
  /// the number of cells occupied by this configuration of the vehicle
  int length;//长度，
  /*!
    \var relPos pos[64]
    \brief The maximum number of occupied cells
    \todo needs to be dynamic
  */
  relPos pos[9999];
};

// map resolution 分辨率现在设置为1 一个格相当于1m
static float mapResolution = 1.0;
// change to set calcIndex resolution
static float yawResolution = 5.0 / 180.0 * M_PI;;
////###################################################
////                      map的参数 改成ros需要改
////###################################################
struct size{
   size(int w, int h):width(w),height(h){}
  int width;
  int height;
};
struct map{
  map(size z):info(z){
      data=new int[z.width*z.height];
  }
  size info;
  int *data;
};
class Size {
public:
  Size(int w, int l,int h ) : width(w), length(l), height(h) {}

  int width;
  int height;
  int length;
};



////###################################################
////                    参数 还需斟酌
////###################################################









// Important!
//###################################################
//                    Holonomic Model参数 
//###################################################
// Holonomic Model
static float holoCarWidth = 2.0;
static float holoStepLength = r * deltat;





//###################################################
//                    Holonomic Model参数 
//###################################################
// Ackermann Model
// width of car
// static float ackerWidth = 0.690000;
static float ackerWidth = 1.75;//改

// distance from rear to vehicle front end
// static float ackerLF = 0.482;
static float ackerLF = 1.68;//改

// distance from rear to vehicle back end
// static float ackerLB = 0.482;
static float ackerLB = 1.68;//改

static float ackerLength = ackerLF + ackerLB;
// static float ackerboating = 0.1;
static float ackerboating = 0.0;


// [#] --- A movement cost penalty for reversing (choosing motion primitives > 2)
static float penaltyReversing = 3.0;

// [#] --- A movement cost penalty for change of direction (changing from
// primitives < 3 to primitives > 2)
static float penaltyCOD = 2.5;

// CONFIG FLAGS 
//true表示可以倒退；false表示只能前进不能倒退
static const bool reverse = true;
//Dubin路径的切换开关: 若车子可以倒退，值为false
static const bool dubins = false;

//最大速度和最大加速度
/******** input0 参数 *****/
static float maxAcc = 3;//改
static float maxVel = 6;//改
// static float maxAcc = 1;
// static float maxVel = 3;
// static float minVel = 2;//改
static float minVel = 0.1;

// static float accResolution = 1;//改
static float accResolution = 0.1;

static float timeResolution = 1.0;
//前轮最大转向角（rad/s）
static float maxSteerphi = 15.f/180.f* M_PI;
//前轮转向角（rad/s）
///******** input0 参数 *****/
// static std::vector<double> steerphi =std::vector<double>({-15.f/180.f* M_PI,-25.f/180.f* M_PI,-35.f/180.f* M_PI,-45.f/180.f* M_PI,15.f/180.f* M_PI,25.f/180.f* M_PI,35.f/180.f* M_PI,45.f/180.f* M_PI});
// static std::vector<double> steerphi =std::vector<double>({15.f/180.f* M_PI});
static std::vector<double> steerphi =std::vector<double>({25.f/180.f* M_PI,  35.f/180.f* M_PI,45.f/180.f* M_PI});



static inline float normalizeHeadingRad(float t) {
  if (t < 0) {
    t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
    return 2.f * M_PI + t;
  }

  return t - 2.f * M_PI * (int)(t / (2.f * M_PI));
}
static inline float Judgment_number(double t) {
    if(t > 0) return 1;
    else if (t < 0) return -1;
    else return 0;
}

}





