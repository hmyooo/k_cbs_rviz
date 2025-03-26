#include <traj_utils/planning_visualization.h>

using std::cout;
using std::endl;
namespace dy_obs
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud_back(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud_back_curr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud_trac(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  PlanningVisualization::PlanningVisualization(ros::NodeHandle &nh,int car_id_,std::string mapback,std::string maptrac)
  {
    node = nh;
    std::string str1 = "goal_point";
    // std::string str6 = "/planning/local_target";
    std::string str2 = "global_list";
    std::string str3 = "init_list";
    std::string str4 = "optimal_list";
    std::string str5 = "a_star_list";

    std::string goal_point_pub_topic=str1+std::to_string(car_id_);
    std::string global_list_pub_topic=str2+std::to_string(car_id_);
    std::string init_list_pub_topic=str3+std::to_string(car_id_);
    std::string optimal_list_pub_topic=str4+std::to_string(car_id_);
    std::string a_star_list_pub_topic=str5+std::to_string(car_id_);

    goal_point_pub = nh.advertise<visualization_msgs::Marker>(goal_point_pub_topic.c_str(), 2);
    global_list_pub = nh.advertise<visualization_msgs::Marker>(global_list_pub_topic.c_str(), 2);
    init_list_pub = nh.advertise<visualization_msgs::Marker>(init_list_pub_topic.c_str(), 2);
    optimal_list_pub = nh.advertise<visualization_msgs::Marker>(optimal_list_pub_topic.c_str(), 2);
    a_star_list_pub = nh.advertise<visualization_msgs::Marker>(a_star_list_pub_topic.c_str(), 20);
    curren_point_pub= nh.advertise<visualization_msgs::Marker>("current_point", 2);
    localstart_point_pub= nh.advertise<visualization_msgs::Marker>("local_start_point", 2);
    localtarget_point_pub= nh.advertise<visualization_msgs::Marker>("localtarget_point", 2);
    occu_point_pub= nh.advertise<visualization_msgs::Marker>("occu_point", 2);
    _all_map_pub = nh.advertise<sensor_msgs::PointCloud2>("local_cloud_back", 10);
    _all_map_pub_trac = nh.advertise<sensor_msgs::PointCloud2>("local_cloud_trac", 10);
    _all_map_pub_local = nh.advertise<sensor_msgs::PointCloud2>("local_cloud_map", 10);
    pred_pub = nh.advertise<visualization_msgs::Marker>("pred_dyobs", 2);
    occu_path= nh.advertise<visualization_msgs::Marker>("occu_path", 2);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(mapback, *pcd_cloud_back) == -1) {
    PCL_ERROR("Couldn't read file0\n");
    
  }
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(maptrac, *pcd_cloud_trac) == -1) {
    PCL_ERROR("Couldn't read file1 \n");
    
  }
   pcl::getMinMax3D(*pcd_cloud_trac,pointmin,pointmax);
    // image_transport::ImageTransport it(nh);
  }

  // // real ids used: {id, id+1000}
  void PlanningVisualization::displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                                                Eigen::Vector4d color, int id, bool show_sphere /* = true */ )
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale ;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);//修改
      // pt.z = list[i](2);//修改

      //if (show_sphere) sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    // if (show_sphere) pub.publish(sphere);
    pub.publish(line_strip);
  }

  // real ids used: {id, id+1}
  void PlanningVisualization::generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                                       const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "map";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 3;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    array.markers.push_back(sphere);
    array.markers.push_back(line_strip);
  }

  // real ids used: {1000*id ~ (arrow nums)+1000*id}
  void PlanningVisualization::generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                                        const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp = ros::Time::now();
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    // geometry_msgs::Point start, end;
    // arrow.points

    arrow.color.r = color(0);
    arrow.color.g = color(1);
    arrow.color.b = color(2);
    arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    arrow.scale.x = scale;
    arrow.scale.y = 2 * scale;
    arrow.scale.z = 2 * scale;

    geometry_msgs::Point start, end;
    for (int i = 0; i < int(list.size() / 2); i++)
    {
      // arrow.color.r = color(0) / (1+i);
      // arrow.color.g = color(1) / (1+i);
      // arrow.color.b = color(2) / (1+i);

      start.x = list[2 * i](0);
      start.y = list[2 * i](1);
      start.z = list[2 * i](2);
      end.x = list[2 * i + 1](0);
      end.y = list[2 * i + 1](1);
      end.z = list[2 * i + 1](2);
      arrow.points.clear();
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      arrow.id = i + id * 1000;

      array.markers.push_back(arrow);
    }
  }

  void PlanningVisualization::displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id)
  {
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;

    sphere.pose.orientation.w = 1.0;
    sphere.color.r = color(0);
    sphere.color.g = color(1);
    sphere.color.b = color(2);
    sphere.color.a = color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = goal_point(0);
    sphere.pose.position.y = goal_point(1);
    sphere.pose.position.z = goal_point(2);

    goal_point_pub.publish(sphere);
  }
  // void PlanningVisualization::mapdiaplay(std::string filemap)
  // {

  //     image_transport::Publisher pub = it.advertise("camera/image", 1);
     
  //     cv::Mat image = cv::imread(filemap, CV_LOAD_IMAGE_COLOR);
  //     if(image.empty()){
  //      printf("open error\n");
  //      }
  //     sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  //     pub.publish(msg);



  // }
  void PlanningVisualization::pubSensedPoints(Eigen::Vector3d curpos) {
      cout<<"curpos 2 "<<curpos(2)<<endl;
      cout<<"pointmax  "<<pointmax.z<<endl;
      if(curpos(2)<(pointmax.z/10))
    {
      //局部点云
      cout<<"pcd_cloud_trac size "<<pcd_cloud_trac->points.size()<<endl;
      cout<<"pcd_cloud_back size "<<pcd_cloud_back->points.size()<<endl;

      pcl::PassThrough<pcl::PointXYZ> passthrough;
      //输入点云1
      passthrough.setInputCloud(pcd_cloud_trac);
      //设置对z轴进行操作
      passthrough.setFilterFieldName("z");
      //设置滤波范围
      passthrough.setFilterLimits(curpos(2)*10,curpos(2)*10+30);//当前位置6s预测地图，20层3Bei即为60;30为3s
      

     //   passthrough.setFilterLimits(DAgentPaths[posebagid].poses.back().pose.position.z*3, DAgentPaths[posebagid].poses.back().pose.position.z*3+60);//当前位置6s预测地图，20层3Bei即为60
      // true表示保留滤波范围外，false表示保留范围内
      passthrough.setFilterLimitsNegative(false);
      //执行滤波并存储
      passthrough.filter(*cloud_after_PassThrough);
      int j=0;
      pcd_cloud_back_curr->clear();
      for(int ipoint=0;ipoint<pcd_cloud_back->points.size();ipoint++)
      {
            pcl::PointXYZ p;
            {
            p.x = pcd_cloud_back->points[ipoint].x;

            p.y = pcd_cloud_back->points[ipoint].y;

            // p.z = pcd_cloud_back->points[ipoint].z+curposez*10;//3s一取
            p.z = pcd_cloud_back->points[ipoint].z/10+curpos(2);//3s一取


            pcd_cloud_back_curr->push_back(p);
            j++;
            }
   

           
      }
      for(int ipoint=0;ipoint<cloud_after_PassThrough->points.size();ipoint++)
      {
           

        cloud_after_PassThrough->points[ipoint].z=cloud_after_PassThrough->points[ipoint].z/10;//3s一取
      }
      pcd_cloud_back_curr->height = 1;//对于无序点云hight默认是1

      pcd_cloud_back_curr->width =j;//cloud_extract点云文件中push_back了j个点，故width=j
      local_cloud->clear();
      (*local_cloud)=(*cloud_after_PassThrough)+(*pcd_cloud_back_curr);
     //   countcurr++;
     //   cout<<"countcurr"<<countcurr<<endl;
      // cout<<"localcloud size is"<<local_cloud->points.size()<<endl;

      // kdtreeLocalMap.setInputCloud(local_cloud->makeShared());
      cout<<"11111111111"<<endl;
      pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
      cout<<"222222222222"<<endl;
      kdtreeLocalMap.setInputCloud(local_cloud);
      cout<<"333333333333333"<<endl;

      pcl::PointCloud<pcl::PointXYZ> localMap;
      pcl::PointXYZ searchPoint(curpos(0), curpos(1), curpos(2));
      vector<int>pointIdxRadiusSearch;//存放搜到的最近的点的地址
      vector<float>pointRadiusSquaredDistance;//存储近邻点对应的距离平方
      double radius = 30;
      cout<<"44444444444444444"<<endl;
      pointIdxRadiusSearch.clear();
        pointRadiusSquaredDistance.clear();
      pcl::PointXYZ pt;
      cout<<"555555555555555555"<<endl;

      if (kdtreeLocalMap.radiusSearch(searchPoint, radius,
                                    pointIdxRadiusSearch,
                                    pointRadiusSquaredDistance) > 0) {
      for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
        pt = local_cloud->points[pointIdxRadiusSearch[i]];
        localMap.points.push_back(pt);
      }
     } else {
      ROS_ERROR("[Map server] No obstacles .");
      return;
     }
      cout<<"6666666666666"<<endl;


     if (pcd_cloud_back_curr->points.size()>0) 
     {
      pcl::toROSMsg(*pcd_cloud_back_curr, localMap_pcd);
     // globalMap_pcd.header.frame_id = "world";
     // _all_map_pub.publish(globalMap_pcd);
      localMap_pcd.header.frame_id = "world";
      _all_map_pub.publish(localMap_pcd);
      // cout<<"if nor"<<endl;
      }

      if (cloud_after_PassThrough->points.size()>0) 
     {
      pcl::toROSMsg(*cloud_after_PassThrough, localMap_pcd_trac);
     // globalMap_pcd.header.frame_id = "world";
     // _all_map_pub.publish(globalMap_pcd);
      localMap_pcd_trac.header.frame_id = "world";
      _all_map_pub_trac.publish(localMap_pcd_trac);
      // cout<<"if nor"<<endl;
      }
      //    if (local_cloud->points.size()>0) 
      // {
      //   pcl::toROSMsg(*local_cloud, localMap_);
      // // globalMap_pcd.header.frame_id = "world";
      // // _all_map_pub.publish(globalMap_pcd);
      //   localMap_.header.frame_id = "world";
      //   _all_map_pub_local.publish(localMap_);
      //   // cout<<"if nor"<<endl;
      //    }
        if (localMap.points.size()>0) 
     {
      pcl::toROSMsg(localMap,localMap_);
     // globalMap_pcd.header.frame_id = "world";
     // _all_map_pub.publish(globalMap_pcd);
      localMap_.header.frame_id = "world";
      _all_map_pub_local.publish(localMap_);
      // cout<<"if nor"<<endl;
      }

    }


}
  void PlanningVisualization::displaycurrPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id)
  {
    // visualization_msgs::Marker sphere,line_strip;
    visualization_msgs::Marker sphere;

        
    // line_strip.header.frame_id = "world";
    // line_strip.header.stamp = ros::Time::now();
    
    // line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    // line_strip.action = visualization_msgs::Marker::ADD;
    // line_strip.id = id + 100;

    // line_strip.pose.orientation.w = 1.0;
    // sphere.color.r = line_strip.color.r = color(0);
    // sphere.color.g = line_strip.color.g = color(1);
    // sphere.color.b = line_strip.color.b = color(2);
    // sphere.color.a = line_strip.color.a = color(3);

    // line_strip.scale.x = scale ;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    sphere.color.r = color(0);
    sphere.color.g = color(1);
    sphere.color.b = color(2);
    sphere.color.a = color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = goal_point(0);
    sphere.pose.position.y = goal_point(1);
    sphere.pose.position.z = goal_point(2);
    sphere.pose.orientation.x = 0.0;
    sphere.pose.orientation.y = 0.0;
    sphere.pose.orientation.z = 0.0;
    sphere.pose.orientation.w = 1.0;
    //   geometry_msgs::Point pt;
    
    //   pt.x = goal_point(0);
    //   pt.y = goal_point(1);
    //   pt.z = goal_point(2);//修改
    //   // pt.z = list[i](2);//修改

    //   //if (show_sphere) sphere.points.push_back(pt);
    //   line_strip.points.push_back(pt);
    // if(line_strip.points.size()>1)
    // {
    //   curren_point_pub.publish(line_strip);
    // }

    curren_point_pub.publish(sphere);
  }
    void PlanningVisualization::displaylocalstartPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id)
  {
    // visualization_msgs::Marker sphere,line_strip;
    visualization_msgs::Marker sphere;

        
    // line_strip.header.frame_id = "world";
    // line_strip.header.stamp = ros::Time::now();
    
    // line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    // line_strip.action = visualization_msgs::Marker::ADD;
    // line_strip.id = id + 100;

    // line_strip.pose.orientation.w = 1.0;
    // sphere.color.r = line_strip.color.r = color(0);
    // sphere.color.g = line_strip.color.g = color(1);
    // sphere.color.b = line_strip.color.b = color(2);
    // sphere.color.a = line_strip.color.a = color(3);

    // line_strip.scale.x = scale ;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    sphere.color.r = color(0);
    sphere.color.g = color(1);
    sphere.color.b = color(2);
    sphere.color.a = color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = goal_point(0);
    sphere.pose.position.y = goal_point(1);
    sphere.pose.position.z = goal_point(2);
    sphere.pose.orientation.x = 0.0;
    sphere.pose.orientation.y = 0.0;
    sphere.pose.orientation.z = 0.0;
    sphere.pose.orientation.w = 1.0;
    //   geometry_msgs::Point pt;
    
    //   pt.x = goal_point(0);
    //   pt.y = goal_point(1);
    //   pt.z = goal_point(2);//修改
    //   // pt.z = list[i](2);//修改

    //   //if (show_sphere) sphere.points.push_back(pt);
    //   line_strip.points.push_back(pt);
    // if(line_strip.points.size()>1)
    // {
    //   curren_point_pub.publish(line_strip);
    // }

    localstart_point_pub.publish(sphere);
  }
  void PlanningVisualization::displayoccuPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id)
  {
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    sphere.color.r = color(0);
    sphere.color.g = color(1);
    sphere.color.b = color(2);
    sphere.color.a = color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = goal_point(0);
    sphere.pose.position.y = goal_point(1);
    sphere.pose.position.z = goal_point(2);
    sphere.pose.orientation.x = 0.0;
    sphere.pose.orientation.y = 0.0;
    sphere.pose.orientation.z = 0.0;
    sphere.pose.orientation.w = 1.0;
    
    occu_point_pub.publish(sphere);
  }
  void PlanningVisualization::displaylocaltargetPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id)
  {
    // visualization_msgs::Marker sphere,line_strip;
    visualization_msgs::Marker sphere;

        
    // line_strip.header.frame_id = "world";
    // line_strip.header.stamp = ros::Time::now();
    
    // line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    // line_strip.action = visualization_msgs::Marker::ADD;
    // line_strip.id = id + 100;

    // line_strip.pose.orientation.w = 1.0;
    // sphere.color.r = line_strip.color.r = color(0);
    // sphere.color.g = line_strip.color.g = color(1);
    // sphere.color.b = line_strip.color.b = color(2);
    // sphere.color.a = line_strip.color.a = color(3);

    // line_strip.scale.x = scale ;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    sphere.color.r = color(0);
    sphere.color.g = color(1);
    sphere.color.b = color(2);
    sphere.color.a = color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = goal_point(0);
    sphere.pose.position.y = goal_point(1);
    sphere.pose.position.z = goal_point(2);
    sphere.pose.orientation.x = 0.0;
    sphere.pose.orientation.y = 0.0;
    sphere.pose.orientation.z = 0.0;
    sphere.pose.orientation.w = 1.0;
    //   geometry_msgs::Point pt;
    
    //   pt.x = goal_point(0);
    //   pt.y = goal_point(1);
    //   pt.z = goal_point(2);//修改
    //   // pt.z = list[i](2);//修改

    //   //if (show_sphere) sphere.points.push_back(pt);
    //   line_strip.points.push_back(pt);
    // if(line_strip.points.size()>1)
    // {
    //   curren_point_pub.publish(line_strip);
    // }

    localtarget_point_pub.publish(sphere);
  }
  void PlanningVisualization::displayGlobalPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    if (global_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 0.5, 0.5, 1);
    displayMarkerList(global_list_pub, init_pts, scale, color, id);
  }

  void PlanningVisualization::displayMultiInitPathList(vector<vector<Eigen::Vector3d>> init_trajs, const double scale)
  {

    if (init_list_pub.getNumSubscribers() == 0)
    {
      cout<<"no subscribe"<<endl;
      return;
    }

    static int last_nums = 0;
    cout<<"init_trajs.size() "<<init_trajs.size()<<endl;
    for ( int id=0; id<last_nums; id++ )
    {
      cout<<"inin1"<<endl;
      Eigen::Vector4d color(0, 0, 0, 0);
      vector<Eigen::Vector3d> blank;
      displayMarkerList(init_list_pub, blank, scale, color, id, false);
      ros::Duration(0.001).sleep();
    }
    last_nums = 0;

    for ( int id=0; id<init_trajs.size(); id++ )
    {
      cout<<"inin2"<<endl;
      Eigen::Vector4d color(0, 0, 1, 0.7);
      displayMarkerList(init_list_pub, init_trajs[id], scale, color, id, false);
      ros::Duration(0.001).sleep();
      last_nums++;
    }

  }

  void PlanningVisualization::displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    if (init_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 0, 1, 1);
    displayMarkerList(init_list_pub, init_pts, scale, color, id);
  }

  void PlanningVisualization::occuPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    if (occu_path.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color= Eigen::Vector4d(0.5 + ((double)rand() / RAND_MAX / 2), 0.5 + ((double)rand() / RAND_MAX / 2), 0, 1);
    displayMarkerList(occu_path, init_pts, scale, color, id);
  }

    void PlanningVisualization::displaypredictpath(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    if (pred_pub.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 0, 1, 1);
    displayMarkerList(pred_pub, init_pts, scale, color, id);
  }

  void PlanningVisualization::displayOptimalList(Eigen::MatrixXd optimal_pts, int id)
  {

    if (optimal_list_pub.getNumSubscribers() == 0)
    {
      return;
    }
    std::ofstream ofs;
    // ofs.open("/home/rancho/1hmy/ego-planner-swarm/record/optimize.txt",std::ios::app);
    //    for (int i = 0; i < optimal_pts.cols(); i++)
    // {
   
    // ofs<<"x " <<optimal_pts.col(i).transpose()(0)<<endl;
    // ofs<<"y " <<optimal_pts.col(i).transpose()(1)<<endl;
    // ofs<<"t " <<optimal_pts.col(i).transpose()(2)<< endl;//修改
    // }
    // ofs<<"end " << endl;//修改
    // ofs<< endl;//修改

    // ofs.close();
    vector<Eigen::Vector3d> list;
    for (int i = 0; i < optimal_pts.cols(); i++)
    {
      Eigen::Vector3d pt = optimal_pts.col(i).transpose();
      list.push_back(pt);
    }
    cout<<"list.size "<<list.size()<<endl;
    cout<<"optimal_pts.size "<<optimal_pts.cols()<<endl;

    Eigen::Vector4d color(1, 0, 0, 1);
    displayMarkerList(optimal_list_pub, list, 1.8, color, id);
  }

  void PlanningVisualization::displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id /* = Eigen::Vector4d(0.5,0.5,0,1)*/)
  {

    if (a_star_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    int i = 0;
    vector<Eigen::Vector3d> list;

    Eigen::Vector4d color = Eigen::Vector4d(0.5 + ((double)rand() / RAND_MAX / 2), 0.5 + ((double)rand() / RAND_MAX / 2), 0, 1); // make the A star pathes different every time.
    double scale = 0.05 + (double)rand() / RAND_MAX / 10;

    for (auto block : a_star_paths)
    {
      list.clear();
      for (auto pt : block)
      {
        list.push_back(pt);
      }
      //Eigen::Vector4d color(0.5,0.5,0,1);
      displayMarkerList(a_star_list_pub, list, scale, color, id + i); // real ids used: [ id ~ id+a_star_paths.size() ]
      i++;
    }
  }

  void PlanningVisualization::displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::MarkerArray array;
    // clear
    pub.publish(array);

    generateArrowDisplayArray(array, list, scale, color, id);

    pub.publish(array);
  }

  // PlanningVisualization::
} // namespace 