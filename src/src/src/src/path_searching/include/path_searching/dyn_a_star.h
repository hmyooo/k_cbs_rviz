#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "opencv2/opencv.hpp"
#include <boost/heap/fibonacci_heap.hpp>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include<fstream>
#include<stdlib.h>
// #include<k-cbs/path_cal.hpp>

using namespace std;
using namespace Eigen;

constexpr double inf = 1 >> 20;
struct gridmap{
    int ifoccupy;
    int time;
};


struct Node {
    Node(const Vector3d& state, double fScore, double gScore)
        : state(state), fScore(fScore), gScore(gScore) {}

    bool operator<(const Node& other) const {
      // Sort order
      // 1. lowest fScore
      // 2. highest gScore

      // Our heap is a maximum heap, so we invert the comperator function here
      if (fScore != other.fScore) {
        return fScore > other.fScore;
      } else {
        return gScore < other.gScore;
      }
    }

    friend std::ostream& operator<<(std::ostream& os, const Node& node) {
      os << "state: " << node.state(0)<<node.state(1)<<node.state(2) << " fScore: " << node.fScore
         << " gScore: " << node.gScore;
      return os;
    }

    Vector3d state;

    double fScore;
    double gScore;

    typename boost::heap::fibonacci_heap<Node>::handle_type handle;

};





// const std::shared_ptr<Model>& single_model;

class AStar{
	public:
	typedef std::shared_ptr<AStar> Ptr;

	AStar(){};
	~AStar(){
		
	}
	

	void initGridMap(std::string filemap){
		if(!filemap.empty()){
			cv::Mat iamge,gray,dst;
			iamge = cv::imread(filemap);
			cv::cvtColor(iamge, gray, cv::COLOR_BGR2GRAY);//先转为灰度图
			cv::threshold(gray, dst, 0, 255, cv::THRESH_OTSU);
			// Gridlocalmap(iamge.cols,iamge.rows);/
			int buffer_size = iamge.cols * iamge.rows ;//传入时间信息
			// md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
			gridmap initgrid;
			initgrid.ifoccupy=0;
			initgrid.time=0;
			map_voxel_num_(0)=iamge.cols;
			map_voxel_num_(1)=iamge.rows;
			map_voxel_num_(2)=1;
  			// ofstream ofs;
            // std::string str11 = "/home/rancho/1hmy/ego-planner-swarm/record/map";
            // std::string str1txt=str11+std::to_string(0);
            // str11=str1txt+".txt";
            // ofs.open(str11,std::ios::app);
            
              
			gridML=vector<gridmap>(buffer_size, initgrid);
			allgridML=vector<gridmap>(buffer_size, initgrid);
			for(int i = 0; i< iamge.cols; i++)
			{
				for(int j = 0; j< iamge.rows; j++){
					{
					if((int)dst.at<uchar>(j, i)==0)
						{gridML[j * iamge.cols + i].ifoccupy=1;
						// ofs<<j * iamge.cols + i<<" " <<1<<" "<<endl;
						}
					else
						{gridML[j * iamge.cols + i].ifoccupy=0;
						// ofs<<j * iamge.cols + i<<" " <<0<<" "<<endl;
						}
					gridML[j * iamge.cols + i].time=0;
					}
			}
			}
			// ofs.close(); 
		} 
		
	}
	bool initdymap=false;
	void initDyGridMap(std::vector<Eigen::Vector3d> localdy,int car_id_){
		// gridMLD.clear();
		// gridMLD=localdy;
		ofstream ofs;
		std::string str11 = "/home/rancho/1hmy/ego-planner-swarm/record/DyGridMap";
		std::string str1txt=str11+std::to_string(car_id_);
		str11=str1txt+".txt";
		ofs.open(str11,std::ios::app);

		allgridML.clear();
		allgridML.swap(gridML);
		for(int i = 0; i< localdy.size(); i++)
		{
		for(double j=-3.0;j<=3.0;j++)
			for(double k=-3.0;k<=3.0;k++)
				{allgridML[(localdy[i](1)+k) * map_voxel_num_(0) + (localdy[i](0)+j)].ifoccupy=1;
				allgridML[(localdy[i](1)+k) * map_voxel_num_(0) + (localdy[i](0)+j)].time=localdy[i](2);
				ofs<<localdy[i](0)+j<<" "<<localdy[i](1)+k<<" "<<localdy[i](2)<<endl;}
		}
		initdymap=true;
		ofs.close();
	}
bool isInMap(const Eigen::Vector3d& pos) {
  if (int(pos(0)) > map_voxel_num_(0) + 1e-4 || int(pos(1)) > map_voxel_num_(1) + 1e-4 ) {
    cout << "large than min range!" << endl;
	cout <<"pos0 "<<pos(0)<<endl;
	cout <<"pos1 "<<pos(1)<<endl;
	cout <<"pos2 "<<pos(2)<<endl;

	cout <<"map_voxel_num_0 "<<map_voxel_num_(0)<<endl;
	cout <<"map_voxel_num_(1) "<<map_voxel_num_(1)<<endl;

    return false;
  }
  if (pos(0) <0 || pos(1)<0) {
    return false;
  }
  return true;
}
	// int getInflateOccupancy(Eigen::Vector3d pos)
	// {
	// // pos(1)=map_voxel_num_(1)-pos(1);
	// // pos(0)-=1;
	// // pos(1)-=1;
	// // cout<<"pos(0)"<<pos(0)<<endl;
	// // cout<<"pos(1)"<<pos(1)<<endl;

 	// if (!isInMap(pos)) return 2;
	// return(gridML[Coord2Indexint(pos)].ifoccupy);
   
	// };
	bool getInflateOccupancy(const Eigen::Vector3d& pos)
	{
	// pos(1)=map_voxel_num_(1)-pos(1);
	// pos(0)-=1;
	// pos(1)-=1;
	// cout<<"pos(0)"<<pos(0)<<endl;
	// cout<<"pos(1)"<<pos(1)<<endl;
	cout<<" getInflateOccupancy "<<endl;
	if (!isInMap(pos)) return -1;
	// if(initdymap)
	// {
		// bool dyocu=getallOccupancy(pos);
		// if(dyocu)
			// return false;
		// else
		// {
		// if(gridML[Coord2Indexint(pos)].ifoccupy==1)
		// return true;
		// if(gridML[Coord2Indexint(pos)].ifoccupy==0)
		// return false;
		// }
		// }
		// else
		// {
		if(gridML[Coord2Indexint(pos)].ifoccupy==1)
		return true;
		if(gridML[Coord2Indexint(pos)].ifoccupy==0)
		return false;
		// }
   
	}
	bool getallOccupancy(const Eigen::Vector3d& pos )
	{
	// pos(1)=map_voxel_num_(1)-pos(1);
	// pos(0)-=1;
	// pos(1)-=1;
	// cout<<"pos(0)"<<pos(0)<<endl;
	// cout<<"pos(1)"<<pos(1)<<endl;
    cout<<" getallOccupancy "<<endl;
	if(initdymap)
{
	if (!isInMap(pos)) return -1;
	for(double i=-5.0;i<=5.0;i=i+1.0)
	{
	if((pos(0)+i>0) && (pos(1)+i>0) && (pos(0)+i<=map_voxel_num_(0)-1e-6) && (pos(1)+i<=map_voxel_num_(1)-1e-6))
	{	
	Vector3d pos_(pos(0)+i,pos(1)+i,pos(2));
	if(allgridML[Coord2Indexint(pos_)].ifoccupy==1 && abs(allgridML[Coord2Indexint(pos_)].time-pos(2))<1e-6)
	  return true;
	if(allgridML[Coord2Indexint(pos_)].ifoccupy==0)
	  return false;
	  }
	else 
	{
		return false;
	}
	  }
}
   else{
	return getInflateOccupancy(pos);
   }

   
	}
	// double diffvx(Eigen::Vector3d  last_but_one ,Eigen::Vector3d that_last)
	// {
		
	// 	return(that_last(0)-last_but_one(0));

	// }
	// double diffvy(Eigen::Vector3d  last_but_one ,Eigen::Vector3d that_last)
	// {
		
	// 	return(that_last(1)-last_but_one(1));

	// }
	// double difft(Eigen::Vector3d  last_but_one ,Eigen::Vector3d that_last)
	// {
		
	// 	return(that_last(2)-last_but_one(2));

	// }


// 	bool swarmcheck(int id,Vector3d chasepos)
// 	{
// 		double safetyRadius;
// 		double LB,LF,width;
// 		width=1.75;
// 		LB=0.65;
// 		LF=2;
// 		safetyRadius=sqrt(pow(width , 2) + pow(LB+LF, 2))/2;
// 		cout<<"swarm_a_path_size is "<<swarm_a_path.size()<<endl;
// 		if(swarm_a_path.size()>0)
// {
// 		for (auto a:swarm_a_path)
// 		{
// 			if(a.car_id>id)
// 			{continue;}
// 			else
// 			{
// 				if(chasepos(2)<a.end_pos_(2)&&chasepos(2)>a.start_pos_(2))
// 				{
// 					if(a.end_pos_(2)-a.start_pos_(2)<3)//局部搜索路径短，认为在全局规划时规避了一些风险
// 					{
// 					cout<<"a.end_pos_(2) is "<<a.end_pos_(2)<<endl;
// 					cout<<"a.start_pos_(2) is "<<a.start_pos_(2)<<endl;

// 					Eigen::Vector2d center_traj;
// 					center_traj(0)=(a.start_pos_(0)+a.end_pos_(0))/2;
// 					center_traj(1)=(a.start_pos_(1)+a.end_pos_(1))/2;
// 					double dist;
// 					dist=sqrt(pow(center_traj(0)-chasepos(0),2)+pow(center_traj(1)-chasepos(1),2))*0.069;//根据速度和车身大小判断,现实世界尺度因素（29像素一米）（0.5缩小后，14.5像素一米，一像素6.9cm）
// 						if(dist<0.5)
// 						{
// 						return false;}
// 					}
// 					else
// 					{
// 					for (int aa=1;aa<a.local_astar_traj_.size();aa++)
// 					{
// 					if(abs(chasepos(2)-a.local_astar_traj_[aa](2))<2)
// 					{
// 					double vx,vy;
// 					vx=diffvx(a.local_astar_traj_[aa-1],a.local_astar_traj_[aa])/difft(a.local_astar_traj_[aa-1],a.local_astar_traj_[aa]);
// 					vy=diffvy(a.local_astar_traj_[aa-1],a.local_astar_traj_[aa])/difft(a.local_astar_traj_[aa-1],a.local_astar_traj_[aa]);
// 					cout<<"vx is "<<vx<<endl;
// 					cout<<"vy is "<<vy<<endl;
// 					if((vx*0.3+vy*0.3)*0.069>safetyRadius*2 ||(vx*0.3+vy*0.3)*0.069<0.1)//2.09*2
// 					{
					
// 						return true;
// 					}
						
// 					else
// 						return false;

// 					}
// 					else
// 					{ continue;}

// 					}

// 					}
					

// 				}
			

// 			}
// 		}


// }		



// 	}
private:
    bool dynamicoccupy(Vector3d chasepos)
	{
		if(initdymap)
		{
		for (int i = 0; i < gridMLD.size(); i++)
		{
		// cout<<"dis "<<sqrt(pow(gridMLD[i][0]-chasepos(0),2)+pow(gridMLD[i][1]-chasepos(1),2))<<endl;
		// cout<<"distime "<<abs(chasepos(2)-gridMLD[i][2])<<endl;

		if(sqrt(pow(gridMLD[i][0]-chasepos(0),2)+pow(gridMLD[i][1]-chasepos(1),2))<1.0 && abs(chasepos(2)-gridMLD[i][2])<1e-6)
			{return false;}
		}

		}
		return true;

	}
	double admissibleHeuristic(Vector3d node){

		 double h_cost = 1.1*sqrt(pow(node(0) - goal(0),2)+pow(node(1) - goal(1),2));
		 return h_cost;
	}
	bool isSolution(Vector3d node){
		// if(startp(0)+(int(node(0))-int(startp(0)))*0.15 == goal(0) && startp(1)+(int(node(1))-int(startp(1)))*0.15 == goal(1) && node(2)>startp(2))
		// 	return true;
		// else 
		// 	return false;
		return int(node(0)) == goal(0) && int(node(1)) == goal(1) && node(2)>startp(2) ;
	}
 	void getNeighbors(Vector3d n, std::vector<pair<Vector3d, double>>  &neighbors,int car_id){
		cout<<"n is "<<n(0)<<" "<<n(1)<<" "<<n(2)<<endl;
		for(int i = -1; i <= 1; i++)
			for(int j = -1; j<=1; j++){
				if(i == 0&&j ==0) continue;
				Vector3d nei = Vector3d(n(0)+i, n(1)+j, n(2)+0.3);
				// if(!gridML[map_voxel_num_(0)*nei(1)+nei(0)].ifoccupy && nei(0)<map_voxel_num_(0) && nei(1)<map_voxel_num_(1)&& nei(0)>1e-6 && nei(1)>1e-6 && dynamicoccupy(nei))
				// if(!gridML[map_voxel_num_(0)*nei(1)+nei(0)].ifoccupy && nei(0)<map_voxel_num_(0) && nei(1)<map_voxel_num_(1)&& nei(0)>0 && nei(1)>0)
				if(nei(0)<map_voxel_num_(0) && nei(1)<map_voxel_num_(1)&& nei(0)>1e-6 && nei(1)>1e-6 && !getallOccupancy(nei) )
				{
					neighbors.push_back(make_pair(nei,sqrt(i*i+j*j)));//判断动态和时间
					}
				if(nei(0)>=map_voxel_num_(0))
				{
					nei(0)=map_voxel_num_(0);
					neighbors.push_back(make_pair(nei,sqrt(i*i+j*j)));//判断动态和时间

				}
				if(nei(1)>=map_voxel_num_(1))
				{
					nei(1)=map_voxel_num_(1);
					neighbors.push_back(make_pair(nei,sqrt(i*i+j*j)));//判断动态和时间

				}

				

			}

	}
	Vector3d Index2Coord(const  int &index){
		Eigen::Vector3d po;
		po(0)=index%map_voxel_num_(0);
		po(1)=floor(index/map_voxel_num_(0));
		po(2)=0;
		return po;
	};
	 int Coord2Indexint(const Eigen::Vector3d &pt){
		int idx;
		// idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;
		// idx = int(pt(1))*map_voxel_num_(0)+map_voxel_num_(0)-int(pt(0));
		// ofstream ofs;
		// std::string str11 = "/home/rancho/1hmy/ego-planner-swarm/record/mapid";
		// std::string str1txt=str11+std::to_string(0);
		// str11=str1txt+".txt";
		// ofs.open(str11,std::ios::app);
		// ofs<<idx<<endl;
		// ofs.close();
		// if (int(pt(1))*map_voxel_num_(0)+int(pt(0))< 0 || int(pt(1))*map_voxel_num_(0)+int(pt(0)) > map_voxel_num_(0)*map_voxel_num_(1) ){
		if (pt(1)*map_voxel_num_(0)+pt(0)< 1e-6 || pt(1)*map_voxel_num_(0)+pt(0) > map_voxel_num_(0)*map_voxel_num_(1)-1e-6 ){

			std::cout<<"pt is "<<pt(0)<<" "<<pt(1)<<" "<<pt(2)<<std::endl;
			std::cout<<"map_voxel_num_ is "<<map_voxel_num_(0)<<" "<<map_voxel_num_(1)<<" "<<map_voxel_num_(2)<<std::endl;
			ROS_ERROR("Ran out of pool, index=%d ", pt(1)*map_voxel_num_(0)+pt(0));
			return -1;
		}
		// idx = int(pt(1))*map_voxel_num_(0)+int(pt(0));
		idx = int(pt(1)*map_voxel_num_(0)+pt(0));

		
		return idx;
	};

public:
	std::vector<gridmap>  gridML,allgridML;
	std::vector<Eigen::Vector3d> gridMLD;
	Vector3i map_voxel_num_;
bool AstarSearch(Vector3d start, Vector3d end,vector<Vector3d> &solution,int car_id_){
		// malloc_trim(0);
		ros::Time time_1 = ros::Time::now();
		goal = Vector3i(int(end(0)),int(end(1)),int(end(2)));
		// 对solution进行初始化 清0
		solution.clear();
	    startp=Vector3d(start(0),start(1),start(2));
		
		cout<<"goal is "<<goal(0)<<" "<<goal(1)<<" "<<goal(2)<<std::endl;
		cout<<"start is "<<start(0)<<" "<<start(1)<<" "<<start(2)<<std::endl;
		// openset      容器存放Node
		// closeset     容器存放Node
		// cameForm     容器是存放 state parent_state action fsocre gscore
		//              （主要是存放neighbor和生成的无碰撞轨迹）  其实跟close 功能一样
		// stateToHeap  容器存放index和node
		//              （存放所有被找过的节点 只要被search过就放入statetoheap） 跟open功能一样

		openSet_t openSet;
    // std::unordered_map<uint64_t, fibHeapHandle_t, std::hash<uint64_t>> stateToHeap;
    // std::unordered_set<uint64_t, std::hash<uint64_t>> closedSet;
    // std::unordered_map<State, std::tuple<State, Cost, Cost>,StateHasher>   cameFrom;
	
		std::unordered_map<int, fibHeapHandle_t, std::hash<int>> stateToHeap;
		std::unordered_set<int, std::hash<int>> closedSet;
		std::unordered_map<int, std::tuple<Vector3d, double, double>,
						std::hash<int>> cameFrom;
		auto handle = openSet.push(Node(start, admissibleHeuristic(start), 0.0));
    	stateToHeap.insert(std::make_pair<>(Coord2Indexint(start), handle));
    	(*handle).handle = handle;
    	std::vector<pair<Vector3d, double>> neighbors;
    	neighbors.reserve(10);
    	while (!openSet.empty()) {
			Node current = openSet.top();

			if (isSolution(current.state)) {
				// cout<<"neighbors.size()"<<endl;
				solution.clear();
    			cout<<" current "<<endl;
				auto iter = cameFrom.find(Coord2Indexint(current.state));
				Vector3d father = current.state;
				while (iter != cameFrom.end()) {
					solution.push_back(father);
					father = std::get<0>(iter->second);
    				cout<<" std::get<0> "<<endl;
					iter = cameFrom.find(Coord2Indexint(std::get<0>(iter->second)));
				}
				solution.push_back(start);
				std::reverse(solution.begin(), solution.end());

				return true;
			}

			openSet.pop();
    		cout<<" stateToHeap::erase "<<endl;
			stateToHeap.erase(Coord2Indexint(current.state));
    		cout<<" stateToHeap::insert "<<endl;
			bool find_ok=(closedSet.find(Coord2Indexint(current.state)) != closedSet.end());
    		cout<<" find_ok "<<endl;

			if (!find_ok)
				{closedSet.insert(Coord2Indexint(current.state));}
			// std::cout<<"current is "<<current.state(0)<<" "<<current.state(1)<<" "<<current.state(2)<<std::endl;
			// traverse neighbors
			neighbors.clear();
			getNeighbors(current.state, neighbors,car_id_);
    		if(neighbors.size()>0)
			{
    		cout<<" find_ok neighbors "<<endl;

		    for (const pair<Vector3d, double>& neighbor : neighbors) {
        		if (closedSet.find(Coord2Indexint(neighbor.first)) == closedSet.end()) {
					double tentative_gScore = current.gScore + neighbor.second;
					auto iter = stateToHeap.find(Coord2Indexint(neighbor.first));
					if (iter == stateToHeap.end()) {  // Discover a new node
						double fScore =
							tentative_gScore + admissibleHeuristic(neighbor.first);
						auto handle =
							openSet.push(Node(neighbor.first, fScore, tentative_gScore));
						(*handle).handle = handle;
						stateToHeap.insert(std::make_pair<>(Coord2Indexint(neighbor.first), handle));
					}
					 else {
						auto handle = iter->second;
						if (tentative_gScore >= (*handle).gScore) 
							continue;

					// update f and gScore
						double delta = (*handle).gScore - tentative_gScore;
						(*handle).gScore = tentative_gScore;
						(*handle).fScore -= delta;
						openSet.increase(handle);
          			}	
					cameFrom.erase(Coord2Indexint(neighbor.first));
					cameFrom.insert(std::make_pair<>(Coord2Indexint(neighbor.first),
					std::make_tuple<>(current.state,neighbor.second,tentative_gScore)));
        		}
      		}

			}
      		
		ros::Time time_2 = ros::Time::now();
        if ((time_2 - time_1).toSec() > 0.2)
        {
            ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            return false;
        }
    	}

 	// ros::Time time_3 = ros::Time::now();
    //     if ((time_3 - time_1).toSec() > 0.2)
    //     {
    //         ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
    //         return false;
    //     }
    	
	};





private:
	
	
	Vector3i goal;
	Vector3d startp;
	

	typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
  	typedef typename openSet_t::handle_type fibHeapHandle_t;
	

	
};

#endif
