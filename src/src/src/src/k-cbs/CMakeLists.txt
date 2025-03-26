#set(CMAKE_BUILD_TYPE Release)
cmake_minimum_required(VERSION 2.8.3)
project(k_cbs)
SET(CMAKE_BUILD_TYPE Debug) 



add_definitions(-DNDEBUG)
add_definitions(-DIL_STD)
set(CPLEX_PREFIX_DIR      /opt/ibm/ILOG/CPLEX_Studio201)
set(CPLEX_INCLUDE_DIR     ${CPLEX_PREFIX_DIR}/cplex/include)
set(CPLEX_LIBRARIES_DIR   ${CPLEX_PREFIX_DIR}/cplex/lib/x86-64_linux/static_pic)
set(CONCERT_INCLUDE_DIR   ${CPLEX_PREFIX_DIR}/concert/include)
set(CONCERT_LIBRARIES_DIR ${CPLEX_PREFIX_DIR}/concert/lib/x86-64_linux/static_pic)
include_directories(${CPLEX_INCLUDE_DIR} ${CONCERT_INCLUDE_DIR})
link_directories(${CPLEX_LIBRARIES_DIR} ${CONCERT_LIBRARIES_DIR})




find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs
    nav_msgs
    visualization_msgs 
    traj_utils
    
)
find_package(ompl REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(PkgConfig REQUIRED)
# find_package(CppAD REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
endif()
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES k_cbs
 

)
if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)
include_directories(
    include 
    SYSTEM 
    third_party
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)

# set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall") 
include_directories(/usr/local/include)
# 添加头文件路径
include_directories(${CppAD_INCLUDE_DIRS} ${IPOPT_INCLUDE_DIRS})
link_directories(/usr/local/lib)
# -Wextra -Werror
# 链接库


add_executable( cbs_node 
    src/main.cpp
)
add_executable( show_node 
    src/show.cpp
)
add_library( k_cbs 
    src/main.cpp
    # src/pathc_cal.hpp
    )



target_link_libraries(cbs_node 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
    ${YAMLCPP_LIBRARIES}
    ${OpenCV_LIBS}
    ${OMPL_LIBRARIES}
    m
    ilocplex
    cplex
    concert
    pthread
    dl
    ipopt
)
target_link_libraries(show_node 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
    ${YAMLCPP_LIBRARIES}
    ${OpenCV_LIBS}
    ${OMPL_LIBRARIES}
    m
    ilocplex
    cplex
    concert
    pthread
    dl
    
)


    