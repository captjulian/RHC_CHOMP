cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME TR_PLANNER_SDF)

project(${PROJECT_NAME})

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(TR_PLANNER_SDF_SOURCE_DIR
        src/sources)

set(TR_PLANNER_SDF_INCLUDE_DIR
        src/include
        )

    set(TR_PLANNER_SDF_SOURCE_FILES
        ${TR_PLANNER_SDF_SOURCE_DIR}/sdf_planner.cpp		
        ${TR_PLANNER_SDF_SOURCE_DIR}/mtrand.cpp
	${TR_PLANNER_SDF_SOURCE_DIR}/rrtstar.cpp
	${TR_PLANNER_SDF_SOURCE_DIR}/chomp_opt.cpp
            )


    set(TR_PLANNER_SDF_HEADER_FILES

	${TR_PLANNER_SDF_INCLUDE_DIR}/sdf_planner.h
	${TR_PLANNER_SDF_INCLUDE_DIR}/mtrand.h
        ${TR_PLANNER_SDF_INCLUDE_DIR}/rrtstar.h
	${TR_PLANNER_SDF_INCLUDE_DIR}/chomp_opt.h
            )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONE	NTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy  
  robot_process
  droneMsgsROS
  opencv_apps
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  message_generation
  cv_bridge
  image_transport
  mav_trajectory_generation_ros
  voxblox_ros
)



#if OpenCV
find_package(OpenCV REQUIRED)
#set(OpenCV_DIR /home/liang/third_part_package/opencv-3.3.0/build)
#find_package(OpenCV REQUIRED PATHS /home/liang/third_part_package/opencv-3.3.0/cmake)
#if(NOT OpenCV_FOUND)
#  message(FATAL_ERROR "Failed to find OpenCV")
#endif(NOT OpenCV_FOUND)
#include_directories(${OpenCV_INCLUDE_DIR})
#MESSAGE(STATUS "OPENCVLIB IS : " ${OpenCV_INCLUDE_DIR})
#MESSAGE(STATUS "OpenCV_DIR is: " ${OpenCV_INCLUDE_DIRS})
#MESSAGE(STATUS "OpenCV_LIBS is: " ${OpenCV_LIBS})


find_package(Eigen3 REQUIRED)
##add_definitions(${EIGEN_DEFINITIONS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#add_message_files(
#  FILES

#)

#add_service_files(
#  FILES
#  nbvp_srv.srv
#)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs

)

###################################
## catkin specific configuration ##


catkin_package(
  CATKIN_DEPENDS roscpp std_msgs 
                 droneMsgsROS robot_process opencv_apps message_runtime
                 geometry_msgs nav_msgs mav_trajectory_generation_ros
                 sensor_msgs visualization_msgs cv_bridge image_transport
	         voxblox_ros
  

  DEPENDS OpenCV Eigen
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

##include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
    ${TR_PLANNER_SDF_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)


#include_directories(${OpenCV_INCLUDE_DIRS})


add_library(TR_PLANNER_SDF_lib ${TR_PLANNER_SDF_SOURCE_FILES} ${TR_PLANNER_SDF_HEADER_FILES})
add_dependencies(TR_PLANNER_SDF_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(TR_PLANNER_SDF_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBS})

add_executable(pose_transform_node src/sources/pose_transform_node.cpp)
add_dependencies(pose_transform_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_transform_node TR_PLANNER_SDF_lib ${OpenCV_LIBRARY})


add_executable(sdf_planner_node src/sources/sdf_planner_node.cpp)
add_dependencies(sdf_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(sdf_planner_node TR_PLANNER_SDF_lib ${OpenCV_LIBRARY})
