cmake_minimum_required(VERSION 2.8.3)
project(generate_pointcloud)

#c++11 standrard
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(OpenCV_DIR "/usr/local/include/opencv2")

# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )

# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io )

# set(PCL_INCLUDE_DIRS "/usr/include/pcl-1.7/pcl")
# 添加头文件和库文件
find_package( PCL REQUIRED COMPONENTS common io )

INCLUDE_DIRECTORIES(# include
		${PCL_INCLUDE_DIRS})
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )

ADD_EXECUTABLE( generate_pointcloud generatePointCloud.cpp )
TARGET_LINK_LIBRARIES( generate_pointcloud  
    ${PCL_LIBRARIES} ${OpenCV_LIBS})
