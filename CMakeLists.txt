# cmake最低版本号要求
cmake_minimum_required(VERSION 3.14)

# 项目名称
project(rendering_demo)

# 设置Eigen3_DIR所在目录，对应eigen安装目录下的cmake目录，目录内包含有Eigen3Config.cmake等文件
set(Eigen3_DIR "./ext/Eigen/install/share/eigen3/cmake")
set(OpenCV_DIR "./ext/OpenCV/install")

# 搜索查询Eigen3
find_package(Eigen3 REQUIRED NO_MODULE)
# OpenCV
find_package(OpenCV REQUIRED)
# 添加OpenCV头文件
include_directories(${OpenCV_INCLUDE_DIRS})

#添加 /include 文件夹下的头文件
include_directories(${PROJECT_SOURCE_DIR}/include/)
# 添加 /src 文件夹下的文件
add_executable(rendering_demo src/main.cpp)

# 为项目可行执行文件引入eigen依赖
target_link_libraries(rendering_demo Eigen3::Eigen)
target_link_libraries(rendering_demo ${OpenCV_LIBS} )