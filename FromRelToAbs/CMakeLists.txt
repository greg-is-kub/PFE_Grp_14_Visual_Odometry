cmake_minimum_required(VERSION 2.8)
project(PosesTreatment)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# Eigen
include_directories("/usr/include/eigen3")

# g2o 
#find_package(G2O REQUIRED)
#include_directories(${G2O_INCLUDE_DIRS})

add_executable(PosesTreatment PosesTreatment.cpp)
#target_link_libraries(PosesTreatment
#        g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES}
#        )


