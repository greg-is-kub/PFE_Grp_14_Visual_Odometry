cmake_minimum_required(VERSION 2.8)
project(vo)

set(CMAKE_BUILD_TYPE "Release")
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${SSE_FLAGS} -msse4")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)



#set(ENV{PKG_CONFIG_PATH} /usr/lib/x86_64-linux-gnu/pkgconfig/)
#find_package(PkgConfig)
#pkg_search_module(OP REQUIRED opencv4)
find_package(Pangolin REQUIRED)
find_package(OpenCV 3 REQUIRED)
#find_package(G2O REQUIRED)


include_directories(
        #${OP_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        "/usr/include/eigen3/"
)

add_executable(test_trajectory trajectory.cpp ORB/orb.cpp Triangulation/triangulation.cpp ransac/ransac.cpp ICP/ICP.cpp video_slicer.cpp)
target_link_libraries(test_trajectory ${OpenCV_LIBS} ${Pangolin_LIBRARIES})




