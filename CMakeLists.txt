include_directories("/usr/include/eigen3")

#add_executable(coordinateTransform coordinateTransform.cpp)

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

#add_executable(trajectory trajectory.cpp)
#target_link_libraries(trajectory ${Pangolin_LIBRARIES})

add_executable(test_trajectory test_trajectory.cpp trajectory.cpp)
target_link_libraries(test_trajectory ${Pangolin_LIBRARIES})
