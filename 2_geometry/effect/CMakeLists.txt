cmake_minimum_required(VERSION 3.10)
project(transform)

add_executable(motion ./main.cpp)
find_package(OpenCV REQUIRED)
target_include_directories(motion PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(motion PRIVATE ${OpenCV_LIBS})

find_package(Eigen3 REQUIRED)
target_include_directories(motion PRIVATE ${EIGEN3_INCLUDE_DIRES})