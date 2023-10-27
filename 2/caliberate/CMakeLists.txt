cmake_minimum_required(VERSION 3.10)
project(cali)

add_executable(cali ./main.cpp)
find_package(OpenCV REQUIRED)
target_include_directories(cali PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(cali PRIVATE ${OpenCV_LIBS})