cmake_minimum_required(VERSION 3.10)
project(pnp)

add_executable(pnp ./main.cpp)

target_include_directories(pnp PRIVATE ./source)
target_compile_features(pnp PRIVATE cxx_std_17)

find_package(OpenCV REQUIRED)
target_include_directories(pnp PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(pnp PRIVATE ${OpenCV_LIBS})

find_package(Eigen3 REQUIRED)
target_include_directories(pnp PRIVATE ${Eigen3_INCLUDE_DIRS})