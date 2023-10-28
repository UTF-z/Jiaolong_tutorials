cmake_minimum_required(VERSION 3.10)
project(kalman_tutorial)

add_subdirectory(pybind11)
pybind11_add_module(Filters kalman_filter.cpp)