cmake_minimum_required(VERSION 3.17)
project(osqp_cmpc)

set(CMAKE_CXX_STANDARD 17)

find_package(osqp REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(osqp_cmpc main.cpp osqp_solver.cpp osqp_solver.h robot_state.cpp robot_state.h)

target_link_libraries(osqp_cmpc PRIVATE osqp::osqp Eigen3::Eigen)