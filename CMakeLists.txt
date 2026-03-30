cmake_minimum_required(VERSION 3.14)
project(slam_case_study)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(slam_sandbox
  src/main.cpp
  src/slam_algorithm.cpp
)

target_link_libraries(slam_sandbox
  Eigen3::Eigen
)
target_include_directories(slam_sandbox PRIVATE
  include/
)
