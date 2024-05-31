cmake_minimum_required(VERSION 3.20)
project(
  wrenfold-extra-examples
  VERSION 0.1
  DESCRIPTION "Sample projects using wrenfold."
  LANGUAGES CXX)

find_package(Eigen3 3.3 REQUIRED)

add_subdirectory(dependencies)
add_subdirectory(examples)
