cmake_minimum_required(VERSION 3.20)
project(
  wrenfold-extra-examples
  VERSION 0.1
  DESCRIPTION "Sample projects using wrenfold."
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED)

set(SHARED_WARNING_FLAGS "")
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL
                                             "AppleClang")
  list(APPEND SHARED_WARNING_FLAGS -Wno-gnu-zero-variadic-macro-arguments)
endif()

add_subdirectory(dependencies)
add_subdirectory(examples)
