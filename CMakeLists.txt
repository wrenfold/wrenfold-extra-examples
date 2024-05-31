cmake_minimum_required(VERSION 3.20)
project(
  wrenfold-extra-examples
  VERSION 0.1
  DESCRIPTION "Sample projects using wrenfold."
  LANGUAGES CXX)

add_subdirectory(dependencies)

if(MSVC)
  set(WARNING_FLAGS /W4 /WX /wd4244 /wd4702)
else()
  set(WARNING_FLAGS -Wall -Wextra -pedantic -Werror -Wno-sign-compare
                    -Wno-unused-parameter)
endif()
