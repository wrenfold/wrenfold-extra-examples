# Create a target for eigen.
function(add_eigen)
  set(EIGEN_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/eigen")
  set(EIGEN3_INCLUDE_DIR
      ${EIGEN_INCLUDE_DIRS}
      PARENT_SCOPE)
  if(NOT TARGET eigen)
    add_library(eigen INTERFACE IMPORTED GLOBAL)
    set_target_properties(eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                           "${EIGEN_INCLUDE_DIRS}")
  endif()
  # Add alias for ceres:
  add_library(Eigen3::Eigen ALIAS eigen)
endfunction()
add_eigen()

function(add_gflags)
  set(BUILD_TESTING
      OFF
      CACHE BOOL "" FORCE)
  set(BUILD_PACKAGING
      OFF
      CACHE BOOL "" FORCE)
  add_subdirectory(gflags)
endfunction()
add_gflags()

function(add_glog)
  set(BUILD_SHARED_LIBS
      OFF
      CACHE BOOL "" FORCE)
  set(BUILD_TESTING
      OFF
      CACHE BOOL "" FORCE)
  set(WITH_GFLAGS OFF CACHE BOOL "" FORCE)
  set(WITH_GTEST OFF CACHE BOOL "" FORCE)
  set(WITH_UNWIND OFF CACHE BOOL "" FORCE)
  add_subdirectory(glog)
endfunction()
add_glog()

# Google Ceres
function(add_ceres)
  set(MINIGLOG
      ON
      CACHE BOOL "" FORCE)
  set(GFLAGS
      OFF
      CACHE BOOL "" FORCE)
  set(SUITESPARSE
      OFF
      CACHE BOOL "" FORCE)
  set(USE_CUDA
      OFF
      CACHE BOOL "" FORCE)
  set(BUILD_TESTING
      OFF
      CACHE BOOL "" FORCE)
  set(BUILD_BENCHMARKS
      OFF
      CACHE BOOL "" FORCE)
  set(BUILD_EXAMPLES
      OFF
      CACHE BOOL "" FORCE)
  set(SCHUR_SPECIALIZATIONS
      OFF
      CACHE BOOL "" FORCE)
  add_subdirectory(ceres-solver)
endfunction()
add_ceres()

# Create an interface library for the wrenfold runtime headers
function(add_wrenfold_headers)
  add_library(wrenfold_runtime INTERFACE)
  target_include_directories(
    wrenfold_runtime
    INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/wrenfold>
              $<INSTALL_INTERFACE:include>)
  target_compile_definitions(wrenfold_runtime INTERFACE -DWF_SPAN_EIGEN_SUPPORT)
  target_compile_features(wrenfold_runtime INTERFACE cxx_std_17)
endfunction()
add_wrenfold_headers()
