function(add_libfmt)
  set(FMT_TEST
      OFF
      CACHE BOOL "" FORCE) # Turn off libfmt tests
  add_subdirectory(fmt EXCLUDE_FROM_ALL) # Don't need to install.
endfunction()
add_libfmt()

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

# GTSAM
function(add_gtsam)
  set(GTSAM_USE_SYSTEM_EIGEN ON CACHE BOOL "" FORCE)
  set(GTSAM_WITH_TBB OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_EXAMPLES_ALWAYS OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_UNSTABLE OFF CACHE BOOL "" FORCE)
  set(GTSAM_UNSTABLE_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
  set(GTSAM_USE_QUATERNIONS ON CACHE BOOL "" FORCE)
  set(GTSAM_THROW_CHEIRALITY_EXCEPTION OFF CACHE BOOL "" FORCE)
  add_subdirectory(gtsam)
  if (NOT MSVC)
    # Silence GTSAM4.2 deprecation warnings:
    target_compile_options(gtsam PUBLIC -Wno-deprecated-declarations)
    # Silence OOB access in BearingRangeFactor...
    target_compile_options(gtsam PRIVATE -Wno-array-bounds -Wno-range-loop-construct)
  endif ()
endfunction()
add_gtsam()

# Create an interface library for the wrenfold runtime headers
function(add_wrenfold_headers)
  add_library(wrenfold_runtime INTERFACE)
  target_sources(wrenfold_runtime INTERFACE wrenfold/wrenfold/span.h
                                            wrenfold/wrenfold/span_detail.h)
  target_include_directories(
    wrenfold_runtime
    INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/wrenfold>
              $<INSTALL_INTERFACE:include>)
  target_compile_definitions(wrenfold_runtime INTERFACE -DWF_SPAN_EIGEN_SUPPORT)
  target_compile_features(wrenfold_runtime INTERFACE cxx_std_17)
endfunction()
add_wrenfold_headers()
