# Add unit test:
add_executable(sophus_pose3_interpolation_test
               sophus_pose3_interpolation_test.cc generated/pose3_interpolate.h)
target_link_libraries(
  sophus_pose3_interpolation_test
  PRIVATE wrenfold_runtime Eigen3::Eigen fmt::fmt Sophus::Sophus gtest_main
          gtsam)
target_compile_options(sophus_pose3_interpolation_test PRIVATE ${WARNING_FLAGS})

if(NOT MSVC)
  target_compile_options(sophus_pose3_interpolation_test
                         PRIVATE -Wno-deprecated-copy -Wno-unused-parameter)
endif()

# Add benchmark:
add_executable(
  sophus_pose3_interpolation_bench sophus_pose3_interpolation_bench.cc
                                   generated/pose3_interpolate.h)
target_link_libraries(
  sophus_pose3_interpolation_bench
  PRIVATE wrenfold_runtime Eigen3::Eigen fmt::fmt Sophus::Sophus
          benchmark::benchmark_main gtsam)
target_compile_options(sophus_pose3_interpolation_bench
                       PRIVATE ${WARNING_FLAGS})

if(NOT MSVC)
  target_compile_options(sophus_pose3_interpolation_bench
                         PRIVATE -Wno-deprecated-copy -Wno-unused-parameter)
endif()
