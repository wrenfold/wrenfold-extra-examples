add_executable(gtsam_bal gtsam_bal.cc)
target_link_libraries(gtsam_bal PRIVATE gtsam wrenfold_runtime Eigen3::Eigen
                                        fmt::fmt-header-only)
target_compile_options(gtsam_bal PRIVATE ${WARNING_FLAGS})
if(NOT MSVC)
  # GTSAM requires non-iso extension vla
  target_compile_options(gtsam_bal PRIVATE -Wno-vla)
  target_compile_options(gtsam_bal PRIVATE -Wno-deprecated-copy
                                           -Wno-unused-parameter)
endif()
