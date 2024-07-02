add_executable(simple_bundle_adjuster simple_bundle_adjuster.cc
                                      generated/snavely_reprojection_error.h)
target_link_libraries(
  simple_bundle_adjuster PRIVATE Ceres::ceres wrenfold_runtime Eigen3::Eigen
                                 glog::glog)
target_compile_options(simple_bundle_adjuster PRIVATE ${SHARED_WARNING_FLAGS})
if(NOT MSVC)
  target_compile_options(simple_bundle_adjuster PRIVATE -Wno-unused-parameter)
endif()
