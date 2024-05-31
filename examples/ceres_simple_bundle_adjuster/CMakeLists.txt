add_executable(simple_bundle_adjuster simple_bundle_adjuster.cc
                                      generated/snavely_reprojection_error.h)
target_link_libraries(
  simple_bundle_adjuster PRIVATE Ceres::ceres wrenfold_runtime Eigen3::Eigen
                                 glog::glog)
