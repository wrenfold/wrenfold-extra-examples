// Benchmark the generated method vs the handwritten GTSAM implementation.
#include <benchmark/benchmark.h>
#include <fmt/ostream.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include "generated/pose3_interpolate.h"

auto get_test_poses() {
  const Sophus::SE3d world_T_a{
      Eigen::Quaterniond(0.410182, 0.280694, 0.857538, 0.132633).normalized(),
      Eigen::Vector3d{-2.0, 3.4, 0.7}};
  const Sophus::SE3d world_T_b{
      Eigen::Quaterniond(0.772165, -0.596797, -0.191188, 0.105078).normalized(),
      Eigen::Vector3d{1.2, -4.32, 0.1}};
  return std::make_tuple(world_T_a, world_T_b);
}

void BM_Generated(benchmark::State& state) {
  const auto [world_T_a, world_T_b] = get_test_poses();

  for (auto _ : state) {
    for (const double alpha : {0.0, 0.5, 1.0}) {
      Eigen::Matrix<double, 6, 6> D_a_analytical, D_b_analytical;
      const Sophus::SE3d world_T_interpolated =
          gen::pose3_interpolate(world_T_a, world_T_b, alpha, D_a_analytical, D_b_analytical);

      benchmark::DoNotOptimize(world_T_interpolated);
      benchmark::DoNotOptimize(D_a_analytical);
      benchmark::DoNotOptimize(D_b_analytical);
    }
  }
}

void BM_GeneratedFirstOrder(benchmark::State& state) {
  const auto [world_T_a, world_T_b] = get_test_poses();

  for (auto _ : state) {
    for (const double alpha : {0.0, 0.5, 1.0}) {
      Eigen::Matrix<double, 6, 6> D_a_analytical, D_b_analytical;
      const Sophus::SE3d world_T_interpolated = gen::pose3_interpolate_first_order(
          world_T_a, world_T_b, alpha, D_a_analytical, D_b_analytical);

      benchmark::DoNotOptimize(world_T_interpolated);
      benchmark::DoNotOptimize(D_a_analytical);
      benchmark::DoNotOptimize(D_b_analytical);
    }
  }
}

void BM_GTSAM(benchmark::State& state) {
  const auto [world_T_a, world_T_b] = get_test_poses();

  for (auto _ : state) {
    for (const double alpha : {0.0, 0.5, 1.0}) {
      Eigen::Matrix<double, 6, 6> D_a_gtsam, D_b_gtsam;
      const gtsam::Pose3 world_T_interoplated_gtsam = gtsam::interpolate(
          gtsam::Pose3{gtsam::Rot3(world_T_a.so3().unit_quaternion()), world_T_a.translation()},
          gtsam::Pose3{gtsam::Rot3(world_T_b.so3().unit_quaternion()), world_T_b.translation()},
          alpha, &D_a_gtsam, &D_b_gtsam);

      benchmark::DoNotOptimize(world_T_interoplated_gtsam);
      benchmark::DoNotOptimize(D_a_gtsam);
      benchmark::DoNotOptimize(D_b_gtsam);
    }
  }
}

BENCHMARK(BM_Generated);
BENCHMARK(BM_GeneratedFirstOrder);
BENCHMARK(BM_GTSAM);
