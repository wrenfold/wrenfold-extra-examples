# sophus_pose3_interpolation

This example generates a function that performs tangent-space interpolation between two elements
of SE(3). To illustrate integration into third-party frameworks, we generate a function that can accept and return [Sophus](https://github.com/strasdat/Sophus) types.

1. We first define a symbolic implementation of [an SE(3) pose](../pose3.py) with a handful of useful methods such as composition and inversion.
2. We then implement a [custom C++ generator](gen_pose3_interpolation.py) that can customize the emitted code to access members on Sophus types.
3. Lastly, we [test](sophus_pose3_interpolation_test.cc) the generated implementation by comparing the numerical results to `gtsam::interpolate`.

## Benchmarks

We include a small [benchmark](sophus_pose3_interpolation_bench.cc) that compares three implementations:

- Two code-generated versions that compute tangent-space derivatives using the "chain rule" and "first order" methods. See sections `B.1` and `B.2` of the [SymForce paper](https://arxiv.org/abs/2204.07889).
- Calling `gtsam::interpolate` with two `gtsam::Pose3` objects.

The following results were collected using GCC 12.3 on Ubuntu 22.04. As always, your mileage will vary depending on choice of compiler and compilation flags applied.
```
Running build/examples/sophus_pose3_interpolation/sophus_pose3_interpolation_bench
Run on (16 X 2304 MHz CPU s)
CPU Caches:
  L1 Data 48 KiB (x8)
  L1 Instruction 32 KiB (x8)
  L2 Unified 1280 KiB (x8)
  L3 Unified 24576 KiB (x1)
Load Average: 0.04, 0.13, 0.09
-----------------------------------------------------------------
Benchmark                       Time             CPU   Iterations
-----------------------------------------------------------------
BM_Generated                 2417 ns         2417 ns       287265
BM_GeneratedFirstOrder       1958 ns         1958 ns       370139
BM_GTSAM                     1996 ns         1996 ns       354310
```

We note that the GTSAM impementation `BM_GTSAM` beats the generated chain-rule `BM_Generated` implementation. This is not terribly surprising, since it is able to leverage certain analytical simplifications that are possible when directly chaining together tangent-space Jacobians. However, applying the first-order method of retraction allows the generated method to achieve competitive results.
