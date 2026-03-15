# wrenfold-extra-examples

This repository is archived, and the supported examples have been merged into the [main repository](https://github.com/wrenfold/wrenfold).

## Building

You will need [Eigen](https://eigen.tuxfamily.org/dox/) and Boost installed in a location where cmake can find them.

Make sure you have cloned the requisite submodules:
```bash
git submodule update --init --recursive
```

To configure and build the examples:
```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -Wno-deprecated
cmake --build .
```

Alternatively, there is an Ubuntu [Dockerfile](./Dockerfile).
