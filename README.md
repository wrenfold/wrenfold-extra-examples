# wrenfold-extra-examples

This repository contains additional examples for the [wrenfold](https://wrenfold.org) code-generation framework. These samples in this repository are intended to illustrate how generated code can be integrated into different optimizers and third-party frameworks.

For examples of using and customizing wrenfold code-generation, see the [main repo](https://github.com/wrenfold/wrenfold/tree/main/examples).

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
