# wrenfold-extra-examples

This repository contains additional examples for the [wrenfold](https://wrenfold.org) code-generation framework. These samples in this repository are intended to illustrate how wrenfold-generated code can be integrated into different optimizers.

For examples of using and customizing wrenfold code-generation, see the [main repo](https://github.com/wrenfold/wrenfold/tree/main/examples).

## Building

Make sure you have cloned the requisite submodules:
```bash
git submodule update --init --recursive
```

To configure and run the benchmark suite:
```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -Wno-deprecated
cmake --build .
```
