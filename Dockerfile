FROM ubuntu:24.04

RUN apt update -y
RUN apt upgrade -y

RUN apt update
RUN apt install -y gcc-14 g++-14

RUN apt install -y cmake ninja-build libboost-all-dev libeigen3-dev

COPY ./ /wrenfold-extra-examples

RUN mkdir /wrenfold-extra-examples/build
WORKDIR /wrenfold-extra-examples/build

RUN cmake .. -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_CXX_COMPILER=g++-14 \
  -Wno-deprecated -Wno-dev

RUN cmake --build . --parallel $(nproc)
