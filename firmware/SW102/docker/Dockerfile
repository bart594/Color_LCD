# syntax = docker/dockerfile:experimental
FROM ubuntu:20.04
RUN apt update\
  && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi  make\
  && rm -rf /var/lib/apt/lists/*
RUN mkdir /build
COPY . /source
#RUN #--mount=type=bind,source=./build,target=/build,rw \
RUN cd /source/SW102 && make