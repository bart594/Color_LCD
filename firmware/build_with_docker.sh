#!/bin/bash
mkdir -p build

DOCKER_BUILDKIT=0 docker build -f docker/Dockerfile . -t sw102_build
docker run --rm -v $PWD/build:/build sw102_build /bin/bash -c "cp /source/SW102/_build/*.hex /build; cp /source/SW102/_release/*.zip /build"
