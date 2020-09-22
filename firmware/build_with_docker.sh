#!/bin/bash
mkdir -p build

DOCKER_BUILDKIT=0 docker build -f docker/Dockerfile .