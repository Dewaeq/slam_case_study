#!/bin/bash

set -e

mkdir -p build
cd build
cmake ..
make

./slam_sandbox

cd ..
python3 visualize.py