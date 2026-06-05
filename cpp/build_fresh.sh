#!/bin/bash

rm -rf build/
mkdir -p build/
cd build
cmake -DCMAKE_PREFIX_PATH=/opt/drake ..
make -j$(nproc)
