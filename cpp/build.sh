#!/bin/bash

cd build
cmake -DCMAKE_PREFIX_PATH=/opt/drake ..
make -j$(nproc)
