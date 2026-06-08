#!/bin/bash

cd build
cmake -DCMAKE_PREFIX_PATH="/opt/drake;/home/noor/ompl-2.0" ..
make -j$(nproc)
