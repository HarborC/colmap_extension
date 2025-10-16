#!/bin/bash

BASE_DIR=$(cd "$(dirname "$0")" && pwd)
cd "${BASE_DIR}"

mkdir build
cd build
cmake .. -GNinja -DBLA_VENDOR=Intel10_64lp
ninja -j$(nproc)
sudo ninja install