#!/bin/bash

BASE_DIR=$(cd "$(dirname "$0")" && pwd)
cd "${BASE_DIR}"

# rm -r build
mkdir build
cd build
cmake .. -GNinja
ninja
# sudo ninja install