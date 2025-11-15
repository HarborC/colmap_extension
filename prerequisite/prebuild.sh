#!/bin/bash

BASE_DIR=$(cd "$(dirname "$0")" && pwd)
cd "${BASE_DIR}"

sed -i '469s/const //' ./colmap/src/colmap/estimators/bundle_adjustment.cc

cd colmap
rm -r build
mkdir build
cd build
cmake .. -GNinja
ninja 
sudo ninja install
cd ../
rm -r build
cd ../

sed -i '469s/Point3D/const Point3D/' ./colmap/src/colmap/estimators/bundle_adjustment.cc