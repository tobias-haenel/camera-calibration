#!/usr/bin/sh

BUILD=Release

set -e
mkdir -p build/${BUILD}
cd build/${BUILD}
cmake -DOpenCV_DIR=/usr/local/share/OpenCV \
      -DOpenIGTLink_DIR=/usr/local/lib/igtl/cmake/igtl-3.1\
      -DCMAKE_BUILD_TYPE=${BUILD}\
      ../../
make -j 5
./camera-calibration ../../data/settings.xml
